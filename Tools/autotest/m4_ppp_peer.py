'''
A minimal PPP peer for the SITL M4 network process (SCRIPTING_M4_PPP=tcp:127.0.0.1:<port>).

It listens on TCP, answers LCP and IPCP as the address provider (our_ip for itself, his_ip for
the M4), records the UDP datagrams the M4 sends and can send UDP datagrams to the M4. Enough for
the status and flood autotests; it is no router.

AP_FLAKE8_CLEAN
'''

import socket
import struct
import threading
import time

PPP_IP = 0x0021
PPP_LCP = 0xC021
PPP_IPCP = 0x8021

CONF_REQ = 1
CONF_ACK = 2
CONF_NAK = 3
CONF_REJ = 4
TERM_REQ = 5
TERM_ACK = 6
PROTO_REJ = 8
ECHO_REQ = 9
ECHO_REPLY = 10

IPCP_ADDRESS = 3
MAX_PENDING = 65536    # bytes without a flag byte: not PPP, start again


def _fcs_table():
    table = []
    for b in range(256):
        v = b
        for _ in range(8):
            v = (v >> 1) ^ 0x8408 if v & 1 else v >> 1
        table.append(v)
    return table


FCS_TABLE = _fcs_table()


def fcs16(data, fcs=0xFFFF):
    for b in data:
        fcs = (fcs >> 8) ^ FCS_TABLE[(fcs ^ b) & 0xFF]
    return fcs


def ip_checksum(header):
    if len(header) % 2:
        header += b'\x00'
    total = sum(struct.unpack('!%uH' % (len(header) // 2), header))
    while total >> 16:
        total = (total & 0xFFFF) + (total >> 16)
    return ~total & 0xFFFF


def ip_bytes(text):
    return socket.inet_aton(text)


class M4PPPPeer(object):
    '''serves one PPP connection at a time; a new connection (an M4 restart) starts PPP again'''

    def __init__(self, port, our_ip, his_ip):
        self.our_ip = ip_bytes(our_ip)
        self.his_ip = ip_bytes(his_ip)
        self.listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind(('127.0.0.1', port))
        self.listener.listen(1)
        self.listener.settimeout(0.2)
        self.lock = threading.Lock()
        self.conn = None
        self.running = True
        self.ident = 0
        self.ipv4_up = False
        self.connections = 0
        self.udp_received = []      # (source port, destination port, payload)
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def close(self):
        self.running = False
        self.thread.join(5)
        self.listener.close()

    # framing
    def _send_frame(self, protocol, payload):
        body = b'\xff\x03' + struct.pack('!H', protocol) + payload
        fcs = fcs16(body) ^ 0xFFFF
        body += struct.pack('<H', fcs)
        out = bytearray(b'\x7e')
        for b in body:
            if b < 0x20 or b in (0x7e, 0x7d):
                out += bytes([0x7d, b ^ 0x20])
            else:
                out.append(b)
        out.append(0x7e)
        with self.lock:
            if self.conn is not None:
                try:
                    self.conn.sendall(bytes(out))
                except OSError:
                    pass

    def _send_control(self, protocol, code, ident, data):
        self._send_frame(protocol, struct.pack('!BBH', code, ident, 4 + len(data)) + data)

    def _next_ident(self):
        self.ident = (self.ident + 1) & 0xFF
        return self.ident

    def _frames(self, conn):
        pending = bytearray()
        while self.running:
            try:
                data = conn.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if not data:
                return
            pending += data
            if len(pending) > MAX_PENDING and b'\x7e' not in pending:
                pending.clear()
            while True:
                end = pending.find(b'\x7e')
                if end < 0:
                    break
                raw = pending[:end]
                del pending[:end + 1]
                frame = bytearray()
                escape = False
                for b in raw:
                    if b == 0x7d:
                        escape = True
                    elif escape:
                        frame.append(b ^ 0x20)
                        escape = False
                    else:
                        frame.append(b)
                if len(frame) < 4 or fcs16(frame) != 0xF0B8:
                    continue
                frame = frame[:-2]
                if frame[:2] == b'\xff\x03':
                    frame = frame[2:]
                if not frame:
                    continue
                if frame[0] & 1:
                    protocol, payload = frame[0], frame[1:]
                else:
                    protocol, payload = struct.unpack('!H', bytes(frame[:2]))[0], frame[2:]
                yield protocol, bytes(payload)

    def _run(self):
        while self.running:
            try:
                conn, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            conn.settimeout(0.2)
            with self.lock:
                self.conn = conn
                self.connections += 1
            self.ipv4_up = False
            lcp_open = False
            ipcp_acked = False
            self._send_control(PPP_LCP, CONF_REQ, self._next_ident(), b'')
            for protocol, payload in self._frames(conn):
                if protocol == PPP_LCP:
                    if self._lcp(payload) and not lcp_open:
                        # LCP is open: ask for our address once
                        lcp_open = True
                        self._send_control(PPP_IPCP, CONF_REQ, self._next_ident(),
                                           bytes([IPCP_ADDRESS, 6]) + self.our_ip)
                elif protocol == PPP_IPCP:
                    ipcp_acked = self._ipcp(payload) or ipcp_acked
                elif protocol == PPP_IP:
                    self._ipv4(payload)
                else:
                    self._send_control(PPP_LCP, PROTO_REJ, self._next_ident(), struct.pack('!H', protocol) + payload)
            with self.lock:
                self.conn = None
            self.ipv4_up = False
            conn.close()

    def _lcp(self, payload):
        '''True when the M4 acknowledged our (empty) Configure-Request'''
        if len(payload) < 4:
            return False
        code, ident = payload[0], payload[1]
        data = payload[4:]
        if code == CONF_REQ:
            self._send_control(PPP_LCP, CONF_ACK, ident, data)   # every option the M4 asks for
        elif code == CONF_ACK:
            return True
        elif code == TERM_REQ:
            self._send_control(PPP_LCP, TERM_ACK, ident, b'')
        elif code == ECHO_REQ:
            self._send_control(PPP_LCP, ECHO_REPLY, ident, b'\x00\x00\x00\x00' + data[4:])
        return False

    def _ipcp(self, payload):
        '''True when the M4 acknowledged our address'''
        if len(payload) < 4:
            return False
        code, ident = payload[0], payload[1]
        data = payload[4:]
        if code == CONF_ACK:
            self.ipv4_up = True
            return True
        if code != CONF_REQ:
            return False
        reject = b''
        nak = b''
        i = 0
        while i + 2 <= len(data):
            kind, length = data[i], data[i + 1]
            if length < 2:
                break
            option = data[i:i + length]
            if kind == IPCP_ADDRESS and length == 6:
                if option[2:] != self.his_ip:
                    nak += bytes([IPCP_ADDRESS, 6]) + self.his_ip
            else:
                reject += option
            i += length
        if reject:
            self._send_control(PPP_IPCP, CONF_REJ, ident, reject)
        elif nak:
            self._send_control(PPP_IPCP, CONF_NAK, ident, nak)
        else:
            self._send_control(PPP_IPCP, CONF_ACK, ident, data)
        return False

    def _ipv4(self, packet):
        if len(packet) < 20 or packet[0] >> 4 != 4 or packet[9] != 17:
            return
        header_len = (packet[0] & 0x0F) * 4
        udp = packet[header_len:]
        if len(udp) < 8:
            return
        sport, dport, length = struct.unpack('!HHH', udp[:6])
        with self.lock:
            self.udp_received.append((sport, dport, bytes(udp[8:length])))

    # the test's side
    def drop_connection(self):
        '''closes the PPP connection, as a peer that went away; the M4 connects again'''
        with self.lock:
            conn = self.conn
        if conn is not None:
            conn.shutdown(socket.SHUT_RDWR)

    def wait_ipv4_up(self, timeout):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.ipv4_up:
                return True
            time.sleep(0.1)
        return False

    def take_udp(self):
        with self.lock:
            got = self.udp_received
            self.udp_received = []
        return got

    def send_udp(self, sport, dport, payload):
        udp = struct.pack('!HHHH', sport, dport, 8 + len(payload), 0) + payload
        header = struct.pack('!BBHHHBBH4s4s', 0x45, 0, 20 + len(udp), 0, 0, 64, 17, 0, self.our_ip, self.his_ip)
        header = header[:10] + struct.pack('!H', ip_checksum(header)) + header[12:]
        self._send_frame(PPP_IP, header + udp)
