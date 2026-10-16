/*
  The net_port thread: every NET_P port's socket, non-blocking, between lwIP and the port's rings
  (as AP_Networking_port.cpp's loops on the M7), and the status block every STATUS_PERIOD_MS.
  The M7 never waits on the M4, and this thread never waits in lwIP: a full RX ring drops UDP
  datagrams and stops reading TCP.
 */

#include <errno.h>
#include <string.h>
#include "net_m4.h"
#include "net_platform.h"
#include "net_sys_arch.h"
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/sockets.h"

using namespace NetM4;

namespace {

// NetM4::PortConfig::type (AP_Networking::NetworkPortType)
enum PortType : uint8_t { PORT_NONE = 0, UDP_CLIENT = 1, UDP_SERVER = 2, TCP_CLIENT = 3, TCP_SERVER = 4 };

const uint32_t RETRY_MS = 1000;          // a failed socket set up again; a TCP connect polled
const uint32_t UDP_PEER_TIMEOUT_MS = 3000;   // a UDP server answers another sender after this silence
const uint8_t MAX_DATAGRAMS_PER_PASS = 16;   // per port and direction: a flood cannot hold the thread
const uint32_t DOORBELL_WAIT_MS = 2;

struct PortState {
    int sock = -1;
    int listen_sock = -1;
    bool connected;       // TCP: a connection; UDP: a datagram went either way
    bool rings_ok;        // both rings attached
    uint8_t pending[MAX_RECORD];
    uint16_t pending_len, pending_sent;
    uint32_t next_try_ms;
    sockaddr_in to;       // UDP client: the destination; UDP server: its peer
    bool have_to;
    uint32_t peer_heard_ms;   // UDP server: the peer's last datagram
    bool send_to;         // UDP: sendto() (broadcast, multicast, server), else send() on a connected socket
    uint32_t rx_dropped;
    uint32_t tx_dropped;
    ByteRing tx, rx;
};

PortState ports[NUM_PORTS];
uint8_t datagram[512];

int last_errno()
{
    return thread_errno_get();
}

bool would_block(int err)
{
    return err == EWOULDBLOCK || err == EAGAIN || err == ENOMEM;
}

sockaddr_in address_of(uint32_t ip, uint16_t port)
{
    sockaddr_in addr {};
    addr.sin_len = sizeof(addr);
    addr.sin_family = AF_INET;
    addr.sin_port = lwip_htons(port);
    addr.sin_addr.s_addr = lwip_htonl(ip);
    return addr;
}

int new_socket(int type)
{
    const int sock = lwip_socket(AF_INET, type, 0);
    if (sock >= 0 && lwip_fcntl(sock, F_SETFL, O_NONBLOCK) != 0) {
        lwip_close(sock);
        return -1;
    }
    return sock;
}

void close_socket(int &sock)
{
    if (sock >= 0) {
        lwip_close(sock);
        sock = -1;
    }
}

// a socket that failed: closed, set up again after RETRY_MS
void retry_later(PortState &port, uint32_t now_ms)
{
    close_socket(port.sock);
    close_socket(port.listen_sock);
    port.connected = false;
    port.pending_len = 0;
    port.pending_sent = 0;
    port.next_try_ms = now_ms + RETRY_MS;
}

bool due(const PortState &port, uint32_t now_ms)
{
    return int32_t(now_ms - port.next_try_ms) >= 0;
}

bool open_udp(PortState &port, const PortConfig &cfg, uint8_t type)
{
    port.sock = new_socket(SOCK_DGRAM);
    if (port.sock < 0) {
        return false;
    }
    const int on = 1;
    lwip_setsockopt(port.sock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    if (type == UDP_SERVER) {
        const sockaddr_in local = address_of(cfg.ip, cfg.port);
        port.send_to = true;
        port.have_to = false;
        return lwip_bind(port.sock, reinterpret_cast<const sockaddr *>(&local), sizeof(local)) == 0;
    }
    port.to = address_of(cfg.ip, cfg.port);
    port.have_to = true;
    if (cfg.ip == 0xFFFFFFFFU) {
        port.send_to = true;
        return lwip_setsockopt(port.sock, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on)) == 0;
    }
    if ((cfg.ip >> 28) == 0xE) {
        // multicast: receive the group on the port, send to it
        port.send_to = true;
        const sockaddr_in local = address_of(0, cfg.port);
        ip_mreq group {};
        group.imr_multiaddr.s_addr = lwip_htonl(cfg.ip);
        group.imr_interface.s_addr = lwip_htonl(0);
        return lwip_bind(port.sock, reinterpret_cast<const sockaddr *>(&local), sizeof(local)) == 0 &&
               lwip_setsockopt(port.sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &group, sizeof(group)) == 0;
    }
    port.send_to = false;
    return lwip_connect(port.sock, reinterpret_cast<const sockaddr *>(&port.to), sizeof(port.to)) == 0;
}

// records from the TX ring, one datagram each; a datagram lwIP cannot take now waits in pending
void udp_send(PortState &port)
{
    for (uint8_t i = 0; i < MAX_DATAGRAMS_PER_PASS; i++) {
        if (port.pending_len == 0) {
            const int32_t len = port.tx.read_record(port.pending, sizeof(port.pending));
            if (len == RECORD_DROPPED) {
                port.tx_dropped++;   // longer than pending (MAX_RECORD today: then RECORD_BAD instead)
                continue;
            }
            if (len == RECORD_BAD) {
                port.tx.consumer_resync();   // the consumer recovers a desynced ring
                return;
            }
            if (len <= 0) {
                return;
            }
            port.pending_len = uint16_t(len);
        }
        if (port.send_to && !port.have_to) {
            port.tx_dropped++;   // a UDP server sends only once a peer has been heard
            port.pending_len = 0;
            continue;
        }
        const int sent = port.send_to ?
            lwip_sendto(port.sock, port.pending, port.pending_len, 0, reinterpret_cast<const sockaddr *>(&port.to), sizeof(port.to)) :
            lwip_send(port.sock, port.pending, port.pending_len, 0);
        if (sent < 0 && would_block(last_errno())) {
            return;   // pending is sent on a later pass
        }
        if (sent < 0) {
            port.tx_dropped++;   // e.g. no route while the link is down
        } else {
            port.connected = true;
        }
        port.pending_len = 0;
    }
}

bool same_sender(const sockaddr_in &a, const sockaddr_in &b)
{
    return a.sin_addr.s_addr == b.sin_addr.s_addr && a.sin_port == b.sin_port;
}

// a UDP server keeps answering its peer, as the M7's port loop: another sender takes over only
// once the peer has been silent for UDP_PEER_TIMEOUT_MS
void track_udp_peer(PortState &port, const sockaddr_in &from, uint32_t now_ms)
{
    if (port.have_to && !same_sender(from, port.to) && now_ms - port.peer_heard_ms < UDP_PEER_TIMEOUT_MS) {
        return;
    }
    port.to = from;
    port.have_to = true;
    port.peer_heard_ms = now_ms;
}

// whole datagrams into the RX ring; with no room for one, it is read and dropped. A datagram over
// sizeof(datagram) is delivered cut, the bytes past it counted as dropped
void udp_receive(PortState &port, bool server, uint32_t now_ms)
{
    for (uint8_t i = 0; i < MAX_DATAGRAMS_PER_PASS; i++) {
        sockaddr_in from {};
        iovec part { datagram, sizeof(datagram) };
        msghdr message {};
        message.msg_name = &from;
        message.msg_namelen = sizeof(from);
        message.msg_iov = &part;
        message.msg_iovlen = 1;
        // the datagram's whole length, MSG_TRUNC when it did not fit
        const int whole = lwip_recvmsg(port.sock, &message, 0);
        if (whole <= 0) {
            return;
        }
        const uint32_t len = uint32_t(whole) < sizeof(datagram) ? uint32_t(whole) : uint32_t(sizeof(datagram));
        port.rx_dropped += uint32_t(whole) - len;
        port.connected = true;
        if (server) {
            track_udp_peer(port, from, now_ms);
        }
        if (port.rx.space() < len || port.rx.write(datagram, len) != len) {
            port.rx_dropped += len;
        }
    }
}

// TCP bytes both ways; false when the connection is gone
bool tcp_transfer(PortState &port)
{
    if (port.pending_sent == port.pending_len) {
        port.pending_sent = 0;
        port.pending_len = uint16_t(port.tx.read(port.pending, sizeof(port.pending)));
        if (port.pending_len == 0 && port.tx.desynced()) {
            port.tx.consumer_resync();
        }
    }
    if (port.pending_sent < port.pending_len) {
        const int sent = lwip_send(port.sock, port.pending + port.pending_sent, port.pending_len - port.pending_sent, 0);
        if (sent > 0) {
            port.pending_sent += uint16_t(sent);
        } else if (sent < 0 && !would_block(last_errno())) {
            return false;
        }
    }
    // only what the ring takes: TCP pushes back on the sender
    const uint32_t space = port.rx.space();
    if (space == 0) {
        return true;
    }
    const int len = lwip_recv(port.sock, datagram, space < sizeof(datagram) ? space : sizeof(datagram), 0);
    if (len > 0) {
        port.rx.write(datagram, uint32_t(len));
        return true;
    }
    return len < 0 && would_block(last_errno());
}

void tcp_client(PortState &port, const PortConfig &cfg, uint32_t now_ms)
{
    if (port.sock < 0) {
        if (!due(port, now_ms)) {
            return;
        }
        port.sock = new_socket(SOCK_STREAM);
        if (port.sock < 0) {
            retry_later(port, now_ms);
            return;
        }
    }
    if (!port.connected) {
        // non-blocking, asked again every RETRY_MS: EINPROGRESS, then EALREADY until lwIP answers
        // EISCONN; anything else (refused, reset) closes the socket for a new one
        if (!due(port, now_ms)) {
            return;
        }
        const sockaddr_in to = address_of(cfg.ip, cfg.port);
        const int ret = lwip_connect(port.sock, reinterpret_cast<const sockaddr *>(&to), sizeof(to));
        const int err = ret == 0 ? 0 : last_errno();
        if (ret == 0 || err == EISCONN) {
            port.connected = true;
            port.pending_len = 0;
            port.pending_sent = 0;
        } else if (err == EINPROGRESS || err == EALREADY) {
            port.next_try_ms = now_ms + RETRY_MS;
        } else {
            retry_later(port, now_ms);
        }
        return;
    }
    if (!tcp_transfer(port)) {
        retry_later(port, now_ms);
    }
}

void tcp_server(PortState &port, const PortConfig &cfg, uint32_t now_ms)
{
    if (port.listen_sock < 0) {
        if (!due(port, now_ms)) {
            return;
        }
        port.listen_sock = new_socket(SOCK_STREAM);
        const int on = 1;
        const sockaddr_in local = address_of(cfg.ip, cfg.port);
        if (port.listen_sock < 0 ||
            lwip_setsockopt(port.listen_sock, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on)) != 0 ||
            lwip_bind(port.listen_sock, reinterpret_cast<const sockaddr *>(&local), sizeof(local)) != 0 ||
            lwip_listen(port.listen_sock, 1) != 0) {
            retry_later(port, now_ms);
            return;
        }
    }
    if (!port.connected) {
        // one connection at a time: the next one is accepted once this one is gone
        const int sock = lwip_accept(port.listen_sock, nullptr, nullptr);
        if (sock < 0) {
            return;
        }
        if (lwip_fcntl(sock, F_SETFL, O_NONBLOCK) != 0) {
            lwip_close(sock);
            return;
        }
        port.sock = sock;
        port.connected = true;
        port.pending_len = 0;
        port.pending_sent = 0;
        return;
    }
    if (!tcp_transfer(port)) {
        close_socket(port.sock);
        port.connected = false;
    }
}

void service_port(PortState &port, const PortConfig &cfg, uint32_t now_ms)
{
    if (!port.rings_ok) {
        return;
    }
    switch (cfg.type) {
    case UDP_CLIENT:
    case UDP_SERVER:
        if (port.sock < 0) {
            if (!due(port, now_ms)) {
                return;
            }
            if (!open_udp(port, cfg, cfg.type)) {
                retry_later(port, now_ms);
                return;
            }
        }
        udp_send(port);
        udp_receive(port, cfg.type == UDP_SERVER, now_ms);
        return;
    case TCP_CLIENT:
        tcp_client(port, cfg, now_ms);
        return;
    case TCP_SERVER:
        tcp_server(port, cfg, now_ms);
        return;
    default:
        return;
    }
}

uint32_t state_of(const PortState &port, uint8_t type)
{
    if (type == PORT_NONE || (port.sock < 0 && port.listen_sock < 0)) {
        return PORT_CLOSED;
    }
    if (port.connected) {
        return PORT_CONNECTED;
    }
    // a TCP client's socket is not open until it connects
    return type == TCP_CLIENT ? PORT_CLOSED : PORT_OPEN;
}

}  // namespace

void NetM4::port_service_main(void *arg)
{
    (void)arg;
    const Config &config = service_config();
    Region &region = *reinterpret_cast<Region *>(net_platform_region());
    for (uint8_t i = 0; i < NUM_PORTS; i++) {
        ports[i].rings_ok = ports[i].tx.attach(&region.ports[i].tx_control, region.ports[i].tx, PORT_RING_DATA) &&
                            ports[i].rx.attach(&region.ports[i].rx_control, region.ports[i].rx, PORT_RING_DATA);
    }
    uint32_t last_status_ms = 0;
    bool status_written = false;
    for (;;) {
        const uint32_t now_ms = os_now_ms();
        for (uint8_t i = 0; i < NUM_PORTS; i++) {
            // between two ports: no ring operation is half done
            if (park_requested()) {
                park_here();
            }
            service_port(ports[i], config.ports[i], now_ms);
        }
        if (!status_written || now_ms - last_status_ms >= STATUS_PERIOD_MS) {
            PortReport report {};
            for (uint8_t i = 0; i < NUM_PORTS; i++) {
                report.state[i] = state_of(ports[i], config.ports[i].type);
                report.rx_dropped[i] = ports[i].rx_dropped;
                report.tx_dropped[i] = ports[i].tx_dropped;
            }
            if (park_requested()) {
                park_here();
            }
            publish_status(report);
            last_status_ms = now_ms;
            status_written = true;
        }
        net_platform_doorbell_wait(DOORBELL_WAIT_MS);
    }
}
