/*
  m4_net_peer: the host side of the SITL M4 process's PPP link (SCRIPTING_M4_PPP=tcp:127.0.0.1:<port>).

  It listens on 127.0.0.1 for the M4 process, runs PPP over that connection with lwIP (NO_SYS,
  raw API, one thread, one poll() loop) as the address provider, and relays traffic between
  the M4's addresses and ports on 127.0.0.1:

    udp-to-host:<peer port>:<host port>         datagrams to the peer's <peer port> go to the host
                                                port; its replies go back to the last M4 sender
    udp-group-to-host:<group>:<port>:<host port> datagrams to a multicast group go to the host port
    tcp-from-host:<host port>:<M4 port>         a host connection to <host port> is carried to
                                                the M4's <M4 port>

  A new connection on the PPP port replaces the current one only once it shows it speaks PPP (its
  first byte is a PPP flag or address byte): the M4 restarted. Anything else is closed.
 */
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <string>
#include <vector>

// lwIP's TCP_MSS (lwipopts.h), not the host's
#undef TCP_MSS

#include "lwip/init.h"
#include "lwip/ip4.h"
#include "lwip/pbuf.h"
#include "lwip/prot/ip4.h"
#include "lwip/prot/udp.h"
#include "lwip/tcp.h"
#include "lwip/timeouts.h"
#include "lwip/udp.h"
#include "netif/ppp/pppos.h"

#ifdef MSG_NOSIGNAL
static const int SEND_FLAGS = MSG_NOSIGNAL;
#else
static const int SEND_FLAGS = 0;
#endif

static const uint32_t LOOPBACK = INADDR_LOOPBACK;   // every host socket binds and sends here only
static const int POLL_MS = 10;
static const size_t HOST_READ_MAX = 4096;
static const uint32_t CANDIDATE_TIMEOUT_MS = 3000;   // a new PPP-port connection must speak by then

enum class RelayKind { UdpToHost, UdpGroupToHost, TcpFromHost };

struct Relay {
    RelayKind kind;
    ip4_addr_t group;           // UdpGroupToHost
    uint16_t m4_side_port;      // the peer port, the group port or the M4 port
    uint16_t host_port;
    int host_fd = -1;           // UDP: a socket on 127.0.0.1 sending to host_port; TCP: the listener
    struct udp_pcb *pcb = nullptr;
    ip_addr_t last_sender;
    uint16_t last_sender_port = 0;
};

struct TcpLink {
    int host_fd;
    struct tcp_pcb *pcb = nullptr;
    bool connected = false;
    bool m4_closed = false;
    bool host_gone = false;     // the host hung up while data for the M4 was still held
    std::string to_host;        // received from the M4, not yet written to the host (acknowledged on write)
    std::string to_m4;          // read from the host, not yet accepted by tcp_write
};

static volatile sig_atomic_t terminate_requested;
static std::vector<Relay> relays;
static std::vector<TcpLink *> links;
static ip4_addr_t our_ip;
static ip4_addr_t his_ip;
static struct netif ppp_netif;
static ppp_pcb *ppp;
static bool ppp_up;
static int listen_fd = -1;
static int ppp_fd = -1;
static int waiting_fd = -1;     // the M4's new connection, held until PPP is dead
static int candidate_fd = -1;   // a new connection while a session runs: promoted only if it speaks PPP
static uint32_t candidate_since_ms;
static uint32_t ppp_bytes_dropped;
static uint8_t datagram[65536];

extern "C" u32_t sys_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return u32_t(uint64_t(ts.tv_sec) * 1000U + uint64_t(ts.tv_nsec) / 1000000U);
}

// PPP's magic number seed
extern "C" u32_t sys_jiffies(void)
{
    return sys_now();
}

static void usage()
{
    fprintf(stderr,
            "usage: m4_net_peer --ppp-listen <port> --our-ip <a.b.c.d> --his-ip <a.b.c.d> [--relay <rule>]...\n"
            "  rules: udp-to-host:<peer port>:<host port>\n"
            "         udp-group-to-host:<group>:<port>:<host port>\n"
            "         tcp-from-host:<host port>:<M4 port>\n");
}

static bool parse_port(const std::string &text, uint16_t &port)
{
    char *end = nullptr;
    const unsigned long value = strtoul(text.c_str(), &end, 10);
    if (text.empty() || *end != 0 || value == 0 || value > 65535) {
        return false;
    }
    port = uint16_t(value);
    return true;
}

static bool parse_ip(const char *text, ip4_addr_t &ip)
{
    struct in_addr addr;
    if (inet_pton(AF_INET, text, &addr) != 1) {
        return false;
    }
    ip.addr = addr.s_addr;
    return true;
}

static std::vector<std::string> split(const std::string &text)
{
    std::vector<std::string> parts;
    size_t start = 0;
    for (;;) {
        const size_t colon = text.find(':', start);
        parts.push_back(text.substr(start, colon - start));
        if (colon == std::string::npos) {
            return parts;
        }
        start = colon + 1;
    }
}

static bool parse_relay(const char *text, Relay &relay)
{
    const std::vector<std::string> parts = split(text);
    if (parts[0] == "udp-to-host" && parts.size() == 3) {
        relay.kind = RelayKind::UdpToHost;
        return parse_port(parts[1], relay.m4_side_port) && parse_port(parts[2], relay.host_port);
    }
    if (parts[0] == "udp-group-to-host" && parts.size() == 4) {
        relay.kind = RelayKind::UdpGroupToHost;
        return parse_ip(parts[1].c_str(), relay.group) && ip4_addr_ismulticast(&relay.group) &&
               parse_port(parts[2], relay.m4_side_port) && parse_port(parts[3], relay.host_port);
    }
    if (parts[0] == "tcp-from-host" && parts.size() == 3) {
        relay.kind = RelayKind::TcpFromHost;
        return parse_port(parts[1], relay.host_port) && parse_port(parts[2], relay.m4_side_port);
    }
    return false;
}

static struct sockaddr_in loopback_address(uint16_t port)
{
    struct sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(LOOPBACK);
    addr.sin_port = htons(port);
    return addr;
}

static void set_nonblocking(int fd)
{
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
}

static void set_nosigpipe(int fd)
{
#ifdef SO_NOSIGPIPE
    const int on = 1;
    setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &on, sizeof(on));
#else
    (void)fd;
#endif
}

// a TCP listener on 127.0.0.1:port, or -1
// a free loopback port refuses at once; a listener accepts, or leaves the SYN pending when its backlog is full
static bool port_in_use(const struct sockaddr_in &addr)
{
    const int probe = socket(AF_INET, SOCK_STREAM, 0);
    if (probe < 0) {
        return false;
    }
    set_nonblocking(probe);
    bool in_use = true;
    if (connect(probe, (const struct sockaddr *)&addr, sizeof(addr)) != 0) {
        if (errno == EINPROGRESS) {
            struct pollfd pfd { probe, POLLOUT, 0 };
            int error = 0;
            socklen_t len = sizeof(error);
            if (poll(&pfd, 1, 500) == 1 && getsockopt(probe, SOL_SOCKET, SO_ERROR, &error, &len) == 0 &&
                error == ECONNREFUSED) {
                in_use = false;
            }
        } else if (errno == ECONNREFUSED) {
            in_use = false;
        }
    }
    close(probe);
    return in_use;
}

// SO_REUSEADDR lets the peer relaunch while its last connections sit in TIME_WAIT, but on macOS it also
// lets a 127.0.0.1 bind shadow another process's *:port listener (a SITL SERIAL port): a port something
// already answers on is refused first.
static int listen_loopback(uint16_t port)
{
    const struct sockaddr_in addr = loopback_address(port);
    if (port_in_use(addr)) {
        errno = EADDRINUSE;
        return -1;
    }
    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        return -1;
    }
    const int on = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    if (bind(fd, (const struct sockaddr *)&addr, sizeof(addr)) != 0 || listen(fd, 4) != 0) {
        close(fd);
        return -1;
    }
    set_nonblocking(fd);
    return fd;
}

// a UDP socket on 127.0.0.1 connected to 127.0.0.1:port, or -1
static int udp_to_loopback(uint16_t port)
{
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        return -1;
    }
    const struct sockaddr_in local = loopback_address(0);
    const struct sockaddr_in remote = loopback_address(port);
    if (bind(fd, (const struct sockaddr *)&local, sizeof(local)) != 0 ||
        connect(fd, (const struct sockaddr *)&remote, sizeof(remote)) != 0) {
        close(fd);
        return -1;
    }
    set_nonblocking(fd);
    return fd;
}

// UDP relays: M4 to host

static void udp_relay_recv(void *arg, struct udp_pcb *, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    Relay &relay = *(Relay *)arg;
    const u16_t len = pbuf_copy_partial(p, datagram, p->tot_len, 0);
    pbuf_free(p);
    ip_addr_copy(relay.last_sender, *addr);
    relay.last_sender_port = port;
    send(relay.host_fd, datagram, len, SEND_FLAGS);
}

extern "C" int m4_net_peer_ip4_input(struct pbuf *p, struct netif *inp)
{
    if (inp != &ppp_netif) {
        return 0;
    }
    // the headers only: the payload is copied on a match
    const u16_t len = pbuf_copy_partial(p, datagram, IP_HLEN_MAX + UDP_HLEN, 0);
    if (len < IP_HLEN) {
        return 0;
    }
    const struct ip_hdr *iphdr = (const struct ip_hdr *)datagram;
    const u16_t header_len = IPH_HL_BYTES(iphdr);
    const u16_t total_len = lwip_ntohs(IPH_LEN(iphdr));
    if (IPH_PROTO(iphdr) != IP_PROTO_UDP || header_len < IP_HLEN || total_len > p->tot_len ||
        total_len < header_len + UDP_HLEN || header_len + UDP_HLEN > len || (lwip_ntohs(IPH_OFFSET(iphdr)) & (IP_MF | IP_OFFMASK)) != 0) {
        return 0;
    }
    ip4_addr_t dest;
    ip4_addr_copy(dest, iphdr->dest);
    const struct udp_hdr *udphdr = (const struct udp_hdr *)(datagram + header_len);
    const u16_t dest_port = lwip_ntohs(udphdr->dest);
    for (const Relay &relay : relays) {
        if (relay.kind == RelayKind::UdpGroupToHost && relay.host_fd >= 0 &&
            ip4_addr_eq(&relay.group, &dest) && relay.m4_side_port == dest_port) {
            const u16_t payload = pbuf_copy_partial(p, datagram, u16_t(total_len - header_len - UDP_HLEN),
                                                    u16_t(header_len + UDP_HLEN));
            send(relay.host_fd, datagram, payload, SEND_FLAGS);
            pbuf_free(p);
            return 1;
        }
    }
    return 0;
}

// host to M4: replies to the last sender
static void udp_relay_from_host(Relay &relay)
{
    const ssize_t n = recv(relay.host_fd, datagram, sizeof(datagram), 0);
    if (n <= 0 || relay.kind != RelayKind::UdpToHost || relay.pcb == nullptr || relay.last_sender_port == 0) {
        return;   // a group relay forwards one way; a refused datagram's ICMP error is dropped here too
    }
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, u16_t(n), PBUF_RAM);
    if (p == nullptr) {
        return;
    }
    memcpy(p->payload, datagram, size_t(n));
    udp_sendto(relay.pcb, p, &relay.last_sender, relay.last_sender_port);
    pbuf_free(p);
}

// TCP relays

static void link_detach_pcb(TcpLink &link)
{
    if (link.pcb == nullptr) {
        return;
    }
    tcp_arg(link.pcb, nullptr);
    tcp_recv(link.pcb, nullptr);
    tcp_sent(link.pcb, nullptr);
    tcp_err(link.pcb, nullptr);
    if (tcp_close(link.pcb) != ERR_OK) {
        tcp_abort(link.pcb);
    }
    link.pcb = nullptr;
}

static void link_close(TcpLink &link)
{
    link_detach_pcb(link);
    if (link.host_fd >= 0) {
        close(link.host_fd);
        link.host_fd = -1;
    }
}

static void link_push_to_m4(TcpLink &link)
{
    if (link.pcb == nullptr || !link.connected || link.to_m4.empty()) {
        return;
    }
    const size_t room = tcp_sndbuf(link.pcb);
    const size_t len = link.to_m4.size() < room ? link.to_m4.size() : room;
    if (len == 0 || tcp_write(link.pcb, link.to_m4.data(), u16_t(len), TCP_WRITE_FLAG_COPY) != ERR_OK) {
        return;   // held until tcp_sent makes room
    }
    link.to_m4.erase(0, len);
    tcp_output(link.pcb);
}

static void link_flush_to_host(TcpLink &link)
{
    while (!link.to_host.empty() && link.host_fd >= 0) {
        const ssize_t n = send(link.host_fd, link.to_host.data(), link.to_host.size(), SEND_FLAGS);
        if (n <= 0) {
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
                return;
            }
            // may run in an lwIP callback: sweep_links() closes the PCB
            close(link.host_fd);
            link.host_fd = -1;
            return;
        }
        link.to_host.erase(0, size_t(n));
        if (link.pcb != nullptr) {
            tcp_recved(link.pcb, u16_t(n));   // n <= TCP_WND: the window bounds to_host
        }
    }
}

static err_t link_m4_recv(void *arg, struct tcp_pcb *, struct pbuf *p, err_t)
{
    TcpLink &link = *(TcpLink *)arg;
    if (p == nullptr) {
        link.m4_closed = true;
        return ERR_OK;
    }
    const size_t old_size = link.to_host.size();
    link.to_host.resize(old_size + p->tot_len);
    pbuf_copy_partial(p, &link.to_host[old_size], p->tot_len, 0);
    pbuf_free(p);
    link_flush_to_host(link);
    return ERR_OK;
}

static err_t link_m4_sent(void *arg, struct tcp_pcb *, u16_t)
{
    link_push_to_m4(*(TcpLink *)arg);
    return ERR_OK;
}

static void link_m4_error(void *arg, err_t)
{
    // lwIP has freed the PCB
    TcpLink &link = *(TcpLink *)arg;
    link.pcb = nullptr;
    link.m4_closed = true;
}

static err_t link_m4_connected(void *arg, struct tcp_pcb *, err_t)
{
    TcpLink &link = *(TcpLink *)arg;
    link.connected = true;
    link_push_to_m4(link);
    return ERR_OK;
}

static void tcp_relay_accept(Relay &relay)
{
    const int fd = accept(relay.host_fd, nullptr, nullptr);
    if (fd < 0) {
        return;
    }
    struct tcp_pcb *pcb = ppp_up ? tcp_new() : nullptr;
    if (pcb == nullptr) {
        close(fd);
        return;
    }
    set_nonblocking(fd);
    set_nosigpipe(fd);
    TcpLink *link = new TcpLink;
    link->host_fd = fd;
    link->pcb = pcb;
    tcp_arg(pcb, link);
    tcp_recv(pcb, link_m4_recv);
    tcp_sent(pcb, link_m4_sent);
    tcp_err(pcb, link_m4_error);
    if (tcp_connect(pcb, &his_ip, relay.m4_side_port, link_m4_connected) != ERR_OK) {
        link_close(*link);
        delete link;
        return;
    }
    links.push_back(link);
}

static void link_from_host(TcpLink &link)
{
    char buf[HOST_READ_MAX];
    const ssize_t n = recv(link.host_fd, buf, sizeof(buf), 0);
    if (n > 0) {
        link.to_m4.append(buf, size_t(n));
        link_push_to_m4(link);
    } else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
        // tcp_close sends what lwIP queued, then FIN; with M4 data not yet written to the host
        // (the receive window not fully reopened) lwIP sends RST instead
        link_close(link);
    }
}

// the only place a link is freed: the main loop holds link pointers across callbacks that close links
static void sweep_links()
{
    for (size_t i = 0; i < links.size();) {
        TcpLink &link = *links[i];
        if (link.m4_closed && (link.to_host.empty() || link.host_fd < 0)) {
            link_close(link);
        }
        if (link.host_gone && (link.to_m4.empty() || link.pcb == nullptr)) {
            link_close(link);
        }
        if (link.host_fd < 0) {
            link_detach_pcb(link);
            delete links[i];
            links.erase(links.begin() + long(i));
        } else {
            i++;
        }
    }
}

// closes every link; sweep_links() frees them
static void close_links()
{
    for (TcpLink *link : links) {
        link_close(*link);
    }
}

// the relays' lwIP PCBs live for one PPP session
static void relays_up()
{
    for (Relay &relay : relays) {
        if (relay.kind != RelayKind::UdpToHost) {
            continue;
        }
        relay.last_sender_port = 0;
        relay.pcb = udp_new();
        if (relay.pcb == nullptr || udp_bind(relay.pcb, &our_ip, relay.m4_side_port) != ERR_OK) {
            fprintf(stderr, "m4_net_peer: cannot bind %s:%u\n", ip4addr_ntoa(&our_ip), relay.m4_side_port);
            exit(1);
        }
        udp_recv(relay.pcb, udp_relay_recv, &relay);
    }
}

static void relays_down()
{
    for (Relay &relay : relays) {
        if (relay.pcb != nullptr) {
            udp_remove(relay.pcb);
            relay.pcb = nullptr;
        }
    }
    close_links();
}

// PPP

static u32_t ppp_output(ppp_pcb *, const void *data, u32_t len, void *)
{
    if (ppp_fd < 0) {
        return len;
    }
    // non-blocking: a stalled M4 loses frames rather than stalling the peer
    const ssize_t n = send(ppp_fd, data, len, SEND_FLAGS);
    const u32_t sent = n < 0 ? 0 : u32_t(n);
    ppp_bytes_dropped += len - sent;
    return sent;
}

static void ppp_status(ppp_pcb *, int err_code, void *)
{
    if (err_code == PPPERR_NONE) {
        if (!ppp_up) {
            ppp_up = true;
            relays_up();
            printf("m4_net_peer: PPP up, his %s\n", ip4addr_ntoa(netif_ip4_gw(&ppp_netif)));
        }
        return;
    }
    if (ppp_up) {
        ppp_up = false;
        relays_down();
    }
    printf("m4_net_peer: PPP down\n");
    if (ppp_bytes_dropped != 0) {
        printf("m4_net_peer: %u bytes to the M4 dropped (socket full)\n", unsigned(ppp_bytes_dropped));
        ppp_bytes_dropped = 0;
    }
}

// the M4 process went away, or a restarted one connected
static void drop_m4_connection()
{
    if (ppp_fd < 0) {
        return;
    }
    close(ppp_fd);
    ppp_fd = -1;
    ppp_close(ppp, 1);
}

static void accept_m4()
{
    const int fd = accept(listen_fd, nullptr, nullptr);
    if (fd < 0) {
        return;
    }
    const int on = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
    set_nonblocking(fd);
    set_nosigpipe(fd);
    if (ppp_fd < 0 && waiting_fd < 0) {
        waiting_fd = fd;   // no session to protect
        return;
    }
    if (candidate_fd >= 0) {
        close(candidate_fd);
    }
    candidate_fd = fd;
    candidate_since_ms = sys_now();
}

static void close_candidate(const char *why)
{
    printf("m4_net_peer: closed a new connection on the PPP port: %s\n", why);
    close(candidate_fd);
    candidate_fd = -1;
}

// a PPPoS stream starts with a flag (0x7E) or, within 100 ms of the sender's start, the address (0xFF)
static void check_candidate(bool readable)
{
    if (candidate_fd < 0) {
        return;
    }
    if (!readable) {
        if (sys_now() - candidate_since_ms > CANDIDATE_TIMEOUT_MS) {
            close_candidate("silent");
        }
        return;
    }
    uint8_t first;
    const ssize_t n = recv(candidate_fd, &first, 1, MSG_PEEK);
    if (n == 1 && (first == 0x7E || first == 0xFF)) {
        // the M4 restarted while its old connection stayed open
        if (waiting_fd >= 0) {
            close(waiting_fd);
        }
        waiting_fd = candidate_fd;
        candidate_fd = -1;
        drop_m4_connection();
    } else if (n == 1) {
        close_candidate("not PPP");
    } else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
        close_candidate("closed");
    }
}

static void read_m4()
{
    uint8_t buf[4096];
    const ssize_t n = recv(ppp_fd, buf, sizeof(buf), 0);
    if (n > 0) {
        pppos_input(ppp, buf, int(n));
    } else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
        drop_m4_connection();
    }
}

// a connection with no PPP session: start one once the last has ended
static void start_ppp()
{
    if (ppp->phase != PPP_PHASE_DEAD) {
        return;
    }
    if (waiting_fd >= 0) {
        ppp_fd = waiting_fd;
        waiting_fd = -1;
    }
    if (ppp_fd >= 0) {
        ppp_connect(ppp, 0);
    }
}

static void on_terminate(int)
{
    terminate_requested = 1;
}

int main(int argc, char **argv)
{
    uint16_t ppp_port = 0;
    bool have_our_ip = false;
    bool have_his_ip = false;
    for (int i = 1; i < argc; i += 2) {
        const char *value = i + 1 < argc ? argv[i + 1] : nullptr;
        bool ok = value != nullptr;
        if (ok && strcmp(argv[i], "--ppp-listen") == 0) {
            ok = parse_port(value, ppp_port);
        } else if (ok && strcmp(argv[i], "--our-ip") == 0) {
            ok = have_our_ip = parse_ip(value, our_ip);
        } else if (ok && strcmp(argv[i], "--his-ip") == 0) {
            ok = have_his_ip = parse_ip(value, his_ip);
        } else if (ok && strcmp(argv[i], "--relay") == 0) {
            Relay relay {};
            ok = parse_relay(value, relay);
            for (const Relay &other : relays) {
                // one lwIP PCB per peer port
                if (ok && relay.kind == RelayKind::UdpToHost && other.kind == RelayKind::UdpToHost &&
                    other.m4_side_port == relay.m4_side_port) {
                    ok = false;
                }
            }
            relays.push_back(relay);
        } else {
            ok = false;
        }
        if (!ok) {
            usage();
            return 2;
        }
    }
    if (ppp_port == 0 || !have_our_ip || !have_his_ip) {
        usage();
        return 2;
    }

    setvbuf(stdout, nullptr, _IOLBF, 0);
    signal(SIGPIPE, SIG_IGN);
    struct sigaction term {};
    term.sa_handler = on_terminate;
    sigaction(SIGTERM, &term, nullptr);
    sigaction(SIGINT, &term, nullptr);

    listen_fd = listen_loopback(ppp_port);
    if (listen_fd < 0) {
        fprintf(stderr, "m4_net_peer: cannot listen on 127.0.0.1:%u: %s\n", ppp_port, strerror(errno));
        return 1;
    }
    for (Relay &relay : relays) {
        relay.host_fd = relay.kind == RelayKind::TcpFromHost ? listen_loopback(relay.host_port)
                                                              : udp_to_loopback(relay.host_port);
        if (relay.host_fd < 0) {
            fprintf(stderr, "m4_net_peer: cannot open the host side of port %u: %s\n", relay.host_port,
                    strerror(errno));
            return 1;
        }
    }

    lwip_init();
    ppp = pppos_create(&ppp_netif, ppp_output, ppp_status, nullptr);
    if (ppp == nullptr) {
        fprintf(stderr, "m4_net_peer: pppos_create failed\n");
        return 1;
    }
    ppp_set_ipcp_ouraddr(ppp, &our_ip);
    ppp_set_ipcp_hisaddr(ppp, &his_ip);
    netif_set_default(&ppp_netif);

    while (!terminate_requested) {
        std::vector<struct pollfd> fds;
        std::vector<Relay *> relay_of;      // per pollfd from first_relay: the relay, or null for a link
        std::vector<TcpLink *> link_of;
        fds.push_back({listen_fd, POLLIN, 0});
        fds.push_back({ppp_fd, POLLIN, 0});   // a negative fd is ignored
        fds.push_back({candidate_fd, POLLIN, 0});
        const size_t first_relay = fds.size();
        for (Relay &relay : relays) {
            // a TCP listener is served only while PPP is up: connections wait in its backlog
            if (relay.kind != RelayKind::TcpFromHost || ppp_up) {
                fds.push_back({relay.host_fd, POLLIN, 0});
                relay_of.push_back(&relay);
                link_of.push_back(nullptr);
            }
        }
        for (TcpLink *link : links) {
            if (link->host_gone) {
                continue;   // its fd would report POLLHUP every pass
            }
            short events = 0;
            // read the host only while lwIP can take it: the rest stays in the host socket
            if (link->connected && link->pcb != nullptr && link->to_m4.empty()) {
                events |= POLLIN;
            }
            if (!link->to_host.empty()) {
                events |= POLLOUT;
            }
            fds.push_back({link->host_fd, events, 0});
            relay_of.push_back(nullptr);
            link_of.push_back(link);
        }

        const int ready = poll(fds.data(), nfds_t(fds.size()), POLL_MS);
        check_candidate(ready > 0 && fds[2].revents != 0);
        if (ready > 0) {
            if (fds[1].revents != 0 && ppp_fd >= 0) {
                read_m4();
            }
            if (fds[0].revents & POLLIN) {
                accept_m4();
            }
            for (size_t i = first_relay; i < fds.size(); i++) {
                const short revents = fds[i].revents;
                if (revents == 0) {
                    continue;
                }
                if (relay_of[i - first_relay] != nullptr) {
                    Relay &relay = *relay_of[i - first_relay];
                    if (relay.kind == RelayKind::TcpFromHost) {
                        tcp_relay_accept(relay);
                    } else {
                        udp_relay_from_host(relay);
                    }
                    continue;
                }
                TcpLink &link = *link_of[i - first_relay];
                if (link.host_fd < 0) {
                    continue;   // closed earlier in this pass (PPP went down)
                }
                if (revents & POLLOUT) {
                    link_flush_to_host(link);
                }
                if (link.host_fd >= 0 && link.pcb != nullptr && (revents & (POLLIN | POLLHUP | POLLERR))) {
                    if (link.to_m4.empty()) {
                        link_from_host(link);
                    } else {
                        link.host_gone = true;   // closed by sweep_links() once the held data is sent
                    }
                }
            }
        }
        sys_check_timeouts();
        start_ppp();
        sweep_links();
    }

    close_links();
    sweep_links();
    for (Relay &relay : relays) {
        close(relay.host_fd);
    }
    if (waiting_fd >= 0) {
        close(waiting_fd);
    }
    if (candidate_fd >= 0) {
        close(candidate_fd);
    }
    if (ppp_fd >= 0) {
        close(ppp_fd);
    }
    close(listen_fd);
    return 0;
}
