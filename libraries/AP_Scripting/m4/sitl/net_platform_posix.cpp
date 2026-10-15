/*
  The M4 network service's platform in the SITL M4 process: the region in the shared window,
  a polled doorbell, and PPP over a TCP connection named by SCRIPTING_M4_PPP (tcp:<host>:<port>)
  in place of the M4's UART.
 */
#include <atomic>
#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <AP_ScriptingM4_Layout.h>
#include "net_platform.h"

static uint8_t *region;
static uint32_t session;
static std::atomic<int> ppp_fd {-1};

void net_platform_init(uint8_t *window, uint32_t grant)
{
    region = window + ScriptingM4::RING_REGION_OFFSET;
    session = grant;
}

void net_platform_doorbell_isr(uint8_t group)
{
    (void)group;
}

uint8_t *net_platform_region()
{
    return region;
}

// the M7 SITL rings no doorbell for the M4 process: a short sleep
void net_platform_doorbell_wait(uint32_t timeout_ms)
{
    struct timespec ts { time_t(timeout_ms / 1000), long(timeout_ms % 1000) * 1000000L };
    nanosleep(&ts, nullptr);
}

uint32_t net_platform_session()
{
    return session;
}

static void ppp_close_transport()
{
    const int fd = ppp_fd.exchange(-1);
    if (fd >= 0) {
        close(fd);
    }
}

bool net_platform_ppp_open()
{
    ppp_close_transport();
    const char *spec = getenv("SCRIPTING_M4_PPP");
    if (spec == nullptr || strncmp(spec, "tcp:", 4) != 0) {
        return false;
    }
    char host[64];
    const char *colon = strrchr(spec + 4, ':');
    if (colon == nullptr || size_t(colon - (spec + 4)) >= sizeof(host)) {
        return false;
    }
    memcpy(host, spec + 4, size_t(colon - (spec + 4)));
    host[colon - (spec + 4)] = 0;
    struct addrinfo hints {};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    struct addrinfo *found = nullptr;
    if (getaddrinfo(host, colon + 1, &hints, &found) != 0 || found == nullptr) {
        return false;
    }
    const int fd = socket(AF_INET, SOCK_STREAM, 0);
    bool connected = fd >= 0 && connect(fd, found->ai_addr, found->ai_addrlen) == 0;
    freeaddrinfo(found);
    if (!connected) {
        if (fd >= 0) {
            close(fd);
        }
        return false;
    }
    const int on = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &on, sizeof(on));
#ifdef SO_NOSIGPIPE
    setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &on, sizeof(on));
#endif
    ppp_fd.store(fd);
    return true;
}

int32_t net_platform_ppp_read(uint8_t *buf, uint32_t len, uint32_t timeout_ms)
{
    const int fd = ppp_fd.load();
    if (fd < 0) {
        return -1;
    }
    struct pollfd pfd { fd, POLLIN, 0 };
    const int ready = poll(&pfd, 1, int(timeout_ms));
    if (ready == 0 || (ready < 0 && errno == EINTR)) {
        return 0;
    }
    const ssize_t n = ready > 0 ? recv(fd, buf, len, 0) : -1;
    if (n > 0) {
        return int32_t(n);
    }
    if (n < 0 && (errno == EINTR || errno == EAGAIN)) {
        return 0;
    }
    ppp_close_transport();   // the peer closed, or an error
    return -1;
}

int32_t net_platform_ppp_write(const uint8_t *buf, uint32_t len)
{
    const int fd = ppp_fd.load();
    if (fd < 0) {
        return -1;
    }
#ifdef MSG_NOSIGNAL
    const int flags = MSG_NOSIGNAL;
#else
    const int flags = 0;
#endif
    uint32_t sent = 0;
    while (sent < len) {
        const ssize_t n = send(fd, buf + sent, len - sent, flags);
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n <= 0) {
            return -1;   // the reader sees the close and reopens
        }
        sent += uint32_t(n);
    }
    return int32_t(sent);
}
