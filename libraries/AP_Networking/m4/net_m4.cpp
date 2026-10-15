#include "net_m4.h"

#include <atomic>
#include <string.h>
#include "net_platform.h"
#include "net_sys_arch.h"
#include "lwip/opt.h"
#include "lwip/def.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "netif/ppp/pppos.h"

using namespace NetM4;

namespace {

// the M7's configuration of this session, set before the threads start and then only read
Config config;
bool started;
uint32_t threads_started;

// the PPP link as ppp_status() last saw it (tcpip thread); the status writer copies it under os_protect
struct Link {
    bool up;
    uint32_t ip, netmask, gateway;   // host byte order
};
Link ppp_link;
ppp_pcb *ppp;
struct netif ppp_if;
std::atomic<bool> ppp_reconnect;   // a PPP error: the ppp_rx thread closes and connects again
OsSem idle;                         // never signalled: the ppp_rx thread's retry wait

// the status the net_port thread wrote last
StatusFields written;
bool written_once;

std::atomic<bool> park_request;
std::atomic<bool> port_parked;
std::atomic<const void *> port_thread;

// below the service thread, which beats the M4 heartbeat (NORMALPRIO): NORMALPRIO - 1
const int NET_THREAD_PRIO = -1;
const uint32_t PPP_RETRY_MS = 1000;
const uint32_t PARK_WAIT_MS = 1000;

uint32_t ppp_output(ppp_pcb *pcb, const void *data, uint32_t len, void *ctx)
{
    (void)pcb;
    (void)ctx;
    const int32_t sent = net_platform_ppp_write(static_cast<const uint8_t *>(data), len);
    return sent > 0 ? uint32_t(sent) : 0;
}

// tcpip thread, as AP_Networking_PPP's ppp_status_callback
void ppp_status(ppp_pcb *pcb, int code, void *ctx)
{
    (void)ctx;
    Link now {};
    if (code == PPPERR_NONE) {
        const struct netif *netif = ppp_netif(pcb);
        now.up = true;
        now.ip = lwip_ntohl(netif_ip4_addr(netif)->addr);
        now.netmask = lwip_ntohl(netif_ip4_netmask(netif)->addr);
        now.gateway = lwip_ntohl(netif_ip4_gw(netif)->addr);
    } else if (code != PPPERR_USER) {
        // PPPERR_USER follows our own ppp_close(); any other error loses the link: connect again
        ppp_reconnect.store(true);
    }
    os_protect_enter();
    ppp_link = now;
    os_protect_exit();
}

// back to PPP_PHASE_DEAD without carrier, then connect; false while lwIP is still terminating
bool ppp_restart()
{
    LOCK_TCPIP_CORE();
    if (ppp->phase != PPP_PHASE_DEAD) {
        ppp_close(ppp, 1);
    }
    const bool connecting = ppp->phase == PPP_PHASE_DEAD && ppp_connect(ppp, 0) == ERR_OK;
    UNLOCK_TCPIP_CORE();
    return connecting;
}

void ppp_rx_main(void *arg)
{
    (void)arg;
    LOCK_TCPIP_CORE();
    ppp = pppos_create(&ppp_if, ppp_output, ppp_status, nullptr);
    if (ppp != nullptr) {
        ppp_set_default(ppp);
        ppp_connect(ppp, 0);
    }
    UNLOCK_TCPIP_CORE();
    if (ppp == nullptr) {
        return;   // no PPP pcb: the ports run on loopback only
    }
    static uint8_t buf[512];
    bool transport_open = true;
    for (;;) {
        if (!transport_open) {
            os_sem_wait_ms(&idle, PPP_RETRY_MS);
            transport_open = net_platform_ppp_open();
            if (transport_open && !ppp_restart()) {
                ppp_reconnect.store(true);
            }
            continue;
        }
        if (ppp_reconnect.exchange(false) && !ppp_restart()) {
            ppp_reconnect.store(true);   // still terminating: the next pass tries again
        }
        const int32_t n = net_platform_ppp_read(buf, sizeof(buf), 100);
        if (n > 0) {
            pppos_input_tcpip(ppp, buf, n);
        } else if (n < 0) {
            // the transport closed: no carrier until it opens again
            LOCK_TCPIP_CORE();
            ppp_close(ppp, 1);
            UNLOCK_TCPIP_CORE();
            transport_open = false;
        }
    }
}

void start_thread(const char *name, void (*fn)(void *))
{
    if (os_thread_start(name, NET_THREAD_PRIO, fn, nullptr)) {
        threads_started++;
    }
}

}  // namespace

void net_m4_session_start()
{
    Region &region = *reinterpret_cast<Region *>(net_platform_region());
    for (PortRings &rings : region.ports) {
        ByteRing tx, rx;
        if (tx.attach(&rings.tx_control, rings.tx, PORT_RING_DATA)) {
            tx.consumer_resync();
        }
        if (rx.attach(&rings.rx_control, rings.rx, PORT_RING_DATA)) {
            rx.producer_reset();
        }
    }
}

void net_m4_configure(const Config &new_config)
{
    // checked again here: the M7's message is not trusted
    if (started || !config_valid(new_config) || new_config.enabled == 0) {
        return;
    }
    started = true;
    config = new_config;
    os_sem_init(&idle, 0);
    // starts the tcpip thread (sys_thread_new); one that cannot start faults
    tcpip_init(nullptr, nullptr);
    threads_started++;
    start_thread("net_port", port_service_main);
    if (net_platform_ppp_open()) {
        start_thread("ppp_rx", ppp_rx_main);
    }
}

uint32_t net_m4_threads_started()
{
    return threads_started;
}

void net_m4_park()
{
    park_request.store(true);
    const void *port = port_thread.load();
    if (port == nullptr || port == os_self()) {
        return;   // no net_port thread yet, or the fault is its own
    }
    const uint32_t start_ms = os_now_ms();
    while (!port_parked.load() && os_now_ms() - start_ms < PARK_WAIT_MS) {
    }
}

const Config &NetM4::service_config()
{
    return config;
}

bool NetM4::park_requested()
{
    if (port_thread.load() == nullptr) {
        port_thread.store(os_self());   // only the net_port thread asks
    }
    return park_request.load();
}

void NetM4::park_here()
{
    port_parked.store(true);
    for (;;) {
        net_platform_doorbell_wait(1000);
    }
}

void NetM4::publish_status(const PortReport &report)
{
    StatusFields status {};
    status.magic = STATUS_MAGIC;
    status.session = net_platform_session();
    os_protect_enter();
    const Link now = ppp_link;
    os_protect_exit();
    status.flags = STATUS_STACK_UP | (now.up ? STATUS_LINK_UP : 0U);
    status.ip = now.ip;
    status.netmask = now.netmask;
    status.gateway = now.gateway;
    status.change_count = written.change_count;
    if (!written_once || status.flags != written.flags || status.ip != written.ip ||
        status.netmask != written.netmask || status.gateway != written.gateway) {
        status.change_count++;
    }
    memcpy(status.port_state, report.state, sizeof(status.port_state));
    memcpy(status.port_rx_dropped, report.rx_dropped, sizeof(status.port_rx_dropped));
    memcpy(status.port_tx_dropped, report.tx_dropped, sizeof(status.port_tx_dropped));
    status.heartbeat = written.heartbeat + 1U;
    Region &region = *reinterpret_cast<Region *>(net_platform_region());
    write_status(region.status, status);
    written = status;
    written_once = true;
}

// lwIP's LWIP_HOOK_IP4_ROUTE (lwipopts.h): no gateway route, lwIP's own netif choice
extern "C" struct netif *net_m4_routing_hook(const struct ip4_addr *)
{
    return nullptr;
}
