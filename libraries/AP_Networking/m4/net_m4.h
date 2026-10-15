#pragma once

/*
  The M4 network service: lwIP (tcpip thread), PPP over the platform's transport (ppp_rx thread)
  and the NET_P ports between their sockets and the network region's rings (net_port thread,
  net_port_service.cpp). Only the net_port thread touches the region after the session start.
 */

#include <stdint.h>
#include "AP_Networking_M4_Config.h"
#include "AP_Networking_M4_Layout.h"

// main(), before the M4 publishes its peer header: the M4's ends of the port rings, so nothing
// of the last session is sent or delivered (the M4 consumes TX and produces RX)
void net_m4_session_start();
// once per session, from the M4 service; never blocks. An invalid or disabled config starts nothing
void net_m4_configure(const NetM4::Config &config);
// the network threads this process started (tcpip, net_port, ppp_rx)
uint32_t net_m4_threads_started();
// SITL fault path, before the fault is recorded: the net_port thread stops between two ring
// operations, so the M7 can clear the region. Waits at most 1 s (a thread blocked in lwIP)
void net_m4_park();

// for net_port_service.cpp
namespace NetM4 {
struct PortReport {
    uint32_t state[NUM_PORTS];        // PortState
    uint32_t rx_dropped[NUM_PORTS];   // bytes
    uint32_t tx_dropped[NUM_PORTS];   // records
};
const Config &service_config();
// the status block, from the net_port thread every STATUS_PERIOD_MS
void publish_status(const PortReport &report);
bool park_requested();
void park_here();
void port_service_main(void *arg);
}  // namespace NetM4
