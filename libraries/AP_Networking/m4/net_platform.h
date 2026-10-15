#pragma once

/*
  What the M4 network service needs from its platform: net_platform_chibios.cpp in the M4
  image, net_platform_posix.cpp in the SITL M4 process (both in libraries/AP_Scripting/m4).
  Thread context only, apart from net_platform_doorbell_isr().
 */

#include <stdint.h>

// main(), before net_m4_session_start(): the shared window (SITL; the image uses
// AP_NETWORKING_M4_RING_BASE) and this session's restart count, as published in the peer header
void net_platform_init(uint8_t *window, uint32_t session);
// the M4 image: the HSEM receive callback for AP_NETWORKING_M4_DOORBELL_GROUP
void net_platform_doorbell_isr(uint8_t group);
// the network region (NetM4::Region)
uint8_t *net_platform_region();
// returns at the M7's network doorbell or after timeout_ms
void net_platform_doorbell_wait(uint32_t timeout_ms);
uint32_t net_platform_session();
// the PPP transport: false when there is none
bool net_platform_ppp_open();
// bytes read, 0 after timeout_ms, -1 once the transport closed (open it again)
int32_t net_platform_ppp_read(uint8_t *buf, uint32_t len, uint32_t timeout_ms);
// bytes written, -1 when the transport is closed
int32_t net_platform_ppp_write(const uint8_t *buf, uint32_t len);
