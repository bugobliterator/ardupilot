/*
  The M4 network service's platform in the M4 image: the region at AP_NETWORKING_M4_RING_BASE,
  the M7's network doorbell (HSEM group AP_NETWORKING_M4_DOORBELL_GROUP, registered by main())
  and no PPP transport yet (the UART driver comes with the PPP UART handover).
 */
#include "ch.h"

#include <AP_ScriptingM4_Placement.h>
#include "net_platform.h"

static binary_semaphore_t doorbell;
static uint32_t session;

void net_platform_init(uint8_t *window, uint32_t restarts)
{
    (void)window;
    session = restarts;
    chBSemObjectInit(&doorbell, true);
}

// HSEM receive callback, in the HSEM interrupt
void net_platform_doorbell_isr(uint8_t group)
{
    (void)group;
    chSysLockFromISR();
    chBSemSignalI(&doorbell);
    chSysUnlockFromISR();
}

uint8_t *net_platform_region()
{
    return reinterpret_cast<uint8_t *>(AP_NETWORKING_M4_RING_BASE);
}

void net_platform_doorbell_wait(uint32_t timeout_ms)
{
    (void)chBSemWaitTimeout(&doorbell, TIME_MS2I(timeout_ms));
}

uint32_t net_platform_session()
{
    return session;
}

bool net_platform_ppp_open()
{
    return false;
}

int32_t net_platform_ppp_read(uint8_t *buf, uint32_t len, uint32_t timeout_ms)
{
    (void)buf;
    (void)len;
    (void)timeout_ms;
    return -1;
}

int32_t net_platform_ppp_write(const uint8_t *buf, uint32_t len)
{
    (void)buf;
    (void)len;
    return -1;
}
