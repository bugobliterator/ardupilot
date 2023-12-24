#include "EventSource.h"
#include <AP_Math/AP_Math.h>

#if CH_CFG_USE_EVENTS == TRUE

void ChibiOS::EventSource::signal(uint32_t evt_mask)
{
    ch_evt_src.broadcastFlags(evt_mask);
}

__RAMFUNC__ void ChibiOS::EventSource::signalI(uint32_t evt_mask)
{
    chSysLockFromISR();
    ch_evt_src.broadcastFlagsI(evt_mask);
    chSysUnlockFromISR();
}
#endif //#if CH_CFG_USE_EVENTS == TRUE
