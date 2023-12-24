#include "EventHandle.h"
#include "EventSource.h"
#include "ch.h"
#include "hal.h"

void ChibiOS::EventHandle::register_events()
{
    if (evt_src == nullptr) {
        osalDbgAssert(false, "EventHandle::register_events() called without setting evt_src");
        return;
    }
    evt_src->ch_evt_src.unregister(&evt_listener); // unregister first to avoid double registration
    evt_src->ch_evt_src.registerMask(&evt_listener, evt_mask);
    handle_registered = true;
}

void ChibiOS::EventHandle::add_events(uint32_t mask)
{
    WITH_SEMAPHORE(sem);
    osalDbgAssert((evt_mask & mask) == 0, "EventHandle::add_events() called with mask that is already registered");
    evt_mask |= mask;
    return;
}

uint32_t ChibiOS::EventHandle::wait(uint16_t duration_us)
{
    if (evt_src == nullptr) {
        osalDbgAssert(false, "EventHandle::wait() called without setting evt_src");
        return 0;
    }
    // if not registered, raise assert
    osalDbgAssert(handle_registered && (evt_listener.ev_listener.listener == chThdGetSelfX()), "EventHandle::wait() called without registering first");
    if (duration_us == 0) {
        return chEvtWaitAnyTimeout(evt_mask, TIME_IMMEDIATE);
    } else {
        return chEvtWaitAnyTimeout(evt_mask, OSAL_US2I(duration_us));
    }
}
