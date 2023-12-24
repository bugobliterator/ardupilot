#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/EventHandle.h>
#include "EventSource.h"
#include "Semaphores.h"
#include "AP_HAL_ChibiOS_Namespace.h"
#include <ch.hpp>

#if CH_CFG_USE_EVENTS == TRUE
class ChibiOS::EventHandle : public AP_HAL::EventHandle {
public:
    // return true if event type was successfully registered
    // to be called from thread context where wait will be called
    void register_events() override;

    // adds event to be registered when register_events is called
    void add_events(uint32_t mask) override;

    uint32_t wait(uint16_t duration_us) override;

    void set_source(ChibiOS::EventSource *src) { evt_src = src; }
private:
    chibios_rt::EventListener evt_listener;
    ChibiOS::EventSource *evt_src;
    uint32_t evt_mask;
    ChibiOS::Semaphore sem;
    bool handle_registered;
    thread_t *thread;
};
#endif
