#pragma once

#include "AP_HAL_Namespace.h"

class AP_HAL::EventHandle {
public:
    // return true if event type was successfully registered
    // to be called from thread context where wait will be called
    virtual void register_events() = 0;

    // add events
    virtual void add_events(uint32_t evt_mask) = 0;

    // return true if event was triggered within the duration
    virtual uint32_t wait(uint16_t duration_us) = 0;
};
