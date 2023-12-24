#pragma once
#include "AP_HAL_Namespace.h"

class AP_HAL::EventSource {
public:
    // generate event from thread context
    virtual void signal(uint32_t evt_mask) = 0;

    // generate event from interrupt context
    virtual void signalI(uint32_t evt_mask) { signal(evt_mask); }
};
