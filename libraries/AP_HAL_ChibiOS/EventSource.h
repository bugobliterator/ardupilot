#pragma once
#include <hal.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include "AP_HAL_ChibiOS_Namespace.h"
#include <AP_HAL/EventSource.h>
#include <ch.hpp>

#if CH_CFG_USE_EVENTS == TRUE
class ChibiOS::EventSource : public AP_HAL::EventSource {
    friend class ChibiOS::EventHandle;
private:
    // Single event source to be shared across multiple users
    chibios_rt::EventSource ch_evt_src;

public:
    // generate event from thread context
    void signal(uint32_t evt_mask) override;

    // generate event from interrupt context
    void signalI(uint32_t evt_mask) override;
};
#endif //#if CH_CFG_USE_EVENTS == TRUE
