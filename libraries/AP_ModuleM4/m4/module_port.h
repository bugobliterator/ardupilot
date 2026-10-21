#pragma once

/*
  The module thread of the M4 module runtime: on the M4 image an
  unprivileged ChibiOS thread whose host callbacks are SVCs (chibios/), in
  the SITL M4 process a pthread calling the same bodies directly (sitl/).
  One module runs at a time; its context is privileged.
 */

#include <stdint.h>
#include <AP_Module/HostApi.h>
#include <AP_ModuleM4_HostCalls.h>
#include <AP_ModuleM4_LoadLayout.h>
#include "m4_service.h"

namespace ModuleM4 {

// once, before the service runs: the link the module thread posts on, the service that sends
// it, and how to wake the service when it could not send at once
void module_port_init(ModuleLink &link, M4Service &service, void (*wake_service)());
// M4Platform::wake_module: the call on the link is Done
void module_port_wake();

// starts the module: start(&module_host_api, module_token(context.index)) at entry (with the
// Thumb bit on the M4) on its stack in the layout, with rw_base as its static base (r9 on the
// M4). context's ranges, heap, request and reply are set by the caller. Refused while a module
// thread still runs
void module_port_start(ModuleContext &context, const LoadLayout &layout, uint32_t entry, uint32_t rw_base);
// M4 image: regions 6 (stack guard) and 7 (code) off, before the loader writes the module area
void module_port_unpublish();
// stop requested: sleep_ms returns STOPPED at once, as do binding_call and text
void module_port_request_stop(ModuleContext &context);
// true once start() returned, with its result
bool module_port_returned(const ModuleContext &context, int32_t &result);

// M4Platform::m7_millis (M4 image): the M7's millis() from this session's Config, received now;
// time_us takes the M7's TIM5 wraps from it rather than assuming none
void module_port_set_m7_millis(uint32_t m7_millis);

// MOD_M4_INJECT 17 (M4 image): the built-in check of the thread path; M4Platform::start_selftest
void module_port_start_selftest();

// SITL M4 process: module code in its own mapping below 4 GB (the entry is a uint32_t), RW to copy
// into, then its 4096-aligned code prefix published PROT_READ | PROT_EXEC
uint8_t *module_port_code_map(uint32_t size);
bool module_port_code_publish(uint8_t *code, uint32_t code_size);

}  // namespace ModuleM4

extern "C" const AP_ModuleHostApi module_host_api;
