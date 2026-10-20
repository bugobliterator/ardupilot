#pragma once

#include <AP_Module/AP_Module_config.h>

#if AP_MODULE_M4_ENABLED && !AP_MODULE_ENABLED
#error "AP_MODULE_M4_ENABLED needs AP_MODULE_ENABLED"
#endif

// test-only MOD_M4_INJECT, the thread-path selftest and the module fault kinds; to be turned off
// before any release. The M4 Makefile and the SITL M4 process define it to 1.
#ifndef AP_MODULE_M4_FAULT_INJECTION_ENABLED
#define AP_MODULE_M4_FAULT_INJECTION_ENABLED AP_MODULE_M4_ENABLED
#endif
