#pragma once

#include <AP_Rust/AP_Rust_config.h>

#if AP_RUST_M4_ENABLED && !AP_SCRIPTING_RUST_ENABLED
#error "AP_RUST_M4_ENABLED needs AP_SCRIPTING_RUST_ENABLED"
#endif

// test-only RUST_M4_INJECT, the thread-path selftest and the module fault kinds; to be turned off
// before any release. The M4 Makefile and the SITL M4 process define it to 1.
#ifndef AP_RUST_M4_FAULT_INJECTION_ENABLED
#define AP_RUST_M4_FAULT_INJECTION_ENABLED AP_RUST_M4_ENABLED
#endif
