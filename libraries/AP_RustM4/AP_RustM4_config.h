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

// test-only: the host thread at main boost + 1 (183) and the worker at main boost (182), above
// the main loop and the timer thread, to measure the transport without their preemption
#ifndef AP_RUST_M4_PRIORITY_TEST
#define AP_RUST_M4_PRIORITY_TEST 0
#endif
