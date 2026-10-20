#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// Only relocation harnesses may load containers without runtime identity.
#ifndef AP_RUST_LEGACY_PROBE_ENABLED
#define AP_RUST_LEGACY_PROBE_ENABLED 0
#endif

// Native modules on the STM32H757 Cortex-M4 (libraries/AP_RustM4) in place of the in-process runtime
#ifndef AP_RUST_M4_ENABLED
#define AP_RUST_M4_ENABLED 0
#endif
