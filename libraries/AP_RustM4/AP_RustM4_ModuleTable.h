#pragma once

#include <stdint.h>

namespace RustM4 {

// the PR's Runtime::MAX_MODULES
static const uint8_t MAX_MODULES = 8;

/*
  One module of an M4 session on the M7: the owner the generated adapters key their
  per-module state on (the context of binding_dispatch() and of the binding_*_cleanup()
  calls), as the PR's Runtime passes its Slot
 */
struct ModuleSlot {
    uint16_t index;
    uint32_t session;
};

}  // namespace RustM4
