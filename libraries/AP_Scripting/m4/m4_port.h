#pragma once

/*
  What the portable M4 Lua runtime needs from its platform. The ChibiOS image
  implements it with SVCs from the unprivileged Lua thread
  (m4/chibios/lua_port.cpp), the SITL process on a pthread
  (m4/sitl/lua_port.cpp). Called from the Lua thread only.
 */

#include <stddef.h>
#include <stdint.h>
#include <AP_ScriptingM4_Protocol.h>

namespace ScriptingM4 {

// send a Call or Notify frame (seq set by the service); for a Call wait for the Reply,
// copied to reply. Returns the reply length (0 for a Notify) or -errno: -ETIMEDOUT
// (no reply within CALL_TIMEOUT_MS), -EMSGSIZE, -EINVAL (bad frame), -EFAULT (bad buffer)
int32_t port_call(const uint8_t *request, uint16_t request_len, uint8_t *reply, uint16_t reply_cap);
uint32_t port_millis();
uint32_t port_micros();
void port_sleep_ms(uint32_t ms);
// a fault the M7 asked the Lua thread to perform, FaultKind::None if none
FaultKind port_fault_request();
// perform a Lua-thread fault (LuaAircr, LuaStackOverflow, LuaRccWrite) or a HardFault; does not return
__attribute__((noreturn)) void port_lua_fault(FaultKind kind);
// bytes the Lua heap may grow to on this platform
uint32_t port_heap_capacity();

// the Lua heap's allocator, with realloc and free semantics: newlib-nano on the M4; SITL
// keeps a model of it, so that what fits in SITL fits on the M4
void *port_heap_realloc(void *block, size_t size);
void port_heap_free(void *block);
// heap bytes a live block takes, its allocator overhead included; a block that realloc
// kept in place at a smaller size still costs what it cost before
uint32_t port_heap_block_cost(void *block);

// the most a new block of size bytes costs: newlib-nano takes ((size + 3) & ~3) + 8,
// the SITL model rounds to 8 and so charges at least as much
static inline uint32_t heap_block_cost(size_t size)
{
    return size == 0 ? 0 : uint32_t(((size + 7) & ~size_t(7)) + 8);
}

}  // namespace ScriptingM4
