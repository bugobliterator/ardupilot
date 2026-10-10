#pragma once

/*
  What the portable M4 Lua runtime needs from its platform. The ChibiOS image
  implements it with SVCs from the unprivileged Lua thread
  (m4/chibios/lua_port.cpp), the SITL process on a pthread
  (m4/sitl/lua_port.cpp). Called from the Lua thread only.
 */

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

}  // namespace ScriptingM4
