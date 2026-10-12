#pragma once

#include <stdint.h>

namespace ScriptingM4 {

// the M4 Lua runtime: configure from the M7, load the scripts, run them; never returns.
// Runs on the Lua thread (unprivileged on the M4 image).
__attribute__((noreturn)) void lua_main();
// sleeps on the Lua thread, taking the M7's fault requests every 100 ms (a binding that waits)
void lua_sleep_ms(uint32_t ms);

}  // namespace ScriptingM4
