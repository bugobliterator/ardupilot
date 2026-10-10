#pragma once

namespace ScriptingM4 {

// the M4 Lua runtime: configure from the M7, load the scripts, run them; never returns.
// Runs on the Lua thread (unprivileged on the M4 image).
__attribute__((noreturn)) void lua_main();

}  // namespace ScriptingM4
