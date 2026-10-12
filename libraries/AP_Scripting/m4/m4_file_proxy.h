#pragma once

/*
  Files for M4 scripts through the M7 (m4_file_proxy.cpp): the C file calls of
  the Lua core (m4/shim/AP_Filesystem/posix_compat.h), and the dirlist and
  remove globals of lua_bindings.cpp. Lua thread only.
 */

extern "C" {
#include <lua/src/lua.h>
}

namespace ScriptingM4 {

// dirlist and remove into the sandbox table on top of the stack
void open_file_functions(lua_State *L);

}  // namespace ScriptingM4
