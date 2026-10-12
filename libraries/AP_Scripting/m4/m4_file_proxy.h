#pragma once

/*
  Files for M4 scripts through the M7 (m4_file_proxy.cpp): the C file calls of
  the Lua core (m4/shim/AP_Filesystem/posix_compat.h), and the dirlist and
  remove globals of lua_bindings.cpp. Lua thread only.
 */

#include <stdint.h>
extern "C" {
#include <lua/src/lua.h>
}

namespace ScriptingM4 {

// dirlist and remove into the sandbox table on top of the stack
void open_file_functions(lua_State *L);

// the M7 file id of a Lua file (sock:sendfile, m4_wrappers.cpp), with the M7's file positioned
// where the script's reads left it (the read-ahead and an ungetc dropped, as fseek does); -1 if
// file is no open stream of this pool
int32_t file_proxy_id(void *file);

}  // namespace ScriptingM4
