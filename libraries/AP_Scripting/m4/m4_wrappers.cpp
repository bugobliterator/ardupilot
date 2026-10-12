/*
  The M4 ends of manual bindings whose in-process argument or result cannot
  cross to the M7 as is (gen-bindings -r remote_m4_wrappers[], ROW_M4_WRAPPER).
  Each runs on the Lua thread as the row's closure, with its upvalues, and
  calls remote_call() for the M7 part.
 */
#include "m4_bindings.h"

namespace ScriptingM4 {

// dev:read_registers(first, count): the in-process binding returns a table of the bytes
// (AP_HAL__I2CDevice_read_registers); the M7 returns them as a string
int wrap_I2CDevice_read_registers(lua_State *L)
{
    const bool multi_register = lua_gettop(L) == 3;
    const int results = remote_call(L);
    if (!multi_register || results != 1 || lua_type(L, -1) != LUA_TSTRING) {
        return results;
    }
    size_t length = 0;
    const char *bytes = lua_tolstring(L, -1, &length);   // stays on the stack below the table
    lua_createtable(L, int(length), 0);
    for (size_t i = 0; i < length; i++) {
        lua_pushinteger(L, lua_Integer(i) + 1);
        lua_pushinteger(L, uint8_t(bytes[i]));
        lua_settable(L, -3);
    }
    return 1;
}

}  // namespace ScriptingM4
