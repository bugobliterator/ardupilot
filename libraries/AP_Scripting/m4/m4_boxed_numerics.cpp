/*
  uint32_t and uint64_t on the M4: lua_boxed_numerics.cpp without AP_HAL, with
  the same conversions and texts. newlib-nano's printf has no %llu, so
  uint64_t is formatted here.
 */
#include "m4_binding_deps.h"

uint32_t coerce_to_uint32_t(lua_State *L, int arg)
{
    {   // userdata
        const uint32_t *ud = static_cast<uint32_t *>(luaL_testudata(L, arg, "uint32_t"));
        if (ud != nullptr) {
            return *ud;
        }
    }
    {   // integer
        int success;
        const lua_Integer v = lua_tointegerx(L, arg, &success);
        if (success) {
            return static_cast<uint32_t>(v);
        }
    }
    {   // float
        int success;
        const lua_Number v = lua_tonumberx(L, arg, &success);
        if (success && v >= 0 && v <= float(UINT32_MAX)) {
            return static_cast<uint32_t>(v);
        }
    }
    return luaL_argerror(L, arg, "Unable to coerce to uint32_t");
}

uint64_t coerce_to_uint64_t(lua_State *L, int arg)
{
    {   // uint64_t userdata
        const uint64_t *ud = static_cast<uint64_t *>(luaL_testudata(L, arg, "uint64_t"));
        if (ud != nullptr) {
            return *ud;
        }
    }
    {   // integer: Lua's int32, so negative numbers are refused rather than wrapped
        int success;
        const lua_Integer v = lua_tointegerx(L, arg, &success);
        if (success && v >= 0) {
            return static_cast<uint64_t>(v);
        }
    }
    {   // uint32_t userdata
        const uint32_t *ud = static_cast<uint32_t *>(luaL_testudata(L, arg, "uint32_t"));
        if (ud != nullptr) {
            return static_cast<uint64_t>(*ud);
        }
    }
    {   // float
        int success;
        const lua_Number v = lua_tonumberx(L, arg, &success);
        if (success && (v >= 0) && (v <= float(UINT64_MAX))) {
            return static_cast<uint64_t>(v);
        }
    }
    return luaL_argerror(L, arg, "Unable to coerce to uint64_t");
}

int lua_new_uint32_t(lua_State *L)
{
    const int args = lua_gettop(L);
    if (args > 1) {
        return luaL_argerror(L, args, "too many arguments");
    }
    *new_uint32_t(L) = (args == 1) ? coerce_to_uint32_t(L, 1) : 0;
    return 1;
}

int lua_new_uint64_t(lua_State *L)
{
    const int args = lua_gettop(L);
    if (args > 2) {
        return luaL_argerror(L, args, "too many arguments");
    }
    uint64_t value = 0;
    if (args == 1) {
        value = coerce_to_uint64_t(L, 1);
    } else if (args == 2) {
        // two uint32 giving the high and low half
        const uint64_t high = coerce_to_uint32_t(L, 1);
        const uint64_t low = coerce_to_uint32_t(L, 2);
        value = (high << 32) | low;
    }
    *new_uint64_t(L) = value;
    return 1;
}

int uint32_t_toint(lua_State *L)
{
    binding_argcheck(L, 1);
    const uint32_t v = *check_uint32_t(L, 1);
    lua_pushinteger(L, static_cast<lua_Integer>(v));
    return 1;
}

int uint64_t_toint(lua_State *L)
{
    binding_argcheck(L, 1);
    const uint64_t v = *check_uint64_t(L, 1);
    if (v > INT32_MAX) {
        return 0;   // too large for an integer: nil rather than an error, as in-process
    }
    lua_pushinteger(L, static_cast<lua_Integer>(v));
    return 1;
}

int uint32_t_tofloat(lua_State *L)
{
    binding_argcheck(L, 1);
    const uint32_t v = *check_uint32_t(L, 1);
    lua_pushnumber(L, static_cast<lua_Number>(v));
    return 1;
}

int uint64_t_tofloat(lua_State *L)
{
    binding_argcheck(L, 1);
    const uint64_t v = *check_uint64_t(L, 1);
    lua_pushnumber(L, static_cast<lua_Number>(v));
    return 1;
}

int uint32_t___tostring(lua_State *L)
{
    binding_argcheck(L, 1);
    const uint32_t v = *check_uint32_t(L, 1);
    char buf[32];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)v);
    lua_pushstring(L, buf);
    return 1;
}

int uint64_t___tostring(lua_State *L)
{
    binding_argcheck(L, 1);
    uint64_t v = *check_uint64_t(L, 1);
    // "%llu" by hand: newlib-nano's printf has no long long
    char digits[21];
    size_t n = 0;
    do {
        digits[n++] = char('0' + v % 10);
        v /= 10;
    } while (v != 0);
    char buf[21];
    for (size_t i = 0; i < n; i++) {
        buf[i] = digits[n - 1 - i];
    }
    buf[n] = 0;
    lua_pushstring(L, buf);
    return 1;
}

int uint64_t_split(lua_State *L)
{
    binding_argcheck(L, 1);
    const uint64_t v = *check_uint64_t(L, 1);
    *new_uint32_t(L) = uint32_t(v >> 32);          // high
    *new_uint32_t(L) = uint32_t(v & 0xFFFFFFFF);   // low
    return 2;
}
