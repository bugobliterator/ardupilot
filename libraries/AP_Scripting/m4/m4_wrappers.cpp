/*
  The M4 ends of bindings whose in-process argument or result cannot cross to
  the M7 as is, or whose in-process call blocks (gen-bindings -r
  remote_m4_wrappers[], ROW_M4_WRAPPER). Each runs on the Lua thread as the
  row's closure, with its upvalues, and calls remote_call() for the M7 part.
 */
#include "m4_bindings.h"
#include "m4_file_proxy.h"
#include "m4_lua.h"
#include "m4_port.h"
#include "m4_socket_wait.h"

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

// sock:sendfile(file): the M7 cannot read an M4 Lua file; the row takes the file's id in the
// M7's file table instead, with the M7's file positioned where the script's reads left it.
// Anything but a file fails as in-process (luaL_checkudata's "FILE* expected"); a closed file
// sends -1, which the M7 refuses with false
int wrap_SocketAPM_sendfile(lua_State *L)
{
    luaL_Stream *stream = static_cast<luaL_Stream *>(luaL_checkudata(L, 2, LUA_FILEHANDLE));
    lua_pushinteger(L, stream->closef != nullptr ? file_proxy_id(stream->f) : -1);
    lua_replace(L, 2);
    return remote_call(L);
}

// a socket handle's state bit: set_blocking(false) succeeded, so its calls return at once
static const uint8_t SOCKET_NONBLOCKING = 0x01;

// the socket at argument 1 waits as an in-process blocking socket would: one the script never
// set non-blocking (in-process sockets start blocking)
static bool socket_blocks(lua_State *L)
{
    const uint8_t *state = handle_state(L, 1, uint8_t(lua_tointeger(L, lua_upvalueindex(3))));
    return state != nullptr && (*state & SOCKET_NONBLOCKING) == 0;
}

// the first result of the socket at argument 1's method name, called with an integer argument
// (or none, argument < 0), as a boolean
static bool socket_method(lua_State *L, const char *name, int argument = -1)
{
    lua_getfield(L, 1, name);
    lua_pushvalue(L, 1);
    if (argument >= 0) {
        lua_pushinteger(L, argument);
    }
    lua_call(L, argument >= 0 ? 2 : 1, 1);
    const bool result = lua_toboolean(L, -1) != 0;
    lua_pop(L, 1);
    return result;
}

// sock:set_blocking(blocking): the M7 keeps the socket non-blocking and returns whether it is
// open; the M4 keeps the mode the script asked for, which it waits by
int wrap_SocketAPM_set_blocking(lua_State *L)
{
    const int results = remote_call(L);
    uint8_t *state = handle_state(L, 1, uint8_t(lua_tointeger(L, lua_upvalueindex(3))));
    if (results == 1 && lua_toboolean(L, -1) && state != nullptr) {
        *state = lua_toboolean(L, 2) ? uint8_t(*state & ~SOCKET_NONBLOCKING) : uint8_t(*state | SOCKET_NONBLOCKING);
    }
    return results;
}

// sock:connect(addr, port): the M7's connect is non-blocking and returns false while the
// handshake runs (is_pending). A blocking socket waits for it here, as in-process connect()
// returns once connected: it polls the socket writable, which ends the pending connect, until
// SOCKET_IDLE_LIMIT_MS have passed (then false, the connect still running on the M7)
int wrap_SocketAPM_connect(lua_State *L)
{
    const int results = remote_call(L);
    if (results != 1 || lua_toboolean(L, -1) || !socket_blocks(L) || !socket_method(L, "is_pending")) {
        return results;
    }
    (void)socket_wait([L]() { return socket_method(L, "pollout", 0) ? SocketStep::Done : SocketStep::Wait; },
                      lua_sleep_ms, port_millis);
    lua_pop(L, 1);
    lua_pushboolean(L, socket_method(L, "is_connected"));
    return 1;
}

// sock:send(data, len) and sock:sendto(data, len, addr, port): the M7's call returns the count
// and whether it would have blocked. A non-blocking socket gets the count. A blocking socket
// sends all of it, as in-process: the rest of the data again after a call that sent part of it
// or would have blocked, until all is sent, a call fails, or SOCKET_IDLE_LIMIT_MS pass without
// progress; the result is the bytes sent, else the failed call's count
static int socket_send(lua_State *L)
{
    const int argc = lua_gettop(L);
    luaL_checkstack(L, argc + 4, "M4 socket send");
    // the row's call, kept at argc + 1 for the tries
    lua_pushvalue(L, lua_upvalueindex(1));
    lua_pushvalue(L, lua_upvalueindex(2));
    lua_pushvalue(L, lua_upvalueindex(3));
    lua_pushcclosure(L, remote_call, 3);
    const int call = argc + 1;
    lua_pushvalue(L, call);
    for (int i = 1; i <= argc; i++) {
        lua_pushvalue(L, i);
    }
    lua_call(L, argc, 2);
    int32_t count = int32_t(lua_tointeger(L, -2));
    const bool would_block = lua_toboolean(L, -1) != 0;
    lua_pop(L, 1);   // the count is the result
    size_t length = 0;
    // the M7 took arguments 2 and 3 as a string and a length; argument 2 stays on the stack
    const char *data = lua_tolstring(L, 2, &length);
    if (!socket_blocks(L) || data == nullptr) {
        return 1;
    }
    const uint32_t requested = coerce_to_uint32_t(L, 3);
    SocketSend progress(requested < length ? size_t(requested) : length);
    if (progress.add(count, would_block) == SocketStep::Done) {
        return 1;
    }
    (void)socket_wait([&]() {
        lua_settop(L, call);
        lua_pushvalue(L, call);
        lua_pushvalue(L, 1);
        lua_pushlstring(L, data + progress.sent(), progress.remaining());
        lua_pushinteger(L, lua_Integer(progress.remaining()));
        for (int i = 4; i <= argc; i++) {
            lua_pushvalue(L, i);
        }
        lua_call(L, argc, 2);
        count = int32_t(lua_tointeger(L, -2));
        return progress.add(count, lua_toboolean(L, -1) != 0);
    }, lua_sleep_ms, port_millis);
    lua_settop(L, call);
    lua_pushinteger(L, progress.result(count));
    return 1;
}

int wrap_SocketAPM_send(lua_State *L)
{
    return socket_send(L);
}

int wrap_SocketAPM_sendto(lua_State *L)
{
    return socket_send(L);
}

}  // namespace ScriptingM4
