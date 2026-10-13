/*
  The M4 ends of bindings whose in-process argument or result cannot cross to
  the M7 as is, or whose in-process call blocks (gen-bindings -r
  remote_m4_wrappers[], ROW_M4_WRAPPER). Each runs on the Lua thread as the
  row's closure, with its upvalues, and calls remote_call() for the M7 part.
 */
#include "m4_bindings.h"
#include <AP_ScriptingM4_Rpc.h>
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

// a table field as the in-process lua_GCS_command_int() reads it (lua_gettable, so __index runs):
// pushed as a value if it is a number (numeric strings included: lua_isnumber) or, with
// integers_only, an integer (lua_isinteger); else nil, which leaves the packet field 0
static void push_command_field(lua_State *L, const char *name, bool integers_only)
{
    lua_pushstring(L, name);
    lua_gettable(L, 3);
    if (integers_only ? lua_isinteger(L, -1) : lua_isnumber(L, -1)) {
        if (integers_only) {
            lua_pushinteger(L, lua_tointeger(L, -1));
        } else {
            lua_pushnumber(L, lua_tonumber(L, -1));
        }
    } else {
        lua_pushnil(L);
    }
    lua_remove(L, -2);
}

// gcs:run_command_int(command, table): a Lua table cannot cross to the M7, so its fields go as
// values 3..10 (p1, p2, p3, p4, x, y, z, frame; command_int_packet() on the M7). Without a table
// the command goes alone, and the M7 checks it and returns nothing, as in-process
int wrap_GCS_run_command_int(lua_State *L)
{
    const int top = lua_gettop(L);
    if (top != 3) {
        // binding_argcheck(L, 3)'s in-process texts
        return luaL_argerror(L, top, top > 3 ? "too many arguments" : "too few arguments");
    }
    if (!lua_istable(L, 3)) {
        lua_settop(L, 2);
        return remote_call(L);
    }
    static const char *const params[] = { "p1", "p2", "p3", "p4" };
    for (const char *name : params) {
        push_command_field(L, name, false);
    }
    push_command_field(L, "x", true);
    push_command_field(L, "y", true);
    push_command_field(L, "z", false);
    push_command_field(L, "frame", true);
    lua_remove(L, 3);   // the table: the values follow the command
    return remote_call(L);
}

// a socket handle's state bits: set_blocking(false) succeeded, so its calls return at once; it
// was made with Socket(1), so its sends are datagrams
static const uint8_t SOCKET_NONBLOCKING = 0x01;
static const uint8_t SOCKET_DATAGRAM = 0x02;

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

// a piece of a send is one call: self, the string, its length, and sendto's address and port
static_assert(sizeof(CallHeader) + 8 + 3 + SOCKET_SEND_CHUNK + 5 + 8 + 5 <= MAX_FRAME,
              "a stream send's piece fits one call frame");

// sock:send(data, len) and sock:sendto(data, len, addr, port) (socket_send_all()). The M7's call
// returns the count and whether it would have blocked; the script gets the count. A stream
// socket's string goes in SOCKET_SEND_CHUNK pieces; a datagram (sendto, or send on a Socket(1))
// goes whole, and one too long for a call frame fails with "M4: call arguments too large".
// A blocking socket tries again where in-process the call blocks
static int socket_send(lua_State *L, bool to_address)
{
    const int argc = lua_gettop(L);
    luaL_checkstack(L, argc + 4, "M4 socket send");
    // the row's call, kept at argc + 1 for every piece
    lua_pushvalue(L, lua_upvalueindex(1));
    lua_pushvalue(L, lua_upvalueindex(2));
    lua_pushvalue(L, lua_upvalueindex(3));
    lua_pushcclosure(L, remote_call, 3);
    const int call = argc + 1;
    const uint8_t *state = handle_state(L, 1, uint8_t(lua_tointeger(L, lua_upvalueindex(3))));
    size_t length = 0;
    const char *data = lua_type(L, 2) == LUA_TSTRING ? lua_tolstring(L, 2, &length) : nullptr;
    if (state == nullptr || data == nullptr || argc < 3) {
        // the M7 refuses what is no socket or no string, with the in-process error
        lua_pushvalue(L, call);
        for (int i = 1; i <= argc; i++) {
            lua_pushvalue(L, i);
        }
        lua_call(L, argc, 2);
        lua_pop(L, 1);
        return 1;
    }
    const bool datagram = to_address || (*state & SOCKET_DATAGRAM) != 0;
    const bool blocking = (*state & SOCKET_NONBLOCKING) == 0;
    const int32_t sent = socket_send_all(length, datagram, blocking,
    [&](size_t offset, size_t piece, bool first, bool &would_block) {
        lua_settop(L, call);
        lua_pushvalue(L, call);
        lua_pushvalue(L, 1);
        lua_pushlstring(L, data + offset, piece);
        if (first) {
            lua_pushvalue(L, 3);   // the script's length: the M7 checks it, and sends at most the piece
        } else {
            lua_pushinteger(L, lua_Integer(piece));
        }
        for (int i = 4; i <= argc; i++) {
            lua_pushvalue(L, i);
        }
        lua_call(L, argc, 2);
        would_block = lua_toboolean(L, -1) != 0;
        return int32_t(lua_tointeger(L, -2));
    }, [L, length]() {
        // the M7 took argument 3 as a uint32_t: the M4's conversion of it agrees
        const uint32_t requested = coerce_to_uint32_t(L, 3);
        return requested < length ? size_t(requested) : length;
    }, lua_sleep_ms, port_millis);
    lua_settop(L, call);
    lua_pushinteger(L, sent);
    return 1;
}

int wrap_SocketAPM_send(lua_State *L)
{
    return socket_send(L, false);
}

int wrap_SocketAPM_sendto(lua_State *L)
{
    return socket_send(L, true);
}

// Socket(datagram): the M4 keeps whether the new socket is a datagram one, which a send never
// cuts into pieces (an accepted socket is always a stream one)
int wrap_SocketAPM_new(lua_State *L)
{
    const int results = remote_call(L);
    if (results == 1 && lua_tointeger(L, 1) != 0) {   // the M7 took it as a uint8_t
        uint8_t *state = handle_state(L, -1, 0);
        if (state != nullptr) {
            *state |= SOCKET_DATAGRAM;
        }
    }
    return results;
}

}  // namespace ScriptingM4
