#include "m4_rpc_client.h"

#include <errno.h>
#include <string.h>
// the Lua headers alone: lua.hpp pulls in AP_Filesystem, which the M4 image does not have
extern "C" {
#include <lua/src/lua.h>
#include <lua/src/lualib.h>
#include <lua/src/lauxlib.h>
}
#include "m4_port.h"

namespace ScriptingM4 {

RpcClient rpc;

RpcWriter &RpcClient::begin(uint16_t method, bool notify)
{
    _method = method;
    _notify = notify;
    _count = 0;
    memset(&_reply_header, 0, sizeof(_reply_header));
    _writer.begin(_request, sizeof(_request), sizeof(CallHeader));
    return _writer;
}

int RpcClient::invoke()
{
    if (!_writer.ok()) {
        return -EMSGSIZE;
    }
    CallHeader call;
    fill_header(call.header, _notify ? MsgType::Notify : MsgType::Call, _writer.length(), 0);
    call.method = _method;
    call.argc = _writer.count();
    call.reserved = 0;
    memcpy(_request, &call, sizeof(call));
    const int32_t ret = port_call(_request, _writer.length(), _reply, sizeof(_reply));
    if (ret < 0) {
        return ret;
    }
    if (_notify) {
        return 0;
    }
    if (ret < int32_t(sizeof(ReplyHeader))) {
        return -EBADMSG;
    }
    memcpy(&_reply_header, _reply, sizeof(_reply_header));
    if (!decode_values(&_reply[sizeof(ReplyHeader)], uint16_t(ret) - sizeof(ReplyHeader),
                       _reply_header.count, _values)) {
        return -EBADMSG;
    }
    _count = _reply_header.count;
    return 0;
}

static void push_value(lua_State *L, const RpcValue &value)
{
    switch (value.tag) {
    case ValueTag::False:
    case ValueTag::True:
        lua_pushboolean(L, value.tag == ValueTag::True);
        return;
    case ValueTag::Integer:
        lua_pushinteger(L, value.integer);
        return;
    case ValueTag::Number:
        lua_pushnumber(L, value.number);
        return;
    case ValueTag::String:
        lua_pushlstring(L, value.string, value.length);
        return;
    case ValueTag::Nil:
    case ValueTag::Opaque:
        break;
    }
    lua_pushnil(L);
}

// raises the error of a failed reply; luaL_argerror() renumbers for method calls, as in-process
static int raise_reply_error(lua_State *L)
{
    char message[128];
    message[0] = 0;
    if (rpc.count() >= 1 && rpc.value(0).tag == ValueTag::String) {
        const RpcValue &text = rpc.value(0);
        const size_t length = text.length < sizeof(message) - 1 ? text.length : sizeof(message) - 1;
        memcpy(message, text.string, length);
        message[length] = 0;
    }
    switch (rpc.status()) {
    case ReplyStatus::ArgError:
        return luaL_argerror(L, rpc.error_arg(), message);
    case ReplyStatus::Error:
        return luaL_error(L, "%s", message);
    case ReplyStatus::UnknownMethod:
        return luaL_error(L, "M4: method not served by the M7");
    case ReplyStatus::Ok:
    case ReplyStatus::Malformed:
        break;
    }
    return luaL_error(L, "M4: malformed call");
}

// upvalue 1: method id; upvalue 2: true if argument 1 is the self of a singleton (sent as nil)
static int remote_stub(lua_State *L)
{
    const uint16_t method = uint16_t(lua_tointeger(L, lua_upvalueindex(1)));
    const bool has_self = lua_toboolean(L, lua_upvalueindex(2)) != 0;
    RpcWriter &args = rpc.begin(method);
    const int top = lua_gettop(L);
    for (int i = 1; i <= top; i++) {
        if (i == 1 && has_self) {
            args.nil();
            continue;
        }
        switch (lua_type(L, i)) {
        case LUA_TNIL:
            args.nil();
            break;
        case LUA_TBOOLEAN:
            args.boolean(lua_toboolean(L, i) != 0);
            break;
        case LUA_TNUMBER:
            if (lua_isinteger(L, i)) {
                args.integer(int32_t(lua_tointeger(L, i)));
            } else {
                args.number(float(lua_tonumber(L, i)));
            }
            break;
        case LUA_TSTRING: {
            size_t length = 0;
            const char *text = lua_tolstring(L, i, &length);
            args.string(text, length);
            break;
        }
        default:
            args.opaque(uint8_t(lua_type(L, i)));   // the M7 names it in "... expected, got table"
            break;
        }
    }
    if (!args.ok()) {
        return luaL_error(L, "M4: call arguments too large");
    }
    const int ret = rpc.invoke();
    if (ret != 0) {
        return luaL_error(L, "M4: call failed (%d)", ret);
    }
    if (rpc.status() != ReplyStatus::Ok) {
        return raise_reply_error(L);
    }
    // a finalizer run while the results are pushed could make a call and reuse the reply
    // buffer; the system methods Lua calls return nothing, Generated Bindings copies first
    luaL_checkstack(L, rpc.count(), "M4 results");
    for (uint8_t i = 0; i < rpc.count(); i++) {
        push_value(L, rpc.value(i));
    }
    return rpc.count();
}

static void push_stub(lua_State *L, SystemMethod method, bool has_self)
{
    lua_pushinteger(L, lua_Integer(method));
    lua_pushboolean(L, has_self);
    lua_pushcclosure(L, remote_stub, 2);
}

void open_remote_functions(lua_State *L)
{
    // stand-ins until the Generated Bindings sub-phase: gcs:send_text and print
    lua_newtable(L);
    push_stub(L, SystemMethod::GcsSendText, true);
    lua_setfield(L, -2, "send_text");
    lua_setfield(L, -2, "gcs");
    push_stub(L, SystemMethod::Print, false);
    lua_setfield(L, -2, "print");
}

}  // namespace ScriptingM4
