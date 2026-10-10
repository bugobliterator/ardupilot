#pragma once

/*
  Lua calls from the M4 runtime: one call at a time, on the Lua thread.
  remote_stub() is the client stub of every remote function: it encodes the
  Lua arguments, calls the M7 and pushes the results, or raises the M7's
  error with the in-process text.
 */

#include <stdint.h>
#include <AP_ScriptingM4_Rpc.h>

struct lua_State;

namespace ScriptingM4 {

class RpcClient {
public:
    // start a frame for method; append its values to the returned writer
    RpcWriter &begin(uint16_t method, bool notify = false);
    RpcWriter &begin(SystemMethod method, bool notify = false) { return begin(uint16_t(method), notify); }
    // send; for a Call wait for and decode the reply. 0 or -errno
    int invoke();
    ReplyStatus status() const { return ReplyStatus(_reply_header.status); }
    uint8_t error_arg() const { return _reply_header.error_arg; }
    uint8_t count() const { return _count; }
    // valid until the next begin()
    const RpcValue &value(uint8_t index) const { return _values[index]; }

private:
    uint8_t _request[MAX_FRAME];
    uint8_t _reply[MAX_FRAME];
    RpcWriter _writer;
    ReplyHeader _reply_header;
    RpcValue _values[MAX_VALUES];
    uint16_t _method;
    uint8_t _count;
    bool _notify;
};

extern RpcClient rpc;

// adds the remote functions to the sandbox table on top of the stack
void open_remote_functions(lua_State *L);

}  // namespace ScriptingM4
