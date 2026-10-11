#pragma once

/*
  Calls from the M4 runtime to the M7: one call at a time, on the Lua thread.
  The Lua side of a remote binding (its stub) is in m4_bindings.cpp.
 */

#include <stdint.h>
#include <AP_ScriptingM4_Rpc.h>

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
    // the reply's values decoded from a copy in buf: a call made while they are pushed
    // (from a finalizer) reuses the client's buffers
    bool copy_reply(uint8_t *buf, uint16_t size, RpcValue *values, uint8_t &count) const;

private:
    uint8_t _request[MAX_FRAME];
    uint8_t _reply[MAX_FRAME];
    RpcWriter _writer;
    ReplyHeader _reply_header;
    RpcValue _values[MAX_VALUES];
    uint16_t _method;
    uint16_t _reply_length;   // of the last reply, header included
    uint8_t _count;
    bool _notify;
};

extern RpcClient rpc;

}  // namespace ScriptingM4
