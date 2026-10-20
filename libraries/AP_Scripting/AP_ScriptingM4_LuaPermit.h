#pragma once

/*
  The Lua permit of an M4 session as the M7 granted it (networking builds, where the M4 also runs
  with Lua held). The M7 keeps its own copy and never reads LUA_PERMIT_SLOT back: the M4 can
  write the shared window. A session started without the permit runs no Lua, so its Call and
  Notify frames are refused. No ArduPilot dependencies (host unit tests).
 */

#include "AP_ScriptingM4_Protocol.h"

namespace ScriptingM4 {

class LuaPermit {
public:
    // the value written to LUA_PERMIT_SLOT for the next M4 start
    void store_next(bool permitted) { _next = permitted; }
    // a start or a restart grant: the new session runs under the stored value
    void session_granted() { _session = _next; }
    bool session_permitted() const { return _session; }
    // Call and Notify carry Lua's calls: bindings, files, texts, the script list. Stats and
    // NetConfigRequest come from the M4 service and are served either way
    bool serves(MsgType type) const
    {
        return _session || (type != MsgType::Call && type != MsgType::Notify);
    }

private:
    bool _next = false;
    bool _session = false;
};

// what refuse_slow_binding() does with a frame while the duty cap refuses slow bindings
enum class SlowCallRoute : uint8_t {
    NotRefused,      // not a binding call: served as usual (handle_message() applies the permit)
    ProtocolError,   // a Lua binding call from a session without Lua: no reply, a protocol error
    DutyCapError,    // refused with the duty cap's Error reply
};
// binding_method: a Call or Notify at or above FIRST_BINDING_METHOD
inline SlowCallRoute slow_call_route(MsgType type, bool binding_method, bool network_wanted, const LuaPermit &permit)
{
    if ((type != MsgType::Call && type != MsgType::Notify) || !binding_method) {
        return SlowCallRoute::NotRefused;
    }
    if (network_wanted && !permit.serves(type)) {
        return SlowCallRoute::ProtocolError;
    }
    return SlowCallRoute::DutyCapError;
}

}  // namespace ScriptingM4
