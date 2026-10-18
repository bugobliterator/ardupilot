#pragma once

/*
  SCR_M4_RST_MAX on networking builds: every M4 failure restarts the whole M4 with backoff, but
  only failures that can be Lua's spend Lua's restart budget. A network stall, and protocol
  errors from a session that ran without Lua, are the network's: a flaky link never turns Lua
  off. C++11, no ArduPilot dependencies (host unit tests).
 */

#include <stdint.h>

namespace ScriptingM4 {

enum class NetFailure : uint8_t {
    Fault,            // a recorded M4 fault or a process exit: Lua's or the network's
    HeartbeatLost,    // the service thread stopped
    ProtocolErrors,
    NetworkStalled,   // the status heartbeat stood still (NetStall)
};

inline bool charges_lua_budget(NetFailure failure, bool session_lua_permitted)
{
    if (failure == NetFailure::NetworkStalled) {
        return false;
    }
    if (failure == NetFailure::ProtocolErrors && !session_lua_permitted) {
        return false;
    }
    return true;
}

// a network failure's charge on SCR_M4_RST_MAX (used of limit): true when Lua goes off until
// reboot. An uncharged failure leaves the count as it is
inline bool spend_lua_restart(uint8_t &used, uint8_t limit, bool charged)
{
    if (!charged) {
        return false;
    }
    if (used >= limit) {
        return true;
    }
    used++;
    return false;
}

}  // namespace ScriptingM4
