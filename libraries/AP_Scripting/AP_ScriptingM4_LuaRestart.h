#pragma once

/*
  Lua-only restarts (networking builds). The M4 parks its Lua thread on a Lua fault, a Lua stall,
  a hold or a requested park and reports it (MsgType::LuaParked) until answered; the M7 answers
  every report it accepts with a new grant in LUA_CONTROL_SLOT, and LUA_HOLD says whether Lua
  stays parked. Faults and stalls are charged to SCR_M4_RST_MAX; at the limit Lua is off and the
  M4 runs on. The M7 keeps this copy and never reads the slot back.
  A new M4 session expects park 1 again under the same grant, so session_started() belongs where
  the M7 resyncs the channel at that session's start (AP_ScriptingM4::check_started(), on the
  thread that serves frames): the old session no longer runs once the new one publishes, and the
  resync drops whatever it queued, so no report of it is read after session_started().
  C++11, no ArduPilot dependencies (host unit tests).
 */

#include <stdint.h>
#include "AP_ScriptingM4_NetBudget.h"
#include "AP_ScriptingM4_Rpc.h"

namespace ScriptingM4 {

class LuaRestart {
public:
    enum class State : uint8_t {
        Running,    // the Lua thread runs scripts
        Stopping,   // LUA_HOLD set: its park report is awaited
        Parked,     // parked and answered: stopped, or its session started stopped
        Off,        // the fault limit, until reboot
    };
    enum class Outcome : uint8_t {
        Ignored,     // a report already answered, of another grant, or of a session about to restart
        Restarted,   // a fresh Lua session runs
        Stopped,     // parked until a restart request
        Off,         // the fault limit
    };
    static const uint32_t PARK_TIMEOUT_MS = 3000;

    // LUA_CONTROL_SLOT for the next M4 session
    uint32_t next_session_word(bool lua_runs) const
    {
        return word(lua_runs && _state != State::Off ? 0U : LUA_HOLD);
    }
    // the M4 published that session; lua_runs as given to next_session_word(). Off stays off
    void session_started(bool lua_runs)
    {
        _held_for_restart = false;
        _restart_wanted = false;
        _park_seq_expected = 1;   // the new M4 session counts its parks from 1
        if (_state != State::Off) {
            _state = lua_runs ? State::Running : State::Parked;
        }
    }
    uint32_t control_word() const
    {
        return word(_state == State::Running && !_held_for_restart ? 0U : LUA_HOLD);
    }
    // the whole M4 restarts after its backoff and the old session may run until then: its Lua parks
    // at its next syscall and its report is ignored. The next session_started() clears it
    void hold_until_restart() { _held_for_restart = true; }
    // a stop (restart false) or restart request; false if Lua cannot take it now
    bool request(bool restart, uint32_t now_ms)
    {
        switch (_state) {
        case State::Running:
            _state = State::Stopping;
            _stopping_since_ms = now_ms;
            _restart_wanted = restart;
            return true;
        case State::Stopping:
            _restart_wanted = restart;   // the latest request wins
            return true;
        case State::Parked:
            if (!restart) {
                return false;
            }
            _grant++;
            _state = State::Running;
            return true;
        case State::Off:
            return false;
        }
        return false;
    }
    // a park report for grant and park_seq; charged when it is a fault or a stall, spent on used of limit.
    // Only the park the current grant expects is accepted: a report read again while the resumed
    // thread still resets carries the new grant and the old sequence
    Outcome parked(uint32_t grant, uint16_t park_seq, LuaParkCause cause, uint8_t &used, uint8_t limit, bool &charged)
    {
        charged = false;
        if (grant != (_grant & LUA_GRANT_MASK) || park_seq != _park_seq_expected || _held_for_restart ||
            _state == State::Off) {
            return Outcome::Ignored;
        }
        charged = cause != LuaParkCause::Requested;
        _grant++;   // every accepted report is answered
        _park_seq_expected++;
        if (spend_lua_restart(used, limit, charged)) {
            _state = State::Off;
            return Outcome::Off;
        }
        if (_state == State::Parked || (_state == State::Stopping && !_restart_wanted)) {
            _state = State::Parked;
            return Outcome::Stopped;
        }
        _state = State::Running;
        return Outcome::Restarted;
    }
    bool park_overdue(uint32_t now_ms) const
    {
        // held for the restart, the old session's park report is ignored and never awaited
        return _state == State::Stopping && !_held_for_restart && now_ms - _stopping_since_ms > PARK_TIMEOUT_MS;
    }
    // Lua calls come from a running Lua, or one asked to park that has not yet
    bool lua_live() const { return _state == State::Running || _state == State::Stopping; }
    State state() const { return _state; }
    uint32_t grant() const { return _grant & LUA_GRANT_MASK; }

private:
    uint32_t word(uint32_t hold) const { return (_grant & LUA_GRANT_MASK) | hold; }

    uint32_t _grant = 0;
    uint32_t _stopping_since_ms = 0;
    uint16_t _park_seq_expected = 1;
    State _state = State::Parked;
    bool _held_for_restart = false;
    bool _restart_wanted = false;
};

}  // namespace ScriptingM4
