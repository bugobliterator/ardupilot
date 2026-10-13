#pragma once

/*
  What the M7 does about requests to stop and restart scripting (MAV_CMD_SCRIPTING STOP and
  STOP_AND_RESTART, scripting:restart_all(), SCR_ENABLE), as AP_Scripting::thread() does
  in-process: the scripts stop when scripting should not run (SCR_ENABLE 0, or a stop asked);
  stopped, they start again on a restart request, or once SCR_ENABLE went to 0 and back to 1. On
  the M4 build both are a requested restart of the M4, into a session that lists no script
  (stopped) or one that runs the scripts. A requested restart is no fault: it spends none of
  SCR_M4_RST_MAX. Host thread only. C++11, no ArduPilot dependencies (host unit tests).
 */

#include <stdint.h>

namespace ScriptingM4 {

// the stopped in-process thread looks at the flags once a second: a session runs at least this
// long before a request ends it, so a script that restarts scripting at its start restarts at most
// once a second
static const uint32_t RUN_CONTROL_MIN_SESSION_MS = 1000;

enum class RunAction : uint8_t {
    None,
    Stop,      // restart the M4 into a session that runs no script ("Scripting: stopped")
    Restart,   // restart the M4 into a session that runs the scripts ("Scripting: restarted")
};

class RunControl {
public:
    // one pass of the host thread while an M4 session runs. enabled: SCR_ENABLE; stop: a request
    // taken from AP_Scripting this pass, restart whether it asked for a restart too (STOP_AND_RESTART,
    // restart_all()); running_ms: how long the session has been running
    RunAction update(bool enabled, bool stop, bool restart, uint32_t running_ms)
    {
        if (stop) {
            _stop = true;
            _restart = restart;   // AP_Scripting's flags: STOP clears a restart not yet acted on
        }
        if (!enabled) {
            _cleared = true;
        }
        if (running_ms < RUN_CONTROL_MIN_SESSION_MS) {
            return RunAction::None;
        }
        if (!_stopped) {
            if (enabled && !_stop) {
                return RunAction::None;
            }
            if (enabled && _restart) {
                return restarted();
            }
            // the scripts stop; they start again on a restart request or once SCR_ENABLE has been 0
            _stopped = true;
            _stop = false;
            _cleared = !enabled;
            return RunAction::Stop;
        }
        _stop = false;   // a STOP while stopped changes nothing
        if (enabled && (_cleared || _restart)) {
            return restarted();
        }
        return RunAction::None;
    }

    bool stopped() const { return _stopped; }

private:
    RunAction restarted()
    {
        _stopped = false;
        _stop = false;
        _restart = false;
        _cleared = false;
        return RunAction::Restart;
    }

    bool _stopped = false;   // the session runs no script
    bool _stop = false;      // a stop asked and not yet acted on
    bool _restart = false;   // ... with a restart
    bool _cleared = false;   // SCR_ENABLE has been 0 since the scripts stopped
};

}  // namespace ScriptingM4
