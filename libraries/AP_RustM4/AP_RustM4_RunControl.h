#pragma once

/*
  What the M7 does about requests to stop and restart scripting (MAV_CMD_SCRIPTING STOP and
  STOP_AND_RESTART, scripting:restart_all(), SCR_ENABLE), as AP_Scripting::thread() does
  in-process: the scripts stop when scripting should not run (SCR_ENABLE 0, or a stop asked);
  stopped, they start again on a restart request, or once SCR_ENABLE went to 0 and back to 1. On
  the M4 build both are a requested restart of the M4, into a session that lists no script
  (stopped) or one that runs the scripts. A requested restart is no fault: it spends none of
  RUST_M4_RST_MAX. C++11, no ArduPilot dependencies (host unit tests).
 */

#include <stdint.h>

namespace RustM4 {

// the stopped in-process thread looks at the flags once a second: a session runs at least this
// long before a restart ends it, so a script that restarts scripting at its start restarts at most
// once a second. A stop needs no wait: it acts at once, as the in-process run loop does
static const uint32_t RUN_CONTROL_MIN_SESSION_MS = 1000;

/*
  The stop and restart requests of AP_Scripting (MAV_CMD_SCRIPTING, the RC stop switch,
  restart_all()) as the M4 host thread takes them. Any thread asks; the host thread takes. A stop
  asked without a restart is kept until taken: a restart asked after it (a script's restart_all())
  does not cancel it, as in-process the stopped script no longer runs to ask
 */
class RunRequests {
public:
    // MAV_CMD_SCRIPTING STOP, the RC stop switch
    void stop() { __atomic_fetch_or(&_asked, STOP_ASKED, __ATOMIC_RELAXED); }
    // MAV_CMD_SCRIPTING STOP_AND_RESTART, restart_all()
    void restart() { __atomic_fetch_or(&_asked, RESTART_ASKED, __ATOMIC_RELAXED); }
    // a request since the last take; restart: whether it is a restart (no stop without one asked)
    bool take(bool &restart)
    {
        const uint8_t asked = __atomic_exchange_n(&_asked, uint8_t(0), __ATOMIC_RELAXED);
        restart = asked == RESTART_ASKED;
        return asked != 0;
    }

private:
    static const uint8_t STOP_ASKED = 1;
    static const uint8_t RESTART_ASKED = 2;
    uint8_t _asked = 0;   // STOP_ASKED, RESTART_ASKED
};

enum class RunAction : uint8_t {
    None,
    Stop,      // restart the M4 into a session that runs no script ("Scripting: stopped")
    Restart,   // restart the M4 into a session that runs the scripts ("Scripting: restarted")
};

class RunControl {
public:
    // one pass of the host thread while an M4 session runs. enabled: SCR_ENABLE; stop: a request
    // taken this pass (RunRequests), restart whether it asked for a restart (STOP_AND_RESTART,
    // restart_all()); running_ms: how long the session has been running
    RunAction update(bool enabled, bool stop, bool restart, uint32_t running_ms)
    {
        if (stop) {
            _stop = true;
            _restart = restart;   // a STOP clears a restart not yet acted on
        }
        if (!enabled) {
            _cleared = true;
        }
        if (!_stopped) {
            // SCR_ENABLE 0 or a stop without restart: the scripts stop at once, whatever the
            // session's age, so no restart asked later can cancel the stop
            if (!enabled || (_stop && !_restart)) {
                // they start again on a restart request or once SCR_ENABLE has been 0
                _stopped = true;
                _stop = false;
                _cleared = !enabled;
                return RunAction::Stop;
            }
            if (_stop && running_ms >= RUN_CONTROL_MIN_SESSION_MS) {
                return restarted();
            }
            return RunAction::None;
        }
        _stop = false;   // a STOP while stopped changes nothing
        if (running_ms >= RUN_CONTROL_MIN_SESSION_MS && enabled && (_cleared || _restart)) {
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

}  // namespace RustM4
