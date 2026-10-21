#pragma once

/*
  What the M7 does about requests to stop and restart the modules: they stop when they should not
  run (disabled, or a stop asked); stopped, they start again on a restart request, or once enabled
  went false and back to true. On the M4 both are a requested restart of the M4, into a session
  that loads no module (stopped) or one that loads them. A change of RUST_RESTART is a restart
  request (AP_RustM4::update()). A requested restart is no fault: it spends none of
  RUST_M4_RST_MAX. C++11, no ArduPilot dependencies (host unit tests).
 */

#include <stdint.h>

namespace RustM4 {

// a session runs at least this long before a restart ends it, so a module that asks for a restart
// at its start restarts at most once a second. A stop needs no wait: it acts at once
static const uint32_t RUN_CONTROL_MIN_SESSION_MS = 1000;

/*
  The stop and restart requests as the M4 host thread takes them. Any thread asks; the host thread
  takes. A stop asked without a restart is kept until taken: a restart asked after it does not
  cancel it
 */
class RunRequests {
public:
    // stop the modules
    void stop() { __atomic_fetch_or(&_asked, STOP_ASKED, __ATOMIC_RELAXED); }
    // restart the modules (a RUST_RESTART change)
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
    Stop,      // restart the M4 into a session that loads no module
    Restart,   // restart the M4 into a session that loads the modules
};

// the wait before the restart that follows the n-th fault restart (restarts_used, from 1): 1 s,
// doubling, at most 30 s. A requested restart (0) waits for nothing
inline uint32_t restart_backoff_ms(uint8_t restarts_used)
{
    if (restarts_used == 0) {
        return 0;
    }
    const uint32_t backoff = restarts_used >= 6 ? 30000U : 1000U << (restarts_used - 1U);
    return backoff < 30000U ? backoff : 30000U;
}

// a running module that does not report its stop within this long is reclaimed by a forced restart
static const uint32_t MODULE_STOP_TIMEOUT_MS = 10000;

class RunControl {
public:
    // one pass of the host thread while an M4 session runs. enabled: whether modules should run;
    // stop: a request taken this pass (RunRequests), restart whether it asked for a restart;
    // running_ms: how long the session has been running
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
            // disabled or a stop without restart: the modules stop at once, whatever the
            // session's age, so no restart asked later can cancel the stop
            if (!enabled || (_stop && !_restart)) {
                // they start again on a restart request or once disabled and enabled again
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

    bool _stopped = false;   // the session loads no module
    bool _stop = false;      // a stop asked and not yet acted on
    bool _restart = false;   // ... with a restart
    bool _cleared = false;   // disabled since the modules stopped
};

}  // namespace RustM4
