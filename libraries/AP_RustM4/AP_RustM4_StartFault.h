#pragma once

/*
  How the M7 reports a fault it finds at the start of an M4 session. After a fault restart, the
  new session's header can show a fault count above the one already handled: the old session's
  fault recorded on its way down (the same incident, absorbed), or the new session's own fault
  after its publish() and before the M7 read its header. The heartbeat tells them apart: a session
  whose heartbeat moves after that read did not fault at its start. One whose heartbeat never moves
  is reported as that fault, with its PC, instead of "heartbeat lost". Either way the incident is
  charged once. A stop or restart request waits while the fault is pending (at most the heartbeat
  timeout): taken before the heartbeat tells, it could report a fault already charged.
  The heartbeat is cumulative across sessions: a session that beat before the M7's read and then
  stalled also never moves it from the value read, so after a fault on the way down such a stall
  is reported as that old fault (old PC) rather than "heartbeat lost", still charged once.
  C++11, no ArduPilot dependencies (host unit tests).
 */

#include <stdint.h>

namespace RustM4 {

class StartFault {
public:
    // a session has started; fault_restart: it follows a fault restart (not the first start, not a
    // requested restart); fault_count: its header's; handled_count: the count already handled. A
    // count below the handled one (a fault record cleared at publish()) is no fault
    void session_started(bool fault_restart, uint32_t fault_count, uint32_t handled_count)
    {
        _pending = fault_restart && fault_count > handled_count;
        _held = _pending;
    }

    // the session's heartbeat moved: the fault seen at its start was the old session's
    void beat()
    {
        _pending = false;
        _held = false;
    }

    // the heartbeat timed out; header_readable: the session's header reads as compatible. True:
    // report the recorded fault instead of "heartbeat lost"
    bool report_on_timeout(bool header_readable)
    {
        const bool report = _pending && header_readable;
        _pending = false;
        return report;
    }

    // stop and restart requests (RunControl) wait until the heartbeat or its timeout resolves the
    // fault
    bool holds_requests() const { return _pending; }

    // the session ends in a fault restart (any cause). True: requests were held and never taken,
    // so that fault restart carries a stop asked meanwhile
    bool take_held()
    {
        const bool held = _held;
        _held = false;
        return held;
    }

private:
    bool _pending = false;
    bool _held = false;   // requests held this session and not yet released by a beat
};

}  // namespace RustM4
