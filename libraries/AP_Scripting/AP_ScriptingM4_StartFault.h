#pragma once

/*
  How the M7 reports a fault it finds at the start of an M4 session. After a fault restart, the
  new session's header can show a fault count above the one already handled: the old session's
  fault recorded on its way down (the same incident, absorbed), or the new session's own fault
  after its publish() and before the M7 read its header. The heartbeat tells them apart: a session
  whose heartbeat moves after that read did not fault at its start. One whose heartbeat never moves
  is reported as that fault, with its PC, instead of "heartbeat lost", and so is one a stop or
  restart request interrupts first. Either way the incident is charged once.
  The heartbeat is cumulative across sessions: a session that beat before the M7's read and then
  stalled also never moves it from the value read, so after a fault on the way down such a stall
  is reported as that old fault (old PC) rather than "heartbeat lost", still charged once.
  C++11, no ArduPilot dependencies (host unit tests).
 */

#include <stdint.h>

namespace ScriptingM4 {

class StartFault {
public:
    // a session has started; fault_restart: it follows a fault restart (not the first start, not a
    // requested restart); fault_count: its header's; handled_count: the count already handled. A
    // count below the handled one (a fault record cleared at publish()) is no fault
    void session_started(bool fault_restart, uint32_t fault_count, uint32_t handled_count)
    {
        _pending = fault_restart && fault_count > handled_count;
    }

    // the session's heartbeat moved: the fault seen at its start was the old session's
    void beat() { _pending = false; }

    // the heartbeat timed out; header_readable: the session's header reads as compatible. True:
    // report the recorded fault instead of "heartbeat lost"
    bool report_on_timeout(bool header_readable) { return take(header_readable); }

    // a stop or restart request (RunControl) interrupts the session before either of the above.
    // True: report and charge the recorded fault first; its fault restart then carries the request
    bool report_before_request(bool header_readable) { return take(header_readable); }

private:
    bool take(bool header_readable)
    {
        const bool report = _pending && header_readable;
        _pending = false;
        return report;
    }

    bool _pending = false;
};

}  // namespace ScriptingM4
