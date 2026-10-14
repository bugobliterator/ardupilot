#pragma once

/*
  How the M7 reports a fault it finds at the start of an M4 session. After a fault restart, the
  new session's header can show a fault count above the one already handled: the old session's
  fault recorded on its way down (the same incident, absorbed), or the new session's own fault
  after its publish() and before the M7 read its header. The heartbeat tells them apart: a session
  that beats did not fault at its start. One that never beats is reported as that fault, with its
  PC, instead of "heartbeat lost". Either way the incident is charged once. C++11, no ArduPilot
  dependencies (host unit tests).
 */

#include <stdint.h>

namespace ScriptingM4 {

class StartFault {
public:
    // a session has started; fault_restart: it follows a fault restart (not the first start, not a
    // requested restart); fault_count: its header's; handled_count: the count already handled
    void session_started(bool fault_restart, uint32_t fault_count, uint32_t handled_count)
    {
        _pending = fault_restart && fault_count != handled_count;
    }

    // the session's heartbeat moved: the fault seen at its start was the old session's
    void beat() { _pending = false; }

    // the heartbeat timed out; header_readable: the session's header reads as compatible. True:
    // report the recorded fault instead of "heartbeat lost"
    bool report_on_timeout(bool header_readable)
    {
        const bool report = _pending && header_readable;
        _pending = false;
        return report;
    }

private:
    bool _pending = false;
};

}  // namespace ScriptingM4
