#pragma once

/*
  Networking-on-M4 builds: whether the next restart grant may clear the network region
  (AP_Networking::m4_region_reset()). Only an M4 that cannot run may have the region cleared
  under it: one in its fault handler waiting for the grant, or a SITL M4 process that exited.
  A fault counts as the running session's own only when the header's fault count has moved
  above the count that session started with. A count carried from the previous session (a
  fault on its way down around a requested restart, the new session already running) does not.
  C++11, no ArduPilot dependencies (host unit tests).
 */

#include <stdint.h>

namespace ScriptingM4 {

class NetRegionClear {
public:
    // a session started; fault_count: its header's
    void session_started(uint32_t fault_count)
    {
        _session_fault_count = fault_count;
        _halted = false;
    }

    // the supervisor charges a fault it read in the header; fault_count: that header's
    void fault_seen(uint32_t fault_count)
    {
        _halted = fault_count > _session_fault_count;
    }

    // the M4 process is gone (SITL)
    void process_exited() { _halted = true; }

    // at a restart grant. True: clear the region now
    bool take_clear()
    {
        const bool clear = _halted;
        _halted = false;
        return clear;
    }

private:
    uint32_t _session_fault_count = 0;
    bool _halted = false;
};

}  // namespace ScriptingM4
