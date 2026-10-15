#pragma once

namespace ScriptingM4 {

enum class HostWait { Doorbell, Backoff, Timeout };

// after a serve pass: back off only while frames remain (a flooding M4); an empty ring waits on
// the doorbell even when the pass ran out of budget on wall time (preempted)
inline HostWait host_wait_after(bool budget_ran_out, bool stuck, bool ring_empty)
{
    if (stuck) {
        return HostWait::Timeout;
    }
    if (budget_ran_out && !ring_empty) {
        return HostWait::Backoff;
    }
    return HostWait::Doorbell;
}

}  // namespace ScriptingM4
