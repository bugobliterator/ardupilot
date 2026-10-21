#pragma once

/*
  The module thread's first wait for its call's reply on the M4 image: it
  polls the RX ring instead of sleeping. The thread is the module's own
  (below the service), so the spin delays no other module code. While it
  polls the M4 reads as awake, so the M7's send() rings no doorbell: the
  reply costs no HSEM ring on the M7 and no interrupt and thread switch here.
 */

#include <atomic>
#include <stdint.h>
#include "m4_service.h"

namespace ModuleM4 {

// how long the module thread polls before it sleeps for the rest of its wait: far above the M7's
// preemption band; the call timeout (CALL_TIMEOUT_MS) stays with the service
static const uint32_t MODULE_POLL_US = 2000;

enum class PollEnd : uint8_t {
    Done,      // the call is Done
    Stopped,   // the module was asked to stop: the caller sleeps for the rest of the wait
    Expired,   // no reply within the bound: the caller sleeps for the rest of the wait
};

// drains any frame the M7 sends while the call is in flight, as a doorbell wake would. A fault
// injection drained here sets wake_service (the service injects it)
inline PollEnd poll_for_reply(M4Service &service, const ModuleLink &link, const std::atomic<bool> &stop,
                              uint32_t (*cycles)(), uint32_t bound_cycles, bool &wake_service)
{
    service.module_awake();
    const uint32_t start = cycles();
    while (true) {
        if (link.state.load() == ModuleLink::Done) {
            return PollEnd::Done;
        }
        if (service.frame_waiting() && service.drain_step_for_module(false, false, 0, 0)) {
            wake_service = true;
        }
        if (link.state.load() == ModuleLink::Done) {
            return PollEnd::Done;
        }
        if (stop.load()) {
            return PollEnd::Stopped;
        }
        if (cycles() - start >= bound_cycles) {
            return PollEnd::Expired;
        }
    }
}

}  // namespace ModuleM4
