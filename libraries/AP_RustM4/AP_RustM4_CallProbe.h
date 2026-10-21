#pragma once

/*
  The M4's side of the call latency probe (RUST_M4_OPTS bit 2): the stamps of
  the binding call in flight and of the last one completed, which the next
  probed call carries to the M7 in its CallProbeTrailer. Written by the module
  thread (in its host call) and by the service while the module thread waits
  for the reply; the ModuleLink state hand-over orders the two. Privileged
  memory on the M4 image. C++11, no ArduPilot dependencies.
 */

#include <atomic>
#include <stdint.h>
#include "AP_RustM4_Protocol.h"

namespace RustM4 {

struct M4CallProbe {
    // TIM5's count (the M7's micros()); null where the M4 cannot read it: no probe
    uint32_t (*clock_us)();
    uint32_t stamp_cycles;            // core cycles of one clock_us(), measured by the service at init
    std::atomic<bool> enabled;        // this session's Config carried M4_OPTION_CALL_PROBE
    // the call in flight: seq and stamps set by the service, 0 until it is sent
    uint32_t seq;
    uint32_t sent_us;
    uint32_t reply_seen_us;
    uint16_t flags;                   // PROBE_M4_*
    // the last probed call that got its reply, for the next trailer; last_seq 0: none
    uint32_t last_seq;
    uint32_t last_sent_us;
    uint32_t last_reply_seen_us;
    uint32_t last_return_us;
    uint16_t last_flags;
};

inline bool probing(const M4CallProbe *probe)
{
    return probe != nullptr && probe->clock_us != nullptr && probe->enabled.load();
}

// the trailer of a call about to be framed; the in-flight stamps start over
inline CallProbeTrailer begin_probed_call(M4CallProbe &probe, uint32_t svc_entry_us)
{
    CallProbeTrailer trailer {};
    trailer.svc_entry_us = svc_entry_us;
    trailer.stamp_cycles = probe.stamp_cycles;
    trailer.prev_seq = probe.last_seq;
    trailer.prev_sent_us = probe.last_sent_us;
    trailer.prev_reply_seen_us = probe.last_reply_seen_us;
    trailer.prev_return_us = probe.last_return_us;
    trailer.prev_flags = probe.last_flags;
    probe.seq = 0;
    probe.sent_us = 0;
    probe.reply_seen_us = 0;
    probe.flags = 0;
    return trailer;
}

// the call returns to the module: replied calls become the next trailer's previous call; a call
// without its reply (timeout, lost session) completes nothing
inline void end_probed_call(M4CallProbe &probe, bool replied, uint32_t return_us)
{
    if (!replied || probe.seq == 0 || probe.reply_seen_us == 0) {
        probe.last_seq = 0;
        return;
    }
    probe.last_seq = probe.seq;
    probe.last_sent_us = probe.sent_us;
    probe.last_reply_seen_us = probe.reply_seen_us;
    probe.last_return_us = return_us;
    probe.last_flags = probe.flags;
}

}  // namespace RustM4
