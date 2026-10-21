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
#include <string.h>
#include "AP_RustM4_Protocol.h"

namespace RustM4 {

// a CallProbeTrailer::prev_flags bit beside PROBE_M4_SLEPT (bit 11 is free in AP_Rust::CallTrace
// flags): the module thread's poll took the reply, it never slept. The M7 keeps only the bits in
// its PROBE_M4_FLAGS mask
static const uint16_t PROBE_M4_POLLED = 1U << 11;

// the M4's stamps of one probed call; seq 0 until the service sent it
struct M4CallStamps {
    uint32_t seq;
    uint32_t sent_us;
    uint32_t reply_seen_us;
    uint32_t return_us;
    uint16_t flags;                   // PROBE_M4_*
    uint16_t stamps;                  // PROBE_STAMP_*: the sub-stage stamps taken
    uint32_t packed_us;
    uint32_t posted_us;
    uint32_t forward_returned_us;
    uint32_t reply_isr_us;
    uint32_t reply_wake_us;
    uint32_t signalled_us;
    uint32_t resumed_us;
    // core cycles per M4CyclePart, each valid with its bit in cycles_taken
    uint16_t cycles[M4_CYCLE_PARTS];
    uint16_t cycles_taken;
    uint32_t drain_copied_cycles;     // the counter as the reply's copy to the link ended; 0: not taken
};
static_assert(M4_CYCLE_PARTS <= 16, "cycles_taken");

struct M4CallProbe {
    // TIM5's count (the M7's micros()); null where the M4 cannot read it: no probe
    uint32_t (*clock_us)();
    // the core's cycle counter, for the stamp and memset costs; may be null
    uint32_t (*cycles)();
    uint32_t stamp_cycles;            // core cycles of one clock_us(), measured by the service at init
    uint32_t svc_cycles;              // cycles() as the SVC handler took the call; 0: not taken
    std::atomic<bool> enabled;        // this session's Config carried M4_OPTION_CALL_PROBE
    bool in_call;                     // the module thread is in a probed binding call
    bool memset_measured;
    uint16_t memset_small_centicycles;
    uint16_t memset_large_centicycles;
    M4CallStamps current;             // the call in flight
    M4CallStamps last;                // the last probed call that got its reply; last.seq 0: none
    uint32_t return_cycles;           // cycles() as host_binding_call() returned a probed call; 0: none
};

inline bool probing(const M4CallProbe *probe)
{
    return probe != nullptr && probe->clock_us != nullptr && probe->enabled.load();
}

inline uint32_t probe_cycles_now(const M4CallProbe &probe)
{
    return probe.cycles != nullptr ? probe.cycles() : 0;
}

inline uint16_t cycles_between(uint32_t from, uint32_t to)
{
    const uint32_t cycles = to - from;
    return cycles > 0xFFFFU ? 0xFFFFU : uint16_t(cycles);
}

// a part of the call in flight, from counter value from to to
inline void probe_cycle_part(M4CallStamps &stamps, M4CyclePart part, uint32_t from, uint32_t to)
{
    stamps.cycles[uint8_t(part)] = cycles_between(from, to);
    stamps.cycles_taken |= uint16_t(1U << uint8_t(part));
}

// a sub-stage stamp of the call in flight
inline void probe_stamp(M4CallProbe &probe, uint32_t &field, uint16_t bit)
{
    field = probe.clock_us();
    probe.current.stamps |= bit;
}

// hundredths of a cycle per byte
inline uint16_t centicycles_per_byte(uint32_t cycles, uint32_t bytes)
{
    const uint64_t value = uint64_t(cycles) * 100U / bytes;
    return value > 0xFFFFU ? 0xFFFFU : uint16_t(value);
}

/*
  The memset cost on this core, once per session at the first probed call, before its entry stamp:
  1000 times 64 B on the caller's stack, then 1000 times 4 KB of large_a and large_b (the link's
  request and reply buffers, half each, free while no call is posted)
 */
inline void measure_memset(M4CallProbe &probe, uint8_t *large_a, uint8_t *large_b, uint32_t half)
{
    probe.memset_measured = true;
    if (probe.cycles == nullptr) {
        return;
    }
    static const uint32_t ROUNDS = 1000;
    uint8_t small[64];
    uint32_t start = probe.cycles();
    for (uint32_t i = 0; i < ROUNDS; i++) {
        memset(small, int(i), sizeof(small));
        __asm__ volatile("" : : "r"(small) : "memory");   // the stores are kept
    }
    probe.memset_small_centicycles = centicycles_per_byte(probe.cycles() - start, ROUNDS * uint32_t(sizeof(small)));
    start = probe.cycles();
    for (uint32_t i = 0; i < ROUNDS; i++) {
        memset(large_a, int(i), half);
        memset(large_b, int(i), half);
    }
    probe.memset_large_centicycles = centicycles_per_byte(probe.cycles() - start, ROUNDS * 2U * half);
}

// the trailer of a call about to be framed; the in-flight stamps start over
inline CallProbeTrailer begin_probed_call(M4CallProbe &probe, uint32_t svc_entry_us)
{
    CallProbeTrailer trailer {};
    trailer.svc_entry_us = svc_entry_us;
    trailer.stamp_cycles = probe.stamp_cycles;
    const M4CallStamps &last = probe.last;
    trailer.prev_seq = last.seq;
    trailer.prev_sent_us = last.sent_us;
    trailer.prev_reply_seen_us = last.reply_seen_us;
    trailer.prev_return_us = last.return_us;
    trailer.prev_flags = last.flags;
    trailer.prev_stamps = last.stamps;
    trailer.prev_packed_us = last.packed_us;
    trailer.prev_posted_us = last.posted_us;
    trailer.prev_forward_returned_us = last.forward_returned_us;
    trailer.prev_reply_isr_us = last.reply_isr_us;
    trailer.prev_reply_wake_us = last.reply_wake_us;
    trailer.prev_signalled_us = last.signalled_us;
    trailer.prev_resumed_us = last.resumed_us;
    trailer.memset_small_centicycles = probe.memset_small_centicycles;
    trailer.memset_large_centicycles = probe.memset_large_centicycles;
    memcpy(trailer.prev_cycles, last.cycles, sizeof(trailer.prev_cycles));
    trailer.prev_cycles_taken = last.cycles_taken;
    probe.current = M4CallStamps {};
    probe.in_call = true;
    return trailer;
}

// the call returns to the module: replied calls become the next trailer's previous call; a call
// without its reply (timeout, lost session) completes nothing
inline void end_probed_call(M4CallProbe &probe, bool replied, uint32_t return_us)
{
    probe.in_call = false;
    if (!replied || probe.current.seq == 0 || probe.current.reply_seen_us == 0) {
        probe.last.seq = 0;
        return;
    }
    probe.last = probe.current;
    probe.last.return_us = return_us;
}

}  // namespace RustM4
