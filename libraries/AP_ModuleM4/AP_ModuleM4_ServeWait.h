#pragma once

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include "AP_ModuleM4_ModuleTable.h"
#include "AP_ModuleM4_Protocol.h"
#include "AP_ModuleM4_StatsRows.h"

namespace ModuleM4 {

// the M4 runtime's own frames, which it queues ahead of binding calls: its Stats rows (a log row
// each, a burst once a second) and its system notifies (module texts and states, Noop). A call
// queued behind them would otherwise wait for a later pass, after the host's backoff
inline bool frame_exempt_from_pass_time(const uint8_t *buf, uint16_t len)
{
    MsgHeader header;
    if (buf == nullptr || !parse_header(buf, len, header)) {
        return false;
    }
    if (MsgType(header.type) == MsgType::Stats || MsgType(header.type) == MsgType::Counters) {
        return true;
    }
    if (MsgType(header.type) != MsgType::SystemCall) {
        return false;
    }
    SystemCallHeader call;
    memcpy(&call, buf, sizeof(call));   // parse_header() checked len >= sizeof(SystemCallHeader)
    return (call.flags & CALL_FLAG_NOTIFY) != 0;
}

// exempt frames per pass: the M4's legitimate worst case ahead of a call is one window's Stats rows
// and one notify per module. More of them, e.g. forged by a faulty M4, end the pass as any other
// frame
static const uint8_t MAX_EXEMPT_PER_PASS = STATS_ROWS_PER_WINDOW + MAX_MODULES;
static_assert(MAX_EXEMPT_PER_PASS == 27, "19 Stats and Counters frames and 8 module notifies");

// a pass of the host thread: at most MAX_REQUESTS_PER_WAKE frames, ending once MAX_SERVE_US have
// passed since the wake after a frame that may end it
static const uint8_t MAX_REQUESTS_PER_WAKE = 32;
static const uint32_t MAX_SERVE_US = 200;

// whether the pass may end on time after this frame (a frame past MAX_SERVE_US that is not exempt,
// or exempt beyond MAX_EXEMPT_PER_PASS in this pass). exempt_in_pass starts at 0 for each pass;
// the pass still ends at MAX_REQUESTS_PER_WAKE frames
inline bool frame_ends_pass_on_time(const uint8_t *buf, uint16_t len, uint8_t &exempt_in_pass)
{
    if (exempt_in_pass < MAX_EXEMPT_PER_PASS && frame_exempt_from_pass_time(buf, len)) {
        exempt_in_pass++;
        return false;
    }
    return true;
}

enum class PassEnd : uint8_t {
    Idle,           // the ring is empty
    Busy,           // a limit ran out with a frame just served; more may remain
    ReceiveError,   // the ring returned an error other than empty: the frame stays queued
};

// the host's serve loop: receive(len) -> 0, -EAGAIN or another -errno, the frame received into
// frame; serve(len) handles it; elapsed_us() is the time since the wake
template <typename Receive, typename Serve, typename Elapsed>
inline PassEnd serve_pass(const uint8_t *frame, Receive receive, Serve serve, Elapsed elapsed_us)
{
    // an empty ring is Idle whatever the time; Busy means a limit ran out with a frame just served
    uint8_t exempt_in_pass = 0;
    for (uint8_t i = 0; i < MAX_REQUESTS_PER_WAKE; i++) {
        uint16_t len = 0;
        const int ret = receive(len);
        if (ret == -EAGAIN) {
            return PassEnd::Idle;
        }
        if (ret != 0) {
            return PassEnd::ReceiveError;
        }
        serve(len);
        if (frame_ends_pass_on_time(frame, len, exempt_in_pass) && elapsed_us() > MAX_SERVE_US) {
            return PassEnd::Busy;
        }
    }
    return PassEnd::Busy;
}

/*
  The slow worker after it served a posted frame: it keeps the ring and serves the posted frames
  that follow, POSTED_BATCH_MAX in all or until MAX_SERVE_US, so a burst of log records costs one
  handoff. The host waits for the worker throughout: one consumer at a time. Returns the length of
  the first other frame, left in frame for the host, or 0 (ring empty, a receive error the host's
  next receive reports, or a limit reached).
 */
static const uint8_t POSTED_BATCH_MAX = 8;

template <typename Receive, typename IsPosted, typename Serve, typename Elapsed>
inline uint16_t serve_posted_batch(const uint8_t *frame, Receive receive, IsPosted is_posted, Serve serve,
                                   Elapsed elapsed_us)
{
    for (uint8_t served = 1; served < POSTED_BATCH_MAX && elapsed_us() <= MAX_SERVE_US; served++) {
        uint16_t len = 0;
        if (receive(len) != 0) {
            return 0;
        }
        if (!is_posted(frame, len)) {
            return len;
        }
        serve(len);
    }
    return 0;
}

/*
  Which thread serves a frame. The host's fast lane, above the I/O threads, takes only what does
  fixed work with no file, storage, SD or bus access: binding calls whose generated M4 lane is Fast
  (binding_fast), the Noop notify, the Ping, the Stats rows and Counters, and the M7 -> M4 types, which only count a
  protocol error. Everything else, the other system methods and frames that do not parse
  included, goes to the slow worker below the I/O threads.
 */
inline bool frame_on_fast_lane(const uint8_t *buf, uint16_t len, bool (*binding_fast)(uint32_t id))
{
    MsgHeader header;
    if (!parse_header(buf, len, header)) {
        return false;
    }
    switch (MsgType(header.type)) {
    case MsgType::Stats:
    case MsgType::Counters:
    case MsgType::FaultInject:
    case MsgType::Ready:
    case MsgType::BindingReply:
    case MsgType::SystemReply:
    case MsgType::Control:
    case MsgType::ParameterChanged:
        return true;
    case MsgType::BindingCall: {
        BindingCallHeader call;
        memcpy(&call, buf, sizeof(call));   // parse_header() checked len >= sizeof(BindingCallHeader)
        return binding_fast(call.id);
    }
    case MsgType::SystemCall: {
        SystemCallHeader call;
        memcpy(&call, buf, sizeof(call));
        return call.method == uint16_t(SystemMethod::Noop) || call.method == uint16_t(SystemMethod::Ping);
    }
    }
    return false;
}

/*
  The M7 thread-statistics clock (ChibiOS CH_DBG_STATISTICS) as the host thread reads it about
  itself: the low word of the cumulative cycles of its finished slices, the cycle its running slice
  started on, and best. ChibiOS only ever lowers best to a slice's length, so the host keeps it at
  0; a reset (chTMObjectInit() from another thread, or Util::thread_info() on this one) sets it to
  all ones.
 */
struct ThreadStatsClock {
    uint32_t cumulative;
    uint32_t last;
    uint32_t best;
};

// one reading, in cycles modulo 2^32; reset tells whether the clock was reset since the previous
// reading. It clears best again: the caller writes the clock back
inline uint32_t read_thread_cycles(ThreadStatsClock &clock, uint32_t now, bool &reset)
{
    reset = clock.best != 0;
    clock.best = 0;
    return clock.cumulative + (now - clock.last);
}

/*
  The host's work besides serving frames (heartbeat, supervision, texts, run control, statistics)
  runs after a wait that timed out, and after a pass a doorbell started only once period has
  passed since it last ran, so a stream of calls is served by frame work alone. now and last are
  on one wrapping clock; a timed-out wait bounds the gap between runs
 */
static const uint32_t HOUSEKEEPING_PERIOD_US = 1000;
inline bool housekeeping_due(uint32_t now, uint32_t last, uint32_t period, bool woke_on_timeout)
{
    return woke_on_timeout || now - last >= period;
}

enum class HostWait { Doorbell, Backoff, Timeout };

// how the host waits before its next serve pass: back off only while frames remain (a flooding
// M4); an empty ring waits on the doorbell even when the pass ran out of budget on wall time
// (preempted)
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

}  // namespace ModuleM4
