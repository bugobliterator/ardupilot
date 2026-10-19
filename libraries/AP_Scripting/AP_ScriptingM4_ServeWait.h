#pragma once

#include <errno.h>
#include <stdint.h>
#include "AP_ScriptingM4_Files.h"
#include "AP_ScriptingM4_Objects.h"
#include "AP_ScriptingM4_Rpc.h"
#include "AP_ScriptingM4_StatsRows.h"

namespace ScriptingM4 {

/*
  Token bucket on the host thread's CPU time. Credit accrues at SHARE_PERCENT
  of elapsed wall time up to BURST_US; each pass is charged in full for the
  CPU time it used, so credit goes negative by that much. While credit is
  below RESUME_US the host serves nothing, however many frames are pending.
  Time a binding spends blocked (a semaphore, the SD card) yields the CPU to
  other threads and is not charged; a binding that spins is.
  One long worker frame (fs:crc32 of a large file) can leave a debt that
  would pause serving past the M4's call timeout. The debt stays, but a pass
  is allowed at the latest MAX_PAUSE_US after the previous one started; while
  the debt is beyond MAX_DEBT_US the host refuses binding calls for the slow
  worker in it (refuses_slow_bindings()): cheap calls are answered, heavy ones
  fail until the debt is paid down. Passes this allows early cost at most
  MAX_SERVE_US plus one frame each, plus up to MAX_EXEMPT_PER_PASS frames that
  do not end a pass (frame_ends_pass_on_time()), and no debt is forgiven, so the
  long-run share holds.
 */
struct ServeDutyCapTestAccess;

class ServeDutyCap {
public:
    static constexpr uint32_t SHARE_PERCENT = 10;
    // 10 ms of CPU: a script's burst of a few hundred synchronous calls once a second runs
    // unpaced; the share holds over a 100 ms horizon and in the long run
    static constexpr int32_t BURST_US = 10000;
    // a pass is at most MAX_SERVE_US plus one frame (and up to MAX_EXEMPT_PER_PASS small frames
    // that do not end it): start one only with that much credit
    static constexpr int32_t RESUME_US = 200;
    // the deepest debt that is paid down by pausing alone
    static constexpr int32_t MAX_DEBT_US = 50000;

    void charge(uint32_t busy_us)
    {
        _credit_us -= busy_us;
    }

    // the longest the cap pauses serving, however deep the debt
    static constexpr uint32_t MAX_PAUSE_US = uint32_t(RESUME_US + MAX_DEBT_US) * (100U / SHARE_PERCENT);

    // 0 when a pass may start now, else the microseconds until it may
    uint32_t wait_us(uint32_t now_us)
    {
        refill(now_us);
        if (_credit_us >= RESUME_US) {
            return 0;
        }
        // a pass at the latest MAX_PAUSE_US after the previous one started, however deep the debt;
        // the debt stays, and while it is beyond MAX_DEBT_US such a pass refuses heavy calls
        const uint32_t since_us = now_us - _pass_us;
        const uint32_t pass_due_us = since_us >= MAX_PAUSE_US ? 0 : MAX_PAUSE_US - since_us;
        if (refuses_slow_bindings()) {
            return pass_due_us;
        }
        const uint32_t credit_due_us = uint32_t((RESUME_US - _credit_us) * int64_t(TIME_PER_CREDIT));   // <= MAX_PAUSE_US
        return credit_due_us < pass_due_us ? credit_due_us : pass_due_us;
    }

    // the debt is too deep to run binding calls on the slow worker: they are refused unrun
    bool refuses_slow_bindings() const
    {
        return _credit_us < -MAX_DEBT_US;
    }

    int64_t credit_us() const { return _credit_us; }

private:
    // a pass starts only through pass_after_wake(), so the host cannot charge its wake before
    // deciding (the unit tests' simulators start passes through ServeDutyCapTestAccess)
    template <typename ChargeWake>
    friend bool pass_after_wake(ServeDutyCap &cap, uint32_t woke_us, ChargeWake charge_wake);
    friend struct ServeDutyCapTestAccess;

    // the host starts a pass, wait_us() having returned 0
    void pass_started(uint32_t now_us)
    {
        _pass_us = now_us;
    }

    static_assert(100U % SHARE_PERCENT == 0, "whole microseconds of time per microsecond of credit");
    static constexpr uint32_t TIME_PER_CREDIT = 100U / SHARE_PERCENT;

    // elapsed time is taken modulo 2^32 us; the host refills at least every platform_wait_us()
    // while it runs, and a gap of 71 min or more only earns less credit than it could
    void refill(uint32_t now_us)
    {
        if (!_started) {
            _started = true;   // the first call starts the clock: no credit for time before it
            _refilled_us = now_us;
            _pass_us = now_us;
            return;
        }
        const uint32_t earned_us = (now_us - _refilled_us) / TIME_PER_CREDIT;
        _refilled_us += earned_us * TIME_PER_CREDIT;   // the remainder counts next time
        _credit_us += earned_us;
        if (_credit_us >= BURST_US) {
            _credit_us = BURST_US;
            _refilled_us = now_us;   // full: only time from here on earns
        }
    }

    int64_t _credit_us = BURST_US;
    uint32_t _refilled_us = 0;
    uint32_t _pass_us = 0;
    bool _started = false;
};

// the M4 runtime's own small frames, which it queues ahead of binding calls: its Stats rows (a log
// row each, a burst once a second) and its system Notifies m4:noop, m4:run_stats and m4:release.
// A call queued behind them would otherwise wait for later passes, 0.5 s each while the duty-cap
// debt is deep, and outlast the M4's call timeout
inline bool frame_exempt_from_pass_time(const uint8_t *buf, uint16_t len)
{
    MsgHeader header;
    if (!parse_header(buf, len, header)) {
        return false;
    }
    if (MsgType(header.type) == MsgType::Stats) {
        return true;
    }
    if (MsgType(header.type) != MsgType::Notify) {
        return false;
    }
    CallHeader call;
    memcpy(&call, buf, sizeof(call));   // parse_header() checked len >= sizeof(CallHeader)
    return call.method == uint16_t(SystemMethod::Noop) || call.method == uint16_t(SystemMethod::RunStats) ||
           call.method == uint16_t(SystemMethod::Release);
}

// exempt frames per pass: the M4's legitimate worst case ahead of a call is one window's Stats rows,
// one m4:run_stats and one flush of its release queue (ObjectPool::SLOTS handles, MAX_VALUES per
// m4:release). More of them, e.g. forged by a faulty M4, end the pass as any other frame; all are
// charged
static const uint8_t MAX_RELEASES_PER_FLUSH = (ObjectPool::SLOTS + MAX_VALUES - 1) / MAX_VALUES;
static const uint8_t MAX_EXEMPT_PER_PASS = STATS_ROWS_PER_WINDOW + 1 + MAX_RELEASES_PER_FLUSH;
static_assert(MAX_EXEMPT_PER_PASS == 15, "10 Stats rows, m4:run_stats and 4 m4:release");

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

// the host's serve loop: receive(frame, len) -> 0, -EAGAIN or another -errno; serve(len) handles
// the frame just received into frame; elapsed_us() is the time since the wake
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

// what the user is told about duty-cap refusals, from the count refused in each second: one
// "refused" text at most every NOTICE_MS while they continue, then one "served again" once a
// whole NOTICE_MS has passed without one
class RefusalNotice {
public:
    enum class Text { None, Refused, ServedAgain };
    static constexpr uint32_t NOTICE_MS = 5000;

    Text update(uint32_t now_ms, uint32_t refused_in_last_second)
    {
        if (refused_in_last_second != 0) {
            _last_refusal_ms = now_ms;
            if (!_told || now_ms - _told_ms >= NOTICE_MS) {
                _told = true;
                _told_ms = now_ms;
                return Text::Refused;
            }
            return Text::None;
        }
        if (_told && now_ms - _last_refusal_ms >= NOTICE_MS) {
            _told = false;
            return Text::ServedAgain;
        }
        return Text::None;
    }

private:
    bool _told = false;
    uint32_t _told_ms = 0;
    uint32_t _last_refusal_ms = 0;
};

// the host after a wait: whether its pass may serve is decided before the wait's own wake and
// switch are charged (charge_wake), so the pass a duty-cap pause waited for runs; that charge
// lands on the next wait
template <typename ChargeWake>
inline bool pass_after_wake(ServeDutyCap &cap, uint32_t woke_us, ChargeWake charge_wake)
{
    const bool may_serve = cap.wait_us(woke_us) == 0;
    charge_wake();
    if (may_serve) {
        cap.pass_started(woke_us);
    }
    return may_serve;
}

// CPU time to charge for the interval between two readings of the thread's CPU clock. When
// the clock was reset in between (statistics cleared), or went backwards, the interval's wall
// time is charged: the thread cannot have used more, so a reset at worst over-charges
inline uint32_t interval_cpu_us(uint64_t start_us, uint64_t end_us, bool reset, uint32_t wall_us)
{
    if (reset || end_us < start_us) {
        return wall_us;
    }
    const uint64_t used = end_us - start_us;
    return used > UINT32_MAX ? UINT32_MAX : uint32_t(used);
}

/*
  The M7 thread-statistics clock (ChibiOS CH_DBG_STATISTICS) as the host thread reads it about
  itself: cumulative cycles of its finished slices, the cycle its running slice started on, and
  best. ChibiOS only ever lowers best to a slice's length, so the host keeps it at 0; a reset
  (chTMObjectInit() from another thread, or Util::thread_info() on this one) sets it to all ones.
 */
struct ThreadStatsClock {
    uint64_t cumulative;
    uint32_t last;
    uint32_t best;
};

// one reading, in cycles; reset tells whether the clock was reset since the previous reading.
// It clears best again: the caller writes the clock back
inline uint64_t read_thread_cycles(ThreadStatsClock &clock, uint32_t now, bool &reset)
{
    reset = clock.best != 0;
    clock.best = 0;
    return clock.cumulative + uint32_t(now - clock.last);
}

/*
  What the duty cap charges for the host's CPU, from readings of its CPU clock at the start and
  the end of each pass. charges_waits (ChibiOS, where waits sleep): all CPU since the previous
  reading, so the wake, the switch and the bookkeeping around a pass are charged too. Otherwise
  (SITL, whose waits spin on the simulated clock): each pass only. An interval with a reset is
  charged its wall time; the first reading only starts the clock.
 */
class HostCpuCharge {
public:
    explicit HostCpuCharge(bool charges_waits) : _charges_waits(charges_waits) {}

    uint32_t pass_start(uint64_t cpu_us, bool reset, uint32_t wall_us)
    {
        if (!_started) {
            _started = true;
            latch(cpu_us, wall_us);
            return 0;
        }
        uint32_t used_us = 0;
        if (_charges_waits) {
            used_us = interval(cpu_us, reset, wall_us);
        } else if (reset || cpu_us < _cpu_us) {
            _resets++;
        }
        latch(cpu_us, wall_us);
        return used_us;
    }

    uint32_t pass_end(uint64_t cpu_us, bool reset, uint32_t wall_us)
    {
        const uint32_t used_us = _started ? interval(cpu_us, reset, wall_us) : 0;
        _started = true;
        latch(cpu_us, wall_us);
        return used_us;
    }

    // resets seen since the last call
    uint16_t take_resets()
    {
        const uint16_t resets = _resets;
        _resets = 0;
        return resets;
    }

private:
    uint32_t interval(uint64_t cpu_us, bool reset, uint32_t wall_us)
    {
        if (reset || cpu_us < _cpu_us) {
            _resets++;
        }
        return interval_cpu_us(_cpu_us, cpu_us, reset, wall_us - _wall_us);
    }

    void latch(uint64_t cpu_us, uint32_t wall_us)
    {
        _cpu_us = cpu_us;
        _wall_us = wall_us;
    }

    const bool _charges_waits;
    bool _started = false;
    uint64_t _cpu_us = 0;
    uint32_t _wall_us = 0;
    uint16_t _resets = 0;
};

// gen-bindings -r's fast-lane method flag (ScriptingM4Remote::METHOD_FAST_LANE)
static const uint8_t METHOD_FLAG_FAST_LANE = 0x04;

/*
  Which thread serves a frame. The host's fast lane, above the I/O threads, takes only what does
  fixed work with no file, storage, SD or bus access: the generated methods marked so
  (METHOD_FAST_LANE, binding_fast here), m4:noop, m4:run_stats and the Stats rows, and the M7 ->
  M4 types, which only count a protocol error. Everything else, unclassified methods and frame
  types included, goes to the slow worker below the I/O threads.
 */
inline bool serve_on_fast_lane(MsgType type, uint16_t method, bool binding_fast)
{
    switch (type) {
    case MsgType::Stats:
    case MsgType::Reply:
    case MsgType::FaultInject:
    case MsgType::Ready:
        return true;
    case MsgType::Call:
    case MsgType::Notify:
        if (method >= FIRST_BINDING_METHOD) {
            return binding_fast;
        }
        return method == uint16_t(SystemMethod::Noop) || method == uint16_t(SystemMethod::RunStats);
    }
    return false;
}

// the lane of one frame as it came off the ring; method_flags is the generated table's lookup
// (ScriptingM4Remote::method_flags(), 0 for a method this M7 does not have). A frame that does
// not parse goes to the worker, whose handle_message() counts it
inline bool frame_on_fast_lane(const uint8_t *buf, uint16_t len, uint8_t (*method_flags)(uint16_t))
{
    MsgHeader header;
    if (!parse_header(buf, len, header)) {
        return false;
    }
    const MsgType type = MsgType(header.type);
    uint16_t method = 0;
    if (type == MsgType::Call || type == MsgType::Notify) {
        CallHeader call;
        memcpy(&call, buf, sizeof(call));   // parse_header() checked len >= sizeof(CallHeader)
        method = call.method;
    }
    const bool binding_fast = method >= FIRST_BINDING_METHOD && (method_flags(method) & METHOD_FLAG_FAST_LANE) != 0;
    return serve_on_fast_lane(type, method, binding_fast);
}

// a call whose string arguments are filesystem paths (gen-bindings -r METHOD_PATH_ARGS: fs:crc32,
// fs:stat): each must pass the check m4:fs_open makes, so no M4 call reaches @SYS/threads.txt
inline bool path_arguments_allowed(const RpcValue *values, uint8_t argc)
{
    for (uint8_t i = 0; i < argc; i++) {
        if (values[i].tag == ValueTag::String &&
            check_script_path(values[i].string, values[i].length, PathUse::Read) != 0) {
            return false;
        }
    }
    return true;
}

enum class HostWait { DutyCap, Doorbell, Backoff, Timeout };

// how the host waits before its next serve pass. The duty cap comes first, whatever the
// last pass returned or the ring holds; otherwise back off only while frames remain (a
// flooding M4): an empty ring waits on the doorbell even when the pass ran out of budget on
// wall time (preempted)
inline HostWait host_wait_after(uint32_t cap_wait_us, bool budget_ran_out, bool stuck, bool ring_empty)
{
    if (cap_wait_us != 0) {
        return HostWait::DutyCap;
    }
    if (stuck) {
        return HostWait::Timeout;
    }
    if (budget_ran_out && !ring_empty) {
        return HostWait::Backoff;
    }
    return HostWait::Doorbell;
}

}  // namespace ScriptingM4
