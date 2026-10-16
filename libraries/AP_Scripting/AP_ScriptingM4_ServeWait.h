#pragma once

#include <stdint.h>
#include "AP_ScriptingM4_Files.h"
#include "AP_ScriptingM4_Rpc.h"

namespace ScriptingM4 {

/*
  Token bucket on the host thread's CPU time. Credit accrues at SHARE_PERCENT
  of elapsed wall time up to BURST_US; each pass is charged in full for the
  CPU time it used, so credit goes negative by that much. While credit is
  below RESUME_US the host serves nothing, however many frames are pending.
  Time a binding spends blocked (a semaphore, the SD card) yields the CPU to
  other threads and is not charged; a binding that spins is.
  One long worker frame (fs:crc32 of a large file) can leave a debt that
  would pause serving past the M4's call timeout. Below -MAX_DEBT_US the debt
  stays, but a pass is still allowed every MAX_PAUSE_US, in which the host
  refuses binding calls for the slow worker (refuses_slow_bindings()): cheap
  calls are answered, heavy ones fail until the debt is paid down.
 */
class ServeDutyCap {
public:
    static constexpr uint32_t SHARE_PERCENT = 10;
    static constexpr int32_t BURST_US = 2000;
    // a pass is at most MAX_SERVE_US plus one frame: start one only with that much credit
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
        if (refuses_slow_bindings()) {
            // a debt pass: one every MAX_PAUSE_US since the previous pass started
            const uint32_t since_us = now_us - _pass_us;
            return since_us >= MAX_PAUSE_US ? 0 : MAX_PAUSE_US - since_us;
        }
        // at most MAX_PAUSE_US
        return uint32_t((RESUME_US - _credit_us) * int64_t(TIME_PER_CREDIT));
    }

    // the host starts a pass, wait_us() having returned 0
    void pass_started(uint32_t now_us)
    {
        _pass_us = now_us;
    }

    // the debt is too deep to run binding calls on the slow worker: they are refused unrun
    bool refuses_slow_bindings() const
    {
        return _credit_us < -MAX_DEBT_US;
    }

    int64_t credit_us() const { return _credit_us; }

private:
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
