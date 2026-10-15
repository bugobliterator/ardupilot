#pragma once

#include <stdint.h>

namespace ScriptingM4 {

/*
  Token bucket on the host thread's CPU time. Credit accrues at SHARE_PERCENT
  of elapsed wall time up to BURST_US; each pass is charged in full for the
  CPU time it used, so credit goes negative by that much. While credit is
  below RESUME_US the host serves nothing, however many frames are pending.
  Time a binding spends blocked (a semaphore, the SD card) yields the CPU to
  other threads and is not charged; a binding that spins is.
 */
class ServeDutyCap {
public:
    static constexpr uint32_t SHARE_PERCENT = 10;
    static constexpr int32_t BURST_US = 2000;
    // a pass is at most MAX_SERVE_US plus one frame: start one only with that much credit
    static constexpr int32_t RESUME_US = 200;

    void charge(uint32_t busy_us)
    {
        _credit_us -= busy_us;
    }

    // 0 when a pass may start now, else the microseconds until it may
    uint32_t wait_us(uint32_t now_us)
    {
        refill(now_us);
        if (_credit_us >= RESUME_US) {
            return 0;
        }
        const int64_t wait = (RESUME_US - _credit_us) * int64_t(TIME_PER_CREDIT);
        return wait > int64_t(UINT32_MAX) ? UINT32_MAX : uint32_t(wait);
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
