#include <AP_gtest.h>
#include <AP_Scripting/AP_ScriptingM4_ServeWait.h>

using namespace ScriptingM4;

// plain SITL builds tests as C++11, where EXPECT_* binding a static constexpr member by reference
// needs a definition: such members are compared through a cast

// a flooding M4: the budget ran out and frames remain, so the host pauses
TEST(ScriptingM4ServeWait, BudgetOutWithFramesLeftBacksOff)
{
    EXPECT_EQ(host_wait_after(0, true, false, false), HostWait::Backoff);
}

// a pass preempted past the budget with nothing left: the next call's doorbell is heard at once
TEST(ScriptingM4ServeWait, BudgetOutWithEmptyRingWaitsOnDoorbell)
{
    EXPECT_EQ(host_wait_after(0, true, false, true), HostWait::Doorbell);
}

TEST(ScriptingM4ServeWait, IdleWaitsOnDoorbell)
{
    EXPECT_EQ(host_wait_after(0, false, false, true), HostWait::Doorbell);
    EXPECT_EQ(host_wait_after(0, false, false, false), HostWait::Doorbell);
}

// a ring that cannot be served waits for the timeout, whatever it holds
TEST(ScriptingM4ServeWait, StuckWaitsForTimeout)
{
    EXPECT_EQ(host_wait_after(0, false, true, true), HostWait::Timeout);
    EXPECT_EQ(host_wait_after(0, false, true, false), HostWait::Timeout);
    EXPECT_EQ(host_wait_after(0, true, true, true), HostWait::Timeout);
    EXPECT_EQ(host_wait_after(0, true, true, false), HostWait::Timeout);
}

// the duty cap comes first on every path: busy, stuck, idle, ring empty or not
TEST(ScriptingM4ServeWait, DutyCapComesFirst)
{
    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(host_wait_after(1, i & 1, i & 2, i & 4), HostWait::DutyCap);
    }
}

// the first pass after construction may run, on the burst credit only
TEST(ScriptingM4ServeWait, FirstPassAfterConstruction)
{
    ServeDutyCap cap;
    EXPECT_EQ(cap.wait_us(123456789U), 0U);
    EXPECT_EQ(cap.credit_us(), int64_t(ServeDutyCap::BURST_US));
    // time before the first call earns nothing: a long first pass is not covered by it
    cap.charge(50000);
    EXPECT_NE(cap.wait_us(123456789U + 50000U), 0U);
}

// the clock wraps at 2^32 us: credit is earned across the wrap as anywhere else
TEST(ScriptingM4ServeWait, WrapAround)
{
    ServeDutyCap cap;
    const uint32_t start = UINT32_MAX - 5000U;
    EXPECT_EQ(cap.wait_us(start), 0U);
    cap.charge(ServeDutyCap::BURST_US + 400U);   // credit -400: the wait crosses the wrap
    const uint32_t wait = cap.wait_us(start);
    EXPECT_EQ(wait, uint32_t(ServeDutyCap::RESUME_US + 400) * (100U / ServeDutyCap::SHARE_PERCENT));
    EXPECT_NE(cap.wait_us(start + wait - 10U), 0U);
    EXPECT_EQ(cap.wait_us(start + wait), 0U);   // past the wrap
    EXPECT_LT(start + wait, start);
    EXPECT_EQ(cap.credit_us(), int64_t(ServeDutyCap::RESUME_US));
}

// a gap with no calls (a long preemption) refills at most the burst
TEST(ScriptingM4ServeWait, LongGapRefillsBurstOnly)
{
    ServeDutyCap cap;
    EXPECT_EQ(cap.wait_us(1000U), 0U);
    cap.charge(5000);
    EXPECT_EQ(cap.wait_us(1000U + 3000000000U), 0U);
    EXPECT_EQ(cap.credit_us(), int64_t(ServeDutyCap::BURST_US));
}

/*
  The host loop with a cap: each iteration waits as the cap says (in steps of at most
  max_pause_us, as the host still supervises), then runs one pass of pass_us wall time that
  uses cpu_us of CPU, and is charged the CPU. Returns the host's CPU share of the time, in
  parts per million, and counts the passes and the cap pauses.
 */
static uint64_t simulate_share_ppm(uint32_t pass_us, uint32_t cpu_us, uint32_t gap_us, uint64_t duration_us,
                                   uint32_t start_us, uint32_t max_pause_us, uint64_t &passes, uint64_t &pauses)
{
    ServeDutyCap cap;
    uint64_t elapsed = 0;
    uint64_t busy = 0;
    passes = 0;
    pauses = 0;
    while (elapsed < duration_us) {
        const uint32_t now = start_us + uint32_t(elapsed);
        const uint32_t wait = cap.wait_us(now);
        if (wait != 0) {
            pauses++;
            elapsed += wait < max_pause_us ? wait : max_pause_us;
            continue;
        }
        elapsed += pass_us;
        busy += cpu_us;
        cap.charge(cpu_us);
        passes++;
        elapsed += gap_us;
    }
    return busy * 1000000U / elapsed;
}

static uint64_t simulate_share_ppm(uint32_t pass_us, uint32_t gap_us, uint64_t duration_us,
                                   uint32_t start_us, uint32_t max_pause_us, uint64_t &passes)
{
    uint64_t pauses;
    return simulate_share_ppm(pass_us, pass_us, gap_us, duration_us, start_us, max_pause_us, passes, pauses);
}

// passes using 50 ms of CPU each (a binding that spins): the long-run share stays within
// SHARE_PERCENT plus one pass (and the starting burst), the charge is never clamped
TEST(ScriptingM4ServeWait, LongPassesKeepTheShare)
{
    uint64_t passes;
    const uint64_t duration = 100000000U;   // 100 s
    const uint64_t ppm = simulate_share_ppm(50000U, 0U, duration, 0U, 10000U, passes);
    // the share, plus one pass and the starting burst over the run
    const uint64_t bound = ServeDutyCap::SHARE_PERCENT * 10000U +
                           (50000U + ServeDutyCap::BURST_US) * uint64_t(1000000U) / duration;
    EXPECT_LE(ppm, bound);
    EXPECT_GT(ppm, ServeDutyCap::SHARE_PERCENT * 10000U * 9U / 10U);   // the cap is the limit
    EXPECT_GT(passes, 150U);
}

// one pass of 1 s: serving waits about 10 s, the whole of it charged
TEST(ScriptingM4ServeWait, OneSecondPassIsChargedInFull)
{
    ServeDutyCap cap;
    EXPECT_EQ(cap.wait_us(0U), 0U);
    cap.charge(1000000U);
    const uint32_t wait = cap.wait_us(1000000U);
    EXPECT_GE(wait, 8900000U);
    EXPECT_NE(cap.wait_us(1000000U + wait - 10U), 0U);
    EXPECT_EQ(cap.wait_us(1000000U + wait), 0U);
}

// a flood: back-to-back passes of 200 us, across a clock wrap
TEST(ScriptingM4ServeWait, FloodKeepsTheShare)
{
    uint64_t passes;
    const uint64_t duration = 10000000U;   // 10 s
    const uint64_t ppm = simulate_share_ppm(200U, 0U, duration, UINT32_MAX - 3000000U, 10000U, passes);
    EXPECT_LE(ppm, ServeDutyCap::SHARE_PERCENT * 10000U +
              (200U + ServeDutyCap::BURST_US) * uint64_t(1000000U) / duration);
    EXPECT_GT(ppm, ServeDutyCap::SHARE_PERCENT * 10000U * 9U / 10U);
}

// the bench's normal rate: about 2000 calls/s at 3.3 % wall time, with a main-loop
// preemption of 900 us landing in one pass in four windows of 10 ms: never capped
TEST(ScriptingM4ServeWait, NormalRateIsNotCapped)
{
    ServeDutyCap cap;
    uint32_t now = 0;
    EXPECT_EQ(cap.wait_us(now), 0U);
    for (uint32_t pass = 0; pass < 200000; pass++) {   // 100 s of passes, one per 500 us
        const uint32_t pass_us = (pass % 80 == 0) ? 900U + 17U : 17U;
        ASSERT_EQ(cap.wait_us(now), 0U) << "capped at pass " << pass;
        cap.charge(pass_us);
        now += 500U;
    }
}

// passes of 50 ms wall time that are blocked (a semaphore, the SD card) and use 100 us of CPU:
// they yield the CPU, so the cap never pauses them
TEST(ScriptingM4ServeWait, BlockedPassesAreNotThrottled)
{
    uint64_t passes, pauses;
    const uint64_t ppm = simulate_share_ppm(50000U, 100U, 0U, 100000000U, 0U, 10000U, passes, pauses);
    EXPECT_EQ(pauses, 0U);
    EXPECT_EQ(passes, 2000U);   // one after another for 100 s
    EXPECT_EQ(ppm, 2000U);      // 0.2 % CPU
}

// a scheduler-semaphore binding at the bench's rate: 2000 calls/s, each pass 17 us of CPU,
// and every fifth one blocked for a 2.5 ms main loop: wall time would trip a wall-time cap
TEST(ScriptingM4ServeWait, SemaphoreWaitsAtNormalRateAreNotCapped)
{
    ServeDutyCap cap;
    uint32_t now = 0;
    EXPECT_EQ(cap.wait_us(now), 0U);
    for (uint32_t pass = 0; pass < 200000; pass++) {
        ASSERT_EQ(cap.wait_us(now), 0U) << "capped at pass " << pass;
        cap.charge(17U);
        now += (pass % 5 == 0) ? 2500U : 500U;
    }
}

// the CPU clock read before and after an interval: a reset, or a clock that went back, charges
// the interval's wall time, never less than the thread used
TEST(ScriptingM4ServeWait, IntervalCpuFromClockReadings)
{
    EXPECT_EQ(interval_cpu_us(1000U, 1250U, false, 400U), 250U);
    EXPECT_EQ(interval_cpu_us(1000U, 1000U, false, 400U), 0U);
    EXPECT_EQ(interval_cpu_us(1000U, 1250U, true, 400U), 400U);      // reset seen: the wall time
    EXPECT_EQ(interval_cpu_us(5000000U, 300U, false, 400U), 400U);   // went back: the wall time
    EXPECT_EQ(interval_cpu_us(0U, uint64_t(UINT32_MAX) + 10U, false, 400U), UINT32_MAX);
}

/*
  A flood whose statistics are reset during every pass (the attack of a script opening
  @SYS/threads.txt each pass, or a GCS reading it as fast as it can): each pass runs 200 us of
  CPU in 300 us of wall time, and the waits sleep. The host charges each pass from a reading
  at its start to one at its end; the meter here is the CPU actually used, which the charges
  do not feed: its share stays within the cap.
 */
TEST(ScriptingM4ServeWait, StatisticsResetEveryPassKeepsTheShare)
{
    ServeDutyCap cap;
    uint64_t elapsed = 0;
    uint64_t used = 0;
    const uint64_t duration = 10000000U;
    EXPECT_EQ(cap.wait_us(0U), 0U);
    while (elapsed < duration) {
        const uint32_t wait = cap.wait_us(uint32_t(elapsed));
        if (wait != 0) {
            elapsed += wait < 10000U ? wait : 10000U;
            continue;
        }
        const uint64_t start = 5000000U;   // the clock at the pass start
        const uint64_t end = 150U;         // reset part way through, 150 us more after it
        elapsed += 300U;
        used += 200U;
        cap.charge(interval_cpu_us(start, end, true, 300U));
    }
    const uint64_t ppm = used * 1000000U / elapsed;
    EXPECT_LE(ppm, ServeDutyCap::SHARE_PERCENT * 10000U + (300U + ServeDutyCap::BURST_US) * uint64_t(1000000U) / duration);
}

// without the reset flag, a clock zeroed between readings under-charges: the case the
// reading's reset marker exists for
TEST(ScriptingM4ServeWait, UnflaggedResetWouldUnderCharge)
{
    EXPECT_EQ(interval_cpu_us(10U, 150U, false, 300U), 140U);   // 200 us used, 140 charged
    EXPECT_EQ(interval_cpu_us(10U, 150U, true, 300U), 300U);
}

AP_GTEST_MAIN()
