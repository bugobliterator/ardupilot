#include <AP_gtest.h>
#include <AP_Scripting/AP_Scripting_config.h>
#include <AP_Scripting/AP_ScriptingM4_ServeWait.h>
#if AP_SCRIPTING_M4_ENABLED
#include <AP_Scripting/AP_ScriptingM4_Bindings.h>
#include <AP_Scripting/AP_ScriptingM4_Remote.h>
// the M4's tables as gen-bindings -r wrote them for this tree (the M7 links no copy)
#include <AP_Scripting/lua_remote_tables.cpp>
#include <AP_HAL/AP_HAL.h>
// method_flags() comes with the generated dispatch, which needs the HAL to link
const AP_HAL::HAL& hal = AP_HAL::get_HAL();
#endif

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
        cap.pass_started(now);
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

// one pass of 1 s (a binding that spins on the fast lane): charged in full, the debt is paid
// down by pausing, and while it is beyond MAX_DEBT_US a pass is still allowed every MAX_PAUSE_US
TEST(ScriptingM4ServeWait, OneSecondPassIsChargedInFull)
{
    ServeDutyCap cap;
    EXPECT_EQ(cap.wait_us(0U), 0U);
    cap.pass_started(0U);
    cap.charge(1000000U);
    EXPECT_EQ(cap.wait_us(1000000U), 0U);   // a second since the pass started: a debt pass
    EXPECT_TRUE(cap.refuses_slow_bindings());
    EXPECT_EQ(cap.credit_us(), int64_t(ServeDutyCap::BURST_US) - 1000000 + 100000);
    cap.pass_started(1000000U);
    EXPECT_EQ(cap.wait_us(1000000U), uint32_t(ServeDutyCap::MAX_PAUSE_US));
}

/*
  The host serving the fast lane after one worker frame of frame_us CPU: a cheap frame is
  waiting at every moment, and each pass serves it (50 us). No pause is longer than
  MAX_PAUSE_US, well inside the M4's 1 s call timeout, and the debt is paid down in full
 */
static void serve_fast_lane_after(uint32_t frame_us)
{
    ServeDutyCap cap;
    uint32_t now = 1000U;
    ASSERT_EQ(cap.wait_us(now), 0U);
    cap.pass_started(now);
    cap.charge(frame_us);
    now += frame_us;
    uint64_t fast_passes = 0;
    uint64_t fast_cpu = 0;
    const uint32_t start = now;
    uint32_t last_pass = now - frame_us;
    uint32_t last_pass_us = frame_us;   // how long the previous pass ran
    while (now - start < 30000000U) {
        const uint32_t wait = cap.wait_us(now);
        ASSERT_LE(wait, uint32_t(ServeDutyCap::MAX_PAUSE_US)) << frame_us;
        if (wait != 0) {
            now += wait < 10000U ? wait : 10000U;   // the host supervises in between
            continue;
        }
        // from one pass to the next, the frame itself aside
        ASSERT_LE(now - last_pass, last_pass_us + ServeDutyCap::MAX_PAUSE_US) << frame_us;
        last_pass = now;
        last_pass_us = 50U;
        cap.pass_started(now);
        cap.charge(50U);
        fast_cpu += 50U;
        fast_passes++;
        now += 50U;
    }
    // at least one pass per MAX_PAUSE_US plus its own time
    EXPECT_GE(fast_passes, 30000000U / (ServeDutyCap::MAX_PAUSE_US + 50U)) << frame_us;
    // the frame and the fast lane together within the share, the frame and the burst over
    EXPECT_LE(fast_cpu + frame_us, (now - 1000U) / 10U + frame_us / 10U + ServeDutyCap::BURST_US + 50U) << frame_us;
    EXPECT_FALSE(cap.refuses_slow_bindings()) << frame_us;
}

TEST(ScriptingM4ServeWait, LongWorkerFrameLeavesTheFastLaneServed)
{
    serve_fast_lane_after(150000U);
    serve_fast_lane_after(1000000U);
}

/*
  A script looping a heavy binding on the worker (fs:crc32 of a large file), frame_us of CPU per
  frame, with a 50 us hand-off and a 30 us refusal. Returns the CPU share in ppm and counts the
  frames run and refused; no pause is longer than MAX_PAUSE_US
 */
static uint64_t heavy_loop_share_ppm(uint32_t frame_us, uint64_t duration_us, uint32_t &run, uint32_t &refused)
{
    ServeDutyCap cap;
    uint32_t now = 1000U;
    uint64_t used = 0;
    run = 0;
    refused = 0;
    const uint32_t start = now;
    while (now - start < duration_us) {
        const uint32_t wait = cap.wait_us(now);
        EXPECT_LE(wait, uint32_t(ServeDutyCap::MAX_PAUSE_US));
        if (wait != 0) {
            now += wait;
            continue;
        }
        cap.pass_started(now);
        if (cap.refuses_slow_bindings()) {
            cap.charge(30U);
            used += 30U;
            now += 30U;
            refused++;
            continue;
        }
        cap.charge(50U + frame_us);
        used += 50U + frame_us;
        now += 50U + frame_us;
        run++;
    }
    return used * 1000000U / (now - start);
}

// 1 s frames: the debt is kept, so the share holds at 10 % plus one frame over the run, and the
// frames asked for while the debt is beyond MAX_DEBT_US are refused
TEST(ScriptingM4ServeWait, RepeatedLongWorkerFramesKeepTheShare)
{
    uint32_t run, refused;
    const uint64_t duration = 100000000U;   // 100 s
    const uint64_t ppm = heavy_loop_share_ppm(1000000U, duration, run, refused);
    EXPECT_LE(ppm, ServeDutyCap::SHARE_PERCENT * 10000U + (1000050U + ServeDutyCap::BURST_US) * uint64_t(1000000U) / duration);
    EXPECT_GT(ppm, ServeDutyCap::SHARE_PERCENT * 10000U * 9U / 10U);
    EXPECT_LE(run, 11U);
    EXPECT_GT(refused, run * 10U);   // a debt pass about every 0.5 s while the debt is deep
}

// frames between MAX_DEBT_US and twice that: the early passes the cap allows run some of them in
// debt, but none is forgiven, so the share still holds
TEST(ScriptingM4ServeWait, MidLengthWorkerFramesKeepTheShare)
{
    for (const uint32_t frame_us : { 60000U, 90000U, 150000U }) {
        uint32_t run, refused;
        const uint64_t duration = 100000000U;
        const uint64_t ppm = heavy_loop_share_ppm(frame_us, duration, run, refused);
        EXPECT_LE(ppm, ServeDutyCap::SHARE_PERCENT * 10000U + (frame_us + 50U + ServeDutyCap::MAX_DEBT_US + ServeDutyCap::BURST_US) * uint64_t(1000000U) / duration) << frame_us;
        EXPECT_GT(ppm, ServeDutyCap::SHARE_PERCENT * 10000U * 9U / 10U) << frame_us;
    }
}

// frames within MAX_DEBT_US are paid down by pausing alone: none is refused, the share holds
TEST(ScriptingM4ServeWait, WorkerFramesWithinTheDebtLimitAreNotRefused)
{
    uint32_t run, refused;
    const uint64_t duration = 100000000U;
    const uint64_t ppm = heavy_loop_share_ppm(40000U, duration, run, refused);
    EXPECT_EQ(refused, 0U);
    EXPECT_LE(ppm, ServeDutyCap::SHARE_PERCENT * 10000U + (40050U + ServeDutyCap::BURST_US) * uint64_t(1000000U) / duration);
    EXPECT_GT(run, 200U);
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
        cap.pass_started(uint32_t(elapsed));
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

// a ChibiOS thread's statistics clock (os/rt/src/chtm.c), in microseconds for clarity: the slice
// chain at a switch, chTMObjectInit() from another thread, and Util::thread_info() on the thread
struct FakeThreadStats {
    ThreadStatsClock clock { 0, 0, UINT32_MAX };   // chTMObjectInit() at thread creation
    void switch_in(uint32_t now) { clock.last = now; }
    void switch_out(uint32_t now)
    {
        const uint32_t slice = now - clock.last;
        clock.cumulative += slice;
        if (slice < clock.best) {
            clock.best = slice;
        }
    }
    void reset_from_other_thread() { clock = ThreadStatsClock { 0, 0, UINT32_MAX }; }
    // Util::thread_info() on this thread: cumulative only, and best since that fix
    void reset_from_this_thread(bool marks_best)
    {
        clock.cumulative = 0;
        if (marks_best) {
            clock.best = UINT32_MAX;
        }
    }
    uint64_t read(uint32_t now, bool &reset) { return read_thread_cycles(clock, now, reset); }
};

// the bench's binding_bench.lua: 416 synchronous calls back to back once a second, each pass
// charged 41 us (the bench's M4ST.CPU / Req) with a 145 us round trip. The 10 ms bucket carries
// the first 370 calls unpaced (10000 - 200 us of credit at 41 - 14.5 us per call); the 2 ms one
// paused from call 68
TEST(ScriptingM4ServeWait, OneSecondCallBurstIsPacedOnlyPastTheBucket)
{
    ServeDutyCap cap;
    uint32_t now = 0;
    int first_pause = -1;
    for (int call = 0; call < 416; call++) {
        uint32_t wait = cap.wait_us(now);
        if (wait != 0 && first_pause < 0) {
            first_pause = call;
        }
        while (wait != 0) {
            now += wait;
            wait = cap.wait_us(now);
        }
        cap.pass_started(now);
        cap.charge(41U);
        now += 145U;
    }
    EXPECT_EQ(first_pause, 370);
}

// a pass that a duty-cap pause waited for runs, although the pause's own wake (10 us) is charged
// before it: the host decides before it charges (fails if pass_after_wake() charges first)
TEST(ScriptingM4ServeWait, PassRunsRightAfterAPause)
{
    ServeDutyCap cap;
    uint32_t now = 1000;
    EXPECT_EQ(cap.wait_us(now), 0U);
    cap.charge(uint32_t(ServeDutyCap::BURST_US) + 3000U);   // a pass that ran the credit to -3000
    uint32_t wait = cap.wait_us(now);
    ASSERT_NE(wait, 0U);
    while (wait != 0) {
        now += wait;
        wait = cap.wait_us(now);
    }
    EXPECT_EQ(cap.credit_us(), int64_t(ServeDutyCap::RESUME_US));
    uint32_t charged = 0;
    EXPECT_TRUE(pass_after_wake(cap, now, [&]() { cap.charge(10U); charged += 10U; }));
    EXPECT_EQ(charged, 10U);   // the wake is still charged, in full
    EXPECT_EQ(cap.credit_us(), int64_t(ServeDutyCap::RESUME_US) - 10);
    // the next wake then waits for the charge to be paid back
    EXPECT_NE(cap.wait_us(now), 0U);
}

// the host as the M7 runs it: sleep, wake (switch-in plus overhead), a reading, a pass, a
// reading, overhead, sleep. Readings come from FakeThreadStats; returns the CPU charged and the
// CPU used. reset_every: a reset in every pass (0: none). marks_best: Util::thread_info's fix
struct HostModel {
    FakeThreadStats stats;
    HostCpuCharge charge { true };
    uint64_t charged = 0;
    uint64_t used = 0;
    uint32_t now = 1000;

    void iteration(uint32_t wake_overhead, uint32_t pass, uint32_t sleep_overhead, uint32_t sleep,
                   int reset_kind, bool marks_best = true)
    {
        stats.switch_in(now);
        now += wake_overhead;
        bool reset;
        uint64_t cpu = stats.read(now, reset);
        charged += charge.pass_start(cpu, reset, now);
        now += pass / 2;
        if (reset_kind == 1) {
            stats.reset_from_this_thread(marks_best);   // the pass generated @SYS/threads.txt itself
        }
        now += pass - pass / 2;
        cpu = stats.read(now, reset);
        charged += charge.pass_end(cpu, reset, now);
        now += sleep_overhead;
        stats.switch_out(now);
        used += wake_overhead + pass + sleep_overhead;
        if (reset_kind == 2) {
            stats.reset_from_other_thread();   // a GCS read while the host sleeps
        }
        now += sleep;
    }
};

// no reset: everything the thread ran is charged, the wake and switch overhead around each pass
// included, and nothing more (fails if read_thread_cycles() stops keeping best at 0)
TEST(ScriptingM4ServeWait, ChibiOSChargesAllHostCpu)
{
    HostModel host;
    host.iteration(5, 20, 4, 500, 0);   // the first reading only starts the clock
    const uint64_t used_before = host.used;
    for (int i = 0; i < 100; i++) {
        host.iteration(5, 20, 4, 500, 0);
    }
    // the first iteration charged its pass only (20 us): its reading started the clock
    EXPECT_EQ(host.charged - 20U, host.used - used_before);
    EXPECT_EQ(host.charge.take_resets(), 0U);
}

// SITL charges passes only: the overhead around them is not charged
TEST(ScriptingM4ServeWait, PassOnlyChargeLeavesOutOverhead)
{
    FakeThreadStats stats;
    HostCpuCharge charge { false };
    uint32_t now = 1000;
    uint64_t charged = 0;
    bool reset;
    for (int i = 0; i < 10; i++) {
        stats.switch_in(now);
        now += 5;
        charged += charge.pass_start(stats.read(now, reset), reset, now);
        now += 20;
        charged += charge.pass_end(stats.read(now, reset), reset, now);
        now += 4;
        stats.switch_out(now);
        now += 500;
    }
    EXPECT_EQ(charged, 10U * 20U);   // the passes, none of the 9 us around each
}

// Util::thread_info() on the host (a pass that generated @SYS/threads.txt) zeroes cumulative
// mid-pass while the pass's slice has run longer than the old cumulative: unflagged, the pass is
// under-charged. With best marked, it is charged its wall time (fails if the reset flag is
// ignored, or without the Util::thread_info() fix)
TEST(ScriptingM4ServeWait, SelfResetIsCharged)
{
    for (const bool marks_best : { true, false }) {
        HostModel host;
        host.iteration(5, 20, 4, 500, 0);
        host.stats.reset_from_other_thread();   // a small cumulative from here on
        host.iteration(5, 20, 4, 500, 0);
        const uint64_t charged_before = host.charged;
        host.iteration(5, 400, 4, 500, 1, marks_best);
        const uint64_t charged = host.charged - charged_before;
        if (marks_best) {
            EXPECT_GE(charged, 400U);
        } else {
            EXPECT_LT(charged, 400U);   // why Util::thread_info() marks best
        }
    }
}

// a reset by another thread while the host sleeps: flagged at the next reading, the interval
// charged its wall time (an over-charge), never less than the host used
TEST(ScriptingM4ServeWait, OtherThreadResetOverCharges)
{
    HostModel host;
    host.iteration(5, 20, 4, 500, 0);
    const uint64_t used_before = host.used;
    for (int i = 0; i < 50; i++) {
        host.iteration(5, 20, 4, 500, 2);
    }
    EXPECT_GE(host.charged, host.used - used_before);
    EXPECT_EQ(host.charge.take_resets(), 49U);   // the last reset is after the last reading
}

// a flood whose every pass resets the host's own statistics (the fs:crc32("@SYS/threads.txt")
// attack, had it not been refused): the CPU it really uses stays within the cap
TEST(ScriptingM4ServeWait, SelfResetEveryPassKeepsTheShare)
{
    HostModel host;
    ServeDutyCap cap;
    EXPECT_EQ(cap.wait_us(host.now), 0U);
    const uint32_t start = host.now;
    while (host.now - start < 10000000U) {
        const uint32_t wait = cap.wait_us(host.now);
        if (wait != 0) {
            host.now += wait < 10000U ? wait : 10000U;
            continue;
        }
        cap.pass_started(host.now);
        const uint64_t before = host.charged;
        // one pass of a varying length, as natural jitter makes it
        host.iteration(3, 150 + (host.now % 97), 2, 0, 1);
        cap.charge(uint32_t(host.charged - before));
    }
    const uint64_t ppm = host.used * 1000000U / (host.now - start);
    EXPECT_LE(ppm, ServeDutyCap::SHARE_PERCENT * 10000U + (300U + ServeDutyCap::BURST_US) * uint64_t(1000000U) / 10000000U);
}

// the lanes: only frames that do fixed work run above the I/O threads
TEST(ScriptingM4ServeWait, FastLaneTakesOnlyBoundedFrames)
{
    EXPECT_TRUE(serve_on_fast_lane(MsgType::Call, FIRST_BINDING_METHOD + 3, true));
    EXPECT_FALSE(serve_on_fast_lane(MsgType::Call, FIRST_BINDING_METHOD + 3, false));
    EXPECT_FALSE(serve_on_fast_lane(MsgType::Notify, FIRST_BINDING_METHOD + 3, false));
    EXPECT_TRUE(serve_on_fast_lane(MsgType::Notify, uint16_t(SystemMethod::Noop), false));
    EXPECT_TRUE(serve_on_fast_lane(MsgType::Notify, uint16_t(SystemMethod::RunStats), false));
    for (const SystemMethod slow : { SystemMethod::FsOpen, SystemMethod::FsRead, SystemMethod::FsDir,
                                     SystemMethod::ReadScript, SystemMethod::ListScript, SystemMethod::Print,
                                     SystemMethod::RuntimeText, SystemMethod::Release, SystemMethod::Bindings,
                                     SystemMethod::Config, SystemMethod::ScriptError }) {
        EXPECT_FALSE(serve_on_fast_lane(MsgType::Call, uint16_t(slow), true));
    }
    EXPECT_FALSE(serve_on_fast_lane(MsgType::Call, 60, true));   // no system method yet: the worker
    EXPECT_TRUE(serve_on_fast_lane(MsgType::Stats, 0, false));
    // M7 -> M4 types only count a protocol error; a type no M7 knows fails closed
    EXPECT_TRUE(serve_on_fast_lane(MsgType::Reply, 0, false));
    EXPECT_TRUE(serve_on_fast_lane(MsgType::FaultInject, 0, false));
    EXPECT_TRUE(serve_on_fast_lane(MsgType::Ready, 0, false));
    EXPECT_FALSE(serve_on_fast_lane(MsgType(99), 0, true));
    EXPECT_FALSE(serve_on_fast_lane(MsgType(0), 0, true));
}

/*
  The worker's CPU is charged to the same duty cap as the host's: a script looping a frame that
  runs 40 ms of CPU on the worker (fs:crc32 of a large file) for every 50 us host hand-off. The
  CPU actually used, host and worker, stays within the cap; were the worker's charge left out,
  the host's hand-offs alone would never pause it
 */
TEST(ScriptingM4ServeWait, WorkerCpuIsCharged)
{
    for (const bool charge_worker : { true, false }) {
        ServeDutyCap cap;
        HostCpuCharge worker { false };
        uint32_t now = 1000;
        uint64_t used = 0;
        uint64_t worker_clock = 0;
        EXPECT_EQ(cap.wait_us(now), 0U);
        const uint32_t start = now;
        while (now - start < 20000000U) {
            const uint32_t wait = cap.wait_us(now);
            if (wait != 0) {
                now += wait < 10000U ? wait : 10000U;
                continue;
            }
            cap.pass_started(now);
            cap.charge(50U);   // the host's pass: the hand-off
            IGNORE_RETURN(worker.pass_start(worker_clock, false, now));
            worker_clock += 40000U;
            now += 40050U;
            const uint32_t worker_us = worker.pass_end(worker_clock, false, now);
            if (charge_worker) {
                cap.charge(worker_us);
            }
            used += 40050U;
        }
        const uint64_t ppm = used * 1000000U / (now - start);
        if (charge_worker) {
            EXPECT_LE(ppm, ServeDutyCap::SHARE_PERCENT * 10000U + (40050U + ServeDutyCap::BURST_US) * uint64_t(1000000U) / 20000000U);
        } else {
            EXPECT_GT(ppm, 900000U);   // nearly all of the time
        }
    }
}

#if AP_SCRIPTING_M4_ENABLED
// the lane of real frames against the flags gen-bindings -r wrote for this tree: kills a lane
// decision that ignores, inverts or misreads METHOD_FAST_LANE
static uint16_t binding_method(const char *type_name, const char *row_name)
{
    for (uint8_t id = 1; id <= BINDING_TYPE_COUNT; id++) {
        const BindingType &type = binding_types[id - 1];
        if (strcmp(type.name, type_name) != 0) {
            continue;
        }
        for (uint16_t i = type.first_row; i < type.first_row + type.row_count; i++) {
            if (strcmp(binding_rows[i].name, row_name) == 0) {
                return binding_rows[i].method;
            }
        }
    }
    return 0;
}

static bool call_frame_on_fast_lane(uint16_t method, MsgType type = MsgType::Call)
{
    CallHeader call {};
    fill_header(call.header, type, sizeof(call), 1);
    call.method = method;
    uint8_t buf[sizeof(call)];
    memcpy(buf, &call, sizeof(call));
    return frame_on_fast_lane(buf, sizeof(buf), ScriptingM4Remote::method_flags);
}

TEST(ScriptingM4ServeWait, GeneratedFlagsChooseTheLane)
{
    const uint16_t millis = binding_method("global", "millis");
    const uint16_t get_location = binding_method("ahrs", "get_location");
    const uint16_t crc32 = binding_method("fs", "crc32");
    const uint16_t param_get = binding_method("param", "get");
    const uint16_t set_mode = binding_method("vehicle", "set_mode");
    const uint16_t reboot = binding_method("vehicle", "reboot");
    for (const uint16_t method : { millis, get_location, crc32, param_get, set_mode, reboot }) {
        ASSERT_GE(method, FIRST_BINDING_METHOD);
    }
    EXPECT_TRUE(call_frame_on_fast_lane(millis));
    EXPECT_TRUE(call_frame_on_fast_lane(get_location));
    EXPECT_TRUE(call_frame_on_fast_lane(get_location, MsgType::Notify));
    EXPECT_FALSE(call_frame_on_fast_lane(crc32));
    EXPECT_FALSE(call_frame_on_fast_lane(param_get));
    EXPECT_FALSE(call_frame_on_fast_lane(set_mode));
    EXPECT_FALSE(call_frame_on_fast_lane(reboot));
    // a method id this M7 does not have, and a frame that does not parse: the worker
    EXPECT_FALSE(call_frame_on_fast_lane(uint16_t(FIRST_BINDING_METHOD + BINDING_METHOD_COUNT)));
    const uint8_t short_frame[3] = { uint8_t(MsgType::Call), 0, 3 };
    EXPECT_FALSE(frame_on_fast_lane(short_frame, sizeof(short_frame), ScriptingM4Remote::method_flags));
}
#endif  // AP_SCRIPTING_M4_ENABLED

// fs:crc32 and fs:stat: their paths are checked as m4:fs_open's
TEST(ScriptingM4ServeWait, PathArgumentsAreChecked)
{
    RpcValue values[2] {};
    values[0].tag = ValueTag::String;
    values[1].tag = ValueTag::Integer;
    const char *refused[] = { "@SYS/threads.txt", "/@SYS/threads.txt", "../x", "" };
    for (const char *path : refused) {
        values[0].string = path;
        values[0].length = uint16_t(strlen(path));
        EXPECT_FALSE(path_arguments_allowed(values, 2)) << path;
    }
    const char *allowed[] = { "@SYS/tasks.txt", "/APM/scripts/x.lua", "@ROMFS/scripts/x.lua" };
    for (const char *path : allowed) {
        values[0].string = path;
        values[0].length = uint16_t(strlen(path));
        EXPECT_TRUE(path_arguments_allowed(values, 2)) << path;
    }
}

AP_GTEST_MAIN()
