#include <AP_gtest.h>
#include <vector>
#include <AP_Common/AP_Common.h>
#include <AP_RustM4/AP_RustM4_ServeWait.h>

using namespace RustM4;

// the simulators below stand in for the host loop and start passes themselves
struct RustM4::ServeDutyCapTestAccess {
    static void pass_started(ServeDutyCap &cap, uint32_t now_us) { cap.pass_started(now_us); }
};

// plain SITL builds tests as C++11, where EXPECT_* binding a static constexpr member by reference
// needs a definition: such members are compared through a cast

// a flooding M4: the budget ran out and frames remain, so the host pauses
TEST(RustM4ServeWait, BudgetOutWithFramesLeftBacksOff)
{
    EXPECT_EQ(host_wait_after(0, true, false, false), HostWait::Backoff);
}

// a pass preempted past the budget with nothing left: the next call's doorbell is heard at once
TEST(RustM4ServeWait, BudgetOutWithEmptyRingWaitsOnDoorbell)
{
    EXPECT_EQ(host_wait_after(0, true, false, true), HostWait::Doorbell);
}

TEST(RustM4ServeWait, IdleWaitsOnDoorbell)
{
    EXPECT_EQ(host_wait_after(0, false, false, true), HostWait::Doorbell);
    EXPECT_EQ(host_wait_after(0, false, false, false), HostWait::Doorbell);
}

// a ring that cannot be served waits for the timeout, whatever it holds
TEST(RustM4ServeWait, StuckWaitsForTimeout)
{
    EXPECT_EQ(host_wait_after(0, false, true, true), HostWait::Timeout);
    EXPECT_EQ(host_wait_after(0, false, true, false), HostWait::Timeout);
    EXPECT_EQ(host_wait_after(0, true, true, true), HostWait::Timeout);
    EXPECT_EQ(host_wait_after(0, true, true, false), HostWait::Timeout);
}

// the duty cap comes first on every path: busy, stuck, idle, ring empty or not
TEST(RustM4ServeWait, DutyCapComesFirst)
{
    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(host_wait_after(1, i & 1, i & 2, i & 4), HostWait::DutyCap);
    }
}

// the first pass after construction may run, on the burst credit only
TEST(RustM4ServeWait, FirstPassAfterConstruction)
{
    ServeDutyCap cap;
    EXPECT_EQ(cap.wait_us(123456789U), 0U);
    EXPECT_EQ(cap.credit_us(), int64_t(ServeDutyCap::BURST_US));
    // time before the first call earns nothing: a long first pass is not covered by it
    cap.charge(50000);
    EXPECT_NE(cap.wait_us(123456789U + 50000U), 0U);
}

// the clock wraps at 2^32 us: credit is earned across the wrap as anywhere else
TEST(RustM4ServeWait, WrapAround)
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
TEST(RustM4ServeWait, LongGapRefillsBurstOnly)
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
        ServeDutyCapTestAccess::pass_started(cap, now);
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
TEST(RustM4ServeWait, LongPassesKeepTheShare)
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
TEST(RustM4ServeWait, OneSecondPassIsChargedInFull)
{
    ServeDutyCap cap;
    EXPECT_EQ(cap.wait_us(0U), 0U);
    ServeDutyCapTestAccess::pass_started(cap, 0U);
    cap.charge(1000000U);
    EXPECT_EQ(cap.wait_us(1000000U), 0U);   // a second since the pass started: a debt pass
    EXPECT_TRUE(cap.refuses_slow_bindings());
    EXPECT_EQ(cap.credit_us(), int64_t(ServeDutyCap::BURST_US) - 1000000 + 100000);
    ServeDutyCapTestAccess::pass_started(cap, 1000000U);
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
    ServeDutyCapTestAccess::pass_started(cap, now);
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
        ServeDutyCapTestAccess::pass_started(cap, now);
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

TEST(RustM4ServeWait, LongWorkerFrameLeavesTheFastLaneServed)
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
        ServeDutyCapTestAccess::pass_started(cap, now);
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
TEST(RustM4ServeWait, RepeatedLongWorkerFramesKeepTheShare)
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
TEST(RustM4ServeWait, MidLengthWorkerFramesKeepTheShare)
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
TEST(RustM4ServeWait, WorkerFramesWithinTheDebtLimitAreNotRefused)
{
    uint32_t run, refused;
    const uint64_t duration = 100000000U;
    const uint64_t ppm = heavy_loop_share_ppm(40000U, duration, run, refused);
    EXPECT_EQ(refused, 0U);
    EXPECT_LE(ppm, ServeDutyCap::SHARE_PERCENT * 10000U + (40050U + ServeDutyCap::BURST_US) * uint64_t(1000000U) / duration);
    EXPECT_GT(run, 200U);
}

// a flood: back-to-back passes of 200 us, across a clock wrap
TEST(RustM4ServeWait, FloodKeepsTheShare)
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
TEST(RustM4ServeWait, NormalRateIsNotCapped)
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
TEST(RustM4ServeWait, BlockedPassesAreNotThrottled)
{
    uint64_t passes, pauses;
    const uint64_t ppm = simulate_share_ppm(50000U, 100U, 0U, 100000000U, 0U, 10000U, passes, pauses);
    EXPECT_EQ(pauses, 0U);
    EXPECT_EQ(passes, 2000U);   // one after another for 100 s
    EXPECT_EQ(ppm, 2000U);      // 0.2 % CPU
}

// a scheduler-semaphore binding at the bench's rate: 2000 calls/s, each pass 17 us of CPU,
// and every fifth one blocked for a 2.5 ms main loop: wall time would trip a wall-time cap
TEST(RustM4ServeWait, SemaphoreWaitsAtNormalRateAreNotCapped)
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
TEST(RustM4ServeWait, IntervalCpuFromClockReadings)
{
    EXPECT_EQ(interval_cpu_us(1000U, 1250U, false, 400U), 250U);
    EXPECT_EQ(interval_cpu_us(1000U, 1000U, false, 400U), 0U);
    // reset seen, or the clock went back: the reading since the reset plus RESET_ALLOWANCE_US,
    // at most the wall time
    EXPECT_EQ(interval_cpu_us(1000U, 1250U, true, 400U), 400U);
    EXPECT_EQ(interval_cpu_us(1000U, 50U, true, 10000U), 50U + RESET_ALLOWANCE_US);
    EXPECT_EQ(interval_cpu_us(5000000U, 300U, false, 400U), 400U);
    EXPECT_EQ(interval_cpu_us(5000000U, 300U, false, 10000U), 300U + RESET_ALLOWANCE_US);
    EXPECT_EQ(interval_cpu_us(0U, uint64_t(UINT32_MAX) + 10U, false, 400U), UINT32_MAX);
}

/*
  A flood whose statistics are reset during every pass (the attack of a script opening
  @SYS/threads.txt each pass, or a GCS reading it as fast as it can): each pass runs 200 us of
  CPU in 300 us of wall time, and the waits sleep. The host charges each pass from a reading
  at its start to one at its end; the meter here is the CPU actually used, which the charges
  do not feed: its share stays within the cap.
 */
TEST(RustM4ServeWait, StatisticsResetEveryPassKeepsTheShare)
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
        ServeDutyCapTestAccess::pass_started(cap, uint32_t(elapsed));
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
TEST(RustM4ServeWait, UnflaggedResetWouldUnderCharge)
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
    uint64_t read(uint32_t now, bool &reset)
    {
        reads++;
        return read_thread_cycles(clock, now, reset);
    }
    uint32_t reads = 0;
};

// the bench's binding_bench.lua: 416 synchronous calls back to back once a second, each pass
// charged 41 us (the bench's M4ST.CPU / Req) with a 145 us round trip. The 10 ms bucket carries
// the first 370 calls unpaced (10000 - 200 us of credit at 41 - 14.5 us per call); the 2 ms one
// paused from call 68
TEST(RustM4ServeWait, OneSecondCallBurstIsPacedOnlyPastTheBucket)
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
        ServeDutyCapTestAccess::pass_started(cap, now);
        cap.charge(41U);
        now += 145U;
    }
    EXPECT_EQ(first_pause, 370);
}

// the refusal cliff of one worker call: from RESUME_US of credit, a call of C us is followed by a
// pass forced MAX_PAUSE_US after the call's pass started; it refuses worker bindings when the
// credit is then below -MAX_DEBT_US, i.e. C > RESUME_US + MAX_PAUSE_US / 10 + MAX_DEBT_US (100.4 ms)
TEST(RustM4ServeWait, RefusalCliffOfOneWorkerCall)
{
    for (const uint32_t call_us : { 100000U, 101000U }) {
        ServeDutyCap cap;
        const uint32_t start = 1000;
        EXPECT_EQ(cap.wait_us(start), 0U);
        cap.charge(uint32_t(ServeDutyCap::BURST_US - ServeDutyCap::RESUME_US));
        ASSERT_EQ(cap.credit_us(), int64_t(ServeDutyCap::RESUME_US));
        ServeDutyCapTestAccess::pass_started(cap, start);
        cap.charge(call_us);
        const uint32_t forced = start + uint32_t(ServeDutyCap::MAX_PAUSE_US);
        EXPECT_EQ(cap.wait_us(forced), 0U) << call_us;   // the forced pass is due
        EXPECT_EQ(cap.refuses_slow_bindings(), call_us > 100400U) << call_us;
    }
}

// a pass that a duty-cap pause waited for runs, although the pause's own wake (10 us) is charged
// before it: the host decides before it charges (fails if pass_after_wake() charges first)
TEST(RustM4ServeWait, PassRunsRightAfterAPause)
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

// the host as the M7 runs it: sleep, wake (switch-in plus overhead), a reading where the charge
// asks for one at a pass start, a pass, a reading, overhead, sleep. Readings come from
// FakeThreadStats; returns the CPU charged and the CPU used. reset_kind: a reset in the pass (1)
// or in the sleep (2), 0: none. marks_best: Util::thread_info's fix
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
        uint64_t cpu;
        if (charge.reads_at_pass_start()) {
            cpu = stats.read(now, reset);
            charged += charge.pass_start(cpu, reset, now);
        }
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
TEST(RustM4ServeWait, ChibiOSChargesAllHostCpu)
{
    HostModel host;
    host.iteration(5, 20, 4, 500, 0);   // the first reading only starts the clock
    const uint64_t charged_before = host.charged;
    const uint64_t used_before = host.used;
    for (int i = 0; i < 100; i++) {
        host.iteration(5, 20, 4, 500, 0);
    }
    EXPECT_EQ(host.charged - charged_before, host.used - used_before);
    EXPECT_EQ(host.charge.take_resets(), 0U);
}

// ChibiOS: a pass a doorbell starts reads the thread's statistics once, at its end; that reading
// charges the wait's wake and switch with the pass (fails if the host reads at the pass start too)
TEST(RustM4ServeWait, DoorbellPassChargesOncePerPass)
{
    HostModel host;
    host.iteration(5, 20, 4, 500, 0);
    const uint32_t reads_before = host.stats.reads;
    const uint64_t charged_before = host.charged;
    for (int i = 0; i < 10; i++) {
        host.iteration(5, 20, 4, 500, 0);
    }
    EXPECT_EQ(host.stats.reads - reads_before, 10U);
    EXPECT_EQ(host.charged - charged_before, 10U * (4U + 5U + 20U));
    // SITL, whose waits spin, still reads at both ends so that only the pass is charged
    EXPECT_TRUE(HostCpuCharge { false }.reads_at_pass_start());
}

// SITL charges passes only: the overhead around them is not charged
TEST(RustM4ServeWait, PassOnlyChargeLeavesOutOverhead)
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
TEST(RustM4ServeWait, SelfResetIsCharged)
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
// charged the reading plus RESET_ALLOWANCE_US: never less than the host used, nor the whole sleep
TEST(RustM4ServeWait, OtherThreadResetChargesTheReading)
{
    HostModel host;
    host.iteration(5, 20, 4, 500, 0);
    const uint64_t used_before = host.used;
    for (int i = 0; i < 50; i++) {
        host.iteration(5, 20, 4, 500, 2);
    }
    EXPECT_GE(host.charged, host.used - used_before);
    EXPECT_LE(host.charged, host.used - used_before + 49U * RESET_ALLOWANCE_US);
    EXPECT_EQ(host.charge.take_resets(), 49U);   // the last reset is after the last reading
}

// a GCS @SYS/threads.txt read during the host's 10 ms doorbell sleep: the next reading charges
// the CPU since the reset plus the allowance, not the sleep, so the duty cap keeps its burst and
// the bench's once-a-second call burst still runs its first 360 calls unpaced
TEST(RustM4ServeWait, ResetDuringASleepKeepsTheBurst)
{
    HostModel host;
    host.iteration(5, 20, 4, 10000, 0);
    host.iteration(5, 20, 4, 10000, 2);   // reset in this sleep
    const uint64_t charged_before = host.charged;
    const uint64_t used_before = host.used;
    host.iteration(5, 20, 4, 10000, 0);
    const uint64_t charged = host.charged - charged_before;
    const uint64_t used = host.used - used_before;
    EXPECT_LE(charged, used + RESET_ALLOWANCE_US);
    EXPECT_LT(charged, 1000U);   // not the 10 ms sleep
    ServeDutyCap cap;
    uint32_t now = 0;
    EXPECT_EQ(cap.wait_us(now), 0U);
    cap.charge(uint32_t(charged));
    int first_pause = -1;
    for (int call = 0; call < 416 && first_pause < 0; call++) {
        if (cap.wait_us(now) != 0) {
            first_pause = call;
        }
        cap.charge(41U);
        now += 145U;
    }
    EXPECT_GE(first_pause, 360);
}

// a reset in the middle of a pass while the host is off the CPU for 2 ms (blocked on the slow
// worker, the resetting thread above it): the pass is charged its CPU plus the allowance, not 2 ms
TEST(RustM4ServeWait, ResetMidPassChargesThePassCpu)
{
    FakeThreadStats stats;
    HostCpuCharge charge { true };
    uint32_t now = 1000;
    bool reset;
    stats.switch_in(now);
    (void)charge.pass_start(stats.read(now, reset), reset, now);   // starts the clock
    now += 100;
    stats.switch_out(now);
    now += 500;   // asleep
    stats.switch_in(now);
    now += 5;
    (void)charge.pass_start(stats.read(now, reset), reset, now);
    now += 100;   // the pass up to the hand-off to the worker
    stats.switch_out(now);
    stats.reset_from_other_thread();
    now += 2000;   // off the CPU
    stats.switch_in(now);
    now += 100;   // the rest of the pass
    const uint32_t pass_charge = charge.pass_end(stats.read(now, reset), reset, now);
    EXPECT_TRUE(reset);
    EXPECT_GE(pass_charge, 100U);
    EXPECT_LE(pass_charge, 200U + RESET_ALLOWANCE_US);
}

// a flood whose every pass resets the host's own statistics (the fs:crc32("@SYS/threads.txt")
// attack, had it not been refused): the CPU it really uses stays within the cap
TEST(RustM4ServeWait, SelfResetEveryPassKeepsTheShare)
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
        ServeDutyCapTestAccess::pass_started(cap, host.now);
        const uint64_t before = host.charged;
        // one pass of a varying length, as natural jitter makes it
        host.iteration(3, 150 + (host.now % 97), 2, 0, 1);
        cap.charge(uint32_t(host.charged - before));
    }
    const uint64_t ppm = host.used * 1000000U / (host.now - start);
    EXPECT_LE(ppm, ServeDutyCap::SHARE_PERCENT * 10000U + (300U + ServeDutyCap::BURST_US) * uint64_t(1000000U) / 10000000U);
}

/*
  The worker's CPU is charged to the same duty cap as the host's: a script looping a frame that
  runs 40 ms of CPU on the worker (fs:crc32 of a large file) for every 50 us host hand-off. The
  CPU actually used, host and worker, stays within the cap; were the worker's charge left out,
  the host's hand-offs alone would never pause it
 */
TEST(RustM4ServeWait, WorkerCpuIsCharged)
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
            ServeDutyCapTestAccess::pass_started(cap, now);
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

// refusals in every second for 12 s: a "refused" text at 0, 5 and 10 s only; then quiet: one
// "served again" once 5 s pass without a refusal, and nothing after it
TEST(RustM4ServeWait, RefusalTextsAreRateLimited)
{
    typedef RefusalNotice::Text Text;
    RefusalNotice notice;
    std::vector<uint32_t> refused_at;
    std::vector<uint32_t> served_at;
    for (uint32_t now = 1000; now <= 30000; now += 1000) {
        const uint32_t refused = now <= 12000 ? 14U : 0U;
        const Text text = notice.update(now, refused);
        if (text == Text::Refused) {
            refused_at.push_back(now);
        } else if (text == Text::ServedAgain) {
            served_at.push_back(now);
        }
    }
    EXPECT_EQ(refused_at, (std::vector<uint32_t> { 1000, 6000, 11000 }));
    EXPECT_EQ(served_at, (std::vector<uint32_t> { 17000 }));
}

// a gap shorter than NOTICE_MS between refusals is not "served again"; normal runs say nothing
TEST(RustM4ServeWait, RefusalTextsNeedRefusals)
{
    typedef RefusalNotice::Text Text;
    RefusalNotice quiet;
    for (uint32_t now = 1000; now <= 60000; now += 1000) {
        EXPECT_EQ(quiet.update(now, 0), Text::None) << now;
    }
    RefusalNotice notice;
    EXPECT_EQ(notice.update(1000, 3), Text::Refused);
    EXPECT_EQ(notice.update(2000, 0), Text::None);
    EXPECT_EQ(notice.update(5000, 0), Text::None);
    EXPECT_EQ(notice.update(6000, 2), Text::Refused);   // 5 s after the last text
    EXPECT_EQ(notice.update(10000, 0), Text::None);
    EXPECT_EQ(notice.update(11000, 0), Text::ServedAgain);
    EXPECT_EQ(notice.update(16000, 0), Text::None);
}

// frames as the M4 sends them
struct Frame {
    uint8_t bytes[sizeof(StatsMsg)];
    uint16_t len;
};
static Frame stats_frame()
{
    Frame f {};
    StatsMsg stats {};
    fill_header(stats.header, MsgType::Stats, sizeof(stats), 1);
    memcpy(f.bytes, &stats, sizeof(stats));
    f.len = sizeof(stats);
    return f;
}
static Frame system_frame(SystemMethod method, uint16_t flags)
{
    Frame f {};
    SystemCallHeader call {};
    fill_header(call.header, MsgType::SystemCall, sizeof(call), 2);
    call.method = uint16_t(method);
    call.flags = flags;
    memcpy(f.bytes, &call, sizeof(call));
    f.len = sizeof(call);
    return f;
}
static Frame binding_frame(uint32_t id)
{
    Frame f {};
    f.len = pack_binding_call(f.bytes, sizeof(f.bytes), 3, 0, 0, id, nullptr, 0, 4);
    return f;
}

// one pass of serve_pass(), serve()'s own loop, over a queue of frames, each frame_us long:
// the frames served and how the pass ended
static size_t frames_in_one_pass(const std::vector<Frame> &queue, uint32_t frame_us = 833, PassEnd *end = nullptr)
{
    uint8_t frame[MAX_FRAME];
    size_t next = 0;
    size_t served = 0;
    uint32_t elapsed_us = 0;
    const PassEnd result = serve_pass(frame,
                                      [&](uint16_t &len) {
                                          if (next == queue.size()) {
                                              return -EAGAIN;
                                          }
                                          memcpy(frame, queue[next].bytes, queue[next].len);
                                          len = queue[next].len;
                                          next++;
                                          return 0;
                                      },
                                      [&](uint16_t) {
                                          served++;
                                          elapsed_us += frame_us;
                                      },
                                      [&]() { return elapsed_us; });
    if (end != nullptr) {
        *end = result;
    }
    return served;
}

// a pass ends at MAX_REQUESTS_PER_WAKE frames while none takes it past MAX_SERVE_US, after the
// first binding call that does, and at once on an empty ring or a receive error
TEST(RustM4ServeWait, PassEndsOnTimeOrAtTheFrameLimit)
{
    const std::vector<Frame> calls(MAX_REQUESTS_PER_WAKE + 4U, binding_frame(100));
    for (const uint32_t frame_us : { 0U, 6U, 7U, 8U, 833U }) {
        PassEnd end = PassEnd::Idle;
        const size_t served = frames_in_one_pass(calls, frame_us, &end);
        EXPECT_EQ(end, PassEnd::Busy) << frame_us;
        // the first frame after which more than MAX_SERVE_US have passed ends the pass
        const size_t expected = frame_us * MAX_REQUESTS_PER_WAKE <= MAX_SERVE_US ? MAX_REQUESTS_PER_WAKE :
                                MAX_SERVE_US / frame_us + 1;
        EXPECT_EQ(served, expected) << frame_us;
    }
    uint8_t frame[MAX_FRAME] {};
    size_t served = 0;
    EXPECT_EQ(serve_pass(frame, [](uint16_t &) { return -EAGAIN; }, [&](uint16_t) { served++; },
                         []() { return uint32_t(0); }), PassEnd::Idle);
    EXPECT_EQ(serve_pass(frame, [](uint16_t &) { return -EIO; }, [&](uint16_t) { served++; },
                         []() { return uint32_t(0); }), PassEnd::ReceiveError);
    EXPECT_EQ(served, 0U);
    // frames, then an empty ring: Idle whatever the time
    PassEnd end = PassEnd::Busy;
    EXPECT_EQ(frames_in_one_pass(std::vector<Frame>(3, binding_frame(100)), 0, &end), 3U);
    EXPECT_EQ(end, PassEnd::Idle);
}

// the M4 between module steps: a module's text and state notifies, then its next call; with the
// debt deep a pass is due only every 0.5 s, so the call must be served in this pass
TEST(RustM4ServeWait, NotifiesAheadOfACallDoNotEndThePass)
{
    std::vector<Frame> queue(STATS_ROWS_PER_WINDOW, stats_frame());
    queue.push_back(system_frame(SystemMethod::Text, CALL_FLAG_NOTIFY));
    queue.push_back(system_frame(SystemMethod::ModuleState, CALL_FLAG_NOTIFY));
    queue.push_back(system_frame(SystemMethod::Noop, CALL_FLAG_NOTIFY));
    queue.push_back(binding_frame(100));
    queue.push_back(binding_frame(101));   // the pass ends after the first call
    EXPECT_EQ(frames_in_one_pass(queue), queue.size() - 1);
    // the worst case: a window's Stats rows and one notify per module
    std::vector<Frame> worst(STATS_ROWS_PER_WINDOW, stats_frame());
    for (uint8_t i = 0; i < MAX_MODULES; i++) {
        worst.push_back(system_frame(SystemMethod::Text, CALL_FLAG_NOTIFY));
    }
    worst.push_back(binding_frame(100));
    worst.push_back(binding_frame(101));
    EXPECT_EQ(frames_in_one_pass(worst), worst.size() - 1);
    EXPECT_EQ(worst.size() - 1, size_t(MAX_EXEMPT_PER_PASS) + 1U);
    EXPECT_EQ(size_t(MAX_EXEMPT_PER_PASS), size_t(STATS_ROWS_PER_WINDOW) + MAX_MODULES);
    EXPECT_EQ(size_t(MAX_EXEMPT_PER_PASS), 27U);
}

TEST(RustM4ServeWait, CountersFrameIsExemptFromPassTime)
{
    CountersMsg counters {};
    fill_header(counters.header, MsgType::Counters, sizeof(counters), 1);
    EXPECT_TRUE(frame_exempt_from_pass_time(reinterpret_cast<const uint8_t *>(&counters), sizeof(counters)));
    // served on the host like Stats, never handed to the worker
    EXPECT_TRUE(frame_on_fast_lane(reinterpret_cast<const uint8_t *>(&counters), sizeof(counters),
                                   [](uint32_t) { return false; }));
}

// binding calls, system calls that wait for a reply and frames that do not parse end a pass on
// time; exempt frames beyond MAX_EXEMPT_PER_PASS (a faulty M4) end it too
TEST(RustM4ServeWait, BindingCallsEndThePassOnTime)
{
    const Frame frames[] = {
        binding_frame(100),
        system_frame(SystemMethod::Resolve, 0),
        system_frame(SystemMethod::FsRead, 0),
    };
    for (const Frame &f : frames) {
        uint8_t exempt_in_pass = 0;
        EXPECT_TRUE(frame_ends_pass_on_time(f.bytes, f.len, exempt_in_pass));
        EXPECT_EQ(exempt_in_pass, 0U);
    }
    Frame short_stats = stats_frame();
    short_stats.len = sizeof(MsgHeader) + 4;
    uint8_t exempt_in_pass = 0;
    EXPECT_FALSE(frame_exempt_from_pass_time(short_stats.bytes, short_stats.len));
    EXPECT_TRUE(frame_ends_pass_on_time(short_stats.bytes, short_stats.len, exempt_in_pass));
    EXPECT_TRUE(frame_ends_pass_on_time(nullptr, 0, exempt_in_pass));
    EXPECT_EQ(frames_in_one_pass({ binding_frame(100), binding_frame(101) }), 1U);
    EXPECT_EQ(frames_in_one_pass(std::vector<Frame>(32, stats_frame())), size_t(MAX_EXEMPT_PER_PASS) + 1U);
    EXPECT_EQ(frames_in_one_pass(std::vector<Frame>(32, system_frame(SystemMethod::Noop, CALL_FLAG_NOTIFY))),
              size_t(MAX_EXEMPT_PER_PASS) + 1U);
}

static bool even_ids_fast(uint32_t id)
{
    return (id & 1U) == 0;
}

// the host serves binding calls of the Fast lane, the Noop notify, Stats rows and the M7 -> M4
// types; every other frame goes to the slow worker
TEST(RustM4ServeWait, FastLaneTakesOnlyFastBindingCalls)
{
    const Frame fast = binding_frame(100);
    const Frame slow = binding_frame(101);
    EXPECT_TRUE(frame_on_fast_lane(fast.bytes, fast.len, even_ids_fast));
    EXPECT_FALSE(frame_on_fast_lane(slow.bytes, slow.len, even_ids_fast));
    const Frame stats = stats_frame();
    EXPECT_TRUE(frame_on_fast_lane(stats.bytes, stats.len, even_ids_fast));
    const Frame noop = system_frame(SystemMethod::Noop, CALL_FLAG_NOTIFY);
    EXPECT_TRUE(frame_on_fast_lane(noop.bytes, noop.len, even_ids_fast));
    for (const SystemMethod method : { SystemMethod::Text, SystemMethod::ModuleState, SystemMethod::Resolve,
                                       SystemMethod::FsRead, SystemMethod::Config }) {
        const Frame f = system_frame(method, CALL_FLAG_NOTIFY);
        EXPECT_FALSE(frame_on_fast_lane(f.bytes, f.len, even_ids_fast)) << unsigned(method);
    }
    ReadyMsg ready {};
    fill_header(ready.header, MsgType::Ready, sizeof(ready), 0);
    EXPECT_TRUE(frame_on_fast_lane(reinterpret_cast<const uint8_t *>(&ready), sizeof(ready), even_ids_fast));
    // a frame that does not parse: the worker counts it
    Frame truncated = fast;
    truncated.len = sizeof(BindingCallHeader) - 1;
    EXPECT_FALSE(frame_on_fast_lane(truncated.bytes, truncated.len, even_ids_fast));
    EXPECT_FALSE(frame_on_fast_lane(fast.bytes, uint16_t(fast.len + 1), even_ids_fast));
}

namespace {
struct PostedRing {
    std::vector<bool> posted;   // the queued frames, true for a posted one
    size_t next;                // an aggregate under gnu++11: brace-initialised to 0
    uint32_t served;
};
}

static uint16_t run_batch(PostedRing &ring, uint32_t elapsed_us = 0)
{
    static uint8_t frame[MAX_FRAME];
    return serve_posted_batch(
        frame,
        [&ring](uint16_t &len) {
            if (ring.next == ring.posted.size()) {
                return -EAGAIN;
            }
            frame[0] = ring.posted[ring.next++] ? 1 : 2;
            len = 100;
            return 0;
        },
        [](const uint8_t *f, uint16_t) { return f[0] == 1; },
        [&ring](uint16_t) { ring.served++; },
        [elapsed_us]() { return elapsed_us; });
}

TEST(RustM4ServeWait, PostedBurstIsServedInOneHandoff)
{
    PostedRing ring { { true, true, true, true, false, true } };   // the worker already served one
    EXPECT_EQ(run_batch(ring), 100U);   // the first other frame goes back to the host
    EXPECT_EQ(ring.served, 4U);         // 5 posted frames in one handoff
    EXPECT_EQ(ring.next, 5U);
}

TEST(RustM4ServeWait, PostedBatchStopsAtItsLimit)
{
    PostedRing ring { std::vector<bool>(20, true) };
    EXPECT_EQ(run_batch(ring), 0U);
    EXPECT_EQ(ring.served, uint32_t(POSTED_BATCH_MAX - 1));
}

TEST(RustM4ServeWait, PostedBatchStopsWhenTheRingEmpties)
{
    PostedRing ring { { true, true } };
    EXPECT_EQ(run_batch(ring), 0U);
    EXPECT_EQ(ring.served, 2U);
}

TEST(RustM4ServeWait, PostedBatchStopsOnTime)
{
    PostedRing ring { { true, true } };
    EXPECT_EQ(run_batch(ring, MAX_SERVE_US + 1), 0U);
    EXPECT_EQ(ring.served, 0U);
    EXPECT_EQ(ring.next, 0U);
// doorbell wakes 100 us apart for 5 ms: the housekeeping runs once a millisecond, not after each
// wake (fails without the period)
TEST(RustM4ServeWait, HousekeepingRunsWithinOneMillisecond)
{
    for (const uint32_t start : { 1000U, UINT32_MAX - 2500U }) {
        uint32_t last = start;   // it ran at the start
        uint32_t runs = 0;
        for (uint32_t elapsed = 100; elapsed <= 5000; elapsed += 100) {
            const uint32_t now = start + elapsed;
            if (housekeeping_due(now, last, HOUSEKEEPING_PERIOD_US, false)) {
                runs++;
                last = now;
            }
        }
        EXPECT_GE(runs, 4U) << start;
        EXPECT_LE(runs, 6U) << start;
    }
}

// a wait that timed out on an empty ring runs the housekeeping, however soon after its last run
// (fails if only the period decides)
TEST(RustM4ServeWait, TimeoutWakeRunsHousekeeping)
{
    uint32_t runs = 0;
    const uint32_t last = 5000;
    if (housekeeping_due(last + 10U, last, HOUSEKEEPING_PERIOD_US, true)) {
        runs++;
    }
    EXPECT_EQ(runs, 1U);
    EXPECT_FALSE(housekeeping_due(last + 10U, last, HOUSEKEEPING_PERIOD_US, false));
}

AP_GTEST_MAIN()
