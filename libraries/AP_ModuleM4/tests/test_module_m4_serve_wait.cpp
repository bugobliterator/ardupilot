#include <AP_gtest.h>
#include <vector>
#include <AP_Common/AP_Common.h>
#include <AP_ModuleM4/AP_ModuleM4_ServeWait.h>

using namespace ModuleM4;

// a flooding M4: the budget ran out and frames remain, so the host pauses
TEST(ModuleM4ServeWait, BudgetOutWithFramesLeftBacksOff)
{
    EXPECT_EQ(host_wait_after(true, false, false), HostWait::Backoff);
}

// a pass preempted past the budget with nothing left: the next call's doorbell is heard at once
TEST(ModuleM4ServeWait, BudgetOutWithEmptyRingWaitsOnDoorbell)
{
    EXPECT_EQ(host_wait_after(true, false, true), HostWait::Doorbell);
}

TEST(ModuleM4ServeWait, IdleWaitsOnDoorbell)
{
    EXPECT_EQ(host_wait_after(false, false, true), HostWait::Doorbell);
    EXPECT_EQ(host_wait_after(false, false, false), HostWait::Doorbell);
}

// a ring that cannot be served waits for the timeout, whatever it holds
TEST(ModuleM4ServeWait, StuckWaitsForTimeout)
{
    EXPECT_EQ(host_wait_after(false, true, true), HostWait::Timeout);
    EXPECT_EQ(host_wait_after(false, true, false), HostWait::Timeout);
    EXPECT_EQ(host_wait_after(true, true, true), HostWait::Timeout);
    EXPECT_EQ(host_wait_after(true, true, false), HostWait::Timeout);
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
    void reset_from_this_thread() { clock.cumulative = 0; clock.best = UINT32_MAX; }
    uint32_t read(uint32_t now, bool &reset) { return read_thread_cycles(clock, now, reset); }
};

// a reading is the finished slices plus the running one; best stays 0 between readings, so no
// switch is mistaken for a reset (fails if read_thread_cycles() stops clearing best)
TEST(ModuleM4ServeWait, ThreadClockReadsFinishedAndRunningSlices)
{
    FakeThreadStats stats;
    bool reset;
    stats.switch_in(1000);
    EXPECT_EQ(stats.read(1005, reset), 5U);
    EXPECT_TRUE(reset);   // creation's chTMObjectInit()
    stats.switch_out(1020);
    stats.switch_in(1500);
    EXPECT_EQ(stats.read(1510, reset), 30U);
    EXPECT_FALSE(reset);
    stats.switch_out(1530);
    stats.switch_in(2000);
    EXPECT_EQ(stats.read(2001, reset), 51U);
    EXPECT_FALSE(reset);
}

// a reset from another thread while this one is off the CPU, or by this thread itself, is flagged
// once, and the reading is the CPU since it
TEST(ModuleM4ServeWait, ThreadClockResetIsFlaggedOnce)
{
    FakeThreadStats stats;
    bool reset;
    stats.switch_in(1000);
    IGNORE_RETURN(stats.read(1000, reset));
    stats.switch_out(1100);
    stats.reset_from_other_thread();
    stats.switch_in(2000);
    EXPECT_EQ(stats.read(2010, reset), 10U);
    EXPECT_TRUE(reset);
    EXPECT_EQ(stats.read(2020, reset), 20U);
    EXPECT_FALSE(reset);
    stats.reset_from_this_thread();
    EXPECT_EQ(stats.read(2030, reset), 30U);   // the running slice from 2000: cumulative is 0
    EXPECT_TRUE(reset);
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
TEST(ModuleM4ServeWait, PassEndsOnTimeOrAtTheFrameLimit)
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

// the M4 between module steps: a module's text and state notifies, then its next call, served in
// this pass rather than after the host's backoff
TEST(ModuleM4ServeWait, NotifiesAheadOfACallDoNotEndThePass)
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

TEST(ModuleM4ServeWait, CountersFrameIsExemptFromPassTime)
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
TEST(ModuleM4ServeWait, BindingCallsEndThePassOnTime)
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

// the host serves binding calls of the Fast lane, the Noop notify, the Ping, Stats rows and the M7 -> M4
// types; every other frame goes to the slow worker
TEST(ModuleM4ServeWait, FastLaneTakesOnlyFastBindingCalls)
{
    const Frame fast = binding_frame(100);
    const Frame slow = binding_frame(101);
    EXPECT_TRUE(frame_on_fast_lane(fast.bytes, fast.len, even_ids_fast));
    EXPECT_FALSE(frame_on_fast_lane(slow.bytes, slow.len, even_ids_fast));
    const Frame stats = stats_frame();
    EXPECT_TRUE(frame_on_fast_lane(stats.bytes, stats.len, even_ids_fast));
    const Frame noop = system_frame(SystemMethod::Noop, CALL_FLAG_NOTIFY);
    EXPECT_TRUE(frame_on_fast_lane(noop.bytes, noop.len, even_ids_fast));
    const Frame ping = system_frame(SystemMethod::Ping, 0);
    EXPECT_TRUE(frame_on_fast_lane(ping.bytes, ping.len, even_ids_fast));
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

TEST(ModuleM4ServeWait, PostedBurstIsServedInOneHandoff)
{
    PostedRing ring { { true, true, true, true, false, true } };   // the worker already served one
    EXPECT_EQ(run_batch(ring), 100U);   // the first other frame goes back to the host
    EXPECT_EQ(ring.served, 4U);         // 5 posted frames in one handoff
    EXPECT_EQ(ring.next, 5U);
}

TEST(ModuleM4ServeWait, PostedBatchStopsAtItsLimit)
{
    PostedRing ring { std::vector<bool>(20, true) };
    EXPECT_EQ(run_batch(ring), 0U);
    EXPECT_EQ(ring.served, uint32_t(POSTED_BATCH_MAX - 1));
}

TEST(ModuleM4ServeWait, PostedBatchStopsWhenTheRingEmpties)
{
    PostedRing ring { { true, true } };
    EXPECT_EQ(run_batch(ring), 0U);
    EXPECT_EQ(ring.served, 2U);
}

TEST(ModuleM4ServeWait, PostedBatchStopsOnTime)
{
    PostedRing ring { { true, true } };
    EXPECT_EQ(run_batch(ring, MAX_SERVE_US + 1), 0U);
    EXPECT_EQ(ring.served, 0U);
    EXPECT_EQ(ring.next, 0U);
}

// doorbell wakes 100 us apart for 5 ms: the housekeeping runs once a millisecond, not after each
// wake (fails without the period)
TEST(ModuleM4ServeWait, HousekeepingRunsWithinOneMillisecond)
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
TEST(ModuleM4ServeWait, TimeoutWakeRunsHousekeeping)
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
