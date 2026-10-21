#include <AP_gtest.h>

#include <AP_Rust/CallLatency.h>

using namespace AP_Rust;

static int32_t stage(const int32_t (&stages)[CALL_STAGES], CallStage which)
{
    return stages[uint8_t(which)];
}

TEST(RustCallLatency, StagesFromOneCounter)
{
    const CallStamps stamps { 100, 103, 110, 112, 130, 131, 135, 140, 150, 160, 190, 200 };
    int32_t stages[CALL_STAGES];
    call_stages(stamps, stages);
    EXPECT_EQ(stage(stages, CallStage::Pre), 3);
    EXPECT_EQ(stage(stages, CallStage::Send), 7);
    EXPECT_EQ(stage(stages, CallStage::Receive), 20);
    EXPECT_EQ(stage(stages, CallStage::Pass), 18);
    EXPECT_EQ(stage(stages, CallStage::Handoff), 1);
    EXPECT_EQ(stage(stages, CallStage::Check), 4);
    EXPECT_EQ(stage(stages, CallStage::Lock), 5);
    EXPECT_EQ(stage(stages, CallStage::Adapter), 10);
    EXPECT_EQ(stage(stages, CallStage::Reply), 10);
    EXPECT_EQ(stage(stages, CallStage::Back), 30);
    EXPECT_EQ(stage(stages, CallStage::Return), 10);
    EXPECT_EQ(stage(stages, CallStage::Total), 100);
    // every stage but Pass adds up to Total
    int32_t sum = 0;
    for (uint8_t i = 0; i < CALL_STAGES; i++) {
        if (CallStage(i) != CallStage::Pass && CallStage(i) != CallStage::Total) {
            sum += stages[i];
        }
    }
    EXPECT_EQ(sum, stage(stages, CallStage::Total));
}

TEST(RustCallLatency, StagesAcrossTheCounterWrap)
{
    CallStamps stamps { 0xFFFFFFF0U, 0xFFFFFFF8U, 0x00000004U, 0x00000004U, 0x00000003U, 0x00000003U, 0x00000003U,
                        0x00000003U, 0x00000003U, 0x00000003U, 0x00000003U, 0x00000010U };
    int32_t stages[CALL_STAGES];
    call_stages(stamps, stages);
    EXPECT_EQ(stage(stages, CallStage::Send), 12);
    // the M7 took the frame a microsecond before the M4 read the counter after its send
    EXPECT_EQ(stage(stages, CallStage::Receive), -1);
    EXPECT_EQ(stage(stages, CallStage::Total), 0x20);
}

TEST(RustCallLatency, InProcessStampsLeaveTheTransportStagesZero)
{
    int32_t stages[CALL_STAGES];
    call_stages(in_process_stamps(1000, 1001, 2501, 2503, 2504), stages);
    for (CallStage zero : { CallStage::Pre, CallStage::Send, CallStage::Receive, CallStage::Pass, CallStage::Handoff,
                            CallStage::Reply, CallStage::Back }) {
        EXPECT_EQ(stage(stages, zero), 0);
    }
    EXPECT_EQ(stage(stages, CallStage::Check), 1);
    EXPECT_EQ(stage(stages, CallStage::Lock), 1500);
    EXPECT_EQ(stage(stages, CallStage::Adapter), 2);
    EXPECT_EQ(stage(stages, CallStage::Return), 1);
    EXPECT_EQ(stage(stages, CallStage::Total), 1504);
}

static CallTrace trace_of(uint32_t seq, uint16_t id, int32_t total)
{
    CallTrace trace {};
    trace.seq = seq;
    trace.id = id;
    trace.stage_us[uint8_t(CallStage::Total)] = total;
    return trace;
}

TEST(RustCallLatency, RingFlushesEachCallOnceOldestFirst)
{
    CallTrace storage[4];
    CallTraceRing<4> ring(storage);
    CallTrace out;
    EXPECT_FALSE(ring.next_unflushed(out));
    ring.push(trace_of(1, 7, 10));
    ring.push(trace_of(2, 7, 20));
    ASSERT_TRUE(ring.next_unflushed(out));
    EXPECT_EQ(out.seq, 1U);
    ring.push(trace_of(3, 7, 30));
    ASSERT_TRUE(ring.next_unflushed(out));
    EXPECT_EQ(out.seq, 2U);
    ASSERT_TRUE(ring.next_unflushed(out));
    EXPECT_EQ(out.seq, 3U);
    EXPECT_FALSE(ring.next_unflushed(out));
    EXPECT_EQ(ring.skipped(), 0U);
    EXPECT_EQ(ring.count(), 3);
}

TEST(RustCallLatency, RingCountsCallsOverwrittenBeforeTheirFlush)
{
    CallTrace storage[4];
    CallTraceRing<4> ring(storage);
    for (uint32_t seq = 1; seq <= 10; seq++) {
        ring.push(trace_of(seq, 7, int32_t(seq)));
    }
    EXPECT_EQ(ring.count(), 4);
    EXPECT_EQ(ring.at(0).seq, 7U);
    EXPECT_EQ(ring.at(3).seq, 10U);
    CallTrace out;
    ASSERT_TRUE(ring.next_unflushed(out));
    EXPECT_EQ(out.seq, 7U);
    EXPECT_EQ(ring.skipped(), 6U);
    EXPECT_EQ(ring.written(), 10U);
}

TEST(RustCallLatency, NearestRankPercentiles)
{
    int32_t values[100];
    for (int32_t i = 0; i < 100; i++) {
        values[i] = 100 - i;   // descending: the sort is exercised
    }
    sort_values(values, 100);
    for (int32_t i = 0; i < 100; i++) {
        ASSERT_EQ(values[i], i + 1);
    }
    EXPECT_EQ(sorted_percentile(values, 100, 50), 50);
    EXPECT_EQ(sorted_percentile(values, 100, 99), 99);
    EXPECT_EQ(sorted_percentile(values, 100, 100), 100);
    EXPECT_EQ(sorted_percentile(values, 1, 99), 1);
    EXPECT_EQ(sorted_percentile(values, 0, 50), 0);
    const int32_t three[] = { 5, 9, 70 };
    // ceil(3 * 0.5) = 2nd, ceil(3 * 0.99) = 3rd
    EXPECT_EQ(sorted_percentile(three, 3, 50), 9);
    EXPECT_EQ(sorted_percentile(three, 3, 99), 70);
}

TEST(RustCallLatency, SortHandlesALargeWindowWithRepeats)
{
    static int32_t values[512];
    for (uint16_t i = 0; i < 512; i++) {
        values[i] = int32_t((i * 7919U) % 97U) - 40;
    }
    sort_values(values, 512);
    for (uint16_t i = 1; i < 512; i++) {
        ASSERT_LE(values[i - 1], values[i]);
    }
}

TEST(RustCallLatency, SummaryPerBindingOverTheWindow)
{
    CallTrace storage[8];
    CallTraceRing<8> ring(storage);
    int32_t scratch[8];
    // id 3: totals 10..60; id 4: one call of 1000
    for (uint32_t i = 1; i <= 6; i++) {
        ring.push(trace_of(i, 3, int32_t(i * 10)));
    }
    ring.push(trace_of(7, 4, 1000));
    StageSummary summary = summarize_stage(ring, 3, CallStage::Total, scratch);
    EXPECT_EQ(summary.count, 6);
    EXPECT_EQ(summary.p50_us, 30);
    EXPECT_EQ(summary.p99_us, 60);
    EXPECT_EQ(summary.max_us, 60);
    summary = summarize_stage(ring, ANY_ID, CallStage::Total, scratch);
    EXPECT_EQ(summary.count, 7);
    EXPECT_EQ(summary.max_us, 1000);
    summary = summarize_stage(ring, 9, CallStage::Total, scratch);
    EXPECT_EQ(summary.count, 0);
    EXPECT_EQ(summary.max_us, 0);
    // two more calls fill the window of 8 and push the oldest call of id 3 out
    ring.push(trace_of(8, 4, 5));
    ring.push(trace_of(9, 4, 6));
    summary = summarize_stage(ring, 3, CallStage::Total, scratch);
    EXPECT_EQ(summary.count, 5);
    EXPECT_EQ(summary.p50_us, 40);
    EXPECT_EQ(summary.max_us, 60);
}

TEST(RustCallLatency, PendingCallCompletesFromTheNextTrailer)
{
    PendingCall pending;
    CallStamps stamps { 100, 103, 0, 112, 130, 131, 135, 140, 150, 160, 0, 0 };
    CallTrace partial = trace_of(41, 242, 0);
    partial.flags = uint16_t(CallLane::Fast) | CALL_LOCK_TAKEN;
    CallTrace evicted;
    EXPECT_FALSE(pending.hold(partial, stamps, evicted));
    CallTrace done;
    EXPECT_FALSE(pending.complete(M4Completion { 40, 110, 190, 200, CALL_M4_SLEPT }, done));   // another call
    ASSERT_TRUE(pending.complete(M4Completion { 41, 110, 190, 200, CALL_M4_SLEPT }, done));
    EXPECT_EQ(done.seq, 41U);
    EXPECT_EQ(done.flags, uint16_t(CallLane::Fast) | CALL_LOCK_TAKEN | CALL_M4_SLEPT);
    EXPECT_EQ(stage(done.stage_us, CallStage::Send), 7);
    EXPECT_EQ(stage(done.stage_us, CallStage::Back), 30);
    EXPECT_EQ(stage(done.stage_us, CallStage::Total), 100);
    EXPECT_FALSE(pending.valid());
    EXPECT_FALSE(pending.complete(M4Completion { 41, 110, 190, 200, 0 }, done));   // once only
}

TEST(RustCallLatency, PendingCallWithoutCompletionIsRecordedMissing)
{
    PendingCall pending;
    CallStamps stamps { 100, 103, 0, 112, 130, 131, 135, 140, 150, 160, 0, 0 };
    CallTrace evicted;
    EXPECT_FALSE(pending.hold(trace_of(41, 242, 0), stamps, evicted));
    ASSERT_TRUE(pending.hold(trace_of(42, 242, 0), stamps, evicted));
    EXPECT_EQ(evicted.seq, 41U);
    EXPECT_NE(evicted.flags & CALL_M4_MISSING, 0);
    EXPECT_EQ(stage(evicted.stage_us, CallStage::Send), 0);
    EXPECT_EQ(stage(evicted.stage_us, CallStage::Receive), 27);   // from pre_send
    EXPECT_EQ(stage(evicted.stage_us, CallStage::Back), 0);
    EXPECT_EQ(stage(evicted.stage_us, CallStage::Return), 0);
    EXPECT_EQ(stage(evicted.stage_us, CallStage::Total), 0);
    EXPECT_EQ(stage(evicted.stage_us, CallStage::Adapter), 10);
    CallTrace taken;
    ASSERT_TRUE(pending.take_missing(taken));
    EXPECT_EQ(taken.seq, 42U);
    EXPECT_FALSE(pending.take_missing(taken));
}

AP_GTEST_MAIN()
