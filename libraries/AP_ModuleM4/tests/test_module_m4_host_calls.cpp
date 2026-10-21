#include <errno.h>
#include <AP_gtest.h>
#include <AP_Common/AP_Common.h>
#include <AP_ModuleM4/AP_ModuleM4_HostCalls.h>

using namespace ModuleM4;

struct FakeM7 {
    uint32_t calls;
    uint32_t notifies;
    uint8_t last_request[MAX_FRAME];
    uint8_t answer[MAX_FRAME];
    int32_t answer_len;
    bool hit;
    uint8_t mirror_out[5];
    uint32_t gets, pendings, settles, refreshes;
    uint64_t last_token;
    uint8_t refresh_in[5];
    uint8_t last_notify_flags;
    uint8_t last_notify[MAX_FRAME];
};

static int32_t fake_call(void *platform, const uint8_t *request, uint16_t len, uint8_t *reply, uint16_t cap)
{
    auto &m7 = *static_cast<FakeM7 *>(platform);
    m7.calls++;
    memcpy(m7.last_request, request, len);
    if (m7.answer_len > 0 && uint16_t(m7.answer_len) <= cap) {
        BindingCallHeader call {};
        memcpy(&call, request, sizeof(call));
        reinterpret_cast<MsgHeader *>(m7.answer)->seq = call.header.seq;
        memcpy(reply, m7.answer, m7.answer_len);
    }
    return m7.answer_len;
}
static void fake_notify(void *platform, const uint8_t *frame, uint16_t len)
{
    auto &m7 = *static_cast<FakeM7 *>(platform);
    m7.notifies++;
    BindingCallHeader call {};
    memcpy(&call, frame, len < sizeof(call) ? len : sizeof(call));
    m7.last_notify_flags = call.flags;
    memcpy(m7.last_notify, frame, len);
}
/*
  The call policy from id 5: id 5 is local with the wire of libm.sinf (4 B in, 4 B out), id 20 the mirrored
  get (8 B in, 5 B out), id 21 a handle writer, id 22 a name writer, id 30 posted
 */
static const uint32_t FAKE_POLICY_FIRST = 5;
static const uint8_t fake_local_wires[][2] = { { 12, 4 }, { 4, 4 } };
static uint8_t fake_policy[26];
static const uint8_t *fake_call_policy()
{
    memset(fake_policy, 0, sizeof(fake_policy));
    fake_policy[5 - FAKE_POLICY_FIRST] = 2U << CALL_POLICY_WIRE_SHIFT;
    fake_policy[20 - FAKE_POLICY_FIRST] = uint8_t(MirrorRole::Get) << CALL_POLICY_ROLE_SHIFT;
    fake_policy[21 - FAKE_POLICY_FIRST] = uint8_t(MirrorRole::HandleWriter) << CALL_POLICY_ROLE_SHIFT;
    fake_policy[22 - FAKE_POLICY_FIRST] = uint8_t(MirrorRole::NameWriter) << CALL_POLICY_ROLE_SHIFT;
    fake_policy[30 - FAKE_POLICY_FIRST] = CALL_POLICY_POSTED;
    return fake_policy;
}
// as a generated adapter: writes its fixed 4 B whatever output_size says
static uint32_t local_calls;
static int32_t fake_local(uint32_t, const uint8_t *, uint32_t, uint8_t *output, uint32_t)
{
    local_calls++;
    memset(output, 0xAB, 4);
    return STATUS_OK;
}

static bool fake_mirror_get(void *platform, uint16_t, uint64_t token, uint8_t *out5)
{
    auto &m7 = *static_cast<FakeM7 *>(platform);
    m7.gets++;
    m7.last_token = token;
    if (m7.hit) {
        memcpy(out5, m7.mirror_out, 5);
    }
    return m7.hit;
}
static void fake_mirror_pending(void *platform, uint16_t, uint64_t token)
{
    static_cast<FakeM7 *>(platform)->pendings++;
    static_cast<FakeM7 *>(platform)->last_token = token;
}
static void fake_mirror_settle(void *platform, uint16_t, uint64_t)
{
    static_cast<FakeM7 *>(platform)->settles++;
}
static void fake_mirror_refresh(void *platform, uint16_t, uint64_t token, const uint8_t *out5)
{
    auto &m7 = *static_cast<FakeM7 *>(platform);
    m7.refreshes++;
    m7.last_token = token;
    memcpy(m7.refresh_in, out5, 5);
}

struct Fixture {
    alignas(16) uint8_t stack[256];
    alignas(16) uint8_t rw[256];
    alignas(16) uint8_t heap_arena[256];
    uint8_t privileged[64];
    uint8_t link_request[MAX_FRAME];   // the ModuleLink's buffers
    uint8_t link_reply[MAX_FRAME];
    uint32_t used[1], starts[1];
    FakeM7 m7 {};
    ModuleContext m;
    HostPlatform platform { fake_call, fake_notify, fake_call_policy(), FAKE_POLICY_FIRST, sizeof(fake_policy),
                            fake_local_wires, fake_local, &m7, fake_mirror_get, fake_mirror_pending, fake_mirror_settle,
                            fake_mirror_refresh, nullptr };
    Fixture()
    {
        local_calls = 0;
        m.token = module_token(0);
        m.index = 0;
        m.request = link_request;
        m.reply = link_reply;
        m.ranges.rx = { uintptr_t(privileged), 0 };
        m.ranges.rw = { uintptr_t(rw), sizeof(rw) };
        m.ranges.heap = { uintptr_t(heap_arena), sizeof(heap_arena) };
        m.ranges.stack = { uintptr_t(stack), sizeof(stack) };
        EXPECT_TRUE(m.heap.init(uintptr_t(heap_arena), sizeof(heap_arena), used, starts, 1));
        m.stop.store(false);
    }
    BindingCallArgs args(uint32_t id, const uint8_t *in, uint32_t in_size, uint8_t *out, uint32_t out_size)
    {
        return BindingCallArgs { m.token, id, in, in_size, out, out_size };
    }
};

TEST(ModuleM4HostCalls, BadPointersAreInvalidAndSendNothing)
{
    Fixture f;
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.privileged, 4, f.rw, 4), f.platform), STATUS_INVALID);
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.rw, 4, f.privileged, 4), f.platform), STATUS_INVALID);
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.rw, 8, f.rw + 4, 8), f.platform), STATUS_INVALID);   // overlap
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.rw, 0xFFFFFFFFU, f.stack, 4), f.platform), STATUS_INVALID);
    EXPECT_EQ(f.m7.calls, 0U);
}

TEST(ModuleM4HostCalls, WrongContextTokenIsInvalid)
{
    Fixture f;
    BindingCallArgs a = f.args(9, f.rw, 4, f.stack, 4);
    a.context = f.privileged;
    EXPECT_EQ(host_binding_call(f.m, a, f.platform), STATUS_INVALID);
    EXPECT_EQ(f.m7.calls, 0U);
}

TEST(ModuleM4HostCalls, StopComesFirstAsInThePR)
{
    Fixture f;
    f.m.stop.store(true);
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.privileged, 4, f.rw, 4), f.platform), STATUS_STOPPED);
    EXPECT_EQ(host_binding_call(f.m, f.args(5, f.rw, 4, f.stack, 4), f.platform), STATUS_STOPPED);
    EXPECT_EQ(f.m7.calls, 0U);
}

TEST(ModuleM4HostCalls, LocalIdsNeverReachTheM7)
{
    Fixture f;
    EXPECT_EQ(host_binding_call(f.m, f.args(5, f.rw, 4, f.stack, 4), f.platform), STATUS_OK);
    EXPECT_EQ(f.stack[0], 0xAB);
    EXPECT_EQ(f.m7.calls, 0U);
}

// containment: a local adapter writes its fixed size, so a zero or short size (which ModuleRanges
// accepts at any address when zero) must never reach it
TEST(ModuleM4HostCalls, LocalIdsNeedTheirExactSizes)
{
    Fixture f;
    memset(f.privileged, 0x11, sizeof(f.privileged));
    EXPECT_EQ(host_binding_call(f.m, f.args(5, f.rw, 4, f.privileged, 0), f.platform), STATUS_INVALID);
    EXPECT_EQ(host_binding_call(f.m, f.args(5, f.privileged, 0, f.stack, 4), f.platform), STATUS_INVALID);
    EXPECT_EQ(host_binding_call(f.m, f.args(5, f.rw, 4, f.stack, 2), f.platform), STATUS_INVALID);
    EXPECT_EQ(host_binding_call(f.m, f.args(5, f.rw, 8, f.stack, 4), f.platform), STATUS_INVALID);
    EXPECT_EQ(local_calls, 0U);
    for (uint8_t byte : f.privileged) {
        ASSERT_EQ(byte, 0x11);
    }
    EXPECT_EQ(f.m7.calls, 0U);
}

// ids below and above the policy table, and one whose slot wraps, are plain calls to the M7
TEST(ModuleM4HostCalls, IdsOutsideThePolicyAreForwarded)
{
    Fixture f;
    f.m7.answer_len = pack_binding_reply(f.m7.answer, sizeof(f.m7.answer), 0, ReplyStatus::Ok, STATUS_OK, nullptr, 0, 0,
                                         0);
    const uint32_t ids[] = { 4U, FAKE_POLICY_FIRST + sizeof(fake_policy), 0xFFFFFFFFU };
    for (uint32_t id : ids) {
        EXPECT_EQ(host_binding_call(f.m, f.args(id, f.rw, 4, f.stack, 0), f.platform), STATUS_OK) << id;
        BindingCallHeader sent {};
        ASSERT_TRUE(parse_binding_call(f.m7.last_request, sizeof(BindingCallHeader) + 4, sent));
        EXPECT_EQ(sent.id, id);
        EXPECT_EQ(sent.flags & CALL_FLAG_NOTIFY, 0);
    }
    EXPECT_EQ(f.m7.calls, 3U);
    EXPECT_EQ(f.m7.notifies, 0U);
    EXPECT_EQ(local_calls, 0U);
}

TEST(ModuleM4HostCalls, LaunchTokenIsNeverNull)
{
    for (uint16_t index = 0; index < 8; index++) {
        EXPECT_NE(module_token(index), nullptr);
    }
    Fixture f;
    EXPECT_EQ(host_binding_call(f.m, f.args(5, f.rw, 4, f.stack, 4), f.platform), STATUS_OK);
}

TEST(ModuleM4HostCalls, RemoteCallCopiesOnlyAMatchingOkReply)
{
    Fixture f;
    const uint8_t output[4] = { 1, 2, 3, 4 };
    f.m7.answer_len = pack_binding_reply(f.m7.answer, sizeof(f.m7.answer), 0, ReplyStatus::Ok, STATUS_OK, output, 4, 0, 0);
    memset(f.stack, 0, 4);
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.rw, 4, f.stack, 4), f.platform), STATUS_OK);
    EXPECT_EQ(memcmp(f.stack, output, 4), 0);
    BindingCallHeader sent {};
    ASSERT_TRUE(parse_binding_call(f.m7.last_request, sizeof(BindingCallHeader) + 4, sent));
    EXPECT_EQ(sent.id, 9U);
    // the M7 answers 4 bytes where 8 were asked for: nothing is copied
    memset(f.stack, 0x55, 8);
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.rw, 4, f.stack, 8), f.platform), STATUS_HOST_ERROR);
    EXPECT_EQ(f.stack[0], 0x55);
    // no reply at all (the service's timeout): host error
    f.m7.answer_len = -110;
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.rw, 4, f.stack, 4), f.platform), STATUS_HOST_ERROR);
}

TEST(ModuleM4HostCalls, TextChecksAndStop)
{
    Fixture f;
    EXPECT_EQ(host_text(f.m, 8, f.rw, 4, f.platform), STATUS_INVALID);          // severity above 7
    EXPECT_EQ(host_text(f.m, 6, f.rw, 513, f.platform), STATUS_INVALID);        // above the PR's 512
    EXPECT_EQ(host_text(f.m, 6, f.privileged, 4, f.platform), STATUS_INVALID);
    EXPECT_EQ(host_text(f.m, 6, f.rw, 4, f.platform), STATUS_OK);
    EXPECT_EQ(f.m7.notifies, 1U);
    f.m.stop.store(true);
    EXPECT_EQ(host_text(f.m, 6, f.rw, 4, f.platform), STATUS_STOPPED);
}

TEST(ModuleM4HostCalls, DeallocationOfAForeignPointerFails)
{
    Fixture f;
    uint8_t *p = host_allocate(f.m, 32, 8, f.platform);
    ASSERT_NE(p, nullptr);
    EXPECT_FALSE(host_deallocate(f.m, p + 16));
    EXPECT_TRUE(host_deallocate(f.m, p));
    EXPECT_FALSE(host_deallocate(f.m, p));
    EXPECT_TRUE(host_deallocate(f.m, nullptr));
}

static ModuleStateNotify last_state(const FakeM7 &m7)
{
    ModuleStateNotify notify;
    memcpy(&notify, m7.last_notify, sizeof(notify));
    return notify;
}

// the PR's heap expansion report: the M4 heap cannot expand, so a failed request reports the
// initial heap the module needs, each time that grows
TEST(ModuleM4HostCalls, FailedAllocationReportsTheHeapNeeded)
{
    Fixture f;   // 256 B heap
    ASSERT_NE(host_allocate(f.m, 200, 8, f.platform), nullptr);
    EXPECT_EQ(f.m7.notifies, 0U);
    EXPECT_EQ(host_allocate(f.m, 100, 8, f.platform), nullptr);
    ASSERT_EQ(f.m7.notifies, 1U);
    ModuleStateNotify notify = last_state(f.m7);
    EXPECT_EQ(notify.call.method, uint16_t(SystemMethod::ModuleState));
    EXPECT_EQ(notify.call.flags, CALL_FLAG_NOTIFY);
    EXPECT_EQ(notify.module, 0U);
    EXPECT_EQ(notify.state, uint8_t(ModuleStateKind::HeapNeeded));
    EXPECT_EQ(notify.heap_size, 256U + 112U);
    // a smaller need is not reported again
    EXPECT_EQ(host_allocate(f.m, 50, 8, f.platform), nullptr);
    EXPECT_EQ(f.m7.notifies, 1U);
    // a larger one is, alignment padding included
    EXPECT_EQ(host_allocate(f.m, 1000, 64, f.platform), nullptr);
    ASSERT_EQ(f.m7.notifies, 2U);
    EXPECT_EQ(last_state(f.m7).heap_size, 256U + 1008U + 48U);
    // invalid requests are not a heap need
    EXPECT_EQ(host_allocate(f.m, 0, 8, f.platform), nullptr);
    EXPECT_EQ(host_allocate(f.m, 4096, 3, f.platform), nullptr);
    EXPECT_EQ(host_allocate(f.m, 4096, 8192, f.platform), nullptr);
    EXPECT_EQ(f.m7.notifies, 2U);
    // a stopping module's destructors are not reported
    f.m.stop.store(true);
    EXPECT_EQ(host_allocate(f.m, 4096, 8, f.platform), nullptr);
    EXPECT_EQ(f.m7.notifies, 2U);
}

TEST(ModuleM4HostCalls, PanicRecordsAndReportsOnce)
{
    Fixture f;
    static const char text[] = "index out of bounds at path.rs:212:9";
    memcpy(f.rw, text, sizeof(text));   // the message lies in the module's RW, as a Rust literal would in RX
    host_panic_record(f.m, f.rw, sizeof(text) - 1, f.platform);
    EXPECT_STREQ(f.m.panic_text, text);
    EXPECT_EQ(f.m.state.load(), uint8_t(ModuleStateKind::Panicked));
    EXPECT_EQ(f.m7.notifies, 1U);
}

TEST(ModuleM4HostCalls, MirroredGetHitSendsNothing)
{
    Fixture f;
    f.m7.hit = true;
    const uint8_t answer[5] = { 1, 0, 0, 0x60, 0x40 };   // 3.5f
    memcpy(f.m7.mirror_out, answer, 5);
    const uint64_t token = 0x20001000ULL;
    memcpy(f.rw, &token, 8);
    EXPECT_EQ(host_binding_call(f.m, f.args(20, f.rw, 8, f.stack, 5), f.platform), STATUS_OK);
    EXPECT_EQ(memcmp(f.stack, answer, 5), 0);
    EXPECT_EQ(f.m7.calls, 0U);
    EXPECT_EQ(f.m7.last_token, token);
}

TEST(ModuleM4HostCalls, MirroredGetMissIsForwarded)
{
    Fixture f;
    f.m7.hit = false;
    f.m7.answer_len = pack_binding_reply(f.m7.answer, MAX_FRAME, 0, ReplyStatus::Ok, STATUS_INVALID, nullptr, 0, 0, 0);
    EXPECT_EQ(host_binding_call(f.m, f.args(20, f.rw, 8, f.stack, 5), f.platform), STATUS_INVALID);
    EXPECT_EQ(f.m7.gets, 1U);
    EXPECT_EQ(f.m7.calls, 1U);
    EXPECT_EQ(f.m7.refreshes, 0U);   // INVALID: nothing to refresh
}

TEST(ModuleM4HostCalls, ForwardedGetAnsweredOkRefreshesTheMirror)
{
    Fixture f;
    f.m7.hit = false;
    const uint64_t token = 0x20001000ULL;
    memcpy(f.rw, &token, 8);
    const uint8_t answer[5] = { 1, 0, 0, 0x80, 0x40 };   // 4.0f
    f.m7.answer_len = pack_binding_reply(f.m7.answer, MAX_FRAME, 0, ReplyStatus::Ok, STATUS_OK, answer, 5, 0, 0);
    EXPECT_EQ(host_binding_call(f.m, f.args(20, f.rw, 8, f.stack, 5), f.platform), STATUS_OK);
    EXPECT_EQ(memcmp(f.stack, answer, 5), 0);
    EXPECT_EQ(f.m7.calls, 1U);
    EXPECT_EQ(f.m7.refreshes, 1U);
    EXPECT_EQ(f.m7.last_token, token);
    EXPECT_EQ(memcmp(f.m7.refresh_in, answer, 5), 0);
    // a failed forward refreshes nothing
    f.m7.answer_len = -ETIMEDOUT;
    IGNORE_RETURN(host_binding_call(f.m, f.args(20, f.rw, 8, f.stack, 5), f.platform));
    EXPECT_EQ(f.m7.refreshes, 1U);
}

TEST(ModuleM4HostCalls, MirroredGetWrongSizeIsForwarded)
{
    Fixture f;
    f.m7.hit = true;
    f.m7.answer_len = pack_binding_reply(f.m7.answer, MAX_FRAME, 0, ReplyStatus::Ok, STATUS_OK, f.rw, 4, 0, 0);
    IGNORE_RETURN(host_binding_call(f.m, f.args(20, f.rw, 4, f.stack, 4), f.platform));
    EXPECT_EQ(f.m7.gets, 0U);
    EXPECT_EQ(f.m7.calls, 1U);
}

TEST(ModuleM4HostCalls, StopAndBadPointersComeBeforeTheMirror)
{
    Fixture f;
    f.m7.hit = true;
    EXPECT_EQ(host_binding_call(f.m, f.args(20, f.privileged, 8, f.stack, 5), f.platform), STATUS_INVALID);
    f.m.stop.store(true);
    EXPECT_EQ(host_binding_call(f.m, f.args(20, f.rw, 8, f.stack, 5), f.platform), STATUS_STOPPED);
    EXPECT_EQ(f.m7.gets, 0U);
}

TEST(ModuleM4HostCalls, HandleWriterMarksPendingAndSettles)
{
    Fixture f;
    const uint64_t token = 0x20001000ULL;
    memcpy(f.rw, &token, 8);
    const uint8_t ok = 1;
    f.m7.answer_len = pack_binding_reply(f.m7.answer, MAX_FRAME, 0, ReplyStatus::Ok, STATUS_OK, &ok, 1, 0, 0);
    EXPECT_EQ(host_binding_call(f.m, f.args(21, f.rw, 12, f.stack, 1), f.platform), STATUS_OK);
    EXPECT_EQ(f.m7.pendings, 1U);
    EXPECT_EQ(f.m7.settles, 1U);
    EXPECT_EQ(f.m7.last_token, token);
    // a failed call settles too: its entry must not stay PENDING
    f.m7.answer_len = -ETIMEDOUT;
    IGNORE_RETURN(host_binding_call(f.m, f.args(21, f.rw, 12, f.stack, 1), f.platform));
    EXPECT_EQ(f.m7.settles, 2U);
    // a name writer has no token: no PENDING
    IGNORE_RETURN(host_binding_call(f.m, f.args(22, f.rw, 21, f.stack, 1), f.platform));
    EXPECT_EQ(f.m7.pendings, 2U);
}


TEST(ModuleM4HostCalls, PostedIdReturnsOkAfterTheNotify)
{
    Fixture f;
    EXPECT_EQ(host_binding_call(f.m, f.args(30, f.rw, 16, nullptr, 0), f.platform), STATUS_OK);
    EXPECT_EQ(f.m7.notifies, 1U);
    EXPECT_EQ(f.m7.calls, 0U);
    EXPECT_EQ(f.m7.last_notify_flags, CALL_FLAG_NOTIFY);
}

TEST(ModuleM4HostCalls, PostedIdWithAnOutputGoesTheCallPath)
{
    Fixture f;
    f.m7.answer_len = pack_binding_reply(f.m7.answer, MAX_FRAME, 0, ReplyStatus::Ok, STATUS_OK, f.rw, 4, 0, 0);
    IGNORE_RETURN(host_binding_call(f.m, f.args(30, f.rw, 16, f.stack, 4), f.platform));
    EXPECT_EQ(f.m7.notifies, 0U);
    EXPECT_EQ(f.m7.calls, 1U);
}

TEST(ModuleM4HostCalls, StopComesBeforePosting)
{
    Fixture f;
    f.m.stop.store(true);
    EXPECT_EQ(host_binding_call(f.m, f.args(30, f.rw, 16, nullptr, 0), f.platform), STATUS_STOPPED);
    EXPECT_EQ(f.m7.notifies, 0U);
}

// the probe's counter: each read moves it on by 10
static uint32_t probe_now;
static uint32_t probe_clock()
{
    probe_now += 10;
    return probe_now;
}

// a call the M4 service sends and the M7 answers: the service's probe stamps
static M4CallProbe *served_probe;
static int32_t probed_call(void *platform, const uint8_t *request, uint16_t len, uint8_t *reply, uint16_t cap)
{
    if (served_probe != nullptr) {
        served_probe->current.seq = 77;
        served_probe->current.sent_us = probe_clock();
        served_probe->current.reply_seen_us = probe_clock();
        served_probe->current.flags |= PROBE_M4_SLEPT;
    }
    return fake_call(platform, request, len, reply, cap);
}

TEST(ModuleM4HostCalls, ProbedCallSendsTheLastCompletedCallInItsTrailer)
{
    Fixture f;
    M4CallProbe probe {};
    probe.clock_us = probe_clock;
    probe.stamp_cycles = 48;
    probe.enabled.store(true);
    probe_now = 1000;
    served_probe = &probe;
    f.platform.call = probed_call;
    f.platform.probe = &probe;
    const uint8_t output[4] = { 1, 2, 3, 4 };
    f.m7.answer_len = pack_binding_reply(f.m7.answer, sizeof(f.m7.answer), 0, ReplyStatus::Ok, STATUS_OK, output, 4, 0, 0);
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.rw, 4, f.stack, 4), f.platform), STATUS_OK);
    EXPECT_EQ(memcmp(f.stack, output, 4), 0);
    BindingCallHeader sent {};
    const uint16_t len = sizeof(BindingCallHeader) + 4 + sizeof(CallProbeTrailer);
    ASSERT_TRUE(parse_binding_call(f.m7.last_request, len, sent));
    CallProbeTrailer trailer;
    ASSERT_TRUE(binding_call_trailer(f.m7.last_request, len, sent, trailer));
    EXPECT_EQ(trailer.svc_entry_us, 1010U);
    EXPECT_EQ(trailer.stamp_cycles, 48U);
    EXPECT_EQ(trailer.prev_seq, 0U);   // the first probed call completes nothing
    // its stamps: packed 1020, sent 1030, reply seen 1040, returned 1050
    EXPECT_EQ(probe.last.seq, 77U);
    EXPECT_EQ(probe.last.packed_us, 1020U);
    EXPECT_EQ(probe.last.sent_us, 1030U);
    EXPECT_EQ(probe.last.reply_seen_us, 1040U);
    EXPECT_EQ(probe.last.return_us, 1050U);
    EXPECT_EQ(probe.last.flags, PROBE_M4_SLEPT);
    EXPECT_EQ(probe.last.stamps, PROBE_STAMP_PACKED);
    EXPECT_FALSE(probe.in_call);
    EXPECT_TRUE(probe.memset_measured);
    // the next call carries them
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.rw, 4, f.stack, 4), f.platform), STATUS_OK);
    ASSERT_TRUE(parse_binding_call(f.m7.last_request, len, sent));
    ASSERT_TRUE(binding_call_trailer(f.m7.last_request, len, sent, trailer));
    EXPECT_EQ(trailer.prev_seq, 77U);
    EXPECT_EQ(trailer.prev_sent_us, 1030U);
    EXPECT_EQ(trailer.prev_reply_seen_us, 1040U);
    EXPECT_EQ(trailer.prev_return_us, 1050U);
    EXPECT_EQ(trailer.prev_flags, PROBE_M4_SLEPT);
    EXPECT_EQ(trailer.prev_stamps, PROBE_STAMP_PACKED);
    EXPECT_EQ(trailer.prev_packed_us, 1020U);
    // a call without its reply completes nothing
    f.m7.answer_len = -110;
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.rw, 4, f.stack, 4), f.platform), STATUS_HOST_ERROR);
    EXPECT_EQ(probe.last.seq, 0U);
    served_probe = nullptr;
}

TEST(ModuleM4HostCalls, ProbeOffOrNoRoomSendsThePlainFrame)
{
    Fixture f;
    M4CallProbe probe {};
    probe.clock_us = probe_clock;
    f.platform.probe = &probe;
    const uint8_t output[4] = { 1, 2, 3, 4 };
    f.m7.answer_len = pack_binding_reply(f.m7.answer, sizeof(f.m7.answer), 0, ReplyStatus::Ok, STATUS_OK, output, 4, 0, 0);
    // not enabled by this session's Config
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.rw, 4, f.stack, 4), f.platform), STATUS_OK);
    BindingCallHeader sent {};
    ASSERT_TRUE(parse_binding_call(f.m7.last_request, sizeof(BindingCallHeader) + 4, sent));
    EXPECT_EQ(sent.flags & CALL_FLAG_PROBE, 0);
    // no counter to read
    probe.enabled.store(true);
    probe.clock_us = nullptr;
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.rw, 4, f.stack, 4), f.platform), STATUS_OK);
    ASSERT_TRUE(parse_binding_call(f.m7.last_request, sizeof(BindingCallHeader) + 4, sent));
    EXPECT_EQ(sent.flags & CALL_FLAG_PROBE, 0);
    // local ids are never probed
    probe.clock_us = probe_clock;
    const uint32_t calls = f.m7.calls;
    EXPECT_EQ(host_binding_call(f.m, f.args(5, f.rw, 4, f.stack, 4), f.platform), STATUS_OK);
    EXPECT_EQ(f.m7.calls, calls);
}

// a cycle counter that moves on by 3 per call
static uint32_t fake_cycles_now;
static uint32_t fake_cycles()
{
    fake_cycles_now += 3;
    return fake_cycles_now;
}

TEST(ModuleM4HostCalls, MemsetCostIsMeasuredOnceBeforeTheFirstProbedCall)
{
    Fixture f;
    M4CallProbe probe {};
    probe.clock_us = probe_clock;
    probe.cycles = fake_cycles;
    probe.enabled.store(true);
    f.platform.probe = &probe;
    const uint8_t output[4] = { 1, 2, 3, 4 };
    f.m7.answer_len = pack_binding_reply(f.m7.answer, sizeof(f.m7.answer), 0, ReplyStatus::Ok, STATUS_OK, output, 4, 0, 0);
    memset(f.link_reply, 0x5A, sizeof(f.link_reply));
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.rw, 4, f.stack, 4), f.platform), STATUS_OK);
    EXPECT_TRUE(probe.memset_measured);
    // two reads of the counter 3 apart, over 64000 and 4096000 bytes: 0 hundredths per byte
    EXPECT_EQ(probe.memset_small_centicycles, 0);
    EXPECT_EQ(probe.memset_large_centicycles, 0);
    EXPECT_EQ(centicycles_per_byte(128000, 64000), 200);
    EXPECT_EQ(centicycles_per_byte(4096000, 4096000), 100);
    EXPECT_EQ(centicycles_per_byte(0xFFFFFFFFU, 1), 0xFFFF);
    // the reply buffer was cleared by the measurement, then received the answer
    EXPECT_EQ(memcmp(f.stack, output, 4), 0);
    const uint32_t reads = fake_cycles_now;
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.rw, 4, f.stack, 4), f.platform), STATUS_OK);
    // not again: only the call's cycle reads, five before the pack, then the platform call's return, the
    // output copy and host_binding_call()'s return
    EXPECT_EQ(fake_cycles_now - reads, 8U * 3U);
    // the trailer reports it
    BindingCallHeader sent {};
    const uint16_t len = sizeof(BindingCallHeader) + 4 + sizeof(CallProbeTrailer);
    ASSERT_TRUE(parse_binding_call(f.m7.last_request, len, sent));
    CallProbeTrailer trailer;
    ASSERT_TRUE(binding_call_trailer(f.m7.last_request, len, sent, trailer));
    EXPECT_EQ(trailer.memset_small_centicycles, probe.memset_small_centicycles);
}

TEST(ModuleM4HostCalls, PrePackCyclesReachTheNextTrailer)
{
    Fixture f;
    M4CallProbe probe {};
    probe.clock_us = probe_clock;
    probe.cycles = fake_cycles;
    probe.enabled.store(true);
    probe.memset_measured = true;
    served_probe = &probe;
    f.platform.call = probed_call;
    f.platform.probe = &probe;
    const uint8_t output[4] = { 1, 2, 3, 4 };
    f.m7.answer_len = pack_binding_reply(f.m7.answer, sizeof(f.m7.answer), 0, ReplyStatus::Ok, STATUS_OK, output, 4, 0, 0);
    // the SVC handler's stamp, 40 cycles before the host call reads the counter
    fake_cycles_now = 1000;
    probe.svc_cycles = 963;
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.rw, 4, f.stack, 4), f.platform), STATUS_OK);
    EXPECT_EQ(probe.svc_cycles, 0U);   // used once
    // each read 3 on from the one before
    const auto part = [&probe](M4CyclePart p) { return probe.last.cycles[uint8_t(p)]; };
    const auto bit = [](M4CyclePart p) { return uint16_t(1U << uint8_t(p)); };
    EXPECT_EQ(part(M4CyclePart::SvcEntry), 40);
    EXPECT_EQ(part(M4CyclePart::Checks), 3);
    EXPECT_EQ(part(M4CyclePart::BeginProbe), 3);
    EXPECT_EQ(part(M4CyclePart::Pack), 3);
    EXPECT_EQ(part(M4CyclePart::RetCopy), 3);
    // no drain stamped the copy to the link: no PortTail; SvcReturn waits for the port
    const uint16_t host_parts = bit(M4CyclePart::SvcEntry) | bit(M4CyclePart::Checks) |
                                bit(M4CyclePart::BeginProbe) | bit(M4CyclePart::Pack) | bit(M4CyclePart::RetCopy);
    EXPECT_EQ(probe.last.cycles_taken, host_parts);
    EXPECT_NE(probe.return_cycles, 0U);
    record_svc_return(probe);
    EXPECT_EQ(part(M4CyclePart::SvcReturn), 3);
    EXPECT_EQ(probe.last.cycles_taken, host_parts | bit(M4CyclePart::SvcReturn));
    EXPECT_EQ(probe.return_cycles, 0U);
    record_svc_return(probe);   // once per call
    EXPECT_EQ(part(M4CyclePart::SvcReturn), 3);
    // the next call carries them; without an SVC stamp its own SvcEntry is not taken. A drain's copy to
    // the link 30 cycles before the platform call returns is its PortTail
    f.platform.call = [](void *platform, const uint8_t *request, uint16_t len, uint8_t *reply, uint16_t cap) {
        const int32_t result = probed_call(platform, request, len, reply, cap);
        served_probe->current.drain_copied_cycles = fake_cycles_now - 27;
        return result;
    };
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.rw, 4, f.stack, 4), f.platform), STATUS_OK);
    BindingCallHeader sent {};
    const uint16_t len = sizeof(BindingCallHeader) + 4 + sizeof(CallProbeTrailer);
    ASSERT_TRUE(parse_binding_call(f.m7.last_request, len, sent));
    CallProbeTrailer trailer;
    ASSERT_TRUE(binding_call_trailer(f.m7.last_request, len, sent, trailer));
    EXPECT_EQ(trailer.prev_cycles[uint8_t(M4CyclePart::SvcEntry)], 40);
    EXPECT_EQ(trailer.prev_cycles[uint8_t(M4CyclePart::Checks)], 3);
    EXPECT_EQ(trailer.prev_cycles[uint8_t(M4CyclePart::Pack)], 3);
    EXPECT_EQ(trailer.prev_cycles[uint8_t(M4CyclePart::SvcReturn)], 3);
    EXPECT_EQ(trailer.prev_cycles_taken, host_parts | bit(M4CyclePart::SvcReturn));
    EXPECT_EQ(probe.last.cycles_taken & bit(M4CyclePart::SvcEntry), 0);
    EXPECT_EQ(part(M4CyclePart::PortTail), 30);
    EXPECT_NE(probe.last.cycles_taken & bit(M4CyclePart::PortTail), 0);
    // a counter far behind saturates
    EXPECT_EQ(cycles_between(0, 0x10000U), 0xFFFF);
    EXPECT_EQ(cycles_between(0xFFFFFFFFU, 2), 3);
    served_probe = nullptr;
}

// the M7's answer to a Ping
static void answer_ping(FakeM7 &m7, ReplyStatus transport)
{
    SystemReplyHeader reply {};
    fill_header(reply.header, MsgType::SystemReply, sizeof(reply), 0);
    reply.method = uint16_t(SystemMethod::Ping);
    reply.transport = uint8_t(transport);
    memcpy(m7.answer, &reply, sizeof(reply));
    m7.answer_len = sizeof(reply);
}

TEST(ModuleM4HostCalls, PingIsForwardedAsASystemCall)
{
    Fixture f;
    answer_ping(f.m7, ReplyStatus::Ok);
    EXPECT_EQ(host_binding_call(f.m, f.args(PING_BINDING_ID, nullptr, 0, nullptr, 0), f.platform), STATUS_OK);
    EXPECT_EQ(f.m7.calls, 1U);
    SystemCallHeader sent {};
    memcpy(&sent, f.m7.last_request, sizeof(sent));
    EXPECT_EQ(sent.header.type, uint8_t(MsgType::SystemCall));
    EXPECT_EQ(sent.header.length, sizeof(SystemCallHeader));
    EXPECT_EQ(sent.method, uint16_t(SystemMethod::Ping));
    EXPECT_EQ(sent.flags, 0);
    // an M7 that does not know Ping, a lost reply, a reply of another kind
    answer_ping(f.m7, ReplyStatus::UnknownMethod);
    EXPECT_EQ(host_binding_call(f.m, f.args(PING_BINDING_ID, nullptr, 0, nullptr, 0), f.platform), STATUS_INVALID);
    f.m7.answer_len = -ETIMEDOUT;
    EXPECT_EQ(host_binding_call(f.m, f.args(PING_BINDING_ID, nullptr, 0, nullptr, 0), f.platform), STATUS_HOST_ERROR);
    f.m7.answer_len = pack_binding_reply(f.m7.answer, sizeof(f.m7.answer), 0, ReplyStatus::Ok, STATUS_OK, nullptr, 0, 0, 0);
    EXPECT_EQ(host_binding_call(f.m, f.args(PING_BINDING_ID, nullptr, 0, nullptr, 0), f.platform), STATUS_HOST_ERROR);
    // no buffers: anything else is refused before the M7
    const uint32_t calls = f.m7.calls;
    EXPECT_EQ(host_binding_call(f.m, f.args(PING_BINDING_ID, f.rw, 4, nullptr, 0), f.platform), STATUS_INVALID);
    EXPECT_EQ(host_binding_call(f.m, f.args(PING_BINDING_ID, nullptr, 0, f.stack, 4), f.platform), STATUS_INVALID);
    EXPECT_EQ(f.m7.calls, calls);
    // a stop and a foreign context come first, as for a binding
    BindingCallArgs foreign = f.args(PING_BINDING_ID, nullptr, 0, nullptr, 0);
    foreign.context = module_token(3);
    EXPECT_EQ(host_binding_call(f.m, foreign, f.platform), STATUS_INVALID);
    f.m.stop.store(true);
    EXPECT_EQ(host_binding_call(f.m, f.args(PING_BINDING_ID, nullptr, 0, nullptr, 0), f.platform), STATUS_STOPPED);
    EXPECT_EQ(f.m7.calls, calls);
}

TEST(ModuleM4HostCalls, ProbedPingCarriesTheTrailer)
{
    Fixture f;
    M4CallProbe probe {};
    probe.clock_us = probe_clock;
    probe.cycles = fake_cycles;
    probe.enabled.store(true);
    probe.memset_measured = true;
    probe_now = 1000;
    served_probe = &probe;
    f.platform.call = probed_call;
    f.platform.probe = &probe;
    answer_ping(f.m7, ReplyStatus::Ok);
    EXPECT_EQ(host_binding_call(f.m, f.args(PING_BINDING_ID, nullptr, 0, nullptr, 0), f.platform), STATUS_OK);
    const uint16_t len = sizeof(SystemCallHeader) + sizeof(CallProbeTrailer);
    SystemCallHeader sent {};
    memcpy(&sent, f.m7.last_request, sizeof(sent));
    EXPECT_EQ(sent.header.length, len);
    CallProbeTrailer trailer;
    ASSERT_TRUE(ping_call_trailer(f.m7.last_request, len, sent, trailer));
    EXPECT_EQ(trailer.svc_entry_us, 1010U);
    EXPECT_EQ(trailer.prev_seq, 0U);
    // completed like a binding call: the next probed call carries it
    EXPECT_EQ(probe.last.seq, 77U);
    EXPECT_EQ(probe.last.cycles[uint8_t(M4CyclePart::Pack)], 3);
    EXPECT_FALSE(probe.in_call);
    const uint8_t output[4] = { 1, 2, 3, 4 };
    f.m7.answer_len = pack_binding_reply(f.m7.answer, sizeof(f.m7.answer), 0, ReplyStatus::Ok, STATUS_OK, output, 4, 0, 0);
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.rw, 4, f.stack, 4), f.platform), STATUS_OK);
    BindingCallHeader call {};
    const uint16_t call_len = sizeof(BindingCallHeader) + 4 + sizeof(CallProbeTrailer);
    ASSERT_TRUE(parse_binding_call(f.m7.last_request, call_len, call));
    ASSERT_TRUE(binding_call_trailer(f.m7.last_request, call_len, call, trailer));
    EXPECT_EQ(trailer.prev_seq, 77U);
    served_probe = nullptr;
}

AP_GTEST_MAIN()
