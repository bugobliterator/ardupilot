#include <errno.h>
#include <AP_gtest.h>
#include <AP_Common/AP_Common.h>
#include <AP_RustM4/AP_RustM4_HostCalls.h>

using namespace RustM4;

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
}
// id 5 is local with the wire of libm.sinf: 4 B in, 4 B out
static bool fake_local_wire(uint32_t id, uint32_t &input_size, uint32_t &output_size)
{
    input_size = 4;
    output_size = 4;
    return id == 5;
}
// as a generated adapter: writes its fixed 4 B whatever output_size says
static uint32_t local_calls;
static int32_t fake_local(uint32_t, const uint8_t *, uint32_t, uint8_t *output, uint32_t)
{
    local_calls++;
    memset(output, 0xAB, 4);
    return STATUS_OK;
}

// id 20 is the mirrored get (8 B in, 5 B out), id 21 a handle writer, id 22 a name writer
static uint8_t fake_mirror_role(uint32_t id)
{
    return id == 20 ? uint8_t(MirrorRole::Get) : id == 21 ? uint8_t(MirrorRole::HandleWriter) :
           id == 22 ? uint8_t(MirrorRole::NameWriter) : uint8_t(MirrorRole::None);
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
// id 30 is posted
static bool fake_is_posted(uint32_t id)
{
    return id == 30;
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
    HostPlatform platform { fake_call, fake_notify, fake_local_wire, fake_local, &m7, fake_mirror_role, fake_mirror_get,
                            fake_mirror_pending, fake_mirror_settle, fake_mirror_refresh, fake_is_posted };
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

TEST(RustM4HostCalls, BadPointersAreInvalidAndSendNothing)
{
    Fixture f;
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.privileged, 4, f.rw, 4), f.platform), STATUS_INVALID);
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.rw, 4, f.privileged, 4), f.platform), STATUS_INVALID);
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.rw, 8, f.rw + 4, 8), f.platform), STATUS_INVALID);   // overlap
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.rw, 0xFFFFFFFFU, f.stack, 4), f.platform), STATUS_INVALID);
    EXPECT_EQ(f.m7.calls, 0U);
}

TEST(RustM4HostCalls, WrongContextTokenIsInvalid)
{
    Fixture f;
    BindingCallArgs a = f.args(9, f.rw, 4, f.stack, 4);
    a.context = f.privileged;
    EXPECT_EQ(host_binding_call(f.m, a, f.platform), STATUS_INVALID);
    EXPECT_EQ(f.m7.calls, 0U);
}

TEST(RustM4HostCalls, StopComesFirstAsInThePR)
{
    Fixture f;
    f.m.stop.store(true);
    EXPECT_EQ(host_binding_call(f.m, f.args(9, f.privileged, 4, f.rw, 4), f.platform), STATUS_STOPPED);
    EXPECT_EQ(host_binding_call(f.m, f.args(5, f.rw, 4, f.stack, 4), f.platform), STATUS_STOPPED);
    EXPECT_EQ(f.m7.calls, 0U);
}

TEST(RustM4HostCalls, LocalIdsNeverReachTheM7)
{
    Fixture f;
    EXPECT_EQ(host_binding_call(f.m, f.args(5, f.rw, 4, f.stack, 4), f.platform), STATUS_OK);
    EXPECT_EQ(f.stack[0], 0xAB);
    EXPECT_EQ(f.m7.calls, 0U);
}

// containment: a local adapter writes its fixed size, so a zero or short size (which ModuleRanges
// accepts at any address when zero) must never reach it
TEST(RustM4HostCalls, LocalIdsNeedTheirExactSizes)
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

TEST(RustM4HostCalls, LaunchTokenIsNeverNull)
{
    for (uint16_t index = 0; index < 8; index++) {
        EXPECT_NE(module_token(index), nullptr);
    }
    Fixture f;
    EXPECT_EQ(host_binding_call(f.m, f.args(5, f.rw, 4, f.stack, 4), f.platform), STATUS_OK);
}

TEST(RustM4HostCalls, RemoteCallCopiesOnlyAMatchingOkReply)
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

TEST(RustM4HostCalls, TextChecksAndStop)
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

TEST(RustM4HostCalls, DeallocationOfAForeignPointerFails)
{
    Fixture f;
    uint8_t *p = host_allocate(f.m, 32, 8);
    ASSERT_NE(p, nullptr);
    EXPECT_FALSE(host_deallocate(f.m, p + 16));
    EXPECT_TRUE(host_deallocate(f.m, p));
    EXPECT_FALSE(host_deallocate(f.m, p));
    EXPECT_TRUE(host_deallocate(f.m, nullptr));
}

TEST(RustM4HostCalls, PanicRecordsAndReportsOnce)
{
    Fixture f;
    static const char text[] = "index out of bounds at path.rs:212:9";
    memcpy(f.rw, text, sizeof(text));   // the message lies in the module's RW, as a Rust literal would in RX
    host_panic_record(f.m, f.rw, sizeof(text) - 1, f.platform);
    EXPECT_STREQ(f.m.panic_text, text);
    EXPECT_EQ(f.m.state.load(), uint8_t(ModuleStateKind::Panicked));
    EXPECT_EQ(f.m7.notifies, 1U);
}

TEST(RustM4HostCalls, MirroredGetHitSendsNothing)
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

TEST(RustM4HostCalls, MirroredGetMissIsForwarded)
{
    Fixture f;
    f.m7.hit = false;
    f.m7.answer_len = pack_binding_reply(f.m7.answer, MAX_FRAME, 0, ReplyStatus::Ok, STATUS_INVALID, nullptr, 0, 0, 0);
    EXPECT_EQ(host_binding_call(f.m, f.args(20, f.rw, 8, f.stack, 5), f.platform), STATUS_INVALID);
    EXPECT_EQ(f.m7.gets, 1U);
    EXPECT_EQ(f.m7.calls, 1U);
    EXPECT_EQ(f.m7.refreshes, 0U);   // INVALID: nothing to refresh
}

TEST(RustM4HostCalls, ForwardedGetAnsweredOkRefreshesTheMirror)
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

TEST(RustM4HostCalls, MirroredGetWrongSizeIsForwarded)
{
    Fixture f;
    f.m7.hit = true;
    f.m7.answer_len = pack_binding_reply(f.m7.answer, MAX_FRAME, 0, ReplyStatus::Ok, STATUS_OK, f.rw, 4, 0, 0);
    IGNORE_RETURN(host_binding_call(f.m, f.args(20, f.rw, 4, f.stack, 4), f.platform));
    EXPECT_EQ(f.m7.gets, 0U);
    EXPECT_EQ(f.m7.calls, 1U);
}

TEST(RustM4HostCalls, StopAndBadPointersComeBeforeTheMirror)
{
    Fixture f;
    f.m7.hit = true;
    EXPECT_EQ(host_binding_call(f.m, f.args(20, f.privileged, 8, f.stack, 5), f.platform), STATUS_INVALID);
    f.m.stop.store(true);
    EXPECT_EQ(host_binding_call(f.m, f.args(20, f.rw, 8, f.stack, 5), f.platform), STATUS_STOPPED);
    EXPECT_EQ(f.m7.gets, 0U);
}

TEST(RustM4HostCalls, HandleWriterMarksPendingAndSettles)
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


TEST(RustM4HostCalls, PostedIdReturnsOkAfterTheNotify)
{
    Fixture f;
    EXPECT_EQ(host_binding_call(f.m, f.args(30, f.rw, 16, nullptr, 0), f.platform), STATUS_OK);
    EXPECT_EQ(f.m7.notifies, 1U);
    EXPECT_EQ(f.m7.calls, 0U);
    EXPECT_EQ(f.m7.last_notify_flags, CALL_FLAG_NOTIFY);
}

TEST(RustM4HostCalls, PostedIdWithAnOutputGoesTheCallPath)
{
    Fixture f;
    f.m7.answer_len = pack_binding_reply(f.m7.answer, MAX_FRAME, 0, ReplyStatus::Ok, STATUS_OK, f.rw, 4, 0, 0);
    IGNORE_RETURN(host_binding_call(f.m, f.args(30, f.rw, 16, f.stack, 4), f.platform));
    EXPECT_EQ(f.m7.notifies, 0U);
    EXPECT_EQ(f.m7.calls, 1U);
}

TEST(RustM4HostCalls, StopComesBeforePosting)
{
    Fixture f;
    f.m.stop.store(true);
    EXPECT_EQ(host_binding_call(f.m, f.args(30, f.rw, 16, nullptr, 0), f.platform), STATUS_STOPPED);
    EXPECT_EQ(f.m7.notifies, 0U);
}

AP_GTEST_MAIN()
