#include <AP_gtest.h>
#include <AP_ModuleM4/AP_ModuleM4_HostCalls.h>

using namespace ModuleM4;

struct FakeM7 {
    uint32_t calls;
    uint32_t notifies;
    uint8_t last_request[MAX_FRAME];
    uint8_t answer[MAX_FRAME];
    int32_t answer_len;
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
static void fake_notify(void *platform, const uint8_t *, uint16_t) { static_cast<FakeM7 *>(platform)->notifies++; }
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
    HostPlatform platform { fake_call, fake_notify, fake_local_wire, fake_local, &m7 };
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
    uint8_t *p = host_allocate(f.m, 32, 8);
    ASSERT_NE(p, nullptr);
    EXPECT_FALSE(host_deallocate(f.m, p + 16));
    EXPECT_TRUE(host_deallocate(f.m, p));
    EXPECT_FALSE(host_deallocate(f.m, p));
    EXPECT_TRUE(host_deallocate(f.m, nullptr));
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

AP_GTEST_MAIN()
