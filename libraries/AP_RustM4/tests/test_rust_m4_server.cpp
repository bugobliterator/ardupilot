#include <AP_gtest.h>
#include <AP_Rust/AP_Rust_config.h>

#if AP_SCRIPTING_RUST_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_Rust/Bindings.h>
#include <AP_Rust/BindingM4Policy_generated.h>
#include <AP_Rust/HostApi.h>
#include <AP_RustM4/AP_RustM4_Server.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace RustM4;

static uint32_t first_with(AP_Rust::M4Lane lane)
{
    for (uint32_t id = 0; id < AP_Rust::M4_BINDING_COUNT; id++) {
        if (AP_Rust::M4_BINDING_POLICY[id].lane == lane) {
            return id;
        }
    }
    return UINT32_MAX;
}

static uint16_t call(uint8_t *frame, uint32_t id, uint16_t in_size, uint16_t out_size, uint16_t module = 0)
{
    static uint8_t input[MAX_CALL_INPUT];
    return pack_binding_call(frame, MAX_FRAME, 42, module, 0, id, input, in_size, out_size);
}

static BindingReplyHeader serve(AP_RustM4_Server &server, const uint8_t *frame, uint16_t len)
{
    static uint8_t reply[MAX_FRAME];
    const uint16_t reply_len = server.serve_binding(frame, len, reply, sizeof(reply), 1000);
    BindingReplyHeader h {};
    const uint8_t *body = nullptr;
    EXPECT_TRUE(parse_binding_reply(reply, reply_len, h, body));
    EXPECT_EQ(h.header.seq, 42U);
    return h;
}

TEST(RustM4Server, StatusCodesMatchThePR)
{
    EXPECT_EQ(STATUS_OK, AP_RUST_STATUS_OK);
    EXPECT_EQ(STATUS_STOPPED, AP_RUST_STATUS_STOPPED);
    EXPECT_EQ(STATUS_INVALID, AP_RUST_STATUS_INVALID);
    EXPECT_EQ(STATUS_NO_MEMORY, AP_RUST_STATUS_NO_MEMORY);
    EXPECT_EQ(STATUS_UNAVAILABLE, AP_RUST_STATUS_UNAVAILABLE);
    EXPECT_EQ(STATUS_HOST_ERROR, AP_RUST_STATUS_HOST_ERROR);
    EXPECT_EQ(STATUS_BUSY, AP_RUST_STATUS_BUSY);
}

TEST(RustM4Server, UnknownAndOutOfRangeIdsAreUnknownMethod)
{
    AP_RustM4_Server server;
    server.reset(1, 1);
    uint8_t frame[MAX_FRAME];
    EXPECT_EQ(serve(server, frame, call(frame, AP_Rust::M4_BINDING_COUNT, 0, 0)).transport,
              uint8_t(ReplyStatus::UnknownMethod));
    const uint32_t gap = first_with(AP_Rust::M4Lane::Unknown);
    ASSERT_NE(gap, UINT32_MAX);
    EXPECT_EQ(serve(server, frame, call(frame, gap, 0, 0)).transport, uint8_t(ReplyStatus::UnknownMethod));
}

TEST(RustM4Server, LocalIdsAreRefusedOnTheM7)
{
    AP_RustM4_Server server;
    server.reset(1, 1);
    uint8_t frame[MAX_FRAME];
    EXPECT_EQ(serve(server, frame, call(frame, first_with(AP_Rust::M4Lane::Local), 4, 4)).transport,
              uint8_t(ReplyStatus::UnknownMethod));
}

TEST(RustM4Server, BadModuleIsMalformed)
{
    AP_RustM4_Server server;
    server.reset(1, 1);
    uint8_t frame[MAX_FRAME];
    const uint16_t len = call(frame, first_with(AP_Rust::M4Lane::Fast), 4, 4, 7);   // module 7: no such module
    EXPECT_EQ(serve(server, frame, len).transport, uint8_t(ReplyStatus::Malformed));
}

// a frame above the limits never reaches an adapter
TEST(RustM4Server, OversizeIsMalformed)
{
    AP_RustM4_Server server;
    server.reset(1, 1);
    static uint8_t frame[MAX_FRAME + 64];
    const uint32_t id = first_with(AP_Rust::M4Lane::Fast);
    // a valid header claiming one byte more input than a frame can carry
    BindingCallHeader h {};
    fill_header(h.header, MsgType::BindingCall, uint16_t(MAX_FRAME + 1), 42);
    h.id = id;
    h.in_size = uint16_t(MAX_CALL_INPUT + 1);
    memcpy(frame, &h, sizeof(h));
    EXPECT_EQ(serve(server, frame, uint16_t(MAX_FRAME + 1)).transport, uint8_t(ReplyStatus::Malformed));
    // output above the reply limit
    const uint16_t len = call(frame, id, 4, 4);
    memcpy(&h, frame, sizeof(h));
    h.out_size = uint16_t(MAX_REPLY_OUTPUT + 1);
    memcpy(frame, &h, sizeof(h));
    EXPECT_EQ(serve(server, frame, len).transport, uint8_t(ReplyStatus::Malformed));
    EXPECT_FALSE(server.last_dispatch_ran());
}

// the adapters do not all check sizes: whatever the frame says, they get 4096 B buffers, and no
// adapter writes past 2048 B. The generated exact sizes exist only for local ids, which the M7
// refuses, so four pairs stand in for them.
TEST(RustM4Server, NoAdapterWritesPastTheFrame)
{
    AP_RustM4_Server server;
    server.reset(1, 1);
    const uint16_t sizes[][2] = { { 0, 0 }, { 4, 4 }, { 64, 64 }, { MAX_CALL_INPUT, MAX_REPLY_OUTPUT } };
    uint32_t served = 0;
    for (uint32_t id = 0; id < AP_Rust::M4_BINDING_COUNT; id++) {
        const auto lane = AP_Rust::M4_BINDING_POLICY[id].lane;
        if (lane == AP_Rust::M4Lane::Unknown || lane == AP_Rust::M4Lane::Local ||
            AP_RustM4_Server::needs_vehicle(id)) {
            continue;
        }
        for (const auto &size : sizes) {
            EXPECT_TRUE(server.dispatch_guarded(id, size[0], size[1]))
                << "adapter " << id << " wrote past 2048 B with sizes " << size[0] << "/" << size[1];
        }
        served++;
    }
    EXPECT_GT(served, 300U);
}

// a short frame's missing input reads as zeros, not as the previous call's bytes
TEST(RustM4Server, ShortInputReadsZeros)
{
    AP_RustM4_Server server;
    server.reset(1, 1);
    server.fill_input_for_test(0xCC);
    uint32_t id = 0;
    while (id < AP_Rust::M4_BINDING_COUNT && (AP_Rust::M4_BINDING_POLICY[id].lane != AP_Rust::M4Lane::Fast ||
                                              AP_RustM4_Server::needs_vehicle(id))) {
        id++;
    }
    ASSERT_LT(id, AP_Rust::M4_BINDING_COUNT);
    uint8_t frame[MAX_FRAME];
    serve(server, frame, call(frame, id, 4, 4));
    EXPECT_TRUE(server.last_dispatch_ran());
    EXPECT_TRUE(server.last_input_tail_zero(4));
}

// a new session ends every slot of the old one, through every PR cleanup
TEST(RustM4Server, EndModuleCallsEveryCleanup)
{
    EXPECT_EQ(AP_RustM4_Server::cleanup_count(), 11U);   // PR Runtime.cpp:549-559
    AP_RustM4_Server server;
    server.run_cleanups_for_test(false);
    server.reset(1, 2);
    EXPECT_EQ(server.ended_modules(), 0U);
    server.reset(2, 1);
    EXPECT_EQ(server.ended_modules(), 2U);
    server.end_module(5);   // no such module: nothing
    EXPECT_EQ(server.ended_modules(), 2U);
}
#endif

AP_GTEST_MAIN()
