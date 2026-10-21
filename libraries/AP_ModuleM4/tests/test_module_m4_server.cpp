#include <AP_gtest.h>
#include <AP_Module/AP_Module_config.h>

#if AP_MODULE_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_Module/Bindings.h>
#include <AP_Module/BindingM4Policy_generated.h>
#include <AP_Module/HostApi.h>
#include <AP_ModuleM4/AP_ModuleM4_Server.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace ModuleM4;

static uint32_t first_with(AP_Module::M4Lane lane)
{
    for (uint32_t id = 0; id < AP_Module::M4_BINDING_COUNT; id++) {
        if (AP_Module::M4_BINDING_POLICY[id].lane == lane) {
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

static BindingReplyHeader serve(AP_ModuleM4_Server &server, const uint8_t *frame, uint16_t len)
{
    static uint8_t reply[MAX_FRAME];
    const uint16_t reply_len = server.serve_binding(frame, len, reply, sizeof(reply), 1000);
    BindingReplyHeader h {};
    const uint8_t *body = nullptr;
    EXPECT_TRUE(parse_binding_reply(reply, reply_len, h, body));
    EXPECT_EQ(h.header.seq, 42U);
    return h;
}

TEST(ModuleM4Server, StatusCodesMatchThePR)
{
    EXPECT_EQ(STATUS_OK, AP_MODULE_STATUS_OK);
    EXPECT_EQ(STATUS_STOPPED, AP_MODULE_STATUS_STOPPED);
    EXPECT_EQ(STATUS_INVALID, AP_MODULE_STATUS_INVALID);
    EXPECT_EQ(STATUS_NO_MEMORY, AP_MODULE_STATUS_NO_MEMORY);
    EXPECT_EQ(STATUS_UNAVAILABLE, AP_MODULE_STATUS_UNAVAILABLE);
    EXPECT_EQ(STATUS_HOST_ERROR, AP_MODULE_STATUS_HOST_ERROR);
    EXPECT_EQ(STATUS_BUSY, AP_MODULE_STATUS_BUSY);
}

TEST(ModuleM4Server, UnknownAndOutOfRangeIdsAreUnknownMethod)
{
    AP_ModuleM4_Server server;
    server.reset(1, 1);
    uint8_t frame[MAX_FRAME];
    EXPECT_EQ(serve(server, frame, call(frame, AP_Module::M4_BINDING_COUNT, 0, 0)).transport,
              uint8_t(ReplyStatus::UnknownMethod));
    const uint32_t gap = first_with(AP_Module::M4Lane::Unknown);
    ASSERT_NE(gap, UINT32_MAX);
    EXPECT_EQ(serve(server, frame, call(frame, gap, 0, 0)).transport, uint8_t(ReplyStatus::UnknownMethod));
}

TEST(ModuleM4Server, LocalIdsAreRefusedOnTheM7)
{
    AP_ModuleM4_Server server;
    server.reset(1, 1);
    uint8_t frame[MAX_FRAME];
    EXPECT_EQ(serve(server, frame, call(frame, first_with(AP_Module::M4Lane::Local), 4, 4)).transport,
              uint8_t(ReplyStatus::UnknownMethod));
}

TEST(ModuleM4Server, BadModuleIsMalformed)
{
    AP_ModuleM4_Server server;
    server.reset(1, 1);
    uint8_t frame[MAX_FRAME];
    const uint16_t len = call(frame, first_with(AP_Module::M4Lane::Fast), 4, 4, 7);   // module 7: no such module
    EXPECT_EQ(serve(server, frame, len).transport, uint8_t(ReplyStatus::Malformed));
}

// a frame above the limits never reaches an adapter
TEST(ModuleM4Server, OversizeIsMalformed)
{
    AP_ModuleM4_Server server;
    server.reset(1, 1);
    static uint8_t frame[MAX_FRAME + 64];
    const uint32_t id = first_with(AP_Module::M4Lane::Fast);
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
TEST(ModuleM4Server, NoAdapterWritesPastTheFrame)
{
    AP_ModuleM4_Server server;
    server.reset(1, 1);
    const uint16_t sizes[][2] = { { 0, 0 }, { 4, 4 }, { 64, 64 }, { MAX_CALL_INPUT, MAX_REPLY_OUTPUT } };
    uint32_t served = 0;
    for (uint32_t id = 0; id < AP_Module::M4_BINDING_COUNT; id++) {
        const auto lane = AP_Module::M4_BINDING_POLICY[id].lane;
        if (lane == AP_Module::M4Lane::Unknown || lane == AP_Module::M4Lane::Local ||
            AP_ModuleM4_Server::needs_vehicle(id)) {
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
TEST(ModuleM4Server, ShortInputReadsZeros)
{
    AP_ModuleM4_Server server;
    server.reset(1, 1);
    server.fill_input_for_test(0xCC);
    uint32_t id = 0;
    while (id < AP_Module::M4_BINDING_COUNT && (AP_Module::M4_BINDING_POLICY[id].lane != AP_Module::M4Lane::Fast ||
                                              AP_ModuleM4_Server::needs_vehicle(id))) {
        id++;
    }
    ASSERT_LT(id, AP_Module::M4_BINDING_COUNT);
    uint8_t frame[MAX_FRAME];
    serve(server, frame, call(frame, id, 4, 4));
    EXPECT_TRUE(server.last_dispatch_ran());
    EXPECT_TRUE(server.last_input_tail_zero(4));
}

// a new session ends every slot of the old one, through every PR cleanup
TEST(ModuleM4Server, EndModuleCallsEveryCleanup)
{
    EXPECT_EQ(AP_ModuleM4_Server::cleanup_count(), 11U);   // PR Runtime.cpp:549-559
    AP_ModuleM4_Server server;
    server.run_cleanups_for_test(false);
    server.reset(1, 2);
    EXPECT_EQ(server.ended_modules(), 0U);
    server.reset(2, 1);
    EXPECT_EQ(server.ended_modules(), 2U);
    server.end_module(5);   // no such module: nothing
    EXPECT_EQ(server.ended_modules(), 2U);
}

// a Parameter.* call with the module's handle address in input[0..8) and, from 8, its argument
static int32_t parameter_call(AP_ModuleM4_Server &server, AP_Module::BindingId id, uint64_t token, const void *argument,
                              uint16_t argument_size, uint16_t in_size, uint16_t out_size, uint8_t *out = nullptr,
                              uint16_t module = 0)
{
    uint8_t input[64] {};
    memcpy(input, &token, sizeof(token));
    memcpy(input + 8, argument, argument_size);
    uint8_t frame[MAX_FRAME];
    static uint8_t reply[MAX_FRAME];
    const uint16_t len = pack_binding_call(frame, MAX_FRAME, 42, module, 0, uint32_t(id), input, in_size, out_size);
    const uint16_t reply_len = server.serve_binding(frame, len, reply, sizeof(reply), 1000);
    BindingReplyHeader h {};
    const uint8_t *body = nullptr;
    EXPECT_TRUE(parse_binding_reply(reply, reply_len, h, body));
    EXPECT_EQ(h.transport, uint8_t(ReplyStatus::Ok));
    if (out != nullptr && body != nullptr) {
        memcpy(out, body, h.out_size);
    }
    return h.status;
}

static int32_t parameter_init(AP_ModuleM4_Server &server, uint64_t token, uint16_t module = 0)
{
    char name[17] = "NO_SUCH_PARAM";
    return parameter_call(server, AP_Module::BindingId::PARAMETER_INIT, token, name, sizeof(name), 25, 1, nullptr,
                          module);
}

static int32_t parameter_get(AP_ModuleM4_Server &server, uint64_t token, uint16_t module = 0)
{
    return parameter_call(server, AP_Module::BindingId::PARAMETER_GET, token, nullptr, 0, 8, 5, nullptr, module);
}

// the module's Parameter address is a token: an address in M4 RAM or on this stack is never touched
TEST(ModuleM4Server, ParameterAddressIsATokenOnly)
{
    AP_ModuleM4_Server server;
    server.bind_parameters_for_test(false);
    server.run_cleanups_for_test(false);
    server.reset(1, 1);
    alignas(16) uint8_t guard[64];
    memset(guard, 0xA5, sizeof(guard));
    const uint64_t tokens[] { 0x30010020ULL, uint64_t(uintptr_t(guard + 16)) };
    for (const uint64_t token : tokens) {
        EXPECT_EQ(parameter_init(server, token), AP_MODULE_STATUS_OK);
        uint8_t out[5] { 0xEE, 0xEE, 0xEE, 0xEE, 0xEE };
        EXPECT_EQ(parameter_call(server, AP_Module::BindingId::PARAMETER_GET, token, nullptr, 0, 8, 5, out),
                  AP_MODULE_STATUS_OK);
        EXPECT_EQ(out[0], 0U);   // NO_SUCH_PARAM: unbound, as the PR's adapter answers
        const float value = 1.0f;
        IGNORE_RETURN(parameter_call(server, AP_Module::BindingId::PARAMETER_SET, token, &value, sizeof(value), 12, 1));
        IGNORE_RETURN(parameter_call(server, AP_Module::BindingId::PARAMETER_SET_AND_SAVE, token, &value, sizeof(value),
                                     12, 1));
        IGNORE_RETURN(parameter_call(server, AP_Module::BindingId::PARAMETER_SET_DEFAULT, token, &value, sizeof(value),
                                     12, 1));
        EXPECT_EQ(parameter_call(server, AP_Module::BindingId::PARAMETER_CONFIGURED, token, nullptr, 0, 8, 1),
                  AP_MODULE_STATUS_OK);
    }
    for (const uint8_t byte : guard) {
        EXPECT_EQ(byte, 0xA5);
    }
    EXPECT_EQ(server.parameter_handles(), 2U);
    // a token that init never named, a misaligned one, zero, and another module's
    EXPECT_EQ(parameter_get(server, 0x30010040ULL), AP_MODULE_STATUS_INVALID);
    EXPECT_EQ(parameter_get(server, 0x30010021ULL), AP_MODULE_STATUS_INVALID);
    EXPECT_EQ(parameter_get(server, 0), AP_MODULE_STATUS_INVALID);
    server.reset(2, 2);
    EXPECT_EQ(parameter_init(server, 0x30010020ULL, 0), AP_MODULE_STATUS_OK);
    EXPECT_EQ(parameter_get(server, 0x30010020ULL, 1), AP_MODULE_STATUS_INVALID);
}

// ending one module removes only its handles: every other module's still resolves
TEST(ModuleM4Server, EndModuleKeepsOtherModulesHandles)
{
    AP_ModuleM4_Server server;
    server.bind_parameters_for_test(false);
    server.run_cleanups_for_test(false);
    server.reset(1, 3);
    for (uint32_t i = 0; i < 70; i++) {
        for (uint16_t module = 0; module < 3; module++) {
            EXPECT_EQ(parameter_init(server, 0x30010000ULL + i * 16, module), AP_MODULE_STATUS_OK);
        }
    }
    EXPECT_EQ(server.parameter_handles(), 210U);
    server.end_module(1);
    EXPECT_EQ(server.parameter_handles(), 140U);
    for (uint32_t i = 0; i < 70; i++) {
        EXPECT_EQ(parameter_get(server, 0x30010000ULL + i * 16, 0), AP_MODULE_STATUS_OK);
        EXPECT_EQ(parameter_get(server, 0x30010000ULL + i * 16, 1), AP_MODULE_STATUS_INVALID);
        EXPECT_EQ(parameter_get(server, 0x30010000ULL + i * 16, 2), AP_MODULE_STATUS_OK);
    }
}

// the AirShow's shape: the layout, then about 110 handles read every tick; freed at module end
TEST(ModuleM4Server, ParameterHandlesAreBoundedAndFreed)
{
    AP_ModuleM4_Server server;
    server.bind_parameters_for_test(false);
    server.run_cleanups_for_test(false);
    server.reset(1, 1);
    uint8_t layout[8] {};
    EXPECT_EQ(parameter_call(server, AP_Module::BindingId::PARAMETER_LAYOUT, 0, nullptr, 0, 0, 8, layout),
              AP_MODULE_STATUS_OK);
    uint32_t size, alignment;
    memcpy(&size, layout, 4);
    memcpy(&alignment, layout + 4, 4);
    EXPECT_EQ(size, sizeof(Parameter));
    EXPECT_EQ(alignment, alignof(Parameter));
    const uint64_t base = 0x30010000ULL;
    for (uint32_t i = 0; i < 110; i++) {
        EXPECT_EQ(parameter_init(server, base + i * size), AP_MODULE_STATUS_OK);
    }
    for (uint32_t tick = 0; tick < 3; tick++) {
        for (uint32_t i = 0; i < 110; i++) {
            EXPECT_EQ(parameter_get(server, base + i * size), AP_MODULE_STATUS_OK);
        }
    }
    // init again on a live handle rebinds it: no new entry
    EXPECT_EQ(parameter_init(server, base), AP_MODULE_STATUS_OK);
    EXPECT_EQ(server.parameter_handles(), 110U);
    server.end_module(0);
    EXPECT_EQ(server.parameter_handles(), 0U);
    EXPECT_EQ(parameter_get(server, base), AP_MODULE_STATUS_INVALID);
    for (uint32_t i = 0; i < AP_ModuleM4_Server::PARAMETER_HANDLES_MAX; i++) {
        EXPECT_EQ(parameter_init(server, base + i * size), AP_MODULE_STATUS_OK);
    }
    EXPECT_EQ(parameter_init(server, base + AP_ModuleM4_Server::PARAMETER_HANDLES_MAX * size),
              AP_MODULE_STATUS_NO_MEMORY);
    for (uint32_t i = 0; i < AP_ModuleM4_Server::PARAMETER_HANDLES_MAX; i++) {
        EXPECT_EQ(parameter_get(server, base + i * size), AP_MODULE_STATUS_OK);
    }
    server.reset(2, 1);
    EXPECT_EQ(server.parameter_handles(), 0U);
}
#endif

AP_GTEST_MAIN()
