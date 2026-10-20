#include <AP_gtest.h>
#include <AP_RustM4/AP_RustM4_Protocol.h>

using namespace RustM4;

TEST(RustM4Frames, HeaderSizesAreFrozen)
{
    EXPECT_EQ(sizeof(BindingCallHeader), 20U);
    EXPECT_EQ(sizeof(BindingReplyHeader), 28U);
    EXPECT_EQ(sizeof(SystemCallHeader), 12U);
    EXPECT_EQ(sizeof(SystemReplyHeader), 16U);
    EXPECT_EQ(sizeof(ControlMsg), 12U);
    EXPECT_EQ(MAX_CALL_INPUT, 2028U);
    EXPECT_EQ(MAX_REPLY_OUTPUT, 2020U);
}

TEST(RustM4Frames, CallRoundTrip)
{
    uint8_t frame[MAX_FRAME];
    const uint8_t input[5] = { 1, 2, 3, 4, 5 };
    const uint16_t len = pack_binding_call(frame, sizeof(frame), 77, 0, 0, 431, input, sizeof(input), 12);
    ASSERT_EQ(len, 25U);
    BindingCallHeader h {};
    ASSERT_TRUE(parse_binding_call(frame, len, h));
    EXPECT_EQ(h.header.seq, 77U);
    EXPECT_EQ(h.id, 431U);
    EXPECT_EQ(h.in_size, 5U);
    EXPECT_EQ(h.out_size, 12U);
    EXPECT_EQ(memcmp(frame + sizeof(h), input, sizeof(input)), 0);
    MsgHeader header {};
    EXPECT_TRUE(parse_header(frame, len, header));
}

TEST(RustM4Frames, CallLimits)
{
    static uint8_t frame[MAX_FRAME + 16];
    static uint8_t input[MAX_FRAME];
    EXPECT_EQ(pack_binding_call(frame, sizeof(frame), 1, 0, 0, 1, input, MAX_CALL_INPUT, 0), MAX_FRAME);
    EXPECT_EQ(pack_binding_call(frame, sizeof(frame), 1, 0, 0, 1, input, MAX_CALL_INPUT + 1, 0), 0U);
    EXPECT_EQ(pack_binding_call(frame, sizeof(frame), 1, 0, 0, 1, input, 0, MAX_REPLY_OUTPUT + 1), 0U);
    EXPECT_EQ(pack_binding_call(frame, 30, 1, 0, 0, 1, input, 16, 0), 0U);         // capacity
    EXPECT_EQ(pack_binding_call(frame, sizeof(frame), 1, 0, 0, 1, nullptr, 4, 0), 0U);
}

TEST(RustM4Frames, MalformedCallsAreRejected)
{
    uint8_t frame[64];
    const uint8_t input[8] {};
    const uint16_t len = pack_binding_call(frame, sizeof(frame), 1, 0, 0, 9, input, 8, 4);
    BindingCallHeader h {};
    EXPECT_FALSE(parse_binding_call(frame, len - 1, h));     // length disagrees
    frame[0] = uint8_t(MsgType::BindingReply);
    EXPECT_FALSE(parse_binding_call(frame, len, h));         // wrong type
}

TEST(RustM4Frames, ReplyCarriesOutputOnlyWithOk)
{
    uint8_t frame[MAX_FRAME];
    const uint8_t output[4] = { 9, 8, 7, 6 };
    uint16_t len = pack_binding_reply(frame, sizeof(frame), 5, ReplyStatus::Ok, 0, output, 4, 10, 20);
    ASSERT_EQ(len, 32U);
    BindingReplyHeader h {};
    const uint8_t *body = nullptr;
    ASSERT_TRUE(parse_binding_reply(frame, len, h, body));
    EXPECT_EQ(h.out_size, 4U);
    EXPECT_EQ(memcmp(body, output, 4), 0);
    len = pack_binding_reply(frame, sizeof(frame), 5, ReplyStatus::Ok, -5, output, 4, 0, 0);
    ASSERT_EQ(len, 28U);
    ASSERT_TRUE(parse_binding_reply(frame, len, h, body));
    EXPECT_EQ(h.status, -5);
    EXPECT_EQ(h.out_size, 0U);
}

AP_GTEST_MAIN()
