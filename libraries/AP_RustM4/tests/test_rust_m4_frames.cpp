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

TEST(RustM4Frames, ProbedCallCarriesItsTrailerAfterTheInput)
{
    EXPECT_EQ(sizeof(CallProbeTrailer), 72U);
    uint8_t frame[MAX_FRAME];
    const uint8_t input[3] = { 1, 2, 3 };
    CallProbeTrailer trailer {};
    trailer.svc_entry_us = 11;
    trailer.prev_seq = 5;
    trailer.prev_reply_seen_us = 0xFFFFFFF0U;
    trailer.prev_flags = PROBE_M4_SLEPT;
    trailer.prev_stamps = PROBE_STAMP_PACKED | PROBE_STAMP_RESUMED;
    trailer.prev_packed_us = 21;
    trailer.prev_posted_us = 22;
    trailer.prev_forward_returned_us = 23;
    trailer.prev_reply_isr_us = 24;
    trailer.prev_reply_wake_us = 25;
    trailer.prev_signalled_us = 26;
    trailer.prev_resumed_us = 27;
    trailer.memset_small_centicycles = 125;
    trailer.memset_large_centicycles = 410;
    trailer.prev_svc_to_entry_cycles = 31;
    trailer.prev_checks_cycles = 32;
    trailer.prev_begin_probe_cycles = 33;
    trailer.prev_pack_cycles = 0xFFFF;
    const uint16_t len = pack_binding_call(frame, sizeof(frame), 9, 0, 0, 242, input, 3, 1, &trailer);
    ASSERT_EQ(len, sizeof(BindingCallHeader) + 3 + sizeof(trailer));
    BindingCallHeader h;
    ASSERT_TRUE(parse_binding_call(frame, len, h));
    EXPECT_EQ(h.flags & CALL_FLAG_PROBE, CALL_FLAG_PROBE);
    EXPECT_EQ(h.in_size, 3);
    EXPECT_EQ(frame[sizeof(h) + 2], 3);
    CallProbeTrailer out;
    ASSERT_TRUE(binding_call_trailer(frame, len, h, out));
    EXPECT_EQ(out.svc_entry_us, 11U);
    EXPECT_EQ(out.prev_seq, 5U);
    EXPECT_EQ(out.prev_reply_seen_us, 0xFFFFFFF0U);
    EXPECT_EQ(out.prev_flags, PROBE_M4_SLEPT);
    EXPECT_EQ(out.prev_stamps, PROBE_STAMP_PACKED | PROBE_STAMP_RESUMED);
    EXPECT_EQ(out.prev_packed_us, 21U);
    EXPECT_EQ(out.prev_posted_us, 22U);
    EXPECT_EQ(out.prev_forward_returned_us, 23U);
    EXPECT_EQ(out.prev_reply_isr_us, 24U);
    EXPECT_EQ(out.prev_reply_wake_us, 25U);
    EXPECT_EQ(out.prev_signalled_us, 26U);
    EXPECT_EQ(out.prev_resumed_us, 27U);
    EXPECT_EQ(out.memset_small_centicycles, 125);
    EXPECT_EQ(out.memset_large_centicycles, 410);
    EXPECT_EQ(out.prev_svc_to_entry_cycles, 31);
    EXPECT_EQ(out.prev_checks_cycles, 32);
    EXPECT_EQ(out.prev_begin_probe_cycles, 33);
    EXPECT_EQ(out.prev_pack_cycles, 0xFFFF);
    // the same frame without the flag: its length disagrees with its input size
    BindingCallHeader unflagged = h;
    unflagged.flags = 0;
    memcpy(frame, &unflagged, sizeof(unflagged));
    EXPECT_FALSE(parse_binding_call(frame, len, h));
}

TEST(RustM4Frames, ProbeFlagFollowsTheTrailer)
{
    uint8_t frame[MAX_FRAME];
    const uint8_t input[4] = { 1, 2, 3, 4 };
    // asked for without a trailer: cleared
    uint16_t len = pack_binding_call(frame, sizeof(frame), 1, 0, CALL_FLAG_PROBE, 9, input, 4, 4);
    BindingCallHeader h;
    ASSERT_TRUE(parse_binding_call(frame, len, h));
    EXPECT_EQ(h.flags & CALL_FLAG_PROBE, 0);
    CallProbeTrailer out;
    EXPECT_FALSE(binding_call_trailer(frame, len, h, out));
    // a flag set on a frame without room for a trailer does not parse
    h.flags = CALL_FLAG_PROBE;
    memcpy(frame, &h, sizeof(h));
    EXPECT_FALSE(parse_binding_call(frame, len, h));
    // the trailer counts toward the frame limit
    static uint8_t big[MAX_CALL_INPUT];
    const CallProbeTrailer trailer {};
    EXPECT_EQ(pack_binding_call(frame, sizeof(frame), 1, 0, 0, 9, big, MAX_CALL_INPUT, 0, &trailer), 0U);
    len = pack_binding_call(frame, sizeof(frame), 1, 0, 0, 9, big, uint16_t(MAX_CALL_INPUT - sizeof(trailer)), 0,
                            &trailer);
    EXPECT_EQ(len, MAX_FRAME);
    EXPECT_TRUE(parse_binding_call(frame, len, h));
}

// the frame as a struct header and memcpy copies build it
static void reference_pack(uint8_t *frame, uint32_t seq, uint16_t module, uint16_t flags, uint32_t id,
                           const uint8_t *input, uint16_t in_size, uint16_t out_size, const CallProbeTrailer *trailer)
{
    const uint32_t trailer_size = trailer != nullptr ? uint32_t(sizeof(*trailer)) : 0U;
    BindingCallHeader h;
    fill_header(h.header, MsgType::BindingCall, uint16_t(sizeof(h) + in_size + trailer_size), seq);
    h.module = module;
    h.flags = trailer != nullptr ? uint16_t(flags | CALL_FLAG_PROBE) : uint16_t(flags & ~CALL_FLAG_PROBE);
    h.id = id;
    h.in_size = in_size;
    h.out_size = out_size;
    memcpy(frame, &h, sizeof(h));
    memcpy(frame + sizeof(h), input, in_size);
    if (trailer != nullptr) {
        memcpy(frame + sizeof(h) + in_size, trailer, sizeof(*trailer));
    }
}

// word and byte copies alike: every frame and input alignment, odd and word sizes, with and without a
// trailer; nothing written past the frame
TEST(RustM4Frames, PackedCallBytesEqualTheReferencePacking)
{
    alignas(4) static uint8_t source[MAX_CALL_INPUT + 8];
    for (uint32_t i = 0; i < sizeof(source); i++) {
        source[i] = uint8_t(i * 7U + 3U);
    }
    alignas(4) CallProbeTrailer trailer;
    memcpy(&trailer, source + 1, sizeof(trailer));
    const CallProbeTrailer *const trailers[] = { nullptr, &trailer };
    const uint16_t sizes[] = { 0, 1, 3, 4, 5, 8, 12, 13, 64, 255, uint16_t(MAX_CALL_INPUT - sizeof(trailer)) };
    alignas(4) static uint8_t packed[MAX_FRAME + 8], expected[MAX_FRAME + 8];
    uint32_t cases = 0;
    for (uint32_t frame_offset = 0; frame_offset < 4; frame_offset++) {
        for (uint32_t input_offset = 0; input_offset < 4; input_offset++) {
            for (uint16_t size : sizes) {
                for (const CallProbeTrailer *with : trailers) {
                    memset(packed, 0xA5, sizeof(packed));
                    memset(expected, 0xA5, sizeof(expected));
                    const uint8_t *input = source + input_offset;
                    const uint16_t len = pack_binding_call(packed + frame_offset, MAX_FRAME, 0x01020304U, 0x0506,
                                                           CALL_FLAG_NOTIFY, 0x0708090AU, input, size, 0x070B, with);
                    reference_pack(expected + frame_offset, 0x01020304U, 0x0506, CALL_FLAG_NOTIFY, 0x0708090AU, input,
                                   size, 0x070B, with);
                    ASSERT_EQ(len, sizeof(BindingCallHeader) + size + (with != nullptr ? sizeof(trailer) : 0U));
                    ASSERT_EQ(memcmp(packed, expected, sizeof(packed)), 0)
                        << "frame +" << frame_offset << " input +" << input_offset << " size " << size
                        << (with != nullptr ? " with trailer" : "");
                    cases++;
                }
            }
        }
    }
    EXPECT_EQ(cases, 4U * 4U * 11U * 2U);
}

TEST(RustM4Frames, CopyFrameBytesCopiesExactlyTheSize)
{
    alignas(4) uint8_t src[40], dst[48];
    for (uint32_t i = 0; i < sizeof(src); i++) {
        src[i] = uint8_t(0x40 + i);
    }
    for (uint32_t offset = 0; offset < 4; offset++) {
        for (uint32_t size = 0; size <= 32; size++) {
            memset(dst, 0xEE, sizeof(dst));
            copy_frame_bytes(dst + offset, src + (size & 1U), size);
            for (uint32_t i = 0; i < sizeof(dst); i++) {
                const bool inside = i >= offset && i < offset + size;
                ASSERT_EQ(dst[i], inside ? src[(size & 1U) + i - offset] : 0xEE) << offset << " " << size << " " << i;
            }
        }
    }
}

AP_GTEST_MAIN()
