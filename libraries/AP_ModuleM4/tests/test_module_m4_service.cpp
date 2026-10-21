// the values compared are set exactly
#pragma GCC diagnostic ignored "-Wfloat-equal"

#include <AP_gtest.h>
#include <AP_ModuleM4/AP_ModuleM4_config.h>
#if AP_MODULE_M4_ENABLED
#include <errno.h>
#include <pthread.h>
#include <atomic>
#include <chrono>
// the M4 service as the M4 image and the SITL M4 process build it (tests/wscript adds its
// source and the include paths its bare-name includes need)
#include <AP_ModuleM4_CallStatus.h>
#include <AP_ModuleM4_Layout.h>
#include <m4_service.h>

using namespace ModuleM4;
using CubePilotFW::Channel;

// the SITL M4 process's platform feed, as m4/sitl/crc_port.cpp
void ModuleM4::crc32_feed(Crc32 &crc, const uint8_t *data, uint32_t length)
{
    crc.state = crc32_software_update(crc.state, data, length);
}

namespace {

// both cores' Channels on one window in this process; no doorbell is rung while neither sleeps
alignas(32) uint8_t window[WINDOW_SIZE];
Channel m7;
Channel m4;
ModuleLink call_link;   // not `link`: <unistd.h> declares link()
M4Service service;
pthread_mutex_t service_mutex = PTHREAD_MUTEX_INITIALIZER;
std::atomic<uint32_t> module_wakes;
const auto epoch = std::chrono::steady_clock::now();
// a test moves the service's clocks on: round trips take fake_cycles (1 cycle per us), and
// millis_ahead skips past a call timeout or to the next Stats window
std::atomic<uint32_t> fake_cycles;
std::atomic<uint32_t> millis_ahead;
std::atomic<uint32_t> m7_millis_given;
std::atomic<uint32_t> m7_millis_calls;
std::atomic<uint32_t> stops;
std::atomic<uint32_t> stopped_module;

uint32_t millis_now()
{
    return millis_ahead.load() +
           uint32_t(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - epoch).count());
}
uint32_t cycles_now()
{
    return fake_cycles.load();
}
void inject_fault(FaultKind)
{
}
void wake_module()
{
    module_wakes++;
}
void m7_millis(uint32_t value)
{
    m7_millis_given = value;
    m7_millis_calls++;
}
void stop_module(uint16_t module)
{
    stops++;
    stopped_module = module;
}
void lock_service()
{
    pthread_mutex_lock(&service_mutex);
}
void unlock_service()
{
    pthread_mutex_unlock(&service_mutex);
}

const uint32_t BINDING = 431;

void start()
{
    memset(window, 0, sizeof(window));
    call_link.state.store(ModuleLink::Idle);
    call_link.fault_request.store(0);
    call_link.request_len = 0;
    call_link.result = 0;
    memset(call_link.request, 0, sizeof(call_link.request));
    memset(call_link.reply, 0, sizeof(call_link.reply));
    ASSERT_EQ(m7.init(window, CHANNEL_WINDOW_SIZE, Channel::Core::M7, DOORBELL_GROUP), 0);
    ASSERT_EQ(m4.init(window, CHANNEL_WINDOW_SIZE, Channel::Core::M4, DOORBELL_GROUP), 0);
    module_wakes = 0;
    fake_cycles = 0;
    millis_ahead = 0;
    m7_millis_given = 0;
    m7_millis_calls = 0;
    stops = 0;
    stopped_module = 0xFFFF;
    const M4Platform platform { millis_now, cycles_now, 1, inject_fault, wake_module, nullptr, m7_millis,
                                lock_service, unlock_service, stop_module };
    service.init(m4, window, call_link, platform);
}

void send_ready(uint32_t session)
{
    ReadyMsg ready;
    fill_header(ready.header, MsgType::Ready, sizeof(ready), session);
    ASSERT_EQ(m7.send(&ready, sizeof(ready)), 0);
}

// the module thread's side: a 4 B in, 4 B out BindingCall of id, posted on the link
void post_call(uint32_t id)
{
    const uint8_t input[4] = { 1, 2, 3, 4 };
    call_link.request_len = pack_binding_call(call_link.request, MAX_FRAME, 0, 0, 0, id, input, 4, 4);
    ASSERT_NE(call_link.request_len, 0);
    call_link.state.store(ModuleLink::Posted);
}

// the service's own Config request of a session
bool is_config_request(const uint8_t *frame, uint16_t len)
{
    SystemCallHeader call;
    if (len < sizeof(call)) {
        return false;
    }
    memcpy(&call, frame, sizeof(call));
    return call.header.type == uint8_t(MsgType::SystemCall) && call.method == uint16_t(SystemMethod::Config);
}

// the next M4 -> M7 frame other than Stats and Config; false if the ring holds no other
bool m7_receive(uint8_t *frame, uint16_t &len)
{
    while (m7.receive(frame, MAX_FRAME, &len) == 0) {
        MsgHeader header;
        if (!parse_header(frame, len, header)) {
            ADD_FAILURE() << "unparseable frame";
            return false;
        }
        if (MsgType(header.type) != MsgType::Stats && !is_config_request(frame, len)) {
            return true;
        }
    }
    return false;
}

// the next Config request in the ring, other frames skipped; false if there is none
bool m7_receive_config(ConfigRequest &request)
{
    uint8_t frame[MAX_FRAME];
    uint16_t len = 0;
    while (m7.receive(frame, MAX_FRAME, &len) == 0) {
        if (is_config_request(frame, len)) {
            EXPECT_EQ(len, sizeof(request));
            memcpy(&request, frame, sizeof(request));
            return len == sizeof(request);
        }
    }
    return false;
}

void m7_config_reply(uint32_t seq, uint32_t m7_millis_value)
{
    ConfigReply reply {};
    fill_header(reply.reply.header, MsgType::SystemReply, sizeof(reply), seq);
    reply.reply.method = uint16_t(SystemMethod::Config);
    reply.reply.transport = uint8_t(ReplyStatus::Ok);
    reply.heap_size = 32768;
    reply.code_size = 114688;
    reply.stack_size = 16384;
    reply.max_modules = 2;
    reply.m4_options = 2;
    reply.m7_millis = m7_millis_value;
    reply.m7_micros = m7_millis_value * 1000U;
    ASSERT_EQ(m7.send(&reply, sizeof(reply)), 0);
}

bool m7_receive_call(BindingCallHeader &call)
{
    uint8_t frame[MAX_FRAME];
    uint16_t len = 0;
    return m7_receive(frame, len) && parse_binding_call(frame, len, call);
}

// the next Stats row in the ring, other frames skipped; false if there is none
bool m7_receive_stats(StatsMsg &stats)
{
    uint8_t frame[MAX_FRAME];
    uint16_t len = 0;
    while (m7.receive(frame, MAX_FRAME, &len) == 0) {
        MsgHeader header;
        if (parse_header(frame, len, header) && MsgType(header.type) == MsgType::Stats && len == sizeof(stats)) {
            memcpy(&stats, frame, sizeof(stats));
            return true;
        }
    }
    return false;
}

void m7_reply(uint32_t seq, uint8_t fill)
{
    const uint8_t output[4] = { fill, fill, fill, fill };
    uint8_t frame[MAX_FRAME];
    const uint16_t len = pack_binding_reply(frame, sizeof(frame), seq, ReplyStatus::Ok, 0, output, 4, 0, 0);
    ASSERT_NE(len, 0);
    ASSERT_EQ(m7.send(frame, len), 0);
}

// one call served by the M7 in round_trip_us
void round_trip(uint32_t id, uint32_t round_trip_us)
{
    post_call(id);
    ASSERT_TRUE(service.forward_posted());
    BindingCallHeader call;
    ASSERT_TRUE(m7_receive_call(call));
    fake_cycles += round_trip_us;
    m7_reply(call.header.seq, 0x11);
    service.step();
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    call_link.state.store(ModuleLink::Idle);
}

}  // namespace

TEST(ModuleM4Service, PostedBindingCallIsSentAndCompletedBySeq)
{
    start();
    send_ready(7);
    service.step();   // takes Ready
    post_call(BINDING);
    EXPECT_TRUE(service.forward_posted());
    EXPECT_EQ(call_link.state.load(), ModuleLink::Sent);
    BindingCallHeader call;
    ASSERT_TRUE(m7_receive_call(call));
    EXPECT_EQ(call.id, BINDING);
    EXPECT_EQ(call.in_size, 4);
    EXPECT_EQ(call.out_size, 4);
    const uint32_t first_seq = call.header.seq;
    m7_reply(first_seq + 1, 0x22);   // another seq: dropped
    service.step();
    EXPECT_EQ(call_link.state.load(), ModuleLink::Sent);
    m7_reply(first_seq, 0x22);
    service.step();
    EXPECT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(call_link.result, int32_t(sizeof(BindingReplyHeader) + 4));
    EXPECT_EQ(call_link.reply[sizeof(BindingReplyHeader)], 0x22);
    EXPECT_EQ(module_wakes.load(), 1U);
    // the next call takes the next sequence number
    call_link.state.store(ModuleLink::Idle);
    post_call(BINDING + 1);
    EXPECT_TRUE(service.forward_posted());
    ASSERT_TRUE(m7_receive_call(call));
    EXPECT_EQ(call.header.seq, first_seq + 1);
}

TEST(ModuleM4Service, LateReplyIsDroppedBySeq)
{
    start();
    send_ready(7);
    service.step();
    post_call(BINDING);
    ASSERT_TRUE(service.forward_posted());
    BindingCallHeader first;
    ASSERT_TRUE(m7_receive_call(first));
    // no reply within CALL_TIMEOUT_MS
    millis_ahead += CALL_TIMEOUT_MS + 1;
    service.step();
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(call_link.result, -ETIMEDOUT);
    EXPECT_EQ(service.lost(), 1U);
    call_link.state.store(ModuleLink::Idle);
    // the second call is Sent when the first call's reply arrives
    post_call(BINDING);
    ASSERT_TRUE(service.forward_posted());
    BindingCallHeader second;
    ASSERT_TRUE(m7_receive_call(second));
    ASSERT_NE(second.header.seq, first.header.seq);
    memset(call_link.reply, 0x55, sizeof(call_link.reply));
    m7_reply(first.header.seq, 0x33);
    service.step();
    EXPECT_EQ(call_link.state.load(), ModuleLink::Sent);
    for (uint16_t i = 0; i < sizeof(BindingReplyHeader) + 4; i++) {
        ASSERT_EQ(call_link.reply[i], 0x55) << i;
    }
    EXPECT_EQ(service.lost(), 1U);
    EXPECT_EQ(service.protocol_errors(), 1U);
    // the second call's own reply completes it
    m7_reply(second.header.seq, 0x44);
    service.step();
    EXPECT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(call_link.reply[sizeof(BindingReplyHeader)], 0x44);
}

TEST(ModuleM4Service, ReplyOfAnOlderSessionIsDropped)
{
    start();
    send_ready(7);
    service.step();
    post_call(BINDING);
    ASSERT_TRUE(service.forward_posted());
    BindingCallHeader call;
    ASSERT_TRUE(m7_receive_call(call));
    // a repeated Ready of the same session changes nothing
    send_ready(7);
    service.step();
    EXPECT_EQ(call_link.state.load(), ModuleLink::Sent);
    // a new session: the call cannot be answered any more
    send_ready(8);
    service.step();
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(call_link.result, -ECONNRESET);
    EXPECT_EQ(service.lost(), 1U);
    memset(call_link.reply, 0x55, sizeof(call_link.reply));
    m7_reply(call.header.seq, 0x33);
    service.step();
    EXPECT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(call_link.result, -ECONNRESET);
    EXPECT_EQ(call_link.reply[0], 0x55);
    EXPECT_EQ(service.protocol_errors(), 1U);
}

TEST(ModuleM4Service, StatsRowsAreKeyedAndNamedByBindingId)
{
    start();
    send_ready(7);
    service.step();
    service.set_binding_name(431, "ahrs:get_roll_rad");
    round_trip(431, 100);
    round_trip(431, 100);
    round_trip(432, 50);
    millis_ahead += 1000;
    service.step();
    StatsMsg rows[4];
    uint8_t row_count = 0;
    uint8_t frame[MAX_FRAME];
    uint16_t len = 0;
    while (m7.receive(frame, sizeof(frame), &len) == 0) {
        MsgHeader header;
        ASSERT_TRUE(parse_header(frame, len, header));
        if (is_config_request(frame, len)) {
            continue;   // unanswered here: sent again after CALL_TIMEOUT_MS
        }
        ASSERT_EQ(MsgType(header.type), MsgType::Stats);
        ASSERT_LT(row_count, 4);
        memcpy(&rows[row_count++], frame, sizeof(StatsMsg));
    }
    ASSERT_EQ(row_count, 2);
    // largest total first
    EXPECT_EQ(rows[0].method, 431);
    EXPECT_STREQ(rows[0].name, "ahrs:get_roll_rad");
    EXPECT_EQ(rows[0].count, 2U);
    EXPECT_EQ(rows[0].total_us, 200U);
    EXPECT_EQ(rows[1].method, 432);
    EXPECT_STREQ(rows[1].name, "id:432");
    EXPECT_EQ(rows[1].count, 1U);
    EXPECT_EQ(rows[1].total_us, 50U);
    millis_ahead = 0;
}

TEST(ModuleM4Service, NotifyCompletesAtOnce)
{
    start();
    send_ready(7);
    service.step();
    TextNotify text {};
    fill_header(text.call.header, MsgType::SystemCall, sizeof(text), 0);
    text.call.method = uint16_t(SystemMethod::Text);
    text.call.flags = CALL_FLAG_NOTIFY;
    text.severity = 6;
    text.length = 2;
    memcpy(text.text, "ok", 2);
    memcpy(call_link.request, &text, sizeof(text));
    call_link.request_len = sizeof(text);
    call_link.state.store(ModuleLink::Posted);
    EXPECT_TRUE(service.forward_posted());
    EXPECT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(call_link.result, 0);
    uint8_t frame[MAX_FRAME];
    uint16_t len = 0;
    ASSERT_TRUE(m7_receive(frame, len));
    EXPECT_EQ(len, sizeof(TextNotify));
    EXPECT_EQ(frame[0], uint8_t(MsgType::SystemCall));
}

TEST(ModuleM4Service, ConfigIsSentOncePerSessionAndCached)
{
    start();
    ConfigRequest request;
    service.step();
    EXPECT_FALSE(m7_receive_config(request));   // not before Ready
    send_ready(7);
    service.step();
    ASSERT_TRUE(m7_receive_config(request));
    EXPECT_EQ(request.crc_short, CRC32_CHECK);
    EXPECT_EQ(request.crc_pattern, CRC32_PATTERN_CHECK);
    EXPECT_EQ(request.call.flags, 0);
    EXPECT_FALSE(service.config_ready());
    // another seq is not its reply
    m7_config_reply(request.call.header.seq + 1, 5000);
    service.step();
    EXPECT_FALSE(service.config_ready());
    m7_config_reply(request.call.header.seq, 5000);
    service.step();
    ASSERT_TRUE(service.config_ready());
    EXPECT_EQ(service.config_reply().code_size, 114688);
    EXPECT_EQ(service.config_reply().stack_size, 16384);
    EXPECT_EQ(service.config_reply().m4_options, 2);
    EXPECT_EQ(m7_millis_calls.load(), 1U);
    EXPECT_EQ(m7_millis_given.load(), 5000U);
    // the same session's Ready again: no second Config, even after the timeout
    send_ready(7);
    millis_ahead += CALL_TIMEOUT_MS + 1;
    service.step();
    EXPECT_FALSE(m7_receive_config(request));
    EXPECT_TRUE(service.config_ready());
    // a new session asks again, and keeps asking until it is answered
    send_ready(8);
    service.step();
    EXPECT_FALSE(service.config_ready());
    ASSERT_TRUE(m7_receive_config(request));
    const uint32_t first_seq = request.call.header.seq;
    service.step();
    EXPECT_FALSE(m7_receive_config(request));
    millis_ahead += CALL_TIMEOUT_MS + 1;
    service.step();
    ASSERT_TRUE(m7_receive_config(request));
    EXPECT_NE(request.call.header.seq, first_seq);
    m7_config_reply(request.call.header.seq, 6000);
    service.step();
    EXPECT_TRUE(service.config_ready());
    EXPECT_EQ(m7_millis_given.load(), 6000U);
    millis_ahead = 0;
}

// a stop of this session reaches the platform with its module; one of another session, or not a stop,
// is dropped as a protocol error
TEST(ModuleM4Service, ControlStopOfThisSessionReachesThePlatform)
{
    start();
    send_ready(9);
    service.step();
    auto send_control = [](uint32_t session, uint16_t action) {
        ControlMsg control {};
        fill_header(control.header, MsgType::Control, sizeof(control), session);
        control.module = 0;
        control.action = action;
        ASSERT_EQ(m7.send(&control, sizeof(control)), 0);
    };
    send_control(8, CONTROL_STOP);
    send_control(9, CONTROL_STOP + 1);
    service.step();
    EXPECT_EQ(stops.load(), 0U);
    EXPECT_EQ(service.protocol_errors(), 2U);
    send_control(9, CONTROL_STOP);
    service.step();
    EXPECT_EQ(stops.load(), 1U);
    EXPECT_EQ(stopped_module.load(), 0U);
}

static void send_parameter_changed(uint32_t session, const ParameterValue *values, uint16_t count)
{
    uint8_t frame[MAX_FRAME];
    const uint16_t len = pack_parameter_changed(frame, sizeof(frame), session, values, count);
    ASSERT_NE(len, 0U);
    ASSERT_EQ(m7.send(frame, len), 0);
}

TEST(ModuleM4Service, ParameterChangedFeedsTheMirror)
{
    start();
    send_ready(7);
    service.step();
    const ParameterValue value { 0x20001000ULL, 3.5f, 0, PARAMETER_VALUE_BOUND, 0 };
    send_parameter_changed(7, &value, 1);
    service.step();
    uint8_t out[5];
    ASSERT_TRUE(service.mirror_get(0, 0x20001000ULL, out));
    EXPECT_EQ(out[0], 1U);
    EXPECT_EQ(service.mirror_hits(), 1U);
    EXPECT_FALSE(service.mirror_get(0, 0x20001010ULL, out));
    EXPECT_EQ(service.mirror_misses(), 1U);
}

TEST(ModuleM4Service, ParameterChangedOfAnotherSessionIsDropped)
{
    start();
    send_ready(7);
    service.step();
    const ParameterValue value { 0x20001000ULL, 3.5f, 0, PARAMETER_VALUE_BOUND, 0 };
    send_parameter_changed(6, &value, 1);
    service.step();
    uint8_t out[5];
    EXPECT_FALSE(service.mirror_get(0, 0x20001000ULL, out));
    EXPECT_EQ(service.protocol_errors(), 1U);
}

TEST(ModuleM4Service, BadEntriesAreDroppedOthersApplied)
{
    start();
    send_ready(7);
    service.step();
    const ParameterValue values[3] = {
        { 0x20001000ULL, 1.0f, MAX_MODULES, PARAMETER_VALUE_BOUND, 0 },
        { 0x20001010ULL, 2.0f, 0, PARAMETER_VALUE_BOUND, 0 },
        { 0x20001020ULL, 3.0f, 0xFFFF, PARAMETER_VALUE_BOUND, 0 },
    };
    send_parameter_changed(7, values, 3);
    service.step();
    uint8_t out[5];
    EXPECT_TRUE(service.mirror_get(0, 0x20001010ULL, out));
    EXPECT_EQ(service.protocol_errors(), 2U);
    // a token above UINTPTR_MAX is refused where pointers are 32-bit (the M4 image); the SITL M4
    // process and this host test have 64-bit pointers, so only the module checks run here
}

TEST(ModuleM4Service, ResyncReplyMakesTheModuleStale)
{
    start();
    send_ready(7);
    service.step();
    const ParameterValue value { 0x20001000ULL, 3.5f, 0, PARAMETER_VALUE_BOUND, 0 };
    send_parameter_changed(7, &value, 1);
    service.step();
    post_call(BINDING);
    ASSERT_TRUE(service.forward_posted());
    BindingCallHeader call;
    ASSERT_TRUE(m7_receive_call(call));
    BindingReplyHeader reply {};
    uint8_t frame[MAX_FRAME];
    const uint8_t output[4] = { 0, 0, 0, 0 };
    uint16_t len = pack_binding_reply(frame, sizeof(frame), call.header.seq, ReplyStatus::Ok, STATUS_OK, output, 4, 0, 0);
    memcpy(&reply, frame, sizeof(reply));
    reply.reserved2 = REPLY_FLAG_MIRROR_RESYNC;
    memcpy(frame, &reply, sizeof(reply));
    ASSERT_EQ(m7.send(frame, len), 0);
    service.step();
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    uint8_t out[5];
    EXPECT_FALSE(service.mirror_get(0, 0x20001000ULL, out));
}

TEST(ModuleM4Service, ModuleEndForgetsItsEntries)
{
    start();
    send_ready(7);
    service.step();
    const ParameterValue value { 0x20001000ULL, 3.5f, 0, PARAMETER_VALUE_BOUND, 0 };
    send_parameter_changed(7, &value, 1);
    service.step();
    ModuleStateNotify notify {};
    fill_header(notify.call.header, MsgType::SystemCall, sizeof(notify), 0);
    notify.call.method = uint16_t(SystemMethod::ModuleState);
    notify.call.flags = CALL_FLAG_NOTIFY;
    notify.module = 0;
    notify.state = uint8_t(ModuleStateKind::Returned);
    memcpy(call_link.request, &notify, sizeof(notify));
    call_link.request_len = sizeof(notify);
    call_link.state.store(ModuleLink::Posted);
    ASSERT_TRUE(service.forward_posted());
    uint8_t out[5];
    EXPECT_FALSE(service.mirror_get(0, 0x20001000ULL, out));
    // the M7 still holds the ended module's slots: its later entries are dropped, not errors
    send_parameter_changed(7, &value, 1);
    service.step();
    EXPECT_FALSE(service.mirror_get(0, 0x20001000ULL, out));
    EXPECT_EQ(service.protocol_errors(), 0U);
}

TEST(ModuleM4Service, ForwardedGetRefreshesAStaleEntryOnly)
{
    start();
    send_ready(7);
    service.step();
    const ParameterValue value { 0x20001000ULL, 3.5f, 0, PARAMETER_VALUE_BOUND, 0 };
    send_parameter_changed(7, &value, 1);
    service.step();
    uint8_t out[5];
    const uint8_t answer[5] = { 1, 0, 0, 0x80, 0x40 };   // 4.0f
    service.mirror_refresh(0, 0x20001000ULL, answer);   // fresh: left as it is
    ASSERT_TRUE(service.mirror_get(0, 0x20001000ULL, out));
    float got;
    memcpy(&got, out + 1, 4);
    EXPECT_EQ(got, 3.5f);
    service.mirror_pending(0, 0x20001000ULL);   // a writer whose frame never came (refused or timed out)
    service.mirror_settle(0, 0x20001000ULL);
    EXPECT_FALSE(service.mirror_get(0, 0x20001000ULL, out));   // STALE
    service.mirror_refresh(0, 0x20001000ULL, answer);
    ASSERT_TRUE(service.mirror_get(0, 0x20001000ULL, out));
    memcpy(&got, out + 1, 4);
    EXPECT_EQ(got, 4.0f);
    service.mirror_refresh(0, 0x20001010ULL, answer);   // never inserts: a never-initialised token stays a miss
    EXPECT_FALSE(service.mirror_get(0, 0x20001010ULL, out));
}

TEST(ModuleM4Service, InitOfAnExistingTokenIsForwardedUntilItsFrame)
{
    start();
    send_ready(7);
    service.step();
    const ParameterValue value { 0x20001000ULL, 3.5f, 0, PARAMETER_VALUE_BOUND, 0 };
    send_parameter_changed(7, &value, 1);
    service.step();
    service.mirror_pending(0, 0x20001000ULL);   // the module thread, before its INIT goes out
    uint8_t out[5];
    EXPECT_FALSE(service.mirror_get(0, 0x20001000ULL, out));
    const ParameterValue rebound { 0x20001000ULL, 1.0f, 0, PARAMETER_VALUE_BOUND, 0 };
    send_parameter_changed(7, &rebound, 1);
    service.step();
    service.mirror_settle(0, 0x20001000ULL);
    ASSERT_TRUE(service.mirror_get(0, 0x20001000ULL, out));
    float got;
    memcpy(&got, out + 1, 4);
    EXPECT_EQ(got, 1.0f);
}

TEST(ModuleM4Service, PostedBindingIsTimedAndMarked)
{
    start();
    send_ready(7);
    service.step();
    uint8_t input[16] {};
    call_link.request_len = pack_binding_call(call_link.request, MAX_FRAME, 0, 0, CALL_FLAG_NOTIFY, BINDING, input,
                                              sizeof(input), 0);
    call_link.state.store(ModuleLink::Posted);
    fake_cycles = 0;
    ASSERT_TRUE(service.forward_posted());
    EXPECT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(call_link.result, 0);
    BindingCallHeader call;
    ASSERT_TRUE(m7_receive_call(call));
    EXPECT_EQ(call.flags, CALL_FLAG_NOTIFY);
    millis_ahead = millis_ahead.load() + 1000;
    service.step();   // the Stats window
    StatsMsg stats;
    ASSERT_TRUE(m7_receive_stats(stats));
    EXPECT_EQ(stats.method, BINDING);
    EXPECT_EQ(stats.count, 1U);
    EXPECT_EQ(stats.reserved & STATS_ROW_POSTED, STATS_ROW_POSTED);
}

TEST(ModuleM4Service, PostedFrameDroppedAfterTheTimeoutCountsPostDrop)
{
    start();   // no Ready: the frame cannot go out
    uint8_t input[16] {};
    call_link.request_len = pack_binding_call(call_link.request, MAX_FRAME, 0, 0, CALL_FLAG_NOTIFY, BINDING, input,
                                              sizeof(input), 0);
    call_link.state.store(ModuleLink::Posted);
    service.step();
    millis_ahead = millis_ahead.load() + CALL_TIMEOUT_MS + 1;
    service.step();
    EXPECT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(service.post_drops(), 1U);
}

#endif  // AP_MODULE_M4_ENABLED

AP_GTEST_MAIN()
