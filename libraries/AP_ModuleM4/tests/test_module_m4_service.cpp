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
std::atomic<uint32_t> injected;   // the last FaultKind injected

uint32_t millis_now()
{
    return millis_ahead.load() +
           uint32_t(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - epoch).count());
}
uint32_t cycles_now()
{
    return fake_cycles.load();
}
void inject_fault(FaultKind kind)
{
    injected = uint32_t(kind);
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

void start(M4CallProbe *probe = nullptr)
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
    injected = 0;
    const M4Platform platform { millis_now, cycles_now, 1, inject_fault, wake_module, nullptr, m7_millis,
                                lock_service, unlock_service, stop_module, probe };
    service.init(m4, window, call_link, platform);
}

// both halves of a service pass, in the loops' order
void step_both()
{
    service.drain_step();
    service.housekeeping_step();
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

// the next M4 -> M7 frame other than Stats, Counters and Config; false if the ring holds no other
bool m7_receive(uint8_t *frame, uint16_t &len)
{
    while (m7.receive(frame, MAX_FRAME, &len) == 0) {
        MsgHeader header;
        if (!parse_header(frame, len, header)) {
            ADD_FAILURE() << "unparseable frame";
            return false;
        }
        if (MsgType(header.type) != MsgType::Stats && MsgType(header.type) != MsgType::Counters &&
            !is_config_request(frame, len)) {
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

void m7_config_reply(uint32_t seq, uint32_t m7_millis_value, uint8_t m4_options = 2)
{
    ConfigReply reply {};
    fill_header(reply.reply.header, MsgType::SystemReply, sizeof(reply), seq);
    reply.reply.method = uint16_t(SystemMethod::Config);
    reply.reply.transport = uint8_t(ReplyStatus::Ok);
    reply.heap_max = 32768;
    reply.code_maxsize = 114688;
    reply.stack_max = 16384;
    reply.max_modules = 2;
    reply.m4_options = m4_options;
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

// the next Counters frame in the ring, other frames skipped; false if there is none
bool m7_receive_counters(CountersMsg &counters)
{
    uint8_t frame[MAX_FRAME];
    uint16_t len = 0;
    while (m7.receive(frame, MAX_FRAME, &len) == 0) {
        MsgHeader header;
        if (parse_header(frame, len, header) && MsgType(header.type) == MsgType::Counters) {
            memcpy(&counters, frame, sizeof(counters));
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
    service.drain_step();
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    call_link.state.store(ModuleLink::Idle);
}

// the probe's counter: each read moves it on by 10
uint32_t probe_now;
uint32_t probe_clock()
{
    probe_now += 10;
    return probe_now;
}

// the session's Config answered with these options
void configure(uint8_t m4_options)
{
    send_ready(7);
    service.drain_step();
    service.housekeeping_step();
    ConfigRequest request;
    ASSERT_TRUE(m7_receive_config(request));
    m7_config_reply(request.call.header.seq, 1000, m4_options);
    service.drain_step();
    ASSERT_TRUE(service.config_ready());
}

// a probed call posted as host_binding_call() frames it
void post_probed_call(M4CallProbe &probe)
{
    const uint8_t input[4] = { 1, 2, 3, 4 };
    const CallProbeTrailer trailer = begin_probed_call(probe, 5);
    call_link.request_len = pack_binding_call(call_link.request, MAX_FRAME, 0, 0, 0, BINDING, input, 4, 4, &trailer);
    ASSERT_NE(call_link.request_len, 0);
    call_link.state.store(ModuleLink::Posted);
}

}  // namespace

TEST(ModuleM4Service, PostedBindingCallIsSentAndCompletedBySeq)
{
    start();
    send_ready(7);
    service.drain_step();   // takes Ready
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
    service.drain_step();
    EXPECT_EQ(call_link.state.load(), ModuleLink::Sent);
    m7_reply(first_seq, 0x22);
    service.drain_step();
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
    service.drain_step();
    post_call(BINDING);
    ASSERT_TRUE(service.forward_posted());
    BindingCallHeader first;
    ASSERT_TRUE(m7_receive_call(first));
    // no reply within CALL_TIMEOUT_MS
    millis_ahead += CALL_TIMEOUT_MS + 1;
    service.housekeeping_step();
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
    service.drain_step();
    EXPECT_EQ(call_link.state.load(), ModuleLink::Sent);
    for (uint16_t i = 0; i < sizeof(BindingReplyHeader) + 4; i++) {
        ASSERT_EQ(call_link.reply[i], 0x55) << i;
    }
    EXPECT_EQ(service.lost(), 1U);
    EXPECT_EQ(service.protocol_errors(), 1U);
    // the second call's own reply completes it
    m7_reply(second.header.seq, 0x44);
    service.drain_step();
    EXPECT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(call_link.reply[sizeof(BindingReplyHeader)], 0x44);
}

TEST(ModuleM4Service, ReplyOfAnOlderSessionIsDropped)
{
    start();
    send_ready(7);
    service.drain_step();
    post_call(BINDING);
    ASSERT_TRUE(service.forward_posted());
    BindingCallHeader call;
    ASSERT_TRUE(m7_receive_call(call));
    // a repeated Ready of the same session changes nothing
    send_ready(7);
    service.drain_step();
    EXPECT_EQ(call_link.state.load(), ModuleLink::Sent);
    // a new session: the call cannot be answered any more
    send_ready(8);
    service.drain_step();
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(call_link.result, -ECONNRESET);
    EXPECT_EQ(service.lost(), 1U);
    memset(call_link.reply, 0x55, sizeof(call_link.reply));
    m7_reply(call.header.seq, 0x33);
    service.drain_step();
    EXPECT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(call_link.result, -ECONNRESET);
    EXPECT_EQ(call_link.reply[0], 0x55);
    EXPECT_EQ(service.protocol_errors(), 1U);
}

TEST(ModuleM4Service, StatsRowsAreKeyedAndNamedByBindingId)
{
    start();
    send_ready(7);
    service.drain_step();
    service.set_binding_name(431, "ahrs:get_roll_rad");
    round_trip(431, 100);
    round_trip(431, 100);
    round_trip(432, 50);
    millis_ahead += 1000;
    service.housekeeping_step();
    StatsMsg rows[4];
    uint8_t row_count = 0;
    uint8_t frame[MAX_FRAME];
    uint16_t len = 0;
    while (m7.receive(frame, sizeof(frame), &len) == 0) {
        MsgHeader header;
        ASSERT_TRUE(parse_header(frame, len, header));
        if (is_config_request(frame, len) || MsgType(header.type) == MsgType::Counters) {
            continue;   // Config: unanswered here, sent again after CALL_TIMEOUT_MS
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

// 17 keys in one window: the 16 with the most round-trip time named, the least folded into m4:other
TEST(ModuleM4Service, SixteenBusiestRowsAreReported)
{
    start();
    send_ready(7);
    step_both();
    for (uint32_t i = 0; i < 17; i++) {
        round_trip(500 + i, 10 * (i + 1));
    }
    millis_ahead += 1000;
    step_both();
    StatsMsg rows[20];
    uint8_t row_count = 0;
    uint8_t frame[MAX_FRAME];
    uint16_t len = 0;
    while (m7.receive(frame, sizeof(frame), &len) == 0) {
        MsgHeader header;
        ASSERT_TRUE(parse_header(frame, len, header));
        if (MsgType(header.type) != MsgType::Stats) {
            continue;
        }
        ASSERT_LT(row_count, 20);
        memcpy(&rows[row_count++], frame, sizeof(StatsMsg));
    }
    ASSERT_EQ(row_count, 17);
    for (uint8_t i = 0; i < 16; i++) {
        // largest total first: 500 + 16 down to 500 + 1
        EXPECT_EQ(rows[i].method, 516 - i);
        EXPECT_EQ(rows[i].total_us, 10U * (17 - i));
    }
    EXPECT_STREQ(rows[16].name, "m4:other");
    EXPECT_EQ(rows[16].count, 1U);
    EXPECT_EQ(rows[16].total_us, 10U);
    millis_ahead = 0;
}

TEST(ModuleM4Service, NotifyCompletesAtOnce)
{
    start();
    send_ready(7);
    service.drain_step();
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
    service.housekeeping_step();
    EXPECT_FALSE(m7_receive_config(request));   // not before Ready
    send_ready(7);
    service.drain_step();
    service.housekeeping_step();
    ASSERT_TRUE(m7_receive_config(request));
    EXPECT_EQ(request.crc_short, CRC32_CHECK);
    EXPECT_EQ(request.crc_pattern, CRC32_PATTERN_CHECK);
    EXPECT_EQ(request.call.flags, 0);
    EXPECT_FALSE(service.config_ready());
    // another seq is not its reply
    m7_config_reply(request.call.header.seq + 1, 5000);
    service.drain_step();
    EXPECT_FALSE(service.config_ready());
    m7_config_reply(request.call.header.seq, 5000);
    service.drain_step();
    ASSERT_TRUE(service.config_ready());
    EXPECT_EQ(service.config_reply().code_maxsize, 114688);
    EXPECT_EQ(service.config_reply().stack_max, 16384);
    EXPECT_EQ(service.config_reply().m4_options, 2);
    EXPECT_EQ(m7_millis_calls.load(), 1U);
    EXPECT_EQ(m7_millis_given.load(), 5000U);
    // the same session's Ready again: no second Config, even after the timeout
    send_ready(7);
    millis_ahead += CALL_TIMEOUT_MS + 1;
    service.drain_step();
    service.housekeeping_step();
    EXPECT_FALSE(m7_receive_config(request));
    EXPECT_TRUE(service.config_ready());
    // a new session asks again, and keeps asking until it is answered
    send_ready(8);
    service.drain_step();
    service.housekeeping_step();
    EXPECT_FALSE(service.config_ready());
    ASSERT_TRUE(m7_receive_config(request));
    const uint32_t first_seq = request.call.header.seq;
    service.housekeeping_step();
    EXPECT_FALSE(m7_receive_config(request));
    millis_ahead += CALL_TIMEOUT_MS + 1;
    service.housekeeping_step();
    ASSERT_TRUE(m7_receive_config(request));
    EXPECT_NE(request.call.header.seq, first_seq);
    m7_config_reply(request.call.header.seq, 6000);
    service.drain_step();
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
    service.drain_step();
    auto send_control = [](uint32_t session, uint16_t action) {
        ControlMsg control {};
        fill_header(control.header, MsgType::Control, sizeof(control), session);
        control.module = 0;
        control.action = action;
        ASSERT_EQ(m7.send(&control, sizeof(control)), 0);
    };
    send_control(8, CONTROL_STOP);
    send_control(9, CONTROL_STOP + 1);
    service.drain_step();
    EXPECT_EQ(stops.load(), 0U);
    EXPECT_EQ(service.protocol_errors(), 2U);
    send_control(9, CONTROL_STOP);
    service.drain_step();
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
    step_both();
    const ParameterValue value { 0x20001000ULL, 3.5f, 0, PARAMETER_VALUE_BOUND, 0 };
    send_parameter_changed(7, &value, 1);
    step_both();
    uint8_t out[5];
    ASSERT_TRUE(service.mirror_get(0, 0x20001000ULL, out));
    EXPECT_EQ(out[0], 1U);
    EXPECT_EQ(service.mirror_hits(), 1U);
    EXPECT_FALSE(service.mirror_get(0, 0x20001010ULL, out));
    EXPECT_EQ(service.mirror_misses(), 1U);
}

// the window's mirror counts go after its Stats rows; the next window counts from zero
TEST(ModuleM4Service, StatsWindowSendsCounters)
{
    start();
    send_ready(7);
    step_both();
    const ParameterValue value { 0x20001000ULL, 3.5f, 0, PARAMETER_VALUE_BOUND, 0 };
    send_parameter_changed(7, &value, 1);
    step_both();
    uint8_t out[5];
    ASSERT_TRUE(service.mirror_get(0, 0x20001000ULL, out));
    EXPECT_FALSE(service.mirror_get(0, 0x20001010ULL, out));
    millis_ahead += 1000;
    step_both();
    CountersMsg counters;
    ASSERT_TRUE(m7_receive_counters(counters));
    EXPECT_EQ(counters.mirror_hits, 1U);
    EXPECT_EQ(counters.mirror_misses, 1U);
    EXPECT_EQ(counters.post_drops, 0U);
    millis_ahead += 1000;
    step_both();
    ASSERT_TRUE(m7_receive_counters(counters));
    EXPECT_EQ(counters.mirror_hits, 0U);
    EXPECT_EQ(counters.mirror_misses, 0U);
    millis_ahead = 0;
}

TEST(ModuleM4Service, ParameterChangedOfAnotherSessionIsDropped)
{
    start();
    send_ready(7);
    step_both();
    const ParameterValue value { 0x20001000ULL, 3.5f, 0, PARAMETER_VALUE_BOUND, 0 };
    send_parameter_changed(6, &value, 1);
    step_both();
    uint8_t out[5];
    EXPECT_FALSE(service.mirror_get(0, 0x20001000ULL, out));
    EXPECT_EQ(service.protocol_errors(), 1U);
}

TEST(ModuleM4Service, BadEntriesAreDroppedOthersApplied)
{
    start();
    send_ready(7);
    step_both();
    const ParameterValue values[3] = {
        { 0x20001000ULL, 1.0f, MAX_MODULES, PARAMETER_VALUE_BOUND, 0 },
        { 0x20001010ULL, 2.0f, 0, PARAMETER_VALUE_BOUND, 0 },
        { 0x20001020ULL, 3.0f, 0xFFFF, PARAMETER_VALUE_BOUND, 0 },
    };
    send_parameter_changed(7, values, 3);
    step_both();
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
    step_both();
    const ParameterValue value { 0x20001000ULL, 3.5f, 0, PARAMETER_VALUE_BOUND, 0 };
    send_parameter_changed(7, &value, 1);
    step_both();
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
    step_both();
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    uint8_t out[5];
    EXPECT_FALSE(service.mirror_get(0, 0x20001000ULL, out));
}

TEST(ModuleM4Service, ModuleEndForgetsItsEntries)
{
    start();
    send_ready(7);
    step_both();
    const ParameterValue value { 0x20001000ULL, 3.5f, 0, PARAMETER_VALUE_BOUND, 0 };
    send_parameter_changed(7, &value, 1);
    step_both();
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
    step_both();
    EXPECT_FALSE(service.mirror_get(0, 0x20001000ULL, out));
    EXPECT_EQ(service.protocol_errors(), 0U);
}

TEST(ModuleM4Service, ForwardedGetRefreshesAStaleEntryOnly)
{
    start();
    send_ready(7);
    step_both();
    const ParameterValue value { 0x20001000ULL, 3.5f, 0, PARAMETER_VALUE_BOUND, 0 };
    send_parameter_changed(7, &value, 1);
    step_both();
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
    step_both();
    const ParameterValue value { 0x20001000ULL, 3.5f, 0, PARAMETER_VALUE_BOUND, 0 };
    send_parameter_changed(7, &value, 1);
    step_both();
    service.mirror_pending(0, 0x20001000ULL);   // the module thread, before its INIT goes out
    uint8_t out[5];
    EXPECT_FALSE(service.mirror_get(0, 0x20001000ULL, out));
    const ParameterValue rebound { 0x20001000ULL, 1.0f, 0, PARAMETER_VALUE_BOUND, 0 };
    send_parameter_changed(7, &rebound, 1);
    step_both();
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
    step_both();
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
    step_both();   // the Stats window
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
    step_both();
    millis_ahead = millis_ahead.load() + CALL_TIMEOUT_MS + 1;
    step_both();
    EXPECT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(service.post_drops(), 1U);
}

TEST(ModuleM4Service, ProbeIsOnOnlyWhenTheConfigAsksForIt)
{
    M4CallProbe probe {};
    probe.clock_us = probe_clock;
    probe.enabled.store(true);
    probe.last.seq = 9;
    start(&probe);
    EXPECT_FALSE(probe.enabled.load());   // a new session starts without
    EXPECT_EQ(probe.last.seq, 0U);
    configure(2);
    EXPECT_FALSE(probe.enabled.load());
    start(&probe);
    configure(2 | M4_OPTION_CALL_PROBE);
    EXPECT_TRUE(probe.enabled.load());
    // no counter: never on
    probe.clock_us = nullptr;
    start(&probe);
    configure(M4_OPTION_CALL_PROBE);
    EXPECT_FALSE(probe.enabled.load());
}

TEST(ModuleM4Service, ProbedCallIsStampedAtSendAndReply)
{
    M4CallProbe probe {};
    probe.clock_us = probe_clock;
    probe_now = 0;
    start(&probe);
    EXPECT_EQ(probe.stamp_cycles, 0U);   // the test's cycle counter stands still
    configure(M4_OPTION_CALL_PROBE);
    post_probed_call(probe);
    ASSERT_TRUE(service.forward_posted());
    uint8_t frame[MAX_FRAME];
    uint16_t len = 0;
    ASSERT_TRUE(m7_receive(frame, len));
    BindingCallHeader call;
    ASSERT_TRUE(parse_binding_call(frame, len, call));
    CallProbeTrailer trailer;
    ASSERT_TRUE(binding_call_trailer(frame, len, call, trailer));
    EXPECT_EQ(trailer.svc_entry_us, 5U);
    EXPECT_NE(trailer.pre_send_us, 0U);
    EXPECT_EQ(probe.current.seq, call.header.seq);
    EXPECT_GT(probe.current.sent_us, trailer.pre_send_us);
    EXPECT_EQ(probe.current.flags, 0);   // sent by the module thread
    // the reply after a wait that ended on the poll timeout
    service.note_wake(true);
    m7_reply(call.header.seq, 0x11);
    service.drain_step();
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_GT(probe.current.reply_seen_us, probe.current.sent_us);
    EXPECT_EQ(probe.current.flags, PROBE_M4_POLL_WAKE);
    // no doorbell stamps; complete() signalled the module thread after the reply was seen
    EXPECT_EQ(probe.current.stamps, PROBE_STAMP_SIGNALLED);
    EXPECT_GT(probe.current.signalled_us, probe.current.reply_seen_us);
    call_link.state.store(ModuleLink::Idle);
    // a call the service's step sends, answered after a stamped doorbell
    post_probed_call(probe);
    service.note_wake(false);
    service.drain_step();
    ASSERT_EQ(call_link.state.load(), ModuleLink::Sent);
    ASSERT_TRUE(m7_receive(frame, len));
    ASSERT_TRUE(parse_binding_call(frame, len, call));
    m7_reply(call.header.seq, 0x11);
    service.note_wake(false, true, 4000, 4007);
    service.drain_step();
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(probe.current.flags, PROBE_M4_SENT_BY_STEP);
    EXPECT_EQ(probe.current.stamps, PROBE_STAMP_REPLY_WAKE | PROBE_STAMP_SIGNALLED);
    EXPECT_EQ(probe.current.reply_isr_us, 4000U);
    EXPECT_EQ(probe.current.reply_wake_us, 4007U);
}

// a cycle counter that moves on by 5 per read
uint32_t probe_cycles_now_value;
uint32_t probe_cycles_fake()
{
    probe_cycles_now_value += 5;
    return probe_cycles_now_value;
}

uint16_t part_bit(M4CyclePart part)
{
    return uint16_t(1U << uint8_t(part));
}

// the send and drain of a probed call are split into cycle parts
TEST(ModuleM4Service, ProbedCallTakesItsSendAndDrainCycles)
{
    M4CallProbe probe {};
    probe.clock_us = probe_clock;
    probe.cycles = probe_cycles_fake;
    probe_now = 0;
    start(&probe);
    configure(M4_OPTION_CALL_PROBE);
    post_probed_call(probe);
    ASSERT_TRUE(service.forward_posted());
    // the M7 is awake: a ring write, no doorbell
    const uint16_t sent = part_bit(M4CyclePart::FwdLock) | part_bit(M4CyclePart::FwdRing) |
                          part_bit(M4CyclePart::FwdUnlock);
    EXPECT_EQ(probe.current.cycles_taken, sent);
    EXPECT_EQ(probe.current.cycles[uint8_t(M4CyclePart::FwdLock)], 5);
    EXPECT_EQ(probe.current.cycles[uint8_t(M4CyclePart::FwdRing)], 5);
    EXPECT_EQ(probe.current.cycles[uint8_t(M4CyclePart::FwdUnlock)], 5);
    BindingCallHeader call;
    ASSERT_TRUE(m7_receive_call(call));
    m7_reply(call.header.seq, 0x11);
    // the module thread drains it, woken 123 cycles after the doorbell interrupt
    EXPECT_FALSE(service.drain_step_for_module(false, true, 4000, 4003, 123));
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    const uint16_t drained = part_bit(M4CyclePart::IsrToThread) | part_bit(M4CyclePart::DrainLock) |
                             part_bit(M4CyclePart::DrainRing) | part_bit(M4CyclePart::DrainParse) |
                             part_bit(M4CyclePart::DrainCopy);
    EXPECT_EQ(probe.current.cycles_taken, sent | drained);
    EXPECT_EQ(probe.current.cycles[uint8_t(M4CyclePart::IsrToThread)], 123);
    EXPECT_EQ(probe.current.cycles[uint8_t(M4CyclePart::DrainLock)], 5);
    EXPECT_EQ(probe.current.cycles[uint8_t(M4CyclePart::DrainRing)], 5);
    EXPECT_EQ(probe.current.cycles[uint8_t(M4CyclePart::DrainCopy)], 5);
    EXPECT_GE(probe.current.cycles[uint8_t(M4CyclePart::DrainParse)], 5);
    EXPECT_NE(probe.current.drain_copied_cycles, 0U);
    EXPECT_LE(probe.current.drain_copied_cycles, probe_cycles_now_value);
    EXPECT_EQ(memcmp(call_link.reply + sizeof(BindingReplyHeader), "\x11\x11\x11\x11", 4), 0);
    // the IsrToThread of one wait belongs to one reply; the service's drain takes its own lock
    call_link.state.store(ModuleLink::Idle);
    post_probed_call(probe);
    ASSERT_TRUE(service.forward_posted());
    ASSERT_TRUE(m7_receive_call(call));
    m7_reply(call.header.seq, 0x11);
    service.drain_step();
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(probe.current.cycles_taken, sent | (drained & uint16_t(~part_bit(M4CyclePart::IsrToThread))));
    // probe off: no part, no read
    call_link.state.store(ModuleLink::Idle);
    probe.enabled.store(false);
    probe.current = M4CallStamps {};
    const uint32_t reads = probe_cycles_now_value;
    post_call(BINDING);
    ASSERT_TRUE(service.forward_posted());
    ASSERT_TRUE(m7_receive_call(call));
    m7_reply(call.header.seq, 0x11);
    service.drain_step();
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(probe.current.cycles_taken, 0);
    EXPECT_EQ(probe_cycles_now_value, reads);
}

// the probe's ping goes out as a Ping system call with its trailer and is completed by its SystemReply
TEST(ModuleM4Service, ProbedPingIsSentWithItsTrailer)
{
    M4CallProbe probe {};
    probe.clock_us = probe_clock;
    probe.cycles = probe_cycles_fake;
    probe_now = 0;
    start(&probe);
    configure(M4_OPTION_CALL_PROBE);
    const CallProbeTrailer trailer = begin_probed_call(probe, 5);
    call_link.request_len = pack_ping_call(call_link.request, MAX_FRAME, &trailer);
    call_link.state.store(ModuleLink::Posted);
    ASSERT_TRUE(service.forward_posted());
    ASSERT_EQ(call_link.state.load(), ModuleLink::Sent);
    uint8_t frame[MAX_FRAME];
    uint16_t len = 0;
    ASSERT_TRUE(m7_receive(frame, len));
    SystemCallHeader call;
    ASSERT_EQ(len, sizeof(call) + sizeof(trailer));
    memcpy(&call, frame, sizeof(call));
    EXPECT_EQ(call.method, uint16_t(SystemMethod::Ping));
    CallProbeTrailer sent;
    ASSERT_TRUE(ping_call_trailer(frame, len, call, sent));
    EXPECT_EQ(sent.svc_entry_us, 5U);
    EXPECT_NE(sent.pre_send_us, 0U);
    EXPECT_EQ(probe.current.seq, call.header.seq);
    EXPECT_NE(probe.current.cycles_taken & part_bit(M4CyclePart::FwdRing), 0);
    SystemReplyHeader reply {};
    fill_header(reply.header, MsgType::SystemReply, sizeof(reply), call.header.seq);
    reply.method = uint16_t(SystemMethod::Ping);
    ASSERT_EQ(m7.send(&reply, sizeof(reply)), 0);
    service.drain_step();
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(call_link.result, int32_t(sizeof(reply)));
    EXPECT_NE(probe.current.reply_seen_us, 0U);
    EXPECT_NE(probe.current.cycles_taken & part_bit(M4CyclePart::DrainCopy), 0);
    EXPECT_EQ(ping_reply_status(call_link.reply, uint16_t(call_link.result)), STATUS_OK);
    // without its trailer: sent, not probed
    call_link.state.store(ModuleLink::Idle);
    probe.current = M4CallStamps {};
    call_link.request_len = pack_ping_call(call_link.request, MAX_FRAME);
    call_link.state.store(ModuleLink::Posted);
    ASSERT_TRUE(service.forward_posted());
    ASSERT_TRUE(m7_receive(frame, len));
    EXPECT_EQ(len, sizeof(call));
    EXPECT_EQ(probe.current.seq, 0U);
}

// the module thread's drain stamps its own wait, and leaves the service's note to the service
TEST(ModuleM4Service, ModuleDrainStampsItsOwnWait)
{
    M4CallProbe probe {};
    probe.clock_us = probe_clock;
    probe_now = 0;
    start(&probe);
    configure(M4_OPTION_CALL_PROBE);
    service.note_wake(false, true, 1000, 1001);
    post_probed_call(probe);
    ASSERT_TRUE(service.forward_posted());
    BindingCallHeader call;
    ASSERT_TRUE(m7_receive_call(call));
    m7_reply(call.header.seq, 0x11);
    EXPECT_FALSE(service.drain_step_for_module(false, true, 4000, 4003));
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(probe.current.flags, 0);
    EXPECT_EQ(probe.current.stamps, PROBE_STAMP_REPLY_WAKE | PROBE_STAMP_SIGNALLED);
    EXPECT_EQ(probe.current.reply_isr_us, 4000U);
    EXPECT_EQ(probe.current.reply_wake_us, 4003U);
    // the service's next reply still takes the service's note
    call_link.state.store(ModuleLink::Idle);
    post_probed_call(probe);
    ASSERT_TRUE(service.forward_posted());
    ASSERT_TRUE(m7_receive_call(call));
    m7_reply(call.header.seq, 0x11);
    service.drain_step();
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(probe.current.reply_isr_us, 1000U);
    EXPECT_EQ(probe.current.reply_wake_us, 1001U);
    // a module wait that timed out marks the reply as a poll wake
    call_link.state.store(ModuleLink::Idle);
    post_probed_call(probe);
    ASSERT_TRUE(service.forward_posted());
    ASSERT_TRUE(m7_receive_call(call));
    m7_reply(call.header.seq, 0x11);
    EXPECT_FALSE(service.drain_step_for_module(true, false, 0, 0));
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(probe.current.flags, PROBE_M4_POLL_WAKE);
    EXPECT_EQ(probe.current.stamps, PROBE_STAMP_SIGNALLED);
}

// a fault drained by the module thread is injected by the service's next drain_step()
TEST(ModuleM4Service, ModuleDrainLeavesFaultsToTheService)
{
    start();
    send_ready(7);
    service.drain_step();
    FaultInjectMsg inject {};
    fill_header(inject.header, MsgType::FaultInject, sizeof(inject), 7);
    inject.kind = uint32_t(FaultKind::Stall);
    ASSERT_EQ(m7.send(&inject, sizeof(inject)), 0);
    EXPECT_TRUE(service.drain_step_for_module(false, false, 0, 0));
    EXPECT_EQ(injected.load(), 0U);
    service.drain_step();
    EXPECT_EQ(injected.load(), uint32_t(FaultKind::Stall));
    injected = 0;
    service.drain_step();   // once
    EXPECT_EQ(injected.load(), 0U);
    // drained by the service, injected at once
    ASSERT_EQ(m7.send(&inject, sizeof(inject)), 0);
    service.drain_step();
    EXPECT_EQ(injected.load(), uint32_t(FaultKind::Stall));
}

// the module thread arms the doorbell again before it sleeps: false while a frame waits
TEST(ModuleM4Service, ModuleSleepIsRefusedWhileAFrameWaits)
{
    start();
    EXPECT_TRUE(service.prepare_module_sleep());
    m4.wake();   // no doorbell in this process
    send_ready(7);
    EXPECT_FALSE(service.prepare_module_sleep());
    service.drain_step();
    EXPECT_TRUE(service.prepare_module_sleep());
    m4.wake();
}

TEST(ModuleM4Service, UnprobedCallsLeaveTheProbeAlone)
{
    M4CallProbe probe {};
    probe.clock_us = probe_clock;
    probe_now = 0;
    start(&probe);
    configure(M4_OPTION_CALL_PROBE);
    post_call(BINDING);   // no trailer: the frame had no room, or the call is not probed
    ASSERT_TRUE(service.forward_posted());
    BindingCallHeader call;
    ASSERT_TRUE(m7_receive_call(call));
    m7_reply(call.header.seq, 0x11);
    service.drain_step();
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(probe.current.seq, 0U);
    EXPECT_EQ(probe.current.reply_seen_us, 0U);
}

// the reply on the ring completes the call in drain_step(), which leaves the Stats window, the
// call timeouts and the Config to housekeeping_step() (fails if drain_step() runs them)
TEST(ModuleM4Service, DrainStepCompletesACallWithoutHousekeeping)
{
    start();
    send_ready(7);
    service.drain_step();   // takes Ready: the Config is housekeeping's
    ConfigRequest request;
    EXPECT_FALSE(m7_receive_config(request));
    post_call(BINDING);
    ASSERT_TRUE(service.forward_posted());
    BindingCallHeader call;
    ASSERT_TRUE(m7_receive_call(call));
    m7_reply(call.header.seq, 0x22);
    millis_ahead += 1000;   // a Stats window is due
    service.drain_step();
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(call_link.reply[sizeof(BindingReplyHeader)], 0x22);
    EXPECT_EQ(module_wakes.load(), 1U);
    uint8_t frame[MAX_FRAME];
    uint16_t len = 0;
    EXPECT_NE(m7.receive(frame, sizeof(frame), &len), 0);   // no Stats row, no Config
    // a Sent call past its timeout stays Sent
    call_link.state.store(ModuleLink::Idle);
    post_call(BINDING);
    ASSERT_TRUE(service.forward_posted());
    ASSERT_TRUE(m7_receive_call(call));
    millis_ahead += CALL_TIMEOUT_MS + 1;
    service.drain_step();
    EXPECT_EQ(call_link.state.load(), ModuleLink::Sent);
    EXPECT_EQ(service.lost(), 0U);
    // a call the module thread could not send is sent by drain_step()
    m7_reply(call.header.seq, 0x33);
    service.drain_step();
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    call_link.state.store(ModuleLink::Idle);
    post_call(BINDING);
    service.drain_step();
    EXPECT_EQ(call_link.state.load(), ModuleLink::Sent);
    millis_ahead = 0;
}

// an unanswered Sent call times out in housekeeping_step(), as it did in step()
TEST(ModuleM4Service, HousekeepingStepStillTimesOutCalls)
{
    start();
    send_ready(7);
    service.drain_step();
    post_call(BINDING);
    ASSERT_TRUE(service.forward_posted());
    BindingCallHeader call;
    ASSERT_TRUE(m7_receive_call(call));
    service.housekeeping_step();
    EXPECT_EQ(call_link.state.load(), ModuleLink::Sent);
    millis_ahead += CALL_TIMEOUT_MS + 1;
    service.housekeeping_step();
    ASSERT_EQ(call_link.state.load(), ModuleLink::Done);
    EXPECT_EQ(call_link.result, -ETIMEDOUT);
    EXPECT_EQ(service.lost(), 1U);
    millis_ahead = 0;
}

#endif  // AP_MODULE_M4_ENABLED

AP_GTEST_MAIN()
