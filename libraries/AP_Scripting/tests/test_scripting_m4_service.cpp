#include <AP_gtest.h>
#include <AP_Scripting/AP_Scripting_config.h>
#if AP_SCRIPTING_M4_ENABLED
#include <errno.h>
#include <pthread.h>
#include <atomic>
#include <chrono>
#include <thread>
// the M4 service as the M4 image and the SITL M4 process build it (tests/wscript adds the
// include path its bare-name includes need)
#include <AP_Scripting/m4/m4_service.cpp>
// the M4's tables as gen-bindings -r wrote them for this tree (the Stats rows' labels)
#include <AP_Scripting/lua_remote_tables.cpp>
#include <AP_HAL/AP_HAL.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace ScriptingM4;
using CubePilotFW::Channel;

namespace {

// both cores' Channels on one window in this process; no doorbell is rung while neither sleeps
alignas(32) uint8_t window[WINDOW_SIZE];
Channel m7;
Channel m4;
LuaLink call_link;   // not `link`: <unistd.h> declares link()
M4Service service;
pthread_mutex_t service_mutex = PTHREAD_MUTEX_INITIALIZER;
std::atomic<uint32_t> lua_wakes;
const auto epoch = std::chrono::steady_clock::now();
// a test moves the service's clocks on: round trips take fake_cycles (1 cycle per us), and
// millis_ahead skips to the next Stats window
std::atomic<uint32_t> fake_cycles;
std::atomic<uint32_t> millis_ahead;

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
void wake_lua()
{
    lua_wakes++;
}
void lock_service()
{
    pthread_mutex_lock(&service_mutex);
}
void unlock_service()
{
    pthread_mutex_unlock(&service_mutex);
}

const uint16_t BINDING = FIRST_BINDING_METHOD;

void start()
{
    memset(window, 0, sizeof(window));
    memset(&call_link, 0, sizeof(call_link));
    ASSERT_EQ(m7.init(window, CHANNEL_WINDOW_SIZE, Channel::Core::M7, DOORBELL_GROUP), 0);
    ASSERT_EQ(m4.init(window, CHANNEL_WINDOW_SIZE, Channel::Core::M4, DOORBELL_GROUP), 0);
    lua_wakes = 0;
    const M4Platform platform { millis_now, cycles_now, 1, inject_fault, wake_lua, lock_service, unlock_service };
    service.init(m4, window, call_link, platform);
}

void send_ready()
{
    ReadyMsg ready;
    fill_header(ready.header, MsgType::Ready, sizeof(ready), 0);
    ASSERT_EQ(m7.send(&ready, sizeof(ready)), 0);
}

// the Lua side: a Call of method with no arguments, posted on the link
void post_call(uint16_t method)
{
    CallHeader call;
    fill_header(call.header, MsgType::Call, sizeof(call), 0);
    call.method = method;
    call.argc = 0;
    call.reserved = 0;
    memcpy(call_link.request, &call, sizeof(call));
    call_link.request_len = sizeof(call);
    call_link.state.store(LuaLink::Posted);
}

// the next M4 -> M7 frame, skipping Stats rows; false if the ring holds no other
bool m7_receive_call(CallHeader &call)
{
    uint8_t frame[MAX_FRAME];
    uint16_t len = 0;
    while (m7.receive(frame, sizeof(frame), &len) == 0) {
        MsgHeader header;
        if (!parse_header(frame, len, header)) {
            ADD_FAILURE() << "unparseable frame";
            return false;
        }
        if (MsgType(header.type) == MsgType::Stats) {
            continue;
        }
        memcpy(&call, frame, sizeof(call));
        return true;
    }
    return false;
}

void m7_reply(uint32_t seq)
{
    ReplyHeader reply {};
    fill_header(reply.header, MsgType::Reply, sizeof(reply), seq);
    reply.status = uint8_t(ReplyStatus::Ok);
    ASSERT_EQ(m7.send(&reply, sizeof(reply)), 0);
}

}  // namespace

TEST(ScriptingM4Service, PostedCallIsSentByTheLuaThread)
{
    start();
    send_ready();
    service.step();   // takes Ready
    post_call(BINDING);
    EXPECT_TRUE(service.forward_posted());
    EXPECT_EQ(call_link.state.load(), LuaLink::Sent);
    CallHeader call;
    ASSERT_TRUE(m7_receive_call(call));
    EXPECT_EQ(call.method, BINDING);
    EXPECT_EQ(MsgType(call.header.type), MsgType::Call);
    const uint32_t first_seq = call.header.seq;
    m7_reply(first_seq);
    service.step();
    EXPECT_EQ(call_link.state.load(), LuaLink::Done);
    EXPECT_EQ(call_link.result, int32_t(sizeof(ReplyHeader)));
    EXPECT_EQ(lua_wakes.load(), 1U);
    // the next call takes the next sequence number
    call_link.state.store(LuaLink::Idle);
    post_call(BINDING + 1);
    EXPECT_TRUE(service.forward_posted());
    ASSERT_TRUE(m7_receive_call(call));
    EXPECT_EQ(call.header.seq, first_seq + 1);
}

TEST(ScriptingM4Service, CallPostedBeforeReadyWaitsForTheService)
{
    start();
    post_call(BINDING);
    EXPECT_FALSE(service.forward_posted());
    EXPECT_EQ(call_link.state.load(), LuaLink::Posted);
    CallHeader call;
    EXPECT_FALSE(m7_receive_call(call));
    service.step();   // no Ready yet
    EXPECT_EQ(call_link.state.load(), LuaLink::Posted);
    send_ready();
    service.step();
    EXPECT_EQ(call_link.state.load(), LuaLink::Sent);
    ASSERT_TRUE(m7_receive_call(call));
    EXPECT_FALSE(m7_receive_call(call));   // sent once
}

TEST(ScriptingM4Service, FullRingFallsBackToTheService)
{
    start();
    send_ready();
    service.step();
    // fill the M4 -> M7 ring with Notify frames
    CallHeader filler;
    fill_header(filler.header, MsgType::Notify, sizeof(filler), 0);
    filler.method = uint16_t(SystemMethod::Noop);
    filler.argc = 0;
    filler.reserved = 0;
    uint32_t sent = 0;
    while (m4.send(&filler, sizeof(filler)) == 0) {
        sent++;
    }
    ASSERT_GT(sent, 0U);
    post_call(BINDING);
    EXPECT_FALSE(service.forward_posted());
    EXPECT_EQ(call_link.state.load(), LuaLink::Posted);
    // the M7 drains; the service's next step sends the call
    CallHeader call;
    uint8_t frame[MAX_FRAME];
    uint16_t len = 0;
    for (uint32_t i = 0; i < sent; i++) {
        ASSERT_EQ(m7.receive(frame, sizeof(frame), &len), 0);
    }
    service.step();
    EXPECT_EQ(call_link.state.load(), LuaLink::Sent);
    ASSERT_TRUE(m7_receive_call(call));
    EXPECT_EQ(call.method, BINDING);
}

TEST(ScriptingM4Service, NotifyCompletesAtOnce)
{
    start();
    send_ready();
    service.step();
    CallHeader notify;
    fill_header(notify.header, MsgType::Notify, sizeof(notify), 0);
    notify.method = BINDING;
    notify.argc = 0;
    notify.reserved = 0;
    memcpy(call_link.request, &notify, sizeof(notify));
    call_link.request_len = sizeof(notify);
    call_link.state.store(LuaLink::Posted);
    EXPECT_TRUE(service.forward_posted());
    EXPECT_EQ(call_link.state.load(), LuaLink::Done);
    EXPECT_EQ(call_link.result, 0);
}

TEST(ScriptingM4Service, BadFrameIsRefusedByTheLuaThread)
{
    start();
    send_ready();
    service.step();
    post_call(BINDING);
    call_link.request_len = sizeof(CallHeader) + 1;   // header length disagrees
    EXPECT_TRUE(service.forward_posted());
    EXPECT_EQ(call_link.state.load(), LuaLink::Done);
    EXPECT_EQ(call_link.result, -EINVAL);
}

// the Lua thread sends calls while the service thread steps (Stats once a second):
// every frame the M7 sees is whole and CRC-good, and every call is answered
TEST(ScriptingM4Service, LuaThreadAndServiceShareTheRing)
{
    start();
    send_ready();
    service.step();
    std::atomic<bool> running { true };
    std::atomic<uint32_t> replies { 0 };
    uint32_t calls = 0;
    uint32_t stats_rows = 0;
    uint32_t bad_frames = 0;
    std::thread service_thread([&]() {
        while (running.load()) {
            service.step();
        }
    });
    std::thread m7_thread([&]() {
        uint8_t frame[MAX_FRAME];
        uint16_t len = 0;
        while (running.load()) {
            const int ret = m7.receive(frame, sizeof(frame), &len);
            if (ret == -EAGAIN) {
                std::this_thread::yield();
                continue;
            }
            if (ret != 0) {
                bad_frames++;
                return;
            }
            MsgHeader header;
            if (!parse_header(frame, len, header)) {
                bad_frames++;
                continue;
            }
            if (MsgType(header.type) == MsgType::Stats) {
                stats_rows++;
                continue;
            }
            ReplyHeader reply {};
            fill_header(reply.header, MsgType::Reply, sizeof(reply), header.seq);
            while (m7.send(&reply, sizeof(reply)) != 0 && running.load()) {
                std::this_thread::yield();
            }
        }
    });
    // past one stats window
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
    while (std::chrono::steady_clock::now() < end) {
        post_call(BINDING + uint16_t(calls % 7));
        (void)service.forward_posted();   // if not sent, the service's next step sends it
        while (call_link.state.load() != LuaLink::Done) {
            std::this_thread::yield();
        }
        if (call_link.result != int32_t(sizeof(ReplyHeader))) {
            ADD_FAILURE() << "call " << calls << " ended " << call_link.result;
            break;
        }
        call_link.state.store(LuaLink::Idle);
        calls++;
        replies++;
    }
    running = false;
    service_thread.join();
    m7_thread.join();
    printf("%u calls, %u Stats rows\n", unsigned(calls), unsigned(stats_rows));
    EXPECT_EQ(bad_frames, 0U);
    EXPECT_GT(calls, 1000U);
    EXPECT_GT(stats_rows, 0U);
    EXPECT_EQ(replies.load(), calls);
}

// one window with more binding methods than STATS_REPORTED: the busiest have their own rows, the
// rest fold into m4:other and the system methods into m4:sysother, and every call is counted once
TEST(ScriptingM4Service, BusyWindowFoldsTheQuietestMethodsIntoOther)
{
    start();
    send_ready();
    service.step();
    const uint8_t BINDING_METHODS = 14;
    const uint32_t CALLS_EACH = 2;
    uint32_t calls = 0;
    auto round_trip = [&](uint16_t method, uint32_t round_trip_us) {
        post_call(method);
        ASSERT_TRUE(service.forward_posted());
        CallHeader call;
        ASSERT_TRUE(m7_receive_call(call));
        fake_cycles += round_trip_us;
        m7_reply(call.header.seq);
        service.step();
        ASSERT_EQ(call_link.state.load(), LuaLink::Done);
        call_link.state.store(LuaLink::Idle);
        calls++;
    };
    // binding method i takes 100 * (i + 1) us per call: totals all differ, the ranking is fixed
    for (uint8_t i = 0; i < BINDING_METHODS; i++) {
        for (uint32_t n = 0; n < CALLS_EACH; n++) {
            round_trip(BINDING + i, 100 * (i + 1));
        }
    }
    // two system methods, quicker than any binding
    round_trip(uint16_t(SystemMethod::ListScript), 10);
    round_trip(uint16_t(SystemMethod::ReadScript), 20);

    millis_ahead += 1000;
    service.step();
    StatsMsg rows[STATS_ROWS_PER_WINDOW + 1];
    uint8_t row_count = 0;
    uint8_t frame[MAX_FRAME];
    uint16_t len = 0;
    while (m7.receive(frame, sizeof(frame), &len) == 0) {
        MsgHeader header;
        ASSERT_TRUE(parse_header(frame, len, header));
        ASSERT_EQ(MsgType(header.type), MsgType::Stats);
        ASSERT_LT(row_count, STATS_ROWS_PER_WINDOW + 1);
        memcpy(&rows[row_count++], frame, sizeof(StatsMsg));
    }
    ASSERT_EQ(row_count, STATS_ROWS_PER_WINDOW);
    // the STATS_REPORTED largest totals, largest first
    for (uint8_t r = 0; r < STATS_REPORTED; r++) {
        const uint8_t i = BINDING_METHODS - 1 - r;
        EXPECT_EQ(rows[r].method, BINDING + i);
        EXPECT_EQ(rows[r].count, CALLS_EACH);
        EXPECT_EQ(rows[r].total_us, CALLS_EACH * 100 * (i + 1));
    }
    // the 6 quietest binding methods in m4:other
    const StatsMsg &other = rows[STATS_REPORTED];
    EXPECT_EQ(other.method, uint16_t(SystemMethod::Other));
    EXPECT_STREQ(other.name, "m4:other");
    const uint8_t folded = BINDING_METHODS - STATS_REPORTED;
    EXPECT_EQ(other.count, folded * CALLS_EACH);
    EXPECT_EQ(other.total_us, CALLS_EACH * 100 * (folded * (folded + 1) / 2));
    // the system calls in m4:sysother, never in m4:other
    const StatsMsg &system_other = rows[STATS_REPORTED + 1];
    EXPECT_EQ(system_other.method, uint16_t(SystemMethod::SystemOther));
    EXPECT_STREQ(system_other.name, "m4:sysother");
    EXPECT_EQ(system_other.count, 2U);
    EXPECT_EQ(system_other.total_us, 30U);
    uint32_t counted = 0;
    for (uint8_t r = 0; r < row_count; r++) {
        counted += rows[r].count;
    }
    EXPECT_EQ(counted, calls);
    fake_cycles = 0;
}

#endif  // AP_SCRIPTING_M4_ENABLED

AP_GTEST_MAIN()
