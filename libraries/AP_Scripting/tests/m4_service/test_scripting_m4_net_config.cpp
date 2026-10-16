#include <AP_gtest.h>
#include <string.h>
#include "m4_service.h"
#include <AP_ScriptingM4_Layout.h>
#include <CubePilotFW/PeerHeader.h>

// the M4 service asks the M7 for NET_* once it is Ready, again every second until answered,
// takes the first answer of its session only, and asks again in the next session

using namespace ScriptingM4;
using CubePilotFW::Channel;

static uint32_t now_ms;
static uint32_t fake_millis() { return now_ms; }
static uint32_t fake_cycles() { return now_ms * 1000U; }
static void no_fault(FaultKind) {}
static void no_wake() {}
static uint32_t configs_taken;
static NetM4::Config config_taken;
static void take_config(const NetM4::Config &config)
{
    configs_taken++;
    config_taken = config;
}

alignas(64) static uint8_t window[WINDOW_SIZE];
static Channel m7;
static Channel m4;
static LuaLink lua_link;
static M4Service service;
static const M4Platform platform { fake_millis, fake_cycles, 1, no_fault, no_wake, take_config };

// NetConfigRequests the M7 end received since the last call (Stats and others are skipped)
static uint32_t requests()
{
    uint32_t count = 0;
    uint8_t frame[MAX_FRAME];
    uint16_t len = 0;
    while (m7.receive(frame, sizeof(frame), &len) == 0) {
        MsgHeader header;
        if (parse_header(frame, len, header) && MsgType(header.type) == MsgType::NetConfigRequest) {
            count++;
        }
    }
    return count;
}

static void send_ready()
{
    ReadyMsg ready;
    fill_header(ready.header, MsgType::Ready, sizeof(ready), 1);
    ASSERT_EQ(m7.send(&ready, sizeof(ready)), 0);
}

static void send_config(uint16_t port)
{
    NetConfigMsg msg {};
    fill_header(msg.header, MsgType::NetConfig, sizeof(msg), 2);
    msg.config.magic = NetM4::CONFIG_MAGIC;
    msg.config.enabled = 1;
    msg.config.ports[0].type = 1;
    msg.config.ports[0].port = port;
    ASSERT_EQ(m7.send(&msg, sizeof(msg)), 0);
}

static void start_session()
{
    service.init(m4, window, lua_link, platform);
}

TEST(ScriptingM4NetConfig, AskedUntilAnsweredOncePerSession)
{
    memset(window, 0, sizeof(window));
    ASSERT_EQ(m7.init(window, CHANNEL_WINDOW_SIZE, Channel::Core::M7, DOORBELL_GROUP), 0);
    ASSERT_EQ(m4.init(window, CHANNEL_WINDOW_SIZE, Channel::Core::M4, DOORBELL_GROUP), 0);
    now_ms = 50000;
    start_session();
    service.step();
    EXPECT_EQ(requests(), 0U);   // not before the M7's Ready
    send_ready();
    service.step();
    EXPECT_EQ(requests(), 1U);   // one request after Ready
    now_ms += 999;
    service.step();
    EXPECT_EQ(requests(), 0U);   // none at +999 ms
    now_ms += 1;
    service.step();
    EXPECT_EQ(requests(), 1U);   // one at +1000 ms
    // both requests answered: the first answer configures, the second is ignored
    send_config(14550);
    send_config(14560);
    service.step();
    EXPECT_EQ(configs_taken, 1U);
    EXPECT_EQ(config_taken.ports[0].port, 14550U);
    now_ms += 5000;
    service.step();
    EXPECT_EQ(requests(), 0U);   // answered: no more requests this session
    EXPECT_EQ(configs_taken, 1U);

    // a new session (init()) asks again once the M7 is Ready
    start_session();
    send_config(14570);          // an answer to the last session's request: not asked for yet
    service.step();
    EXPECT_EQ(configs_taken, 1U);
    send_ready();
    service.step();
    EXPECT_EQ(requests(), 1U);
    send_config(14580);
    service.step();
    EXPECT_EQ(configs_taken, 2U);
    EXPECT_EQ(config_taken.ports[0].port, 14580U);
}

// the system contract chain of AP_ScriptingM4_Rpc.h, with or without the Lua permit slot term
static uint32_t contract(bool with_permit)
{
    struct Info {
        uint16_t id;
        const char *name;
        MethodKind kind;
    };
    static const Info table[] = {
#define TEST_METHOD_INFO(id, symbol, name, kind) { id, name, MethodKind::kind },
        SCRIPTING_M4_SYSTEM_METHODS(TEST_METHOD_INFO)
#undef TEST_METHOD_INFO
    };
    uint32_t hash = fnv1a_word(AP_SCRIPTING_M4_CHANNEL_REGION_SIZE, FNV_OFFSET);
    hash = fnv1a_word(sizeof(NetM4::Config), hash);
    hash = fnv1a_word(NetM4::STATUS_LAYOUT_VERSION, hash);
    if (with_permit) {
        hash = fnv1a_word(LUA_PERMIT_SLOT, hash);
    }
    hash = fnv1a_word(MAX_VALUES, fnv1a_word(MAX_BLOB, fnv1a_word(RPC_PROTOCOL_VERSION,
           fnv1a_word(MAX_FRAME, fnv1a_word(sizeof(CallHeader), fnv1a_word(sizeof(ReplyHeader), hash))))));
    for (int i = int(sizeof(table) / sizeof(table[0])) - 1; i >= 0; i--) {
        hash = fnv1a_string(table[i].name, fnv1a_word((uint32_t(table[i].id) << 1) | uint32_t(table[i].kind), hash));
    }
    return fnv1a_word(BINDING_HASH, hash);
}

// an M4 image built before the Lua permit ignores reserved[LUA_PERMIT_SLOT] and would start Lua
// the M7 holds: its contract differs only by the permit term, and the networking M7 refuses it
TEST(ScriptingM4NetConfig, ImageWithoutThePermitIsRefused)
{
    namespace PeerHeader = CubePilotFW::PeerHeader;
    ASSERT_EQ(contract(true), CONTRACT_HASH);
    ASSERT_NE(contract(false), CONTRACT_HASH);
    alignas(64) static uint8_t headers[CHANNEL_WINDOW_SIZE];
    PeerHeader::View view {};
    PeerHeader::publish(headers, PeerHeader::Core::M4, PROTOCOL_VERSION, contract(false), 0);
    EXPECT_EQ(PeerHeader::readPeer(headers, PeerHeader::Core::M7, PROTOCOL_VERSION, CONTRACT_HASH, &view),
              PeerHeader::Status::Mismatch);
    PeerHeader::publish(headers, PeerHeader::Core::M4, PROTOCOL_VERSION, contract(true), 0);
    EXPECT_EQ(PeerHeader::readPeer(headers, PeerHeader::Core::M7, PROTOCOL_VERSION, CONTRACT_HASH, &view),
              PeerHeader::Status::Compatible);
}

AP_GTEST_MAIN()
