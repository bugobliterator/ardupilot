#include <AP_gtest.h>
#include <AP_Scripting/AP_ScriptingM4_Protocol.h>

using namespace ScriptingM4;

TEST(ScriptingM4Protocol, AcceptsWholeKnownMessages)
{
    uint8_t buf[MAX_FRAME] {};
    MsgHeader header;
    ReadyMsg ready {};
    fill_header(ready.header, MsgType::Ready, sizeof(ready), 7);
    memcpy(buf, &ready, sizeof(ready));
    EXPECT_TRUE(parse_header(buf, sizeof(ready), header));
    EXPECT_EQ(header.seq, 7U);
    EXPECT_EQ(header.type, uint8_t(MsgType::Ready));
    // a Reply carries values after its header: any length from the header up to MAX_FRAME
    ReplyHeader reply {};
    fill_header(reply.header, MsgType::Reply, sizeof(reply) + 5, 9);
    memcpy(buf, &reply, sizeof(reply));
    EXPECT_TRUE(parse_header(buf, sizeof(reply) + 5, header));
    CallHeader call {};
    fill_header(call.header, MsgType::Notify, sizeof(call), 3);
    memcpy(buf, &call, sizeof(call));
    EXPECT_TRUE(parse_header(buf, sizeof(call), header));
}

TEST(ScriptingM4Protocol, RejectsShortWrongLengthAndUnknown)
{
    uint8_t buf[MAX_FRAME + 4] {};
    MsgHeader header;
    EXPECT_FALSE(parse_header(buf, 3, header));                       // shorter than a header
    ReadyMsg ready {};
    fill_header(ready.header, MsgType::Ready, sizeof(ready), 1);
    memcpy(buf, &ready, sizeof(ready));
    EXPECT_FALSE(parse_header(buf, sizeof(ready) + 4, header));       // trailing bytes
    CallHeader call {};
    fill_header(call.header, MsgType::Call, sizeof(call) - 1, 1);
    memcpy(buf, &call, sizeof(call));
    EXPECT_FALSE(parse_header(buf, sizeof(call) - 1, header));        // shorter than a call header
    fill_header(call.header, MsgType::Call, MAX_FRAME + 4, 1);
    memcpy(buf, &call, sizeof(call));
    EXPECT_FALSE(parse_header(buf, MAX_FRAME + 4, header));           // longer than a frame
    fill_header(call.header, MsgType::Call, sizeof(call), 1);
    call.header.type = 99;
    memcpy(buf, &call, sizeof(call));
    EXPECT_FALSE(parse_header(buf, sizeof(call), header));            // unknown type
    call.header.type = 1;                                             // the retired Ping
    memcpy(buf, &call, sizeof(call));
    EXPECT_FALSE(parse_header(buf, sizeof(call), header));
}

TEST(ScriptingM4Protocol, FaultNames)
{
    EXPECT_STREQ(fault_name(0x00010000, 0x40000000), "HardFault");   // UDF, escalated (HFSR.FORCED)
    EXPECT_STREQ(fault_name(0x00000082, 0), "MemManage");             // DACCVIOL | MMARVALID
    EXPECT_STREQ(fault_name(0x00008200, 0), "BusFault");              // PRECISERR | BFARVALID
    EXPECT_STREQ(fault_name(0x00010000, 0), "UsageFault");            // UNDEFINSTR
    EXPECT_STREQ(fault_name(0, 0), "HardFault");
    EXPECT_TRUE(lua_thread_fault(FaultKind::LuaAircr));
    EXPECT_FALSE(lua_thread_fault(FaultKind::Flood));
}

TEST(ScriptingM4Protocol, HandlerFaultIsAServiceFault)
{
    EXPECT_EQ(uint8_t(FaultKind::HandlerFault), 11);
    EXPECT_EQ(FAULT_KIND_MAX, 11);
    EXPECT_FALSE(lua_thread_fault(FaultKind::HandlerFault));   // the service pends it, not the Lua thread
}

#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
TEST(ScriptingM4Protocol, NetConfigMessagesHaveFixedSizes)
{
    uint8_t buf[MAX_FRAME] {};
    MsgHeader header;
    NetConfigRequestMsg request {};
    fill_header(request.header, MsgType::NetConfigRequest, sizeof(request), 1);
    memcpy(buf, &request, sizeof(request));
    EXPECT_TRUE(parse_header(buf, sizeof(request), header));
    NetConfigMsg config {};
    fill_header(config.header, MsgType::NetConfig, sizeof(config), 2);
    memcpy(buf, &config, sizeof(config));
    EXPECT_TRUE(parse_header(buf, sizeof(config), header));
    fill_header(config.header, MsgType::NetConfig, sizeof(config) - 1, 2);
    memcpy(buf, &config, sizeof(config) - 1);
    EXPECT_FALSE(parse_header(buf, sizeof(config) - 1, header));
}
#else
TEST(ScriptingM4Protocol, PlainBuildsRejectNetConfigTypes)
{
    uint8_t buf[MAX_FRAME] {};
    MsgHeader header;
    ReadyMsg ready {};
    for (uint8_t type = 9; type <= 10; type++) {
        fill_header(ready.header, MsgType::Ready, sizeof(ready), 1);
        ready.header.type = type;
        memcpy(buf, &ready, sizeof(ready));
        EXPECT_FALSE(parse_header(buf, sizeof(ready), header));
    }
}
#endif

AP_GTEST_MAIN()
