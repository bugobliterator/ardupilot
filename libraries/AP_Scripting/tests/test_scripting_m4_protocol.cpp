#include <AP_gtest.h>
#include <AP_Scripting/AP_ScriptingM4_Protocol.h>

using namespace ScriptingM4;

TEST(ScriptingM4Protocol, AcceptsWholeKnownMessages)
{
    PongMsg pong {};
    fill_header(pong.header, MsgType::Pong, sizeof(pong), 7);
    uint8_t buf[MAX_MESSAGE] {};
    memcpy(buf, &pong, sizeof(pong));
    MsgHeader header;
    EXPECT_TRUE(parse_header(buf, sizeof(pong), header));
    EXPECT_EQ(header.seq, 7U);
    EXPECT_EQ(header.type, uint8_t(MsgType::Pong));
}

TEST(ScriptingM4Protocol, RejectsShortWrongLengthAndUnknown)
{
    uint8_t buf[MAX_MESSAGE] {};
    MsgHeader header;
    EXPECT_FALSE(parse_header(buf, 3, header));                  // shorter than a header
    PingMsg ping {};
    fill_header(ping.header, MsgType::Ping, sizeof(ping), 1);
    memcpy(buf, &ping, sizeof(ping));
    EXPECT_FALSE(parse_header(buf, sizeof(ping) + 4, header));   // trailing bytes
    ping.header.length = sizeof(ping) + 4;
    memcpy(buf, &ping, sizeof(ping));
    EXPECT_FALSE(parse_header(buf, sizeof(ping), header));       // header length disagrees
    ping.header.length = sizeof(ping);
    ping.header.type = 99;
    memcpy(buf, &ping, sizeof(ping));
    EXPECT_FALSE(parse_header(buf, sizeof(ping), header));       // unknown type
}

TEST(ScriptingM4Protocol, FaultNames)
{
    EXPECT_STREQ(fault_name(0x00010000, 0x40000000), "HardFault");   // UDF, escalated (HFSR.FORCED)
    EXPECT_STREQ(fault_name(0x00000082, 0), "MemManage");             // DACCVIOL | MMARVALID
    EXPECT_STREQ(fault_name(0x00008200, 0), "BusFault");              // PRECISERR | BFARVALID
    EXPECT_STREQ(fault_name(0x00010000, 0), "UsageFault");            // UNDEFINSTR
    EXPECT_STREQ(fault_name(0, 0), "HardFault");
}

AP_GTEST_MAIN()
