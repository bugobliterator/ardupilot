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

TEST(ScriptingM4Protocol, FaultOnStacking)
{
    EXPECT_TRUE(fault_on_stacking(0x00000093));    // MSTKERR | MMARVALID | DACCVIOL
    EXPECT_TRUE(fault_on_stacking(0x00000010));    // MSTKERR
    EXPECT_TRUE(fault_on_stacking(0x00001000));    // STKERR
    EXPECT_FALSE(fault_on_stacking(0x00000082));   // DACCVIOL | MMARVALID
    EXPECT_FALSE(fault_on_stacking(0x00008200));   // PRECISERR | BFARVALID
}

TEST(ScriptingM4Protocol, FaultReasonText)
{
    char text[80];
    // a Lua stack overflow: the exception frame went below the Lua stack guard
    fault_reason_text(text, sizeof(text), 0x93, 0, 0, 0x30007F60);
    EXPECT_STREQ(text, "MemManage on stacking SP=0x30007F60 CFSR=0x00000093 (Lua stack overflow)");
    // in the guard itself
    fault_reason_text(text, sizeof(text), 0x93, 0, 0, 0x30008000);
    EXPECT_STREQ(text, "MemManage on stacking SP=0x30008000 CFSR=0x00000093 (Lua stack overflow)");
    // a stacking fault with an SP elsewhere: no overflow suffix
    fault_reason_text(text, sizeof(text), 0x93, 0, 0, 0x3000A000);
    EXPECT_STREQ(text, "MemManage on stacking SP=0x3000A000 CFSR=0x00000093");
    fault_reason_text(text, sizeof(text), 0x93, 0, 0, 0x30008020);   // just above the guard
    EXPECT_STREQ(text, "MemManage on stacking SP=0x30008020 CFSR=0x00000093");
    fault_reason_text(text, sizeof(text), 0x10, 0, 0, 0x300007E0);   // the exception stack
    EXPECT_STREQ(text, "MemManage on stacking SP=0x300007E0 CFSR=0x00000010");
    fault_reason_text(text, sizeof(text), 0x1000, 0, 0, 0x30007F60);
    EXPECT_STREQ(text, "BusFault on stacking SP=0x30007F60 CFSR=0x00001000 (Lua stack overflow)");
    // every other fault keeps its PC text
    fault_reason_text(text, sizeof(text), 0x82, 0, 0x081D047C, 0x081D0461);
    EXPECT_STREQ(text, "MemManage PC=0x081D047C");
    fault_reason_text(text, sizeof(text), 0x00010000, 0x40000000, 0x081D0476, 0);
    EXPECT_STREQ(text, "HardFault PC=0x081D0476");
}

TEST(ScriptingM4Protocol, HandlerFaultIsAServiceFault)
{
    EXPECT_EQ(uint8_t(FaultKind::HandlerFault), 11);
    EXPECT_EQ(FAULT_KIND_MAX, 11);
    EXPECT_FALSE(lua_thread_fault(FaultKind::HandlerFault));   // the service pends it, not the Lua thread
}

AP_GTEST_MAIN()
