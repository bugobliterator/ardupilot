#include <AP_gtest.h>
#include <AP_ModuleM4/AP_ModuleM4_Protocol.h>

using namespace ModuleM4;

TEST(ModuleM4Protocol, AcceptsWholeKnownMessages)
{
    uint8_t buf[MAX_FRAME] {};
    MsgHeader header;
    ReadyMsg ready {};
    fill_header(ready.header, MsgType::Ready, sizeof(ready), 7);
    memcpy(buf, &ready, sizeof(ready));
    EXPECT_TRUE(parse_header(buf, sizeof(ready), header));
    EXPECT_EQ(header.seq, 7U);
    EXPECT_EQ(header.type, uint8_t(MsgType::Ready));
    StatsMsg stats {};
    fill_header(stats.header, MsgType::Stats, sizeof(stats), 8);
    memcpy(buf, &stats, sizeof(stats));
    EXPECT_TRUE(parse_header(buf, sizeof(stats), header));
    FaultInjectMsg inject {};
    fill_header(inject.header, MsgType::FaultInject, sizeof(inject), 9);
    memcpy(buf, &inject, sizeof(inject));
    EXPECT_TRUE(parse_header(buf, sizeof(inject), header));
}

TEST(ModuleM4Protocol, RejectsShortWrongLengthAndUnknown)
{
    uint8_t buf[MAX_FRAME + 4] {};
    MsgHeader header;
    EXPECT_FALSE(parse_header(buf, 3, header));                       // shorter than a header
    ReadyMsg ready {};
    fill_header(ready.header, MsgType::Ready, sizeof(ready), 1);
    memcpy(buf, &ready, sizeof(ready));
    EXPECT_FALSE(parse_header(buf, sizeof(ready) + 4, header));       // trailing bytes
    StatsMsg stats {};
    fill_header(stats.header, MsgType::Stats, sizeof(stats) - 1, 1);
    memcpy(buf, &stats, sizeof(stats));
    EXPECT_FALSE(parse_header(buf, sizeof(stats) - 1, header));       // shorter than its type
    fill_header(ready.header, MsgType::Ready, MAX_FRAME + 4, 1);
    memcpy(buf, &ready, sizeof(ready));
    EXPECT_FALSE(parse_header(buf, MAX_FRAME + 4, header));           // longer than a frame
    // unknown types: the retired Ping, Lua's Call, Reply and Notify, anything else
    for (uint8_t type : { 0, 1, 2, 5, 6, 7, 14, 99 }) {
        fill_header(ready.header, MsgType::Ready, sizeof(ready), 1);
        ready.header.type = type;
        memcpy(buf, &ready, sizeof(ready));
        EXPECT_FALSE(parse_header(buf, sizeof(ready), header)) << unsigned(type);
        ready.header.length = 64;
        memcpy(buf, &ready, sizeof(ready));
        EXPECT_FALSE(parse_header(buf, 64, header)) << unsigned(type);
    }
}

TEST(ModuleM4Protocol, FaultNames)
{
    EXPECT_STREQ(fault_name(0x00010000, 0x40000000), "HardFault");   // UDF, escalated (HFSR.FORCED)
    EXPECT_STREQ(fault_name(0x00000082, 0), "MemManage");             // DACCVIOL | MMARVALID
    EXPECT_STREQ(fault_name(0x00008200, 0), "BusFault");              // PRECISERR | BFARVALID
    EXPECT_STREQ(fault_name(0x00010000, 0), "UsageFault");            // UNDEFINSTR
    EXPECT_STREQ(fault_name(0, 0), "HardFault");
}

TEST(ModuleM4Protocol, FaultOnStacking)
{
    EXPECT_TRUE(fault_on_stacking(0x00000092));    // MMARVALID | MSTKERR | DACCVIOL
    EXPECT_TRUE(fault_on_stacking(0x00000010));    // MSTKERR
    EXPECT_TRUE(fault_on_stacking(0x00001000));    // STKERR
    EXPECT_FALSE(fault_on_stacking(0x00000082));   // DACCVIOL | MMARVALID
    EXPECT_FALSE(fault_on_stacking(0x00008200));   // PRECISERR | BFARVALID
}

TEST(ModuleM4Protocol, FaultReasonText)
{
    char text[80];
    // a module stack at 0x30030000 (its guard: 0x30030000 to 0x3003001F)
    const uint32_t base = 0x30030000;
    // a module stack overflow: the exception frame went below the guard
    fault_reason_text(text, sizeof(text), 0x92, 0, 0, 0x3002FF60, base);
    EXPECT_STREQ(text, "MemManage on stacking SP=0x3002FF60 CFSR=0x00000092 (module stack overflow)");
    // in the guard itself
    fault_reason_text(text, sizeof(text), 0x92, 0, 0, 0x30030000, base);
    EXPECT_STREQ(text, "MemManage on stacking SP=0x30030000 CFSR=0x00000092 (module stack overflow)");
    fault_reason_text(text, sizeof(text), 0x92, 0, 0, 0x3003001C, base);
    EXPECT_STREQ(text, "MemManage on stacking SP=0x3003001C CFSR=0x00000092 (module stack overflow)");
    // the edges of the window: MODULE_STACK_OVERFLOW_REACH below, the guard's end above
    fault_reason_text(text, sizeof(text), 0x92, 0, 0, base - MODULE_STACK_OVERFLOW_REACH, base);
    EXPECT_STREQ(text, "MemManage on stacking SP=0x3002FC00 CFSR=0x00000092 (module stack overflow)");
    fault_reason_text(text, sizeof(text), 0x92, 0, 0, base - MODULE_STACK_OVERFLOW_REACH - 4, base);
    EXPECT_STREQ(text, "MemManage on stacking SP=0x3002FBFC CFSR=0x00000092");
    fault_reason_text(text, sizeof(text), 0x92, 0, 0, base + AP_MODULE_M4_STACK_GUARD_SIZE, base);   // just above
    EXPECT_STREQ(text, "MemManage on stacking SP=0x30030020 CFSR=0x00000092");
    // a stacking fault with an SP elsewhere: no overflow suffix
    fault_reason_text(text, sizeof(text), 0x92, 0, 0, 0x3000A000, base);
    EXPECT_STREQ(text, "MemManage on stacking SP=0x3000A000 CFSR=0x00000092");
    fault_reason_text(text, sizeof(text), 0x10, 0, 0, 0x300007E0, base);   // the exception stack
    EXPECT_STREQ(text, "MemManage on stacking SP=0x300007E0 CFSR=0x00000010");
    fault_reason_text(text, sizeof(text), 0x1000, 0, 0, 0x3002FF60, base);
    EXPECT_STREQ(text, "BusFault on stacking SP=0x3002FF60 CFSR=0x00001000 (module stack overflow)");
    // no module stack (0): never an overflow
    fault_reason_text(text, sizeof(text), 0x92, 0, 0, 0x00000010, 0);
    EXPECT_STREQ(text, "MemManage on stacking SP=0x00000010 CFSR=0x00000092");
    fault_reason_text(text, sizeof(text), 0x92, 0, 0, 0xFFFFFF00, 0);
    EXPECT_STREQ(text, "MemManage on stacking SP=0xFFFFFF00 CFSR=0x00000092");
    // every other fault keeps its PC text
    fault_reason_text(text, sizeof(text), 0x82, 0, 0x081D047C, 0x081D0461, base);
    EXPECT_STREQ(text, "MemManage PC=0x081D047C");
    fault_reason_text(text, sizeof(text), 0x00010000, 0x40000000, 0x081D0476, 0, base);
    EXPECT_STREQ(text, "HardFault PC=0x081D0476");
}

TEST(ModuleM4Protocol, HandlerFaultIsAServiceFault)
{
    EXPECT_EQ(uint8_t(FaultKind::HandlerFault), 11);
    EXPECT_EQ(FAULT_KIND_MAX, 11);
    EXPECT_FALSE(module_fault_kind(uint8_t(FaultKind::HandlerFault)));   // the service pends it, not the module
}

TEST(ModuleM4Protocol, InjectValuesOutsideFaultKind)
{
    for (uint8_t kind : { 7, 8, 9, 13, 14, 15, 16 }) {
        EXPECT_TRUE(module_fault_kind(kind)) << unsigned(kind);
    }
    for (uint8_t kind : { 0, 1, 6, 10, 11, 12, 17 }) {
        EXPECT_FALSE(module_fault_kind(kind)) << unsigned(kind);
    }
    EXPECT_EQ(FAULT_KIND_MAX, uint8_t(FaultKind::HandlerFault));
}

AP_GTEST_MAIN()
