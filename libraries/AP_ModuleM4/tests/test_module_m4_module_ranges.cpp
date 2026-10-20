#include <AP_gtest.h>
#include <AP_ModuleM4/AP_ModuleM4_ModuleRanges.h>

using namespace ModuleM4;

static ModuleRanges ranges()
{
    ModuleRanges r {};
    r.stack = { 0x30008020U, 32768U };
    r.heap = { 0x30010020U, 32768U };
    r.rw = { 0x30027448U, 3000U };
    r.rx = { 0x30028000U, 90000U };
    return r;
}

TEST(ModuleM4ModuleRanges, InputsMayComeFromAnyModuleRange)
{
    const auto r = ranges();
    EXPECT_TRUE(r.input_ok(0x30028000U, 16));          // RX: constant data
    EXPECT_TRUE(r.input_ok(0x30027448U, 3000));        // all of RW
    EXPECT_TRUE(r.input_ok(0x30010020U + 32752U, 16)); // the heap's last 16 B
    EXPECT_TRUE(r.input_ok(0x30008020U, 4));
}

TEST(ModuleM4ModuleRanges, OutputsNeverGoToCode)
{
    const auto r = ranges();
    EXPECT_FALSE(r.output_ok(0x30028000U, 4));
    EXPECT_TRUE(r.output_ok(0x30027448U, 4));
    EXPECT_TRUE(r.output_ok(0x30008020U, 32768));
}

TEST(ModuleM4ModuleRanges, OutsideStraddlingAndWrappingAreRefused)
{
    const auto r = ranges();
    EXPECT_FALSE(r.input_ok(0x30000000U, 4));                  // privileged RAM
    EXPECT_FALSE(r.input_ok(0x30040000U, 4));                  // the shared window
    EXPECT_FALSE(r.input_ok(0x30008000U, 4));                  // the stack guard
    EXPECT_FALSE(r.input_ok(0x30010020U + 32760U, 16));        // runs past the heap
    EXPECT_FALSE(r.input_ok(0x30027448U, 0xFFFFFFFFU));        // length wraps
    EXPECT_FALSE(r.output_ok(0xFFFFFFF0U, 0x20U));             // address + length wraps
    EXPECT_FALSE(r.input_ok(0x30027448U - 1U, 2));             // straddles RW's start
}

TEST(ModuleM4ModuleRanges, EmptyBuffersAreAlwaysAccepted)
{
    const auto r = ranges();
    EXPECT_TRUE(r.input_ok(0, 0));
    EXPECT_TRUE(r.output_ok(0x30000000U, 0));
}

TEST(ModuleM4ModuleRanges, Overlap)
{
    EXPECT_TRUE(buffers_overlap(100, 10, 105, 10));
    EXPECT_TRUE(buffers_overlap(105, 10, 100, 10));
    EXPECT_FALSE(buffers_overlap(100, 10, 110, 10));
    EXPECT_FALSE(buffers_overlap(100, 0, 100, 10));
}

AP_GTEST_MAIN()
