#include <AP_gtest.h>
#include <AP_RustM4/AP_RustM4_CodeWindow.h>

using namespace RustM4;

static const uint32_t TOP = 0x30040000U;

TEST(RustM4CodeWindow, OneByteTakesTheSmallestRegion)
{
    CodeWindow w {};
    ASSERT_TRUE(code_window(TOP, 1, 4, w));
    EXPECT_EQ(w.size, 256U);
    EXPECT_EQ(w.base, TOP - 256U);
    EXPECT_EQ(w.rx_base, TOP - 32U);
    EXPECT_EQ(w.subregion_disable, 0x7FU);
    EXPECT_EQ(w.size_field, 7U);   // 2^(7+1) = 256
}

TEST(RustM4CodeWindow, HundredKilobytesDisablesOneSubregion)
{
    CodeWindow w {};
    ASSERT_TRUE(code_window(TOP, 102400, 8, w));
    EXPECT_EQ(w.size, 131072U);
    EXPECT_EQ(w.rx_base, TOP - 114688U);
    EXPECT_EQ(w.subregion_disable, 0x01U);
    EXPECT_EQ(w.size_field, 16U);
}

TEST(RustM4CodeWindow, LimitsAndBadArguments)
{
    CodeWindow w {};
    EXPECT_TRUE(code_window(TOP, 131072, 8, w));
    EXPECT_EQ(w.subregion_disable, 0U);
    EXPECT_FALSE(code_window(TOP, 131073, 8, w));
    EXPECT_FALSE(code_window(TOP, 0, 8, w));
    EXPECT_FALSE(code_window(TOP, 100, 3, w));          // alignment not a power of two
    EXPECT_FALSE(code_window(TOP + 4096U, 100, 4, w));  // top not aligned to the largest window
}

TEST(RustM4CodeWindow, AlignmentAboveTheSubregionMovesToABiggerWindow)
{
    CodeWindow w {};
    ASSERT_TRUE(code_window(TOP, 300, 4096, w));
    EXPECT_EQ(w.rx_base % 4096U, 0U);
    EXPECT_GE(w.rx_base, w.base);
}

TEST(RustM4CodeWindow, EverySizeKeepsTheInvariants)
{
    for (uint32_t rx = 1; rx <= 131072; rx += 61) {
        CodeWindow w {};
        ASSERT_TRUE(code_window(TOP, rx, 8, w)) << rx;
        EXPECT_EQ(w.base % w.size, 0U) << rx;
        EXPECT_GE(w.rx_base, w.base) << rx;
        EXPECT_LE(w.rx_base + rx, TOP) << rx;
        const uint32_t subregion = w.size / 8U;
        EXPECT_LT(TOP - w.rx_base - rx, subregion > 8U ? subregion : 8U) << rx;
        // disabled subregions are exactly those wholly below the code
        EXPECT_EQ(w.base + __builtin_popcount(w.subregion_disable) * subregion, w.rx_base) << rx;
        EXPECT_EQ(2U << w.size_field, w.size) << rx;
    }
}

AP_GTEST_MAIN()
