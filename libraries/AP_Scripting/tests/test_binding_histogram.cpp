#include <AP_gtest.h>
#include <AP_Scripting/AP_Scripting_BindingHistogram.h>

TEST(BindingHistogram, Empty)
{
    BindingHistogram h;
    h.reset();
    EXPECT_EQ(h.count(), 0U);
    EXPECT_EQ(h.percentile(50), 0U);
    EXPECT_EQ(h.max_us(), 0U);
}

TEST(BindingHistogram, BucketBounds)
{
    BindingHistogram h;
    h.reset();
    h.add(0);       // bucket 0, bound 1
    EXPECT_EQ(h.percentile(100), 1U);
    h.reset();
    h.add(1);       // bucket 1: [1,2), bound 2
    EXPECT_EQ(h.percentile(100), 2U);
    h.reset();
    h.add(13);      // bucket 4: [8,16), bound 16
    EXPECT_EQ(h.percentile(100), 16U);
    h.reset();
    h.add(UINT32_MAX);  // clamps to last bucket, bound 1<<15
    EXPECT_EQ(h.percentile(100), 1U << 15);
    EXPECT_EQ(h.max_us(), UINT32_MAX);
}

TEST(BindingHistogram, Percentiles)
{
    BindingHistogram h;
    h.reset();
    for (uint32_t i = 0; i < 98; i++) {
        h.add(10);          // bound 16
    }
    h.add(1000);            // bound 1024
    h.add(3000);            // bound 4096
    EXPECT_EQ(h.count(), 100U);
    EXPECT_EQ(h.percentile(50), 16U);
    EXPECT_EQ(h.percentile(99), 1024U);
    EXPECT_EQ(h.percentile(100), 4096U);
    EXPECT_EQ(h.max_us(), 3000U);
}

TEST(BindingHistogram, ManySamples)
{
    BindingHistogram h;
    h.reset();
    for (uint32_t i = 0; i < 100000; i++) {
        h.add(5);
    }
    EXPECT_EQ(h.count(), 100000U);
    EXPECT_EQ(h.percentile(99), 8U);
}

AP_GTEST_MAIN()
