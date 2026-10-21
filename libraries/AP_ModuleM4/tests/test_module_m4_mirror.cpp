// the values compared are set exactly
#pragma GCC diagnostic ignored "-Wfloat-equal"

#include <AP_gtest.h>
#include <AP_ModuleM4/AP_ModuleM4_ParamMirror.h>

using namespace ModuleM4;

static ParamMirror mirror;

class MirrorTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        mirror.clear();
    }
};

TEST_F(MirrorTest, UpsertLookupRemoveInSortOrder)
{
    mirror.upsert(1, 0x300, 3.0f, true);
    mirror.upsert(0, 0x200, 2.0f, true);
    mirror.upsert(0, 0x100, 1.0f, true);
    mirror.upsert(1, 0x100, 4.0f, true);
    EXPECT_EQ(mirror.count(), 4U);
    uint8_t out[5];
    ASSERT_TRUE(mirror.get(0, 0x100, out));
    float value;
    memcpy(&value, out + 1, 4);
    EXPECT_EQ(out[0], 1U);
    EXPECT_EQ(value, 1.0f);
    ASSERT_TRUE(mirror.get(1, 0x100, out));
    memcpy(&value, out + 1, 4);
    EXPECT_EQ(value, 4.0f);
    mirror.upsert(0, 0x100, 5.0f, true);   // update in place
    EXPECT_EQ(mirror.count(), 4U);
    mirror.remove_module(0);
    EXPECT_EQ(mirror.count(), 2U);
    EXPECT_FALSE(mirror.get(0, 0x200, out));
    EXPECT_TRUE(mirror.get(1, 0x300, out));
}

TEST_F(MirrorTest, UnboundEntryAnswersZeros)
{
    mirror.upsert(0, 0x100, 9.0f, false);
    uint8_t out[5];
    memset(out, 0xAA, sizeof(out));
    ASSERT_TRUE(mirror.get(0, 0x100, out));
    for (uint8_t byte : out) {
        EXPECT_EQ(byte, 0U);
    }
}

TEST_F(MirrorTest, AMissWritesNothing)
{
    uint8_t out[5];
    memset(out, 0xAA, sizeof(out));
    EXPECT_FALSE(mirror.get(0, 0x100, out));
    EXPECT_EQ(out[0], 0xAA);
}

TEST_F(MirrorTest, AnotherModulesTokenIsAMiss)
{
    mirror.upsert(1, 0x100, 1.0f, true);
    uint8_t out[5];
    EXPECT_FALSE(mirror.get(0, 0x100, out));
}

TEST_F(MirrorTest, CapacityAndOverflow)
{
    for (uint16_t i = 0; i < ParamMirror::CAPACITY; i++) {
        mirror.upsert(0, 0x1000 + 16U * i, float(i), true);
    }
    EXPECT_FALSE(mirror.overflowed());
    mirror.upsert(0, 0x10, 1.0f, true);
    EXPECT_TRUE(mirror.overflowed());
    EXPECT_EQ(mirror.count(), ParamMirror::CAPACITY);
    uint8_t out[5];
    EXPECT_FALSE(mirror.get(0, 0x10, out));
    EXPECT_TRUE(mirror.get(0, 0x1000, out));
}

TEST_F(MirrorTest, PendingAndStaleAreMisses)
{
    mirror.upsert(0, 0x100, 1.0f, true);
    uint8_t out[5];
    mirror.mark_pending(0, 0x100);
    EXPECT_FALSE(mirror.get(0, 0x100, out));
    mirror.upsert(0, 0x100, 2.0f, true);   // the frame ahead of the reply
    mirror.settle(0, 0x100);
    EXPECT_TRUE(mirror.get(0, 0x100, out));
}

TEST_F(MirrorTest, SettleMakesAPendingEntryStale)
{
    mirror.upsert(0, 0x100, 1.0f, true);
    mirror.mark_pending(0, 0x100);
    mirror.settle(0, 0x100);   // the reply came without the frame
    uint8_t out[5];
    EXPECT_FALSE(mirror.get(0, 0x100, out));
    mirror.upsert(0, 0x100, 1.0f, true);
    EXPECT_TRUE(mirror.get(0, 0x100, out));
}

TEST_F(MirrorTest, ModuleStaleTouchesOnlyThatModule)
{
    mirror.upsert(0, 0x100, 1.0f, true);
    mirror.upsert(1, 0x100, 1.0f, true);
    mirror.mark_module_stale(0);
    uint8_t out[5];
    EXPECT_FALSE(mirror.get(0, 0x100, out));
    EXPECT_TRUE(mirror.get(1, 0x100, out));
}

AP_GTEST_MAIN()
