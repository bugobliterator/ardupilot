#include <AP_gtest.h>
#include <AP_Scripting/AP_ScriptingM4_Handles.h>
#include <AP_Scripting/AP_ScriptingM4_Quota.h>

using namespace ScriptingM4;

TEST(ScriptingM4Handles, InsertDeduplicates)
{
    HandleTable table;
    table.reset(5);
    int a = 0, b = 0;
    EXPECT_EQ(table.insert(3, &a), 0);
    EXPECT_EQ(table.insert(3, &b), 1);
    EXPECT_EQ(table.insert(3, &a), 0);            // a getter called every loop takes one entry
    EXPECT_EQ(table.insert(4, &a), 2);            // the same pointer as another type is another handle
    EXPECT_EQ(table.count(), 3);
    EXPECT_EQ(table.insert(3, nullptr), -1);
    EXPECT_EQ(table.insert(0, &a), -1);
}

TEST(ScriptingM4Handles, LookupChecksEverything)
{
    HandleTable table;
    table.reset(5);
    int a = 0;
    ASSERT_EQ(table.insert(3, &a), 0);
    EXPECT_EQ(table.lookup(3, 0, 5), &a);
    EXPECT_EQ(table.lookup(4, 0, 5), nullptr);   // wrong type
    EXPECT_EQ(table.lookup(3, 1, 5), nullptr);   // past the entries
    EXPECT_EQ(table.lookup(3, 0, 6), nullptr);   // another session
    EXPECT_EQ(table.lookup(3, HandleTable::CAPACITY, 5), nullptr);
    EXPECT_EQ(table.lookup(3, 0xFFFF, 5), nullptr);
    EXPECT_EQ(table.lookup(0, 0, 5), nullptr);
    HandleTable fresh;
    EXPECT_EQ(fresh.lookup(3, 0, 0), nullptr);   // generation 0 before any reset
}

TEST(ScriptingM4Handles, HandlesAreSessionScoped)
{
    HandleTable table;
    table.reset(5);
    int a = 0;
    ASSERT_EQ(table.insert(3, &a), 0);
    table.reset(6);                               // the M4 restarted
    EXPECT_EQ(table.count(), 0);
    EXPECT_EQ(table.lookup(3, 0, 5), nullptr);
    EXPECT_EQ(table.lookup(3, 0, 6), nullptr);
    EXPECT_EQ(table.generation(), 6U);
}

TEST(ScriptingM4Handles, FullTableRefuses)
{
    HandleTable table;
    table.reset(1);
    static int objects[HandleTable::CAPACITY + 1];
    for (uint8_t i = 0; i < HandleTable::CAPACITY; i++) {
        EXPECT_EQ(table.insert(1, &objects[i]), i);
    }
    EXPECT_EQ(table.insert(1, &objects[HandleTable::CAPACITY]), -1);
    EXPECT_EQ(table.insert(1, &objects[0]), 0);   // known objects still resolve
}

TEST(ScriptingM4Quota, BurstThenRate)
{
    Quota quota;
    quota.reset(1000);
    const QuotaLimit &text = QUOTA_LIMITS[uint8_t(QuotaClass::Text)];
    for (uint16_t i = 0; i < text.burst; i++) {
        EXPECT_TRUE(quota.allow(QuotaClass::Text, 1000)) << i;
    }
    EXPECT_FALSE(quota.allow(QuotaClass::Text, 1000));            // burst spent
    EXPECT_FALSE(quota.allow(QuotaClass::Text, 1000 + 1000 / text.per_second - 1));
    EXPECT_TRUE(quota.allow(QuotaClass::Text, 1000 + 1000 / text.per_second));   // one call's worth refilled
    EXPECT_EQ(quota.take_hits(), 2U);
    EXPECT_EQ(quota.take_hits(), 0U);
}

TEST(ScriptingM4Quota, RefillAcrossMillisWrap)
{
    Quota quota;
    const uint32_t start = 0xFFFFFFF0U;
    quota.reset(start);
    while (quota.allow(QuotaClass::Text, start)) {
    }
    EXPECT_FALSE(quota.allow(QuotaClass::Text, start + 49));   // wraps to 0x21, 49 ms
    EXPECT_TRUE(quota.allow(QuotaClass::Text, start + 50));    // 50 ms: one call
}

TEST(ScriptingM4Quota, ClassesAreIndependent)
{
    Quota quota;
    quota.reset(0);
    while (quota.allow(QuotaClass::Text, 0)) {
    }
    EXPECT_TRUE(quota.allow(QuotaClass::Storage, 0));
    EXPECT_TRUE(quota.allow(QuotaClass::None, 0));               // no quota
    for (uint32_t i = 0; i < 100000; i++) {
        ASSERT_TRUE(quota.allow(QuotaClass::None, 0));
    }
    EXPECT_TRUE(quota.allow(QuotaClass(9), 0));                  // unknown class: no quota
}

TEST(ScriptingM4Quota, RefillStopsAtTheBurst)
{
    Quota quota;
    quota.reset(0);
    const QuotaLimit &storage = QUOTA_LIMITS[uint8_t(QuotaClass::Storage)];
    uint32_t allowed = 0;
    while (quota.allow(QuotaClass::Storage, 3600000)) {          // an hour idle
        allowed++;
    }
    EXPECT_EQ(allowed, storage.burst);
}

AP_GTEST_MAIN()
