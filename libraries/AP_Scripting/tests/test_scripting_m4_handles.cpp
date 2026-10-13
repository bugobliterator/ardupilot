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
    const uint32_t g = table.generation(0);
    EXPECT_EQ(g >> 8, 5U);                       // the session in the high bits
    EXPECT_NE(g & 0xFFU, 0U);                    // a serial, never 0
    EXPECT_EQ(table.lookup(3, 0, g), &a);
    EXPECT_EQ(table.lookup(4, 0, g), nullptr);   // wrong type
    EXPECT_EQ(table.lookup(3, 1, g), nullptr);   // a free entry
    EXPECT_EQ(table.lookup(3, 0, g + 1), nullptr);
    EXPECT_EQ(table.lookup(3, 0, (6U << 8) | (g & 0xFFU)), nullptr);   // another session
    EXPECT_EQ(table.lookup(3, HandleTable::CAPACITY, g), nullptr);
    EXPECT_EQ(table.lookup(3, 0xFFFF, g), nullptr);
    EXPECT_EQ(table.lookup(0, 0, g), nullptr);
    HandleTable fresh;
    EXPECT_EQ(fresh.lookup(3, 0, fresh.generation(0)), nullptr);   // nothing before any insert
}

TEST(ScriptingM4Handles, ResetWithTheSameSessionEmpties)
{
    // the M4 picks its session number: a restart may repeat it, and the previous
    // session's handles must stay stale even when their entry is used again
    HandleTable table;
    table.reset(5);
    int a = 0, b = 0;
    ASSERT_EQ(table.insert(3, &a), 0);
    const uint32_t old_generation = table.generation(0);
    table.reset(5);
    EXPECT_EQ(table.count(), 0);
    EXPECT_EQ(table.lookup(3, 0, old_generation), nullptr);
    ASSERT_EQ(table.insert(3, &b), 0);
    // without the serial bump at reset, the old handle would name b
    EXPECT_EQ(table.lookup(3, 0, old_generation), nullptr);
    EXPECT_EQ(table.lookup(3, 0, table.generation(0)), &b);
}

TEST(ScriptingM4Handles, HandlesAreSessionScoped)
{
    HandleTable table;
    table.reset(5);
    int a = 0;
    ASSERT_EQ(table.insert(3, &a), 0);
    const uint32_t old_generation = table.generation(0);
    table.reset(6);                               // the M4 restarted
    EXPECT_EQ(table.count(), 0);
    EXPECT_EQ(table.lookup(3, 0, old_generation), nullptr);
    EXPECT_EQ(table.session(), 6U);
    EXPECT_EQ(table.generation(0) >> 8, 6U);
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
    EXPECT_EQ(table.insert_owned(2, &objects[HandleTable::CAPACITY]), -1);
    EXPECT_EQ(table.insert(1, &objects[0]), 0);   // known objects still resolve
}

TEST(ScriptingM4Handles, OwnedEntriesAreNeverShared)
{
    HandleTable table;
    table.reset(2);
    int a = 0;
    const int16_t owned = table.insert_owned(7, &a);
    ASSERT_GE(owned, 0);
    // a second owned entry for the same object, and a kept entry: none shares the owned one,
    // so releasing one userdata never invalidates another
    const int16_t again = table.insert_owned(7, &a);
    const int16_t kept = table.insert(7, &a);
    EXPECT_NE(again, owned);
    EXPECT_NE(kept, owned);
    EXPECT_NE(kept, again);
    EXPECT_EQ(table.insert(7, &a), kept);         // the kept one is deduplicated
    EXPECT_EQ(table.insert_owned(7, nullptr), -1);
    EXPECT_EQ(table.insert_owned(0, &a), -1);
}

TEST(ScriptingM4Handles, ReleasedEntriesNeverComeBack)
{
    HandleTable table;
    table.reset(3);
    int a = 0, b = 0, kept_object = 0;
    const int16_t owned = table.insert_owned(7, &a);
    const int16_t kept = table.insert(8, &kept_object);
    ASSERT_GE(owned, 0);
    ASSERT_GE(kept, 0);
    const uint32_t g = table.generation(owned);
    // the wrong type, the wrong generation and a kept entry are refused; without the owned
    // flag the kept release would hand back the vehicle's object for destruction
    EXPECT_EQ(table.release_owned(8, owned, g), nullptr);
    EXPECT_EQ(table.release_owned(7, owned, g + 1), nullptr);
    EXPECT_EQ(table.release_owned(8, kept, table.generation(kept)), nullptr);
    EXPECT_EQ(table.lookup(8, kept, table.generation(kept)), &kept_object);
    EXPECT_EQ(table.release_owned(7, owned, g), &a);
    EXPECT_EQ(table.release_owned(7, owned, g), nullptr);   // twice
    EXPECT_EQ(table.lookup(7, owned, g), nullptr);
    // the entry is used again under a new serial: the released handle names nothing
    const int16_t next = table.insert_owned(7, &b);
    EXPECT_EQ(next, owned);
    EXPECT_NE(table.generation(next), g);
    EXPECT_EQ(table.lookup(7, owned, g), nullptr);
    EXPECT_EQ(table.lookup(7, next, table.generation(next)), &b);
}

TEST(ScriptingM4Handles, ForgetRetiresAnObjectTheM7Deleted)
{
    HandleTable table;
    table.reset(4);
    int socket = 0;
    const int16_t index = table.insert(9, &socket);
    ASSERT_GE(index, 0);
    const uint32_t g = table.generation(index);
    EXPECT_TRUE(table.forget(&socket));
    EXPECT_FALSE(table.forget(&socket));
    EXPECT_FALSE(table.forget(nullptr));
    EXPECT_EQ(table.lookup(9, index, g), nullptr);
    // a new object at the same address (accept() after close()) gets a new generation
    EXPECT_EQ(table.insert(9, &socket), index);
    EXPECT_NE(table.generation(index), g);
}

TEST(ScriptingM4Handles, SerialsWrapWithoutZero)
{
    HandleTable table;
    table.reset(1);
    int a = 0;
    uint32_t previous = 0;
    for (uint16_t i = 0; i < 600; i++) {
        const int16_t index = table.insert_owned(5, &a);
        ASSERT_EQ(index, 0) << i;
        const uint32_t g = table.generation(0);
        ASSERT_NE(g & 0xFFU, 0U) << i;
        ASSERT_NE(g, previous) << i;
        ASSERT_EQ(table.release_owned(5, 0, g), &a) << i;
        previous = g;
    }
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

TEST(ScriptingM4Quota, FileLogFloodIsLimited)
{
    Quota quota;
    quota.reset(0);
    uint16_t allowed = 0;
    for (uint16_t i = 0; i < 200; i++) {
        allowed += quota.allow(QuotaClass::FileLog, 0) ? 1 : 0;
    }
    EXPECT_EQ(allowed, QUOTA_LIMITS[uint8_t(QuotaClass::FileLog)].burst);
    EXPECT_EQ(quota.take_hits(), 200U - allowed);
    EXPECT_FALSE(quota.allow(QuotaClass::FileLog, 999));
    EXPECT_TRUE(quota.allow(QuotaClass::FileLog, 1000));    // one file a second
}

// CAN transmits from the M4 (write_frame, DroneCAN broadcast and request) share the bus with flight
// traffic: the class counts frames. Without the cost a 145-frame broadcast takes one unit
TEST(ScriptingM4Quota, CanFramesAreLimited)
{
    Quota quota;
    quota.reset(0);
    const QuotaLimit &can = QUOTA_LIMITS[uint8_t(QuotaClass::Can)];
    EXPECT_EQ(can.per_second, 800);
    EXPECT_EQ(can.burst, 800);
    for (uint8_t i = 0; i < 5; i++) {
        EXPECT_TRUE(quota.allow(QuotaClass::Can, 0, 145)) << int(i);   // 725 frames
    }
    EXPECT_FALSE(quota.allow(QuotaClass::Can, 0, 145));    // 75 left
    EXPECT_EQ(quota.take_hits(), 1U);
    EXPECT_TRUE(quota.allow(QuotaClass::Can, 0, 75));
    EXPECT_FALSE(quota.allow(QuotaClass::Can, 0));
    EXPECT_FALSE(quota.allow(QuotaClass::Can, 1, 2));      // 0.8 frames refilled
    EXPECT_TRUE(quota.allow(QuotaClass::Can, 3, 2));       // 2.4 frames
}

TEST(ScriptingM4Quota, HitsSurviveReset)
{
    Quota quota;
    quota.reset(0);
    while (quota.allow(QuotaClass::Text, 0)) {
    }
    quota.reset(10);                                             // the M4 restarted
    EXPECT_EQ(quota.take_hits(), 1U);
    EXPECT_TRUE(quota.allow(QuotaClass::Text, 10));              // buckets full again
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

TEST(ScriptingM4Quota, FileIoBurstThenRate)
{
    Quota quota;
    quota.reset(0);
    const QuotaLimit &file = QUOTA_LIMITS[uint8_t(QuotaClass::FileIo)];
    EXPECT_EQ(file.per_second, 50);
    EXPECT_EQ(file.burst, 100);
    for (uint16_t i = 0; i < file.burst; i++) {
        ASSERT_TRUE(quota.allow(QuotaClass::FileIo, 0)) << i;
    }
    EXPECT_FALSE(quota.allow(QuotaClass::FileIo, 0));             // burst spent
    EXPECT_FALSE(quota.allow(QuotaClass::FileIo, 19));
    EXPECT_TRUE(quota.allow(QuotaClass::FileIo, 20));             // 20 ms at 50/s: one call
    EXPECT_TRUE(quota.allow(QuotaClass::Text, 20));               // the other classes are untouched
    EXPECT_EQ(quota.take_hits(), 2U);
}

// mavlink:send_chan: 200 sends at once, then 200 a second. Without the class the 201st send of one
// run goes out, and a script can take the channel lock as often as the host thread serves it
TEST(ScriptingM4Quota, MavTxBurstThenRate)
{
    Quota quota;
    quota.reset(0);
    const QuotaLimit &tx = QUOTA_LIMITS[uint8_t(QuotaClass::MavTx)];
    EXPECT_EQ(tx.per_second, 200);
    EXPECT_EQ(tx.burst, 200);
    for (int i = 0; i < 200; i++) {
        ASSERT_TRUE(quota.allow(QuotaClass::MavTx, 0)) << i;
    }
    EXPECT_FALSE(quota.allow(QuotaClass::MavTx, 0));
    EXPECT_FALSE(quota.allow(QuotaClass::MavTx, 4));     // 0.8 of a send refilled
    EXPECT_TRUE(quota.allow(QuotaClass::MavTx, 5));      // one send per 5 ms
    EXPECT_EQ(quota.take_hits(), 2U);
}

// logger:write: 500 rows at once, then 500 a second; the in-tree actuator_chirp.lua writes 400 a
// second and passes untouched. Without the class a script writes as fast as the host serves it
TEST(ScriptingM4Quota, LogBurstThenRate)
{
    Quota quota;
    quota.reset(0);
    const QuotaLimit &log = QUOTA_LIMITS[uint8_t(QuotaClass::Log)];
    EXPECT_EQ(log.per_second, 500);
    EXPECT_EQ(log.burst, 500);
    for (int i = 0; i < 500; i++) {
        ASSERT_TRUE(quota.allow(QuotaClass::Log, 0)) << i;
    }
    EXPECT_FALSE(quota.allow(QuotaClass::Log, 0));
    EXPECT_TRUE(quota.allow(QuotaClass::Log, 2));        // one row per 2 ms
    // 400 a second for 10 s never touches the quota
    quota.reset(1000);
    for (uint32_t ms = 1000; ms < 11000; ms += 5) {
        ASSERT_TRUE(quota.allow(QuotaClass::Log, ms)) << ms;
        ASSERT_TRUE(quota.allow(QuotaClass::Log, ms)) << ms;
    }
}

// osd: writes and clears share Osd (300/s); flush and draw_screen, each a whole-screen render,
// share OsdFrame (40/s). Spending one leaves the other whole
TEST(ScriptingM4Quota, OsdClassesAreSeparate)
{
    Quota quota;
    quota.reset(0);
    EXPECT_EQ(QUOTA_LIMITS[uint8_t(QuotaClass::Osd)].per_second, 300);
    EXPECT_EQ(QUOTA_LIMITS[uint8_t(QuotaClass::Osd)].burst, 300);
    EXPECT_EQ(QUOTA_LIMITS[uint8_t(QuotaClass::OsdFrame)].per_second, 40);
    EXPECT_EQ(QUOTA_LIMITS[uint8_t(QuotaClass::OsdFrame)].burst, 40);
    for (int i = 0; i < 40; i++) {
        ASSERT_TRUE(quota.allow(QuotaClass::OsdFrame, 0)) << i;
    }
    EXPECT_FALSE(quota.allow(QuotaClass::OsdFrame, 0));
    for (int i = 0; i < 300; i++) {
        ASSERT_TRUE(quota.allow(QuotaClass::Osd, 0)) << i;
    }
    EXPECT_FALSE(quota.allow(QuotaClass::Osd, 0));
    EXPECT_EQ(QUOTA_CLASSES, 11);
}

AP_GTEST_MAIN()
