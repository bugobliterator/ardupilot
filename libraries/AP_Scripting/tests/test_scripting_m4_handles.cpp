#include <AP_gtest.h>
#include <AP_Scripting/AP_ScriptingM4_Handles.h>

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

AP_GTEST_MAIN()
