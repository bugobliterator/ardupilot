#include <AP_gtest.h>
#include <AP_Scripting/AP_ScriptingM4_ScriptTable.h>

using namespace ScriptingM4;

TEST(ScriptingM4ScriptTable, ChecksumsFollowTheInProcessRules)
{
    static ScriptTable table;
    table.clear();
    ASSERT_TRUE(table.add("./scripts/a.lua", 0x11));
    ASSERT_TRUE(table.add("./scripts/b.lua", 0x22));
    EXPECT_EQ(table.count(), 2U);
    EXPECT_STREQ(table.path(1), "./scripts/b.lua");
    EXPECT_EQ(table.path(2), nullptr);
    EXPECT_EQ(table.path(-1), nullptr);
    EXPECT_TRUE(table.mark_loaded(0));
    EXPECT_TRUE(table.mark_loaded(1));
    EXPECT_EQ(table.loaded_checksum(), 0x33U);
    EXPECT_EQ(table.running_checksum(), 0x33U);
    EXPECT_TRUE(table.mark_stopped(0));
    EXPECT_EQ(table.loaded_checksum(), 0x33U);
    EXPECT_EQ(table.running_checksum(), 0x22U);
}

TEST(ScriptingM4ScriptTable, UntrustedReportsChangeNothing)
{
    static ScriptTable table;
    table.clear();
    ASSERT_TRUE(table.add("./scripts/a.lua", 0x11));
    EXPECT_FALSE(table.mark_stopped(0));     // never loaded
    EXPECT_TRUE(table.mark_loaded(0));
    EXPECT_FALSE(table.mark_loaded(0));      // twice
    EXPECT_FALSE(table.mark_loaded(5));      // out of range
    EXPECT_FALSE(table.mark_loaded(-1));
    EXPECT_TRUE(table.mark_stopped(0));
    EXPECT_FALSE(table.mark_stopped(0));     // twice
    EXPECT_EQ(table.loaded_checksum(), 0x11U);
    EXPECT_EQ(table.running_checksum(), 0U);
}

TEST(ScriptingM4ScriptTable, LimitsAndClear)
{
    static ScriptTable table;
    table.clear();
    char path[MAX_SCRIPT_PATH + 1];
    memset(path, 'a', sizeof(path) - 1);
    path[sizeof(path) - 1] = 0;
    EXPECT_FALSE(table.add(path, 1));        // 64 characters: no room for the terminator
    for (uint8_t i = 0; i < MAX_SCRIPTS; i++) {
        EXPECT_TRUE(table.add("./scripts/x.lua", i));
    }
    EXPECT_FALSE(table.add("./scripts/y.lua", 1));
    table.clear();
    EXPECT_EQ(table.count(), 0U);
    EXPECT_EQ(table.loaded_checksum(), 0U);
}

AP_GTEST_MAIN()
