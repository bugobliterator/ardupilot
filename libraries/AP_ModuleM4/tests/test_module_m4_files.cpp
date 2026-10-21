#include <AP_gtest.h>
#include <AP_ModuleM4/AP_ModuleM4_Files.h>

using namespace ModuleM4;

TEST(ModuleM4Files, ModulePathsOnly)
{
    // the module directories the in-process runtime discovers
    EXPECT_TRUE(module_path_allowed("modules/airshow.apmod"));
    EXPECT_TRUE(module_path_allowed("@ROMFS/modules/airshow.apmod"));
    EXPECT_TRUE(module_path_allowed("/APM/modules/airshow.apmod"));
    EXPECT_TRUE(module_path_allowed("/APM/modules/a.apmod"));
    // anything else: other directories, other suffixes, subdirectories, climbing
    EXPECT_FALSE(module_path_allowed(nullptr));
    EXPECT_FALSE(module_path_allowed(""));
    EXPECT_FALSE(module_path_allowed("airshow.apmod"));
    EXPECT_FALSE(module_path_allowed("/modules/airshow.apmod"));
    EXPECT_FALSE(module_path_allowed("APM/modules/airshow.apmod"));
    EXPECT_FALSE(module_path_allowed("/APM/scripts/airshow.apmod"));
    EXPECT_FALSE(module_path_allowed("/@ROMFS/modules/airshow.apmod"));
    EXPECT_FALSE(module_path_allowed("@SYS/threads.txt"));
    EXPECT_FALSE(module_path_allowed("/APM/modules/airshow.lua"));
    EXPECT_FALSE(module_path_allowed("/APM/modules/airshow.apmod.bak"));
    EXPECT_FALSE(module_path_allowed("/APM/modules/airshow.APMOD"));
    EXPECT_FALSE(module_path_allowed("/APM/modules/.apmod"));
    EXPECT_FALSE(module_path_allowed("/APM/modules/"));
    EXPECT_FALSE(module_path_allowed("/APM/modules/sub/airshow.apmod"));
    EXPECT_FALSE(module_path_allowed("/APM/modules/../LOGS/x.apmod"));
    EXPECT_FALSE(module_path_allowed("modules/..\\x.apmod"));
    EXPECT_FALSE(module_path_allowed("/APM/modules//airshow.apmod"));
}

TEST(ModuleM4Files, OldRustDirectoryRefused)
{
    // AP_Module reads modules/ only; the earlier rust/ directory is not served
    EXPECT_FALSE(module_path_allowed("rust/airshow.apmod"));
    EXPECT_FALSE(module_path_allowed("@ROMFS/rust/airshow.apmod"));
    EXPECT_FALSE(module_path_allowed("/APM/rust/airshow.apmod"));
    EXPECT_FALSE(module_path_allowed("modulesx/airshow.apmod"));
    EXPECT_FALSE(module_path_allowed("/APM/modulesx/airshow.apmod"));
}

TEST(ModuleM4Files, ModuleNameLength)
{
    // the in-process runtime keeps a name in 64 bytes, terminator included
    char path[sizeof("/APM/modules/") + MODULE_NAME_MAX + 1];
    const size_t prefix = strlen("/APM/modules/");
    memcpy(path, "/APM/modules/", prefix);
    for (size_t length = 7; length < MODULE_NAME_MAX + 2; length++) {
        memset(path + prefix, 'a', length - 6);
        memcpy(path + prefix + length - 6, ".apmod", 7);
        EXPECT_EQ(module_path_allowed(path), length < MODULE_NAME_MAX) << length;
    }
    EXPECT_LT(strlen("@ROMFS/modules/") + MODULE_NAME_MAX, size_t(MAX_FS_PATH));
}

TEST(ModuleM4Files, OpenFlagsAreReadOnly)
{
    EXPECT_EQ(MODULE_OPEN_FLAGS, O_RDONLY);
    EXPECT_EQ(MODULE_OPEN_FLAGS & (O_WRONLY | O_RDWR | O_CREAT | O_TRUNC | O_APPEND), 0);
}

TEST(ModuleM4Files, IdsNameOneOpenFile)
{
    FileTable table;
    const int32_t a = table.insert(10);
    const int32_t b = table.insert(11);
    ASSERT_GE(a, 256);
    ASSERT_GE(b, 256);
    EXPECT_NE(a, b);
    EXPECT_EQ(table.lookup(a), 10);
    EXPECT_EQ(table.lookup(b), 11);
    EXPECT_EQ(table.remove(a), 10);
    EXPECT_EQ(table.lookup(a), -1);              // closed
    EXPECT_EQ(table.remove(a), -1);              // closed twice
    const int32_t c = table.insert(12);          // a's slot again, under a new serial
    EXPECT_NE(c, a);
    EXPECT_EQ(table.lookup(a), -1);
    EXPECT_EQ(table.lookup(c), 12);
    EXPECT_EQ(table.lookup(-1), -1);
    EXPECT_EQ(table.lookup(c | 0x07), -1);       // a free slot with c's serial
    EXPECT_EQ(table.lookup(c + (1 << 8)), -1);   // another serial
    EXPECT_EQ(table.lookup(INT32_MAX), -1);
    EXPECT_EQ(table.lookup(0), -1);              // no id is below 256
}

TEST(ModuleM4Files, FullTableRefuses)
{
    FileTable table;
    EXPECT_EQ(table.insert(-1), -1);             // never a failed open
    EXPECT_EQ(table.open_count(), 0);
    for (uint8_t i = 0; i < FILE_SLOTS; i++) {
        ASSERT_GE(table.insert(20 + i), 0) << int(i);
    }
    EXPECT_TRUE(table.full());
    EXPECT_EQ(table.open_count(), FILE_SLOTS);
    EXPECT_EQ(table.insert(99), -1);
}

TEST(ModuleM4Files, CloseAllForgetsEveryFile)
{
    FileTable table;
    const int32_t ids[] = { table.insert(30), table.insert(31), table.insert(32) };
    int closed[FILE_SLOTS];
    uint8_t closed_count = 0;
    table.close_all([&](int fd) { closed[closed_count++] = fd; });
    ASSERT_EQ(closed_count, 3);
    EXPECT_EQ(closed[0] + closed[1] + closed[2], 30 + 31 + 32);
    EXPECT_EQ(table.open_count(), 0);
    for (const int32_t id : ids) {
        EXPECT_EQ(table.lookup(id), -1);         // an earlier session's id
    }
    const int32_t again = table.insert(33);
    for (const int32_t id : ids) {
        EXPECT_NE(again, id);
    }
    closed_count = 0;
    table.close_all([&](int fd) { closed[closed_count++] = fd; });
    EXPECT_EQ(closed_count, 1);
}

TEST(ModuleM4Files, SerialsWrapWithoutZero)
{
    FileTable table;
    int32_t previous = -1;
    for (uint32_t i = 0; i < 70000; i++) {
        const int32_t id = table.insert(40);
        ASSERT_GE(id, 256) << i;
        ASSERT_NE(id, previous) << i;
        ASSERT_EQ(table.remove(id), 40) << i;
        previous = id;
    }
}

TEST(ModuleM4Files, LookupChecksFreeSlotAndSerialApart)
{
    FileTable table;
    const int32_t first = table.insert(10);                  // slot 0, serial 1
    ASSERT_EQ(first, 256);
    // slot 1 was never used: free, and its serial (1) is the one this id carries
    EXPECT_EQ(table.lookup((1 << 8) | 1), -1);
    // slot 0 is open: only its serial differs
    EXPECT_EQ(table.lookup((2 << 8) | 0), -1);
    EXPECT_EQ(table.lookup(first), 10);
    EXPECT_EQ(table.remove((1 << 8) | 1), -1);               // nor can a free slot be closed
    EXPECT_EQ(table.lookup(first), 10);
}

AP_GTEST_MAIN()
