#include <AP_gtest.h>
#include <AP_Scripting/AP_ScriptingM4_Files.h>

using namespace ScriptingM4;

static int check(const char *path, PathUse use)
{
    return check_script_path(path, strlen(path), use);
}

TEST(ScriptingM4Files, PathsReachWhatInProcessReaches)
{
    // what in-process scripting opens, the M4 opens: the scripts directory, relative names, other directories
    EXPECT_EQ(check("./scripts/a.lua", PathUse::Read), 0);
    EXPECT_EQ(check("/APM/scripts/modules/test/nested.lua", PathUse::Read), 0);
    EXPECT_EQ(check("miss.txt", PathUse::Write), 0);
    EXPECT_EQ(check("/miss.txt", PathUse::Write), 0);
    EXPECT_EQ(check("APM/LOGS/00000001.BIN", PathUse::Read), 0);
    EXPECT_EQ(check("./scripts/../miss.txt", PathUse::Write), 0);       // stays inside
    EXPECT_EQ(check("a/./b//c/", PathUse::Read), 0);
    EXPECT_EQ(check("/", PathUse::List), 0);
    EXPECT_EQ(check(".", PathUse::List), 0);
    EXPECT_EQ(check("./scripts\\x.txt", PathUse::Write), 0);
    EXPECT_EQ(check("//@SYS/x.txt", PathUse::Write), 0);               // AP_Filesystem skips one '/': a FatFs path
    EXPECT_EQ(check("./@SYS/x.txt", PathUse::Write), 0);
    EXPECT_EQ(check("/@ROMFS/scripts/x.lua", PathUse::Read), 0);
    // no climbing above the root (or the working directory a relative path starts from)
    EXPECT_EQ(check("..", PathUse::List), EACCES);
    EXPECT_EQ(check("../miss.txt", PathUse::Write), EACCES);
    EXPECT_EQ(check("/../miss.txt", PathUse::Read), EACCES);
    EXPECT_EQ(check("./scripts/../../x.txt", PathUse::Read), EACCES);
    EXPECT_EQ(check("/APM/scripts/../../../x", PathUse::Read), EACCES);
    EXPECT_EQ(check("a\\..\\..\\x", PathUse::Write), EACCES);           // FatFs separator
    EXPECT_EQ(check("a/b/../../..", PathUse::List), EACCES);
    // the @ filesystems: read, never written
    EXPECT_EQ(check("@ROMFS/scripts/modules/x.lua", PathUse::Read), 0);
    EXPECT_EQ(check("@SYS/threads.txt", PathUse::Read), 0);
    EXPECT_EQ(check("@ROMFS/scripts", PathUse::List), 0);
    EXPECT_EQ(check("@ROMFS/scripts/x.lua", PathUse::Write), EROFS);
    EXPECT_EQ(check("@SYS/threads.txt", PathUse::Write), EROFS);
    EXPECT_EQ(check("@PARAM/param.pck", PathUse::Write), EROFS);
    EXPECT_EQ(check("@MISSION/mission.dat", PathUse::Write), EROFS);
    // AP_Filesystem skips one leading '/' before it selects the filesystem
    EXPECT_EQ(check("/@SYS/x.txt", PathUse::Write), EROFS);
    EXPECT_EQ(check("/@PARAM/x", PathUse::Write), EROFS);
    EXPECT_EQ(check("/@MISSION/x", PathUse::Write), EROFS);
    EXPECT_EQ(check("/@PARAM/param.pck", PathUse::Read), 0);
    // bytes the M7 does not take
    EXPECT_EQ(check_script_path("./scripts/a\0../x", 16, PathUse::Read), EINVAL);
    EXPECT_EQ(check("", PathUse::Read), ENOENT);
    EXPECT_EQ(check("", PathUse::Write), ENOENT);
    EXPECT_EQ(check("", PathUse::List), 0);       // dirlist(""): the filesystem root, as in-process
    char long_path[MAX_FS_PATH + 1];
    memset(long_path, 'a', sizeof(long_path) - 1);
    long_path[MAX_FS_PATH] = 0;
    EXPECT_EQ(check(long_path, PathUse::Read), ENAMETOOLONG);
}

TEST(ScriptingM4Files, ModesMatchPosixCompat)
{
    struct Case {
        const char *mode;
        int flags;
    };
    // AP_Filesystem/posix_compat.cpp posix_fopen_modes_to_open(), r+ truncation included
    static const Case cases[] = {
        { "r", O_RDONLY }, { "rb", O_RDONLY },
        { "r+", O_RDWR | O_TRUNC }, { "r+b", O_RDWR | O_TRUNC }, { "rb+", O_RDWR | O_TRUNC },
        { "w", O_WRONLY | O_CREAT | O_TRUNC }, { "wb", O_WRONLY | O_CREAT | O_TRUNC },
        { "w+", O_RDWR | O_CREAT | O_TRUNC }, { "w+b", O_RDWR | O_CREAT | O_TRUNC }, { "wb+", O_RDWR | O_CREAT | O_TRUNC },
        { "a", O_WRONLY | O_CREAT | O_APPEND }, { "ab", O_WRONLY | O_CREAT | O_APPEND },
        { "a+", O_RDWR | O_CREAT | O_APPEND }, { "a+b", O_RDWR | O_CREAT | O_APPEND }, { "ab+", O_RDWR | O_CREAT | O_APPEND },
    };
    for (const Case &c : cases) {
        EXPECT_EQ(fopen_mode_flags(c.mode, strlen(c.mode)), c.flags) << c.mode;
        EXPECT_EQ(mode_writes(c.flags), c.flags != O_RDONLY) << c.mode;
    }
    EXPECT_EQ(fopen_mode_flags("x", 1), -1);
    EXPECT_EQ(fopen_mode_flags("rw", 2), -1);
    EXPECT_EQ(fopen_mode_flags("", 0), -1);
    EXPECT_EQ(fopen_mode_flags("r\0", 2), -1);     // the length counts
}

TEST(ScriptingM4Files, IdsNameOneOpenFile)
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

TEST(ScriptingM4Files, FullTableRefuses)
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

TEST(ScriptingM4Files, CloseAllForgetsEveryFile)
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

TEST(ScriptingM4Files, SerialsWrapWithoutZero)
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

TEST(ScriptingM4Files, PathLengthAndSelectorEdges)
{
    char path[MAX_FS_PATH + 1];
    memset(path, 'a', sizeof(path));
    EXPECT_EQ(check_script_path(path, MAX_FS_PATH - 1, PathUse::Write), 0);               // 127 bytes
    EXPECT_EQ(check_script_path(path, MAX_FS_PATH, PathUse::Write), ENAMETOOLONG);        // 128 bytes
    EXPECT_EQ(check_script_path(path, MAX_FS_PATH, PathUse::List), ENAMETOOLONG);
    EXPECT_EQ(check("@X/../@Y", PathUse::Write), EROFS);                                  // selected by the first '@'
    EXPECT_EQ(check("@SYS/", PathUse::Write), EROFS);
    EXPECT_EQ(check("@SYS", PathUse::Write), EROFS);
    EXPECT_EQ(check("//@PARAM/x", PathUse::Write), 0);                                    // one '/' skipped: a plain path
    for (const PathUse use : { PathUse::List, PathUse::Read, PathUse::Write }) {
        EXPECT_EQ(check("..", use), EACCES) << int(use);
        EXPECT_EQ(check("/../x", use), EACCES) << int(use);
        EXPECT_EQ(check("a/../../x", use), EACCES) << int(use);
    }
}

TEST(ScriptingM4Files, LookupChecksFreeSlotAndSerialApart)
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
