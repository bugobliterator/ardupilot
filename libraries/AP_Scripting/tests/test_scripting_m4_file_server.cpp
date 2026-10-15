#include <AP_gtest.h>

#include <AP_Scripting/AP_Scripting_config.h>

#if AP_SCRIPTING_M4_ENABLED

#include <errno.h>
#include <set>
#include <string>
#include <string.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Scripting/AP_ScriptingM4_FileServer.h>

using namespace ScriptingM4;

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// the M4's calls on the file server, as the untrusted wire delivers them
class FileServerTest : public ::testing::Test {
protected:
    static constexpr const char *dir_name = "m4_file_server_test";

    void SetUp() override
    {
        AP::FS().mkdir(dir_name);
        clean();
        quota.reset(AP_HAL::millis());
    }

    void TearDown() override
    {
        server.reset();
        clean();
    }

    void clean()
    {
        auto *d = AP::FS().opendir(dir_name);
        if (d == nullptr) {
            return;
        }
        std::set<std::string> names;
        for (struct dirent *de = AP::FS().readdir(d); de != nullptr; de = AP::FS().readdir(d)) {
            names.insert(de->d_name);
        }
        AP::FS().closedir(d);
        for (const std::string &name : names) {
            AP::FS().unlink((std::string(dir_name) + "/" + name).c_str());
        }
    }

    static RpcValue integer(int32_t value)
    {
        RpcValue v {};
        v.tag = ValueTag::Integer;
        v.integer = value;
        return v;
    }

    static RpcValue string(const char *text, size_t length)
    {
        RpcValue v {};
        v.tag = ValueTag::String;
        v.string = text;
        v.length = uint16_t(length);
        return v;
    }

    static RpcValue string(const std::string &text) { return string(text.data(), text.size()); }

    static RpcValue table()
    {
        RpcValue v {};
        v.tag = ValueTag::Opaque;
        v.lua_type = 5;   // LUA_TTABLE
        return v;
    }

    // one call; the decoded reply values are in reply[0 .. reply_count)
    ReplyStatus call(SystemMethod method, std::initializer_list<RpcValue> arguments)
    {
        RpcValue values[MAX_VALUES];
        uint8_t count = 0;
        for (const RpcValue &v : arguments) {
            values[count++] = v;
        }
        RpcArgs args(values, count);
        RpcWriter results;
        results.begin(buffer, sizeof(buffer), 0);
        uint8_t error_arg = 0;
        const ReplyStatus status = server.handle(method, args, results, error_arg, quota);
        last_error_arg = error_arg;
        EXPECT_TRUE(results.ok());
        reply_count = results.count();
        EXPECT_TRUE(decode_values(buffer, results.length(), results.count(), reply));
        return status;
    }

    // a call whose reply is one integer: the value
    int32_t call_integer(SystemMethod method, std::initializer_list<RpcValue> arguments)
    {
        EXPECT_EQ(call(method, arguments), ReplyStatus::Ok);
        EXPECT_EQ(reply_count, 1);
        EXPECT_EQ(reply[0].tag, ValueTag::Integer);
        return reply[0].integer;
    }

    std::string path(const char *name) const { return std::string(dir_name) + "/" + name; }

    int32_t open_file(const std::string &file, const char *mode)
    {
        return call_integer(SystemMethod::FsOpen, { string(file), string(mode, strlen(mode)) });
    }

    void write_file(const std::string &file, const std::string &contents)
    {
        const int32_t id = open_file(file, "w");
        ASSERT_GE(id, 256);
        ASSERT_EQ(call_integer(SystemMethod::FsWrite, { integer(id), string(contents) }), int32_t(contents.size()));
        ASSERT_EQ(call_integer(SystemMethod::FsClose, { integer(id) }), 0);
    }

    AP_ScriptingM4_FileServer server;
    Quota quota;
    uint8_t buffer[MAX_FRAME];
    RpcValue reply[MAX_VALUES];
    uint8_t reply_count = 0;
    uint8_t last_error_arg = 0;
};

TEST_F(FileServerTest, WriteReadSeekRemove)
{
    const int32_t id = open_file(path("a.txt"), "w+");
    ASSERT_GE(id, 256);
    EXPECT_EQ(call_integer(SystemMethod::FsWrite, { integer(id), string("hello world", 11) }), 11);
    EXPECT_EQ(call_integer(SystemMethod::FsFlush, { integer(id) }), 0);
    EXPECT_EQ(call_integer(SystemMethod::FsSeek, { integer(id), integer(6), integer(SEEK_SET) }), 6);
    ASSERT_EQ(call(SystemMethod::FsRead, { integer(id), integer(100) }), ReplyStatus::Ok);
    ASSERT_EQ(reply[0].tag, ValueTag::String);
    EXPECT_EQ(std::string(reply[0].string, reply[0].length), "world");
    ASSERT_EQ(call(SystemMethod::FsRead, { integer(id), integer(100) }), ReplyStatus::Ok);
    EXPECT_EQ(reply[0].length, 0U);                                          // end of file
    EXPECT_EQ(call_integer(SystemMethod::FsClose, { integer(id) }), 0);
    EXPECT_EQ(call_integer(SystemMethod::FsRead, { integer(id), integer(1) }), -EBADF);   // closed
    EXPECT_EQ(call_integer(SystemMethod::FsRemove, { string(path("a.txt")) }), 0);
    EXPECT_EQ(call_integer(SystemMethod::FsRemove, { string(path("a.txt")) }), -ENOENT);
    EXPECT_EQ(open_file(path("a.txt"), "r"), -ENOENT);
}

TEST_F(FileServerTest, NegativeSeekFromStartIsInvalid)
{
    const int32_t id = open_file(path("n.txt"), "w+");
    ASSERT_GE(id, 256);
    EXPECT_EQ(call_integer(SystemMethod::FsWrite, { integer(id), string("abc", 3) }), 3);
    EXPECT_EQ(call_integer(SystemMethod::FsSeek, { integer(id), integer(-1), integer(SEEK_SET) }), -EINVAL);
    EXPECT_EQ(call_integer(SystemMethod::FsSeek, { integer(id), integer(INT32_MIN), integer(SEEK_SET) }), -EINVAL);
    // the position is unchanged, and seeking back from the end is still allowed
    EXPECT_EQ(call_integer(SystemMethod::FsSeek, { integer(id), integer(0), integer(SEEK_CUR) }), 3);
    EXPECT_EQ(call_integer(SystemMethod::FsSeek, { integer(id), integer(-2), integer(SEEK_END) }), 1);
}

TEST_F(FileServerTest, ForgedAndStaleIdsAreBadDescriptors)
{
    const int32_t id = open_file(path("b.txt"), "w");
    ASSERT_GE(id, 256);
    const int32_t forged[] = { 0, 1, 255, -1, INT32_MIN, INT32_MAX, id + 1, id + (1 << 8), id | 0x7F, 257 };
    for (const int32_t bad : forged) {
        EXPECT_EQ(call_integer(SystemMethod::FsRead, { integer(bad), integer(1) }), -EBADF) << bad;
        EXPECT_EQ(call_integer(SystemMethod::FsWrite, { integer(bad), string("x", 1) }), -EBADF) << bad;
        EXPECT_EQ(call_integer(SystemMethod::FsSeek, { integer(bad), integer(0), integer(SEEK_SET) }), -EBADF) << bad;
        EXPECT_EQ(call_integer(SystemMethod::FsFlush, { integer(bad) }), -EBADF) << bad;
        EXPECT_EQ(call_integer(SystemMethod::FsClose, { integer(bad) }), -EBADF) << bad;
    }
    // the real file is untouched by all that, and closes once
    EXPECT_EQ(call_integer(SystemMethod::FsWrite, { integer(id), string("x", 1) }), 1);
    EXPECT_EQ(call_integer(SystemMethod::FsClose, { integer(id) }), 0);
    EXPECT_EQ(call_integer(SystemMethod::FsClose, { integer(id) }), -EBADF);
    EXPECT_EQ(call_integer(SystemMethod::FsWrite, { integer(id), string("x", 1) }), -EBADF);
}

TEST_F(FileServerTest, ResetMakesEarlierIdsStale)
{
    const int32_t id = open_file(path("c.txt"), "w");
    ASSERT_GE(id, 256);
    server.reset();
    EXPECT_EQ(call_integer(SystemMethod::FsWrite, { integer(id), string("x", 1) }), -EBADF);
    const int32_t again = open_file(path("c.txt"), "r");
    ASSERT_GE(again, 256);
    EXPECT_NE(again, id);
    EXPECT_EQ(call_integer(SystemMethod::FsRead, { integer(id), integer(1) }), -EBADF);
}

TEST_F(FileServerTest, SessionLimitOfOpenFiles)
{
    write_file(path("d.txt"), "d");
    int32_t ids[FILE_SLOTS];
    for (uint8_t i = 0; i < FILE_SLOTS; i++) {
        ids[i] = open_file(path("d.txt"), "r");
        ASSERT_GE(ids[i], 256) << int(i);
    }
    EXPECT_EQ(open_file(path("d.txt"), "r"), -EMFILE);
    EXPECT_EQ(call_integer(SystemMethod::FsClose, { integer(ids[3]) }), 0);
    EXPECT_GE(open_file(path("d.txt"), "r"), 256);
}

TEST_F(FileServerTest, PathsFromTheM4AreChecked)
{
    // a path without a NUL: only the first length bytes are the path
    write_file(path("e.txt"), "e");
    const std::string wire = path("e.txt") + "XYZ";
    const int32_t id = call_integer(SystemMethod::FsOpen, { string(wire.data(), wire.size() - 3), string("r", 1) });
    EXPECT_GE(id, 256);
    EXPECT_EQ(call_integer(SystemMethod::FsOpen, { string(wire), string("r", 1) }), -ENOENT);   // XYZ is part of the name
    // a NUL inside the path
    EXPECT_EQ(call_integer(SystemMethod::FsOpen, { string("m4_file_server_test\0/e.txt", 25), string("r", 1) }), -EINVAL);
    EXPECT_EQ(call_integer(SystemMethod::FsRemove, { string("a\0b", 3) }), -EINVAL);
    // an empty path, and 127 / 128 bytes
    EXPECT_EQ(open_file("", "r"), -ENOENT);
    std::string name(MAX_FS_PATH - 1, 'n');
    EXPECT_EQ(open_file(name, "r"), -ENOENT);                                 // looked up: not refused
    const int32_t created = open_file(name, "w");
    EXPECT_GE(created, 256);
    EXPECT_EQ(call_integer(SystemMethod::FsClose, { integer(created) }), 0);
    EXPECT_EQ(call_integer(SystemMethod::FsRemove, { string(name) }), 0);
    name.push_back('n');
    EXPECT_EQ(open_file(name, "r"), -ENAMETOOLONG);
    EXPECT_EQ(open_file(name, "w"), -ENAMETOOLONG);
    EXPECT_EQ(call_integer(SystemMethod::FsRemove, { string(name) }), -ENAMETOOLONG);
    EXPECT_EQ(call_integer(SystemMethod::FsDir, { string(name), integer(0) }), -ENAMETOOLONG);
    // above the root, for each use
    EXPECT_EQ(open_file("../x.txt", "r"), -EACCES);
    EXPECT_EQ(open_file("../x.txt", "w"), -EACCES);
    EXPECT_EQ(call_integer(SystemMethod::FsRemove, { string("../x.txt", 8) }), -EACCES);
    EXPECT_EQ(call_integer(SystemMethod::FsDir, { string("..", 2), integer(0) }), -EACCES);
    // the "@" filesystems are not written, also as "/@..."
    EXPECT_EQ(open_file("@SYS/x.txt", "w"), -EROFS);
    EXPECT_EQ(open_file("/@SYS/x.txt", "a"), -EROFS);
    EXPECT_EQ(open_file("@PARAM/m4_escape.txt", "w"), -EROFS);
    EXPECT_EQ(open_file("/@PARAM/m4_escape.txt", "w"), -EROFS);
    EXPECT_EQ(open_file("@MISSION/x", "r+"), -EROFS);
    EXPECT_EQ(open_file("@X/../@Y", "w"), -EROFS);
    EXPECT_EQ(open_file("@SYS/", "w"), -EROFS);
    EXPECT_EQ(call_integer(SystemMethod::FsRemove, { string("@SYS/x.txt", 10) }), -EROFS);
    // "//@PARAM/x" is not @PARAM: AP_Filesystem skips one '/', and what is left is no filesystem's file
    const int32_t odd = open_file("//@PARAM/m4_escape.txt", "w");
    EXPECT_LT(odd, 0);
    EXPECT_NE(odd, -EROFS);
    // reads of the "@" filesystems stay allowed
    EXPECT_NE(open_file("@SYS/memory.txt", "r"), -EROFS);
    EXPECT_NE(open_file("@SYS/memory.txt", "r"), -EACCES);
    // except @SYS/threads.txt: it resets the thread statistics the host's duty cap charges
    EXPECT_EQ(open_file("@SYS/threads.txt", "r"), -EACCES);
}

TEST_F(FileServerTest, BadArgumentsRaiseArgumentErrors)
{
    EXPECT_EQ(call(SystemMethod::FsOpen, { table(), string("r", 1) }), ReplyStatus::ArgError);
    EXPECT_EQ(last_error_arg, 1);
    EXPECT_EQ(call(SystemMethod::FsOpen, { string("x", 1), table() }), ReplyStatus::ArgError);
    EXPECT_EQ(last_error_arg, 2);
    EXPECT_EQ(call(SystemMethod::FsOpen, { string("x", 1) }), ReplyStatus::ArgError);
    EXPECT_EQ(call(SystemMethod::FsOpen, { string("x", 1), string("r", 1), integer(1) }), ReplyStatus::ArgError);
    EXPECT_EQ(call(SystemMethod::FsRead, { string("id", 2), integer(1) }), ReplyStatus::ArgError);
    EXPECT_EQ(last_error_arg, 1);
    EXPECT_EQ(call(SystemMethod::FsRead, { integer(256), integer(FILE_CHUNK + 1) }), ReplyStatus::ArgError);
    EXPECT_EQ(last_error_arg, 2);
    EXPECT_EQ(call(SystemMethod::FsRead, { integer(256), integer(-1) }), ReplyStatus::ArgError);
    EXPECT_EQ(call(SystemMethod::FsWrite, { integer(256), table() }), ReplyStatus::ArgError);
    EXPECT_EQ(last_error_arg, 2);
    EXPECT_EQ(call(SystemMethod::FsSeek, { integer(256), integer(0), integer(3) }), ReplyStatus::ArgError);
    EXPECT_EQ(last_error_arg, 3);
    EXPECT_EQ(call(SystemMethod::FsSeek, { integer(256), integer(0) }), ReplyStatus::ArgError);
    EXPECT_EQ(call(SystemMethod::FsFlush, { table() }), ReplyStatus::ArgError);
    EXPECT_EQ(call(SystemMethod::FsClose, {}), ReplyStatus::ArgError);
    EXPECT_EQ(call(SystemMethod::FsRemove, { integer(1), integer(2) }), ReplyStatus::ArgError);
    EXPECT_EQ(call(SystemMethod::FsDir, { string("x", 1), table() }), ReplyStatus::ArgError);
    EXPECT_EQ(last_error_arg, 2);
    EXPECT_EQ(call(SystemMethod::FsDir, { string("x", 1), integer(-1) }), ReplyStatus::ArgError);
    EXPECT_EQ(call(SystemMethod::FsDir, { table(), integer(0) }), ReplyStatus::ArgError);
    // a write over one chunk, a mode that is no fopen mode
    write_file(path("f.txt"), "f");
    const int32_t id = open_file(path("f.txt"), "r+");
    ASSERT_GE(id, 256);
    const std::string big(FILE_CHUNK + 1, 'b');
    EXPECT_EQ(call_integer(SystemMethod::FsWrite, { integer(id), string(big) }), -EINVAL);
    EXPECT_EQ(open_file(path("f.txt"), "rw"), -EINVAL);
    EXPECT_EQ(open_file(path("f.txt"), ""), -EINVAL);
    // a method that is no file call
    EXPECT_EQ(call(SystemMethod::Config, {}), ReplyStatus::UnknownMethod);
}

TEST_F(FileServerTest, WritesShareTheFileIoQuota)
{
    write_file(path("g.txt"), "g");
    const int32_t id = open_file(path("g.txt"), "r");
    ASSERT_GE(id, 256);
    while (quota.allow(QuotaClass::FileIo, AP_HAL::millis())) {
    }
    EXPECT_EQ(open_file(path("g.txt"), "w"), -EAGAIN);                       // opens for writing are quota'd
    EXPECT_EQ(call_integer(SystemMethod::FsRemove, { string(path("g.txt")) }), -EAGAIN);
    EXPECT_GE(open_file(path("g.txt"), "r"), 256);                           // reads are not
    EXPECT_EQ(call(SystemMethod::FsRead, { integer(id), integer(1) }), ReplyStatus::Ok);
    EXPECT_GT(quota.take_hits(), 0U);
}

TEST_F(FileServerTest, DirectoryPagesCoverEveryEntryOnce)
{
    std::set<std::string> created;
    for (int i = 0; i < 40; i++) {
        // short names: the value count ends a page before DIR_REPLY_BYTES does
        const std::string name = std::string(40, 'a' + i % 26) + std::to_string(i);
        write_file(path(name.c_str()), "");
        quota.reset(AP_HAL::millis());
        created.insert(name);
    }
    std::set<std::string> listed;
    uint16_t pages = 0;
    int32_t start = 0;
    do {
        ASSERT_EQ(call(SystemMethod::FsDir, { string(dir_name, strlen(dir_name)), integer(start) }), ReplyStatus::Ok);
        ASSERT_GE(reply_count, 1);
        ASSERT_EQ(reply[reply_count - 1].tag, ValueTag::Integer);
        uint16_t bytes = 0;
        for (uint8_t i = 0; i + 1 < reply_count; i++) {
            ASSERT_EQ(reply[i].tag, ValueTag::String);
            bytes += reply[i].length + 3;
            EXPECT_TRUE(listed.insert(std::string(reply[i].string, reply[i].length)).second);   // no repeats
        }
        EXPECT_LE(bytes, DIR_REPLY_BYTES);
        const int32_t next = reply[reply_count - 1].integer;
        ASSERT_TRUE(next == 0 || next > start) << "paging does not advance";
        start = next;
        ASSERT_LT(++pages, 100);
    } while (start != 0);
    EXPECT_GT(pages, 1U);
    for (const std::string &name : created) {
        EXPECT_EQ(listed.count(name), 1U) << name;
    }
    // a start past the end: an empty last page
    ASSERT_EQ(call(SystemMethod::FsDir, { string(dir_name, strlen(dir_name)), integer(1000000) }), ReplyStatus::Ok);
    EXPECT_EQ(reply_count, 1);
    EXPECT_EQ(reply[0].integer, 0);
    EXPECT_EQ(call_integer(SystemMethod::FsDir, { string("m4_no_such_directory", 20), integer(0) }), -ENOENT);
}

TEST_F(FileServerTest, DirectoryPagesOfLongNamesStayWithinTheByteLimit)
{
    // 12 names of 200 bytes: DIR_REPLY_BYTES ends a page after 3, long before the value count does.
    // They are made directly: the server refuses a path this long
    std::set<std::string> created;
    for (int i = 0; i < 12; i++) {
        const std::string name = std::string(1, 'a' + i) + std::string(199, 'n');
        const int fd = AP::FS().open(path(name.c_str()).c_str(), O_WRONLY | O_CREAT | O_TRUNC);
        ASSERT_GE(fd, 0);
        AP::FS().close(fd);
        created.insert(name);
    }
    std::set<std::string> listed;
    uint16_t pages = 0;
    int32_t start = 0;
    do {
        ASSERT_EQ(call(SystemMethod::FsDir, { string(dir_name, strlen(dir_name)), integer(start) }), ReplyStatus::Ok);
        ASSERT_GE(reply_count, 1);
        ASSERT_EQ(reply[reply_count - 1].tag, ValueTag::Integer);
        uint16_t bytes = 0;
        for (uint8_t i = 0; i + 1 < reply_count; i++) {
            ASSERT_EQ(reply[i].tag, ValueTag::String);
            bytes += reply[i].length + 3;
            EXPECT_TRUE(listed.insert(std::string(reply[i].string, reply[i].length)).second);
        }
        EXPECT_LE(bytes, DIR_REPLY_BYTES);
        const int32_t next = reply[reply_count - 1].integer;
        ASSERT_TRUE(next == 0 || next > start) << "paging does not advance";
        start = next;
        ASSERT_LT(++pages, 100);
    } while (start != 0);
    EXPECT_GE(pages, 4U);
    for (const std::string &name : created) {
        EXPECT_EQ(listed.count(name), 1U);
    }
}

TEST_F(FileServerTest, EmptyDirectoryPathIsLeftToTheFilesystem)
{
    // dirlist("") is AP_Filesystem's call in-process (FatFs lists the root with the "@" ones, the
    // SITL POSIX backend has no such directory): the server refuses nothing itself
    errno = 0;
    auto *d = AP::FS().opendir("");
    const int expected_error = d == nullptr ? errno : 0;
    if (d != nullptr) {
        AP::FS().closedir(d);
    }
    ASSERT_EQ(call(SystemMethod::FsDir, { string("", 0), integer(0) }), ReplyStatus::Ok);
    ASSERT_GE(reply_count, 1);
    ASSERT_EQ(reply[reply_count - 1].tag, ValueTag::Integer);
    if (expected_error != 0) {
        EXPECT_EQ(reply_count, 1);
        EXPECT_EQ(reply[0].integer, -expected_error);
    } else {
        EXPECT_GE(reply[reply_count - 1].integer, 0);
    }
    // only a listing may name the root this way
    EXPECT_EQ(call_integer(SystemMethod::FsOpen, { string("", 0), string("r", 1) }), -ENOENT);
    EXPECT_EQ(call_integer(SystemMethod::FsRemove, { string("", 0) }), -ENOENT);
}

#endif  // AP_SCRIPTING_M4_ENABLED

AP_GTEST_MAIN()
