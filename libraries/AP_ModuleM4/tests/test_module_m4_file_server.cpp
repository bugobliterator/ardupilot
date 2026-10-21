#include <AP_gtest.h>
#include <AP_Module/AP_Module_config.h>

#if AP_MODULE_ENABLED
#include <errno.h>
#include <fcntl.h>
#include <map>
#include <string>
#include <AP_HAL/AP_HAL.h>
#include <AP_ModuleM4/AP_ModuleM4_FileServer.h>
#include <AP_ModuleM4/AP_ModuleM4_Protocol.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace ModuleM4;

static uint8_t pattern_byte(uint32_t i)
{
    return uint8_t((i * 7U + 3U) & 0xFFU);
}

// the M4's calls on the file server, over a fake filesystem of whole files in memory
class FileServerFixture {
public:
    FileServerFixture() : server(ops())
    {
        fixture = this;
        files["/APM/rust/digits.apmod"] = "123456789";
        std::string pattern;
        for (uint32_t i = 0; i < 2048; i++) {
            pattern.push_back(char(pattern_byte(i)));
        }
        files["/APM/rust/pattern.apmod"] = pattern;
        files["rust/sitl.apmod"] = "sitl";
        files["@ROMFS/rust/romfs.apmod"] = "romfs";
        files["/APM/scripts/x.lua"] = "lua";
    }
    ~FileServerFixture()
    {
        server.reset();
        fixture = nullptr;
    }

    int32_t open(const char *path) { return server.open(path, O_RDONLY); }
    int32_t open_for_write(const char *path) { return server.open(path, O_WRONLY | O_CREAT); }
    int32_t read(int32_t id, uint8_t *buffer, uint16_t count) { return server.read(id, buffer, count); }
    int32_t seek(int32_t id, int32_t offset) { return server.seek(id, offset); }
    AP_ModuleM4_FileServer::Closed close(int32_t id) { return server.close(id); }

    // zlib's CRC-32 of the first n pattern bytes, bitwise
    static uint32_t zlib_crc_of_pattern(uint32_t n)
    {
        uint32_t crc = 0xFFFFFFFFU;
        for (uint32_t i = 0; i < n; i++) {
            crc ^= pattern_byte(i);
            for (uint8_t bit = 0; bit < 8; bit++) {
                crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
            }
        }
        return ~crc;
    }

    std::map<std::string, std::string> files;
    std::map<int, std::pair<std::string, size_t>> open_files;   // fd -> (path, offset)
    int next_fd = 3;
    uint32_t opens = 0;
    AP_ModuleM4_FileServer server;

private:
    static FileServerFixture *fixture;

    static const FileOps &ops()
    {
        static const FileOps file_ops { fake_open, fake_read, fake_close };
        return file_ops;
    }

    static int fake_open(const char *path, int flags)
    {
        if (flags != O_RDONLY) {
            errno = EROFS;
            return -1;
        }
        if (fixture->files.count(path) == 0) {
            errno = ENOENT;
            return -1;
        }
        fixture->opens++;
        const int fd = fixture->next_fd++;
        fixture->open_files[fd] = { path, 0 };
        return fd;
    }

    static int32_t fake_read(int fd, void *buffer, uint32_t count)
    {
        auto found = fixture->open_files.find(fd);
        if (found == fixture->open_files.end()) {
            errno = EBADF;
            return -1;
        }
        const std::string &contents = fixture->files[found->second.first];
        size_t &offset = found->second.second;
        const size_t n = std::min(size_t(count), contents.size() - offset);
        memcpy(buffer, contents.data() + offset, n);
        offset += n;
        return int32_t(n);
    }

    static int fake_close(int fd)
    {
        if (fixture->open_files.erase(fd) == 0) {
            errno = EBADF;
            return -1;
        }
        return 0;
    }
};

FileServerFixture *FileServerFixture::fixture;

TEST(ModuleM4FileServer, CrcCoversServedBytes)
{
    FileServerFixture f;
    const int32_t fd = f.open("/APM/rust/digits.apmod");
    ASSERT_GE(fd, 0);
    uint8_t buffer[16];
    EXPECT_EQ(f.read(fd, buffer, 4), 4);
    EXPECT_EQ(f.read(fd, buffer, 16), 5);
    const auto closed = f.close(fd);
    EXPECT_EQ(closed.served, 9U);
    EXPECT_EQ(closed.crc, 0xCBF43926U);   // zlib.crc32(b"123456789")
}

TEST(ModuleM4FileServer, CrcOfAPartialReadIsOfTheServedBytesOnly)
{
    FileServerFixture f;
    const int32_t fd = f.open("/APM/rust/pattern.apmod");
    uint8_t buffer[2000];
    EXPECT_EQ(f.read(fd, buffer, 1001), 1001);
    const auto closed = f.close(fd);
    EXPECT_EQ(closed.served, 1001U);
    EXPECT_EQ(closed.crc, f.zlib_crc_of_pattern(1001));   // computed bitwise in the test
    const int32_t again = f.open("/APM/rust/pattern.apmod");
    EXPECT_EQ(f.read(again, buffer, 2000), 2000);
    EXPECT_EQ(f.read(again, buffer, 2000), 48);
    EXPECT_EQ(f.close(again).crc, 0xB9D45861U);   // zlib.crc32 of the whole pattern
}

TEST(ModuleM4FileServer, OutOfOrderReadIsRefused)
{
    FileServerFixture f;
    const int32_t fd = f.open("/APM/rust/pattern.apmod");
    uint8_t buffer[64];
    EXPECT_EQ(f.read(fd, buffer, 64), 64);
    EXPECT_EQ(f.seek(fd, 0), -EINVAL);       // backwards
    EXPECT_EQ(f.seek(fd, 128), -EINVAL);     // forwards past the served bytes
    EXPECT_EQ(f.seek(fd, 64), 64);           // to the current offset only
    EXPECT_EQ(f.read(fd, buffer, 64), 64);
    EXPECT_EQ(f.close(fd).served, 128U);
}

TEST(ModuleM4FileServer, WritesAndOtherPathsAreRefused)
{
    FileServerFixture f;
    EXPECT_LT(f.open("/APM/scripts/x.lua"), 0);
    EXPECT_LT(f.open_for_write("/APM/rust/a.apmod"), 0);
}

TEST(ModuleM4FileServer, ForgedAndStaleIdsAreBadDescriptors)
{
    FileServerFixture f;
    const int32_t id = f.open("/APM/rust/digits.apmod");
    ASSERT_GE(id, 256);
    const int32_t forged[] = { 0, 1, 255, -1, INT32_MIN, INT32_MAX, id + 1, id + (1 << 8), id | 0x7F, 257 };
    uint8_t buffer[4];
    for (const int32_t bad : forged) {
        EXPECT_EQ(f.read(bad, buffer, 1), -EBADF) << bad;
        EXPECT_EQ(f.seek(bad, 0), -EBADF) << bad;
        EXPECT_EQ(f.close(bad).result, -EBADF) << bad;
    }
    // the real file is untouched by all that, and closes once
    EXPECT_EQ(f.read(id, buffer, 1), 1);
    const auto closed = f.close(id);
    EXPECT_EQ(closed.result, 0);
    EXPECT_EQ(closed.served, 1U);
    EXPECT_EQ(f.close(id).result, -EBADF);
    EXPECT_EQ(f.read(id, buffer, 1), -EBADF);
}

TEST(ModuleM4FileServer, ResetMakesEarlierIdsStale)
{
    FileServerFixture f;
    const int32_t id = f.open("/APM/rust/digits.apmod");
    ASSERT_GE(id, 256);
    f.server.reset();
    EXPECT_TRUE(f.open_files.empty());
    uint8_t buffer[4];
    EXPECT_EQ(f.read(id, buffer, 1), -EBADF);
    const int32_t again = f.open("/APM/rust/digits.apmod");
    ASSERT_GE(again, 256);
    EXPECT_NE(again, id);
    EXPECT_EQ(f.read(id, buffer, 1), -EBADF);
    // a reopened file is served and checked from its start
    EXPECT_EQ(f.read(again, buffer, 4), 4);
    EXPECT_EQ(f.close(again).served, 4U);
}

TEST(ModuleM4FileServer, SessionLimitOfOpenFiles)
{
    FileServerFixture f;
    int32_t ids[FILE_SLOTS];
    for (uint8_t i = 0; i < FILE_SLOTS; i++) {
        ids[i] = f.open("/APM/rust/digits.apmod");
        ASSERT_GE(ids[i], 256) << int(i);
    }
    EXPECT_EQ(f.open("/APM/rust/digits.apmod"), -EMFILE);
    EXPECT_EQ(f.close(ids[3]).result, 0);
    EXPECT_GE(f.open("/APM/rust/digits.apmod"), 256);
}

TEST(ModuleM4FileServer, PathsFromTheM4AreChecked)
{
    FileServerFixture f;
    // the three module directories
    EXPECT_GE(f.open("rust/sitl.apmod"), 256);
    EXPECT_GE(f.open("@ROMFS/rust/romfs.apmod"), 256);
    EXPECT_EQ(f.open("/APM/rust/missing.apmod"), -ENOENT);
    // refused before the filesystem sees them
    const uint32_t opens = f.opens;
    EXPECT_EQ(f.open(nullptr), -EINVAL);
    EXPECT_EQ(f.open(""), -EACCES);
    EXPECT_EQ(f.open("/APM/rust/../scripts/x.lua"), -EACCES);
    EXPECT_EQ(f.open("/APM/rust/sub/a.apmod"), -EACCES);
    EXPECT_EQ(f.open("@SYS/threads.txt"), -EACCES);
    std::string name(MAX_FS_PATH, 'n');
    EXPECT_EQ(f.open(name.c_str()), -ENAMETOOLONG);
    EXPECT_EQ(f.open_for_write("/APM/rust/digits.apmod"), -EACCES);
    EXPECT_EQ(f.server.open("/APM/rust/digits.apmod", O_RDWR), -EACCES);
    EXPECT_EQ(f.opens, opens);
    // reads above the frame's limit
    const int32_t id = f.open("/APM/rust/pattern.apmod");
    uint8_t buffer[FS_READ_MAX + 1];
    EXPECT_EQ(f.read(id, buffer, FS_READ_MAX + 1), -EINVAL);
    EXPECT_EQ(f.read(id, nullptr, 1), -EINVAL);
    EXPECT_EQ(f.read(id, buffer, 0), 0);
    EXPECT_EQ(f.close(id).served, 0U);
}

#endif  // AP_MODULE_ENABLED

AP_GTEST_MAIN()
