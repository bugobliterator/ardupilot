#include <AP_gtest.h>
#include <algorithm>
#include <string>
#include <vector>
#include <AP_Scripting/m4/m4_file_stream.h>

using namespace ScriptingM4;

// an M7 file in memory, with the m4:fs_* contract: >= 0, or -errno
struct MemoryBackend {
    std::string content;
    size_t position = 0;
    bool open_file = false;
    int32_t open_result = 300;     // the id
    int32_t write_error = 0;       // a negative errno makes every write fail
    unsigned reads = 0;
    std::vector<uint16_t> write_sizes;

    int32_t open(const char *, const char *mode)
    {
        if (open_result < 0) {
            return open_result;
        }
        open_file = true;
        if (mode[0] == 'w') {
            content.clear();
        }
        position = mode[0] == 'a' ? content.size() : 0;
        return open_result;
    }
    int32_t read(int32_t, uint8_t *buf, uint16_t count)
    {
        reads++;
        EXPECT_LE(count, FILE_CHUNK);
        const size_t n = std::min(size_t(count), content.size() - position);
        memcpy(buf, content.data() + position, n);
        position += n;
        return int32_t(n);
    }
    int32_t write(int32_t, const uint8_t *buf, uint16_t count)
    {
        write_sizes.push_back(count);
        EXPECT_LE(count, FILE_CHUNK);
        if (write_error < 0) {
            return write_error;
        }
        content.replace(position, std::min(size_t(count), content.size() - position),
                        reinterpret_cast<const char *>(buf), count);
        position += count;
        return count;
    }
    int32_t seek(int32_t, int32_t offset, int whence)
    {
        const long base = whence == SEEK_SET ? 0 : whence == SEEK_CUR ? long(position) : long(content.size());
        if (base + offset < 0) {
            return -EINVAL;
        }
        position = size_t(base + offset);
        return int32_t(position);
    }
    int32_t flush(int32_t) { return 0; }
    int32_t close(int32_t)
    {
        open_file = false;
        return 0;
    }
};

static std::string pattern(size_t length)
{
    std::string text(length, 0);
    for (size_t i = 0; i < length; i++) {
        text[i] = char('a' + i % 26);
    }
    return text;
}

TEST(ScriptingM4FileStream, ReadAheadServesGetc)
{
    MemoryBackend backend;
    backend.content = pattern(600);
    FileStream file;
    ASSERT_TRUE(file.open(backend, "s/f.txt", "r"));
    for (size_t i = 0; i < 600; i++) {
        ASSERT_EQ(file.getc(backend), backend.content[i]) << i;
    }
    EXPECT_EQ(file.getc(backend), EOF);
    EXPECT_TRUE(file.eof());
    EXPECT_FALSE(file.error());
    EXPECT_EQ(backend.reads, 4U);      // 256 + 256 + 88 bytes, then the read that finds the end
}

TEST(ScriptingM4FileStream, OtherModesReadPerByte)
{
    MemoryBackend backend;
    backend.content = pattern(10);
    FileStream file;
    ASSERT_TRUE(file.open(backend, "s/f.txt", "a+"));
    ASSERT_EQ(file.seek(backend, 0, SEEK_SET), 0);
    EXPECT_EQ(file.getc(backend), 'a');
    EXPECT_EQ(file.getc(backend), 'b');
    EXPECT_EQ(backend.reads, 2U);      // nothing read ahead: a write lands where the script is
}

TEST(ScriptingM4FileStream, ReadLoopsPastTheChunk)
{
    MemoryBackend backend;
    backend.content = pattern(3200);
    FileStream file;
    ASSERT_TRUE(file.open(backend, "s/f.txt", "rb"));
    std::string back(4000, 0);
    EXPECT_EQ(file.read(backend, &back[0], 1, back.size()), 3200U);   // read("a"): all of it, as in-process
    back.resize(3200);
    EXPECT_EQ(back, backend.content);
    EXPECT_TRUE(file.eof());
    EXPECT_EQ(file.read(backend, &back[0], 1, 1), 0U);
}

TEST(ScriptingM4FileStream, TellAndSeekAccountForReadAhead)
{
    MemoryBackend backend;
    backend.content = pattern(100);
    FileStream file;
    ASSERT_TRUE(file.open(backend, "s/f.txt", "r"));
    for (int i = 0; i < 5; i++) {
        file.getc(backend);
    }
    EXPECT_EQ(file.tell(backend), 5);                  // the M7 is at 100: 95 bytes are only read ahead
    EXPECT_EQ(file.seek(backend, 2, SEEK_CUR), 0);
    EXPECT_EQ(file.tell(backend), 7);
    EXPECT_EQ(file.getc(backend), backend.content[7]);
    EXPECT_EQ(file.seek(backend, 0, SEEK_END), 0);
    EXPECT_EQ(file.tell(backend), 100);
    EXPECT_EQ(file.seek(backend, -500, SEEK_SET), 0);  // refused by the M7: posix_compat.cpp returns 0 too
    EXPECT_EQ(file.tell(backend), 100);
}

TEST(ScriptingM4FileStream, UngetcComesBackFirst)
{
    MemoryBackend backend;
    backend.content = "42\nx";
    FileStream file;
    ASSERT_TRUE(file.open(backend, "s/f.txt", "r"));
    EXPECT_EQ(file.getc(backend), '4');
    EXPECT_EQ(file.getc(backend), '2');
    EXPECT_EQ(file.getc(backend), '\n');
    EXPECT_EQ(file.ungetc('\n'), '\n');                // read("n") reads one byte too far
    EXPECT_EQ(file.tell(backend), 3);                  // as posix_compat.cpp: the unget is not counted
    char rest[4] = {};
    EXPECT_EQ(file.read(backend, rest, 1, 3), 2U);     // the unget first, then the file
    EXPECT_EQ(std::string(rest, 2), "\nx");
    EXPECT_EQ(file.ungetc(EOF), EOF);
}

TEST(ScriptingM4FileStream, WritesGoInChunks)
{
    MemoryBackend backend;
    FileStream file;
    ASSERT_TRUE(file.open(backend, "s/f.txt", "w"));
    const std::string text = pattern(1300);
    EXPECT_EQ(file.write(backend, text.data(), 1, text.size()), 1300U);
    EXPECT_EQ(backend.content, text);
    EXPECT_EQ(backend.write_sizes, (std::vector<uint16_t>{ 512, 512, 276 }));
    EXPECT_FALSE(file.error());
    EXPECT_EQ(file.close(backend), 0);
    EXPECT_FALSE(file.is_open());
    EXPECT_FALSE(backend.open_file);
}

TEST(ScriptingM4FileStream, FailuresSetErrnoAndError)
{
    MemoryBackend backend;
    backend.open_result = -EACCES;
    FileStream file;
    errno = 0;
    EXPECT_FALSE(file.open(backend, "miss.txt", "w"));
    EXPECT_EQ(errno, EACCES);
    EXPECT_FALSE(file.is_open());
    backend.open_result = 300;
    ASSERT_TRUE(file.open(backend, "s/f.txt", "w"));
    backend.write_error = -EAGAIN;                    // over the FileIo quota
    errno = 0;
    EXPECT_EQ(file.write(backend, "x", 1, 1), 0U);
    EXPECT_EQ(errno, EAGAIN);
    EXPECT_TRUE(file.error());
    file.clear_error();
    EXPECT_FALSE(file.error());
}

TEST(ScriptingM4FileStream, ClosedStreamFailsCleanly)
{
    MemoryBackend backend;
    FileStream file;                                  // never opened: as a stream the pool has free
    EXPECT_FALSE(file.is_open());
    errno = 0;
    EXPECT_EQ(file.getc(backend), EOF);
    EXPECT_EQ(errno, EBADF);
    EXPECT_EQ(file.write(backend, "x", 1, 1), 0U);
    EXPECT_EQ(file.close(backend), EOF);
    EXPECT_EQ(backend.reads, 0U);                     // the M7 is never asked about a stream that is not open
    EXPECT_TRUE(backend.write_sizes.empty());
}

AP_GTEST_MAIN()
