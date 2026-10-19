/*
  Unit test for @SYS files opened, read and closed from several threads at
  once (scripting, logging and MAVFTP all reach @SYS): every open that
  succeeds reads the whole file it opened, and nothing crashes.
 */

#include <AP_gtest.h>

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_Filesystem/AP_Filesystem_Sys.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/AP_Scheduler.h>

#include <atomic>
#include <chrono>
#include <string.h>
#include <thread>
#include <vector>
#include <fcntl.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if AP_FILESYSTEM_SYS_ENABLED && AP_SCHEDULER_ENABLED

static AP_Filesystem fs;
static AP_Scheduler scheduler;   // @SYS/tasks.txt is its task_info()

TEST(AP_FilesystemSys, ConcurrentOpenReadClose)
{
    static const char header[] = "TasksV2\n";
    std::atomic<uint32_t> opened { 0 };
    std::atomic<uint32_t> bad_reads { 0 };
    std::vector<std::thread> threads;
    for (int t = 0; t < 6; t++) {
        threads.emplace_back([&]() {
            for (int i = 0; i < 20000; i++) {
                const int fd = fs.open("@SYS/tasks.txt", O_RDONLY);
                if (fd < 0) {
                    continue;   // all slots busy (ENFILE)
                }
                opened++;
                char buf[64] {};
                const int32_t n = fs.read(fd, buf, sizeof(buf) - 1);
                if (n != int32_t(sizeof(header) - 1) || memcmp(buf, header, sizeof(header) - 1) != 0) {
                    bad_reads++;
                }
                if (fs.close(fd) != 0) {
                    bad_reads++;
                }
            }
        });
    }
    for (std::thread &thread : threads) {
        thread.join();
    }
    EXPECT_GT(opened.load(), 1000U);
    EXPECT_EQ(bad_reads.load(), 0U);
}

struct AP_FilesystemSysTestAccess {
    static HAL_Semaphore &sem(AP_Filesystem_Sys &sys) { return sys.sem; }
};

// read() and lseek() of an fd this thread opened do not wait for the slot lock that open() and
// close() take: a 2 MB @SYS/flash.bin crc32 makes 32768 64-byte reads, and a lock each cost the
// M7 about 33 ms of CPU. Another thread holds the lock (an open() mid-claim) while this fd is read
TEST(AP_FilesystemSys, ReadDoesNotTakeSlotLock)
{
    static AP_Filesystem_Sys sys;
    const int fd = sys.open("tasks.txt", O_RDONLY);
    ASSERT_GE(fd, 0);
    HAL_Semaphore &sem = AP_FilesystemSysTestAccess::sem(sys);
    sem.take_blocking();
    std::atomic<bool> done { false };
    std::atomic<int32_t> got { -1 };
    std::atomic<int32_t> position { -1 };
    std::thread reader([&]() {
        char buf[16] {};
        got = sys.read(fd, buf, sizeof(buf));
        position = sys.lseek(fd, 0, SEEK_SET);
        done = true;
    });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!done.load() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const bool finished_while_held = done.load();
    sem.give();   // before join(): a reader that does wait for the lock still finishes
    reader.join();
    EXPECT_TRUE(finished_while_held);
    EXPECT_EQ(got.load(), 8);   // "TasksV2\n"
    EXPECT_EQ(position.load(), 0);
    EXPECT_EQ(sys.close(fd), 0);
}

#endif  // AP_FILESYSTEM_SYS_ENABLED && AP_SCHEDULER_ENABLED

AP_GTEST_MAIN()
