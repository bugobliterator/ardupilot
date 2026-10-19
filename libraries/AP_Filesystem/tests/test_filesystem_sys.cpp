/*
  Unit test for @SYS files opened, read and closed from several threads at
  once (scripting, logging and MAVFTP all reach @SYS): every open that
  succeeds reads the whole file it opened, and nothing crashes.
 */

#include <AP_gtest.h>

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Scheduler/AP_Scheduler.h>

#include <atomic>
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

#endif  // AP_FILESYSTEM_SYS_ENABLED && AP_SCHEDULER_ENABLED

AP_GTEST_MAIN()
