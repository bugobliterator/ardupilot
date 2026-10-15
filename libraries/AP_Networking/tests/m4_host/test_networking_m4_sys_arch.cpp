#include <AP_gtest.h>
#include <thread>
#include "net_sys_arch.h"

using namespace NetM4;

TEST(NetM4SysArch, MailboxDepthByKind)
{
    EXPECT_EQ(mbox_slots_for(TCPIP_SLOTS, false), int(TCPIP_SLOTS));
    EXPECT_EQ(mbox_slots_for(DEFAULT_TCP_RECVMBOX_SIZE, false), int(TCP_RECV_SLOTS));
    EXPECT_EQ(mbox_slots_for(DEFAULT_TCP_RECVMBOX_SIZE, true), int(TCP_RECV_SLOTS));
    EXPECT_EQ(mbox_slots_for(DEFAULT_UDP_RECVMBOX_SIZE, false), int(UDP_RECV_SLOTS));
    EXPECT_EQ(mbox_slots_for(DEFAULT_UDP_RECVMBOX_SIZE, true), int(LUA_UDP_RECV_SLOTS));
    EXPECT_EQ(mbox_slots_for(DEFAULT_ACCEPTMBOX_SIZE, true), int(ACCEPT_SLOTS));
    EXPECT_EQ(mbox_slots_for(128, false), int(TCPIP_SLOTS));   // never more than the pool's slots
}

TEST(NetM4SysArch, PoolsRunOutWithoutMalloc)
{
    sys_init();
    sys_mbox_t boxes[NUM_MBOXES + 1];
    int made = 0;
    for (auto &box : boxes) {
        if (sys_mbox_new(&box, DEFAULT_UDP_RECVMBOX_SIZE) == ERR_OK) {
            made++;
        }
    }
    EXPECT_EQ(made, int(NUM_MBOXES));
    for (int i = 0; i < made; i++) {
        sys_mbox_free(&boxes[i]);
    }
    EXPECT_EQ(sys_mbox_new(&boxes[0], DEFAULT_UDP_RECVMBOX_SIZE), ERR_OK);   // freed boxes are reused
    EXPECT_EQ(mbox_depth(&boxes[0]), int(UDP_RECV_SLOTS));
    sys_mbox_free(&boxes[0]);
}

TEST(NetM4SysArch, MailboxPassesMessagesInOrder)
{
    sys_init();
    sys_mbox_t box;
    ASSERT_EQ(sys_mbox_new(&box, DEFAULT_UDP_RECVMBOX_SIZE), ERR_OK);
    int a = 1, b = 2;
    void *msg = nullptr;
    EXPECT_EQ(sys_mbox_trypost(&box, &a), ERR_OK);
    EXPECT_EQ(sys_mbox_trypost(&box, &b), ERR_OK);
    EXPECT_NE(sys_arch_mbox_fetch(&box, &msg, 10), SYS_ARCH_TIMEOUT);
    EXPECT_EQ(msg, &a);
    EXPECT_NE(sys_arch_mbox_tryfetch(&box, &msg), SYS_MBOX_EMPTY);
    EXPECT_EQ(msg, &b);
    EXPECT_EQ(sys_arch_mbox_tryfetch(&box, &msg), SYS_MBOX_EMPTY);
    EXPECT_EQ(sys_arch_mbox_fetch(&box, &msg, 5), SYS_ARCH_TIMEOUT);
    sys_mbox_free(&box);
}

TEST(NetM4SysArch, ErrnoIsPerThread)
{
    thread_errno_set(11);
    int other = -1;
    std::thread t([&other] { net_m4_set_errno(22); other = thread_errno_get(); });
    t.join();
    EXPECT_EQ(other, 22);
    EXPECT_EQ(thread_errno_get(), 11);
}

TEST(NetM4SysArch, SuccessKeepsErrno)
{
    net_m4_set_errno(11);
    net_m4_set_errno(0);
    EXPECT_EQ(thread_errno_get(), 11);
}

static void thread_returns(void *)
{
}

// a tcpip thread that cannot start faults (lwIP's tcpip_init ignores the result)
TEST(NetM4SysArch, ThreadThatCannotStartFaults)
{
    ::testing::GTEST_FLAG(death_test_style) = "threadsafe";
    EXPECT_DEATH(sys_thread_new("lua", thread_returns, nullptr, 2048, 0), "");
    EXPECT_DEATH({
        sys_thread_new(TCPIP_THREAD_NAME, thread_returns, nullptr, TCPIP_THREAD_STACKSIZE, TCPIP_THREAD_PRIO);
        sys_thread_new(TCPIP_THREAD_NAME, thread_returns, nullptr, TCPIP_THREAD_STACKSIZE, TCPIP_THREAD_PRIO);
    }, "");
}

AP_GTEST_MAIN()
