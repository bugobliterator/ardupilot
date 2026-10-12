#include <AP_gtest.h>
#include <AP_Scripting/m4/m4_socket_wait.h>

using namespace ScriptingM4;

// a Lua-thread clock that only sleeping moves on
struct TestClock {
    uint32_t now_ms = 1000;
    uint32_t sleeps = 0;
    void sleep(uint32_t ms) { now_ms += ms; sleeps++; }
};

// a blocking call that would block twice and then completes: tried again after a sleep each time
TEST(ScriptingM4SocketWait, SleepsBetweenTriesUntilDone)
{
    TestClock clock;
    int tries = 0;
    const bool done = socket_wait([&]() { return ++tries < 3 ? SocketStep::Wait : SocketStep::Done; },
                                  [&](uint32_t ms) { clock.sleep(ms); }, [&]() { return clock.now_ms; });
    EXPECT_TRUE(done);
    EXPECT_EQ(tries, 3);
    EXPECT_EQ(clock.sleeps, 3U);
    EXPECT_EQ(clock.now_ms, 1000 + 3 * SOCKET_RETRY_MS);
}

// a peer that never takes the data: the wait gives up after SOCKET_IDLE_LIMIT_MS. Without the
// limit the Lua thread waits for ever (the test's own stop makes it return true)
TEST(ScriptingM4SocketWait, GivesUpWithoutProgress)
{
    TestClock clock;
    int tries = 0;
    const bool done = socket_wait([&]() { return ++tries > 100000 ? SocketStep::Done : SocketStep::Wait; },
                                  [&](uint32_t ms) { clock.sleep(ms); }, [&]() { return clock.now_ms; });
    EXPECT_FALSE(done);
    EXPECT_GE(clock.now_ms - 1000, SOCKET_IDLE_LIMIT_MS);
    EXPECT_LT(clock.now_ms - 1000, SOCKET_IDLE_LIMIT_MS + SOCKET_RETRY_MS);
}

// a slow peer: every bit of progress starts the idle time over, so a long send completes
TEST(ScriptingM4SocketWait, ProgressStartsTheIdleTimeOver)
{
    TestClock clock;
    int tries = 0;
    const int tries_per_progress = int(SOCKET_IDLE_LIMIT_MS / SOCKET_RETRY_MS) - 1;
    const bool done = socket_wait([&]() {
        tries++;
        if (tries == 3 * tries_per_progress) {
            return SocketStep::Done;
        }
        return tries % tries_per_progress == 0 ? SocketStep::Progress : SocketStep::Wait;
    }, [&](uint32_t ms) { clock.sleep(ms); }, [&]() { return clock.now_ms; });
    EXPECT_TRUE(done);
    EXPECT_GT(clock.now_ms - 1000, 2 * SOCKET_IDLE_LIMIT_MS);
}

// the counts of a blocking send's M7 calls: partial sends add up, a call that would block waits,
// any other failure ends it with what was sent
TEST(ScriptingM4SocketWait, SendAddsUpItsCalls)
{
    SocketSend send(10);
    EXPECT_EQ(send.add(4, false), SocketStep::Progress);
    EXPECT_EQ(send.sent(), 4U);
    EXPECT_EQ(send.remaining(), 6U);
    EXPECT_EQ(send.add(-1, true), SocketStep::Wait);
    EXPECT_EQ(send.add(0, false), SocketStep::Wait);
    EXPECT_EQ(send.add(6, false), SocketStep::Done);
    EXPECT_EQ(send.result(6), 10);

    SocketSend failed(10);
    EXPECT_EQ(failed.add(3, false), SocketStep::Progress);
    EXPECT_EQ(failed.add(-1, false), SocketStep::Done);
    EXPECT_EQ(failed.result(-1), 3);                       // what a blocking send returns
    SocketSend nothing(10);
    EXPECT_EQ(nothing.add(-1, false), SocketStep::Done);
    EXPECT_EQ(nothing.result(-1), -1);

    SocketSend empty(0);
    EXPECT_EQ(empty.add(0, false), SocketStep::Done);
    EXPECT_EQ(empty.result(0), 0);

    SocketSend over(4);                                    // a count above the rest is not trusted
    EXPECT_EQ(over.add(9, false), SocketStep::Done);
    EXPECT_EQ(over.sent(), 4U);
    EXPECT_EQ(over.remaining(), 0U);
}

AP_GTEST_MAIN()
