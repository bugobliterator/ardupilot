#include <AP_gtest.h>
#include <functional>
#include <vector>
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

// an M7 send as socket_send_all() calls it: each call takes at most take bytes of the piece (-1:
// would block) and records the pieces
struct TestSend {
    std::vector<size_t> pieces;
    std::vector<size_t> offsets;
    std::vector<bool> firsts;
    std::function<int32_t(size_t piece, size_t call)> take;
    int32_t operator()(size_t offset, size_t piece, bool first, bool &would_block)
    {
        const int32_t count = take(piece, pieces.size());
        pieces.push_back(piece);
        offsets.push_back(offset);
        firsts.push_back(first);
        would_block = count < 0;
        return count;
    }
};

static int32_t send_all(TestSend &send, TestClock &clock, size_t length, size_t wanted, bool datagram, bool blocking)
{
    return socket_send_all(length, datagram, blocking, std::ref(send), [wanted]() { return wanted; },
                           [&](uint32_t ms) { clock.sleep(ms); }, [&]() { return clock.now_ms; });
}

// a stream send longer than one call frame goes in SOCKET_SEND_CHUNK pieces, back to back.
// Without the pieces the whole string goes in one call, which the M4 refuses for its length
TEST(ScriptingM4SocketWait, StreamSendGoesInPieces)
{
    TestClock clock;
    TestSend send;
    send.take = [](size_t piece, size_t) { return int32_t(piece); };
    EXPECT_EQ(send_all(send, clock, 2000, 2000, false, false), 2000);
    ASSERT_EQ(send.pieces.size(), 3U);
    EXPECT_EQ(send.pieces[0], SOCKET_SEND_CHUNK);
    EXPECT_EQ(send.pieces[1], SOCKET_SEND_CHUNK);
    EXPECT_EQ(send.pieces[2], 2000 - 2 * SOCKET_SEND_CHUNK);
    EXPECT_EQ(send.offsets[2], 2 * SOCKET_SEND_CHUNK);
    EXPECT_TRUE(send.firsts[0]);    // the first carries the script's length: the M7 checks it
    EXPECT_FALSE(send.firsts[1]);
    EXPECT_EQ(clock.sleeps, 0U);
}

// the script's length is shorter than the string: the M7 sends that much of the first piece
TEST(ScriptingM4SocketWait, StreamSendStopsAtTheScriptsLength)
{
    TestClock clock;
    TestSend send;
    send.take = [](size_t, size_t) { return int32_t(10); };
    EXPECT_EQ(send_all(send, clock, 2000, 10, false, true), 10);
    EXPECT_EQ(send.pieces.size(), 1U);
}

// a non-blocking stream send ends at the first short, would-block or failed piece, with what was
// sent, as one in-process send does
TEST(ScriptingM4SocketWait, NonBlockingSendEndsAtAShortPiece)
{
    TestClock clock;
    TestSend send;
    send.take = [](size_t piece, size_t call) { return call == 0 ? int32_t(piece) : int32_t(100); };
    EXPECT_EQ(send_all(send, clock, 2000, 2000, false, false), int32_t(SOCKET_SEND_CHUNK + 100));
    EXPECT_EQ(send.pieces.size(), 2U);
    TestSend blocked;
    blocked.take = [](size_t piece, size_t call) { return call == 0 ? int32_t(piece) : -1; };
    EXPECT_EQ(send_all(blocked, clock, 2000, 2000, false, false), int32_t(SOCKET_SEND_CHUNK));
    TestSend none;
    none.take = [](size_t, size_t) { return -1; };
    EXPECT_EQ(send_all(none, clock, 2000, 2000, false, false), -1);
    EXPECT_EQ(none.pieces.size(), 1U);
    EXPECT_EQ(clock.sleeps, 0U);
}

// an empty string is one call of nothing; a multiple of the piece size makes no empty last call
TEST(ScriptingM4SocketWait, EmptyAndWholePieceSends)
{
    TestClock clock;
    TestSend empty;
    empty.take = [](size_t piece, size_t) { return int32_t(piece); };
    EXPECT_EQ(send_all(empty, clock, 0, 0, false, true), 0);
    ASSERT_EQ(empty.pieces.size(), 1U);
    EXPECT_EQ(empty.pieces[0], 0U);
    TestSend exact;
    exact.take = [](size_t piece, size_t) { return int32_t(piece); };
    EXPECT_EQ(send_all(exact, clock, 2 * SOCKET_SEND_CHUNK, 2 * SOCKET_SEND_CHUNK, false, false),
              int32_t(2 * SOCKET_SEND_CHUNK));
    EXPECT_EQ(exact.pieces.size(), 2U);
    EXPECT_EQ(clock.sleeps, 0U);
}

// a short first piece: a non-blocking send returns its count; a blocking one sleeps and goes on
// from where it stopped, so no byte is lost or sent twice
TEST(ScriptingM4SocketWait, ShortFirstPiece)
{
    TestClock clock;
    TestSend nonblocking;
    nonblocking.take = [](size_t, size_t) { return int32_t(100); };
    EXPECT_EQ(send_all(nonblocking, clock, 2000, 2000, false, false), 100);
    EXPECT_EQ(nonblocking.pieces.size(), 1U);
    TestSend blocking;
    blocking.take = [](size_t piece, size_t call) { return call == 0 ? int32_t(100) : int32_t(piece); };
    EXPECT_EQ(send_all(blocking, clock, 2000, 2000, false, true), 2000);
    ASSERT_EQ(blocking.pieces.size(), 4U);
    EXPECT_EQ(blocking.offsets[1], 100U);
    EXPECT_EQ(blocking.pieces[1], SOCKET_SEND_CHUNK);
    EXPECT_EQ(blocking.offsets[3], 100 + 2 * SOCKET_SEND_CHUNK);
    EXPECT_EQ(blocking.pieces[3], 2000 - 100 - 2 * SOCKET_SEND_CHUNK);
    EXPECT_EQ(clock.sleeps, 1U);
}

// a datagram goes whole: cutting it would make several datagrams of one
TEST(ScriptingM4SocketWait, DatagramGoesWhole)
{
    TestClock clock;
    TestSend send;
    send.take = [](size_t piece, size_t) { return int32_t(piece); };
    EXPECT_EQ(send_all(send, clock, 2000, 2000, true, true), 2000);
    ASSERT_EQ(send.pieces.size(), 1U);
    EXPECT_EQ(send.pieces[0], 2000U);
}

// a blocking send waits for the peer between tries, and gives up after SOCKET_SEND_LIMIT_MS in
// all, however slowly the peer takes the data. Without the overall limit a peer taking a byte a
// second holds every M4 script for the whole string
TEST(ScriptingM4SocketWait, BlockingSendHasAnOverallLimit)
{
    TestClock clock;
    TestSend send;
    send.take = [](size_t, size_t call) { return call % 2 == 0 ? -1 : 1; };   // a byte every other try
    const int32_t sent = send_all(send, clock, 100000, 100000, false, true);
    EXPECT_GT(sent, 0);
    EXPECT_LT(sent, 100000);
    EXPECT_GE(clock.now_ms - 1000, SOCKET_SEND_LIMIT_MS);
    EXPECT_LT(clock.now_ms - 1000, SOCKET_SEND_LIMIT_MS + SOCKET_RETRY_MS);

    TestClock fast;
    TestSend twice;
    twice.take = [](size_t piece, size_t call) { return call < 2 ? -1 : int32_t(piece); };
    EXPECT_EQ(send_all(twice, fast, 1000, 1000, false, true), 1000);
    EXPECT_EQ(fast.sleeps, 2U);
}

AP_GTEST_MAIN()
