#include <AP_gtest.h>
#include <AP_ModuleM4/AP_ModuleM4_StartFault.h>

using namespace ModuleM4;

// a fault restart's new session shows a new fault count and never beats: the fault is its own,
// reported (with its PC) instead of "heartbeat lost", once
TEST(ModuleM4StartFault, EarlyFaultIsReported)
{
    StartFault start;
    start.session_started(true, 2, 1);
    EXPECT_TRUE(start.report_on_timeout(true));
    EXPECT_FALSE(start.report_on_timeout(true));
}

// the new session beats: the new count was the old session's fault on its way down, absorbed; a
// later stall is "heartbeat lost"
TEST(ModuleM4StartFault, FaultOnTheWayDownIsAbsorbed)
{
    StartFault start;
    start.session_started(true, 2, 1);
    start.beat();
    EXPECT_FALSE(start.report_on_timeout(true));
}

// no new fault count at the start: a stall is "heartbeat lost"
TEST(ModuleM4StartFault, NoFaultAtTheStart)
{
    StartFault start;
    start.session_started(true, 1, 1);
    EXPECT_FALSE(start.report_on_timeout(true));
}

// the first start and a requested restart keep their rules: a fault count there is charged by the
// fault-count check, not reported at the heartbeat timeout
TEST(ModuleM4StartFault, OnlyAfterAFaultRestart)
{
    StartFault start;
    start.session_started(false, 2, 1);
    EXPECT_FALSE(start.report_on_timeout(true));
}

// a header that does not read as compatible (Busy) has no fault record to report
TEST(ModuleM4StartFault, UnreadableHeaderIsHeartbeatLost)
{
    StartFault start;
    start.session_started(true, 2, 1);
    EXPECT_FALSE(start.report_on_timeout(false));
}

// each session starts afresh: a pending fault of one session does not carry into the next
TEST(ModuleM4StartFault, EachSessionStartsAfresh)
{
    StartFault start;
    start.session_started(true, 2, 1);
    start.session_started(true, 2, 2);
    EXPECT_FALSE(start.report_on_timeout(true));
}

// a stop or restart request waits while a fault is pending: taken before the heartbeat tells, it
// could charge the old session's fault (absorbed) a second time
TEST(ModuleM4StartFault, RequestWaitsWhileAFaultIsPending)
{
    StartFault start;
    EXPECT_FALSE(start.holds_requests());
    start.session_started(true, 2, 1);
    EXPECT_TRUE(start.holds_requests());
}

// the session beats: the fault was the old session's, and a waiting request then acts alone
TEST(ModuleM4StartFault, RequestActsAloneAfterABeat)
{
    StartFault start;
    start.session_started(true, 2, 1);
    start.beat();
    EXPECT_FALSE(start.holds_requests());
    EXPECT_FALSE(start.report_on_timeout(true));
}

// the heartbeat times out: the fault is reported once, and a waiting request no longer waits (the
// fault restart carries a stop)
TEST(ModuleM4StartFault, RequestStopsWaitingAtTheTimeout)
{
    StartFault start;
    start.session_started(true, 2, 1);
    EXPECT_TRUE(start.report_on_timeout(true));
    EXPECT_FALSE(start.holds_requests());
    EXPECT_FALSE(start.report_on_timeout(true));
}

// no fault at the start, or not a fault restart: requests act at once
TEST(ModuleM4StartFault, NoWaitWithoutAPendingFault)
{
    StartFault start;
    start.session_started(true, 1, 1);
    EXPECT_FALSE(start.holds_requests());
    start.session_started(false, 2, 1);
    EXPECT_FALSE(start.holds_requests());
}

// a held session ended by a fault restart whatever its cause (here "heartbeat lost", the header
// unreadable at the timeout): that fault restart carries the held requests, once
TEST(ModuleM4StartFault, AnyFaultRestartCarriesHeldRequests)
{
    StartFault start;
    start.session_started(true, 2, 1);
    EXPECT_FALSE(start.report_on_timeout(false));
    EXPECT_TRUE(start.take_held());
    EXPECT_FALSE(start.take_held());
}

// the reported start fault's restart carries them too
TEST(ModuleM4StartFault, ReportedFaultRestartCarriesHeldRequests)
{
    StartFault start;
    start.session_started(true, 2, 1);
    EXPECT_TRUE(start.report_on_timeout(true));
    EXPECT_TRUE(start.take_held());
}

// after a beat, or with nothing pending, requests were taken as they came: nothing to carry
TEST(ModuleM4StartFault, NothingHeldNothingCarried)
{
    StartFault start;
    start.session_started(true, 2, 1);
    start.beat();
    EXPECT_FALSE(start.take_held());
    start.session_started(true, 1, 1);
    EXPECT_FALSE(start.take_held());
    start.session_started(false, 2, 1);
    EXPECT_FALSE(start.take_held());
}

// a fault record cleared at publish() (count 0, below the handled one) is no fault: a stall is
// "heartbeat lost", not a report with every field zero
TEST(ModuleM4StartFault, ClearedFaultRecordIsNoFault)
{
    StartFault start;
    start.session_started(true, 0, 3);
    EXPECT_FALSE(start.report_on_timeout(true));
}

AP_GTEST_MAIN()
