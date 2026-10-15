#include <AP_gtest.h>
#include <AP_Scripting/AP_ScriptingM4_ServeWait.h>

using namespace ScriptingM4;

// a flooding M4: the budget ran out and frames remain, so the host pauses
TEST(ScriptingM4ServeWait, BudgetOutWithFramesLeftBacksOff)
{
    EXPECT_EQ(host_wait_after(true, false, false), HostWait::Backoff);
}

// a pass preempted past the budget with nothing left: the next call's doorbell is heard at once
TEST(ScriptingM4ServeWait, BudgetOutWithEmptyRingWaitsOnDoorbell)
{
    EXPECT_EQ(host_wait_after(true, false, true), HostWait::Doorbell);
}

TEST(ScriptingM4ServeWait, IdleWaitsOnDoorbell)
{
    EXPECT_EQ(host_wait_after(false, false, true), HostWait::Doorbell);
    EXPECT_EQ(host_wait_after(false, false, false), HostWait::Doorbell);
}

// a ring that cannot be served waits for the timeout, whatever it holds
TEST(ScriptingM4ServeWait, StuckWaitsForTimeout)
{
    EXPECT_EQ(host_wait_after(false, true, true), HostWait::Timeout);
    EXPECT_EQ(host_wait_after(false, true, false), HostWait::Timeout);
    EXPECT_EQ(host_wait_after(true, true, true), HostWait::Timeout);
    EXPECT_EQ(host_wait_after(true, true, false), HostWait::Timeout);
}

AP_GTEST_MAIN()
