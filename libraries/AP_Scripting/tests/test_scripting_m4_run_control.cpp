#include <AP_gtest.h>
#include <AP_Scripting/AP_ScriptingM4_RunControl.h>

using namespace ScriptingM4;

// a session that has run long enough for a request to be acted on
static const uint32_t RUNNING = RUN_CONTROL_MIN_SESSION_MS;

// STOP: the scripts stop and stay stopped; a second STOP changes nothing; STOP_AND_RESTART starts
// them again. Without the stopped state STOP would be a restart with the scripts
TEST(ScriptingM4RunControl, StopThenRestart)
{
    RunControl control;
    EXPECT_EQ(control.update(true, false, false, RUNNING), RunAction::None);
    EXPECT_EQ(control.update(true, true, false, RUNNING), RunAction::Stop);
    EXPECT_TRUE(control.stopped());
    EXPECT_EQ(control.update(true, false, false, RUNNING), RunAction::None);
    EXPECT_EQ(control.update(true, true, false, RUNNING), RunAction::None);
    EXPECT_EQ(control.update(true, true, true, RUNNING), RunAction::Restart);
    EXPECT_FALSE(control.stopped());
    EXPECT_EQ(control.update(true, false, false, RUNNING), RunAction::None);
}

// STOP_AND_RESTART and restart_all() while the scripts run: one restart, no stopped session
TEST(ScriptingM4RunControl, StopAndRestartIsOneRestart)
{
    RunControl control;
    EXPECT_EQ(control.update(true, true, true, RUNNING), RunAction::Restart);
    EXPECT_FALSE(control.stopped());
}

// a request in a session's first second waits for it, as the in-process thread looks once a
// second: a script restarting scripting at its start restarts at most once a second. Without the
// wait the first update restarts
TEST(ScriptingM4RunControl, RequestsWaitForOneSecondOfSession)
{
    RunControl control;
    EXPECT_EQ(control.update(true, true, true, RUNNING - 1), RunAction::None);
    EXPECT_EQ(control.update(true, false, false, RUNNING - 1), RunAction::None);
    EXPECT_EQ(control.update(true, false, false, RUNNING), RunAction::Restart);
}

// a stopped session waits its first second too: SCR_ENABLE toggled 0, 1, 0, 1 restarts the M4 at
// most once a second. Without the wait the stopped session restarts at its first update
TEST(ScriptingM4RunControl, StoppedSessionsWaitToo)
{
    RunControl control;
    EXPECT_EQ(control.update(false, false, false, RUNNING), RunAction::Stop);
    // the stopped session starts
    EXPECT_EQ(control.update(true, false, false, 0), RunAction::None);
    EXPECT_EQ(control.update(true, false, false, RUNNING - 1), RunAction::None);
    EXPECT_EQ(control.update(true, false, false, RUNNING), RunAction::Restart);
}

// the later of two requests before they are acted on wins, as AP_Scripting's flags are
// overwritten: STOP after STOP_AND_RESTART stops
TEST(ScriptingM4RunControl, TheLaterRequestWins)
{
    RunControl control;
    EXPECT_EQ(control.update(true, true, true, 10), RunAction::None);
    EXPECT_EQ(control.update(true, true, false, RUNNING), RunAction::Stop);
}

// SCR_ENABLE 0 stops the scripts; back to 1 starts them again (in-process: cleared). Without the
// enable check SCR_ENABLE changes nothing until a reboot
TEST(ScriptingM4RunControl, ScrEnableStopsAndStarts)
{
    RunControl control;
    EXPECT_EQ(control.update(false, false, false, RUNNING), RunAction::Stop);
    EXPECT_EQ(control.update(false, false, false, RUNNING), RunAction::None);
    EXPECT_EQ(control.update(true, false, false, RUNNING), RunAction::Restart);
}

// stopped by STOP, an enable that never went to 0 starts nothing; a restart asked while disabled
// waits for the enable
TEST(ScriptingM4RunControl, RestartNeedsTheEnable)
{
    RunControl control;
    EXPECT_EQ(control.update(true, true, false, RUNNING), RunAction::Stop);
    for (int i = 0; i < 5; i++) {
        EXPECT_EQ(control.update(true, false, false, RUNNING), RunAction::None);
    }
    RunControl disabled;
    EXPECT_EQ(disabled.update(false, true, true, RUNNING), RunAction::Stop);
    EXPECT_EQ(disabled.update(false, false, false, RUNNING), RunAction::None);
    EXPECT_EQ(disabled.update(true, false, false, RUNNING), RunAction::Restart);
}

AP_GTEST_MAIN()
