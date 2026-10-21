#include <AP_gtest.h>
#include <AP_RustM4/AP_RustM4_RunControl.h>

using namespace RustM4;

// a session that has run long enough for a request to be acted on
static const uint32_t RUNNING = RUN_CONTROL_MIN_SESSION_MS;

// STOP: the scripts stop and stay stopped; a second STOP changes nothing; STOP_AND_RESTART starts
// them again. Without the stopped state STOP would be a restart with the scripts
TEST(RustM4RunControl, StopThenRestart)
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
TEST(RustM4RunControl, StopAndRestartIsOneRestart)
{
    RunControl control;
    EXPECT_EQ(control.update(true, true, true, RUNNING), RunAction::Restart);
    EXPECT_FALSE(control.stopped());
}

// a request in a session's first second waits for it, as the in-process thread looks once a
// second: a script restarting scripting at its start restarts at most once a second. Without the
// wait the first update restarts
TEST(RustM4RunControl, RequestsWaitForOneSecondOfSession)
{
    RunControl control;
    EXPECT_EQ(control.update(true, true, true, RUNNING - 1), RunAction::None);
    EXPECT_EQ(control.update(true, false, false, RUNNING - 1), RunAction::None);
    EXPECT_EQ(control.update(true, false, false, RUNNING), RunAction::Restart);
}

// a stopped session waits its first second too: SCR_ENABLE toggled 0, 1, 0, 1 restarts the M4 at
// most once a second. Without the wait the stopped session restarts at its first update
TEST(RustM4RunControl, StoppedSessionsWaitToo)
{
    RunControl control;
    EXPECT_EQ(control.update(false, false, false, RUNNING), RunAction::Stop);
    // the stopped session starts
    EXPECT_EQ(control.update(true, false, false, 0), RunAction::None);
    EXPECT_EQ(control.update(true, false, false, RUNNING - 1), RunAction::None);
    EXPECT_EQ(control.update(true, false, false, RUNNING), RunAction::Restart);
}

// a STOP acts at once, in a session's first second too, as the in-process run loop stops at its
// next check. Without it the stop waits a second, and a restart asked meanwhile replaces it
TEST(RustM4RunControl, StopActsAtOnce)
{
    RunControl control;
    EXPECT_EQ(control.update(true, true, false, 0), RunAction::Stop);
    EXPECT_TRUE(control.stopped());
    RunControl disabled;
    EXPECT_EQ(disabled.update(false, false, false, 0), RunAction::Stop);
}

// a STOP after a restart not yet acted on stops at once: the restart is dropped, and a stop is
// never replaced by a later restart
TEST(RustM4RunControl, AStopIsNeverCancelledByARestart)
{
    RunControl control;
    EXPECT_EQ(control.update(true, true, true, 10), RunAction::None);
    EXPECT_EQ(control.update(true, true, false, 20), RunAction::Stop);
    EXPECT_EQ(control.update(true, false, false, RUNNING), RunAction::None);
    EXPECT_TRUE(control.stopped());
}

// the requests as the host takes them: a stop asked without a restart wins over a restart asked
// before or after it (a script's restart_all() racing a GCS STOP). Without the stop bit kept, the
// restart after the stop takes it as a restart
TEST(RustM4RunControl, TakenRequestsKeepAStop)
{
    RunRequests requests;
    bool restart = true;
    EXPECT_FALSE(requests.take(restart));
    requests.restart();
    requests.stop();
    requests.restart();
    EXPECT_TRUE(requests.take(restart));
    EXPECT_FALSE(restart);
    EXPECT_FALSE(requests.take(restart));   // taken once
    requests.restart();
    EXPECT_TRUE(requests.take(restart));
    EXPECT_TRUE(restart);
}

// a script that calls restart_all() every 100 ms restarts scripting once a second; a GCS STOP stops
// it at the next pass, and it stays stopped. Without the at-once stop the STOP waits for the
// second, and the script's next restart_all() turns it into a restart
TEST(RustM4RunControl, ARestartingScriptIsStoppedAndStaysStopped)
{
    RunRequests requests;
    RunControl control;
    uint32_t session_ms = 0;
    uint32_t last_restart_ms = 0;
    uint32_t now_ms = 0;
    unsigned restarts = 0;
    bool restart = false;
    // the host passes every 100 ms; the script runs while the session is not stopped
    for (int pass = 0; pass < 100; pass++, now_ms += 100, session_ms += 100) {
        if (!control.stopped()) {
            requests.restart();   // the script's restart_all()
        }
        if (pass == 35) {
            requests.stop();      // MAV_CMD_SCRIPTING STOP, between two of the script's calls
            requests.restart();
        }
        const bool stop = requests.take(restart);
        const RunAction action = control.update(true, stop, restart, session_ms);
        if (pass < 35) {
            EXPECT_NE(action, RunAction::Stop);
        } else if (pass == 35) {
            EXPECT_EQ(action, RunAction::Stop);
        } else {
            EXPECT_EQ(action, RunAction::None);
        }
        if (action == RunAction::Restart) {
            if (restarts > 0) {
                EXPECT_GE(now_ms - last_restart_ms, RUN_CONTROL_MIN_SESSION_MS);
            }
            restarts++;
            last_restart_ms = now_ms;
        }
        if (action != RunAction::None) {
            session_ms = 0;   // the M4 restarts into a new session
        }
    }
    EXPECT_GE(restarts, 3U);
    EXPECT_TRUE(control.stopped());
}

// SCR_ENABLE 0 stops the scripts; back to 1 starts them again (in-process: cleared). Without the
// enable check SCR_ENABLE changes nothing until a reboot
TEST(RustM4RunControl, ScrEnableStopsAndStarts)
{
    RunControl control;
    EXPECT_EQ(control.update(false, false, false, RUNNING), RunAction::Stop);
    EXPECT_EQ(control.update(false, false, false, RUNNING), RunAction::None);
    EXPECT_EQ(control.update(true, false, false, RUNNING), RunAction::Restart);
}

// stopped by STOP, an enable that never went to 0 starts nothing; a restart asked while disabled
// waits for the enable
TEST(RustM4RunControl, RestartNeedsTheEnable)
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

TEST(RustM4RunControl, BackoffDoublesToThirtySeconds)
{
    EXPECT_EQ(restart_backoff_ms(1), 1000U);
    EXPECT_EQ(restart_backoff_ms(2), 2000U);
    EXPECT_EQ(restart_backoff_ms(3), 4000U);
    EXPECT_EQ(restart_backoff_ms(5), 16000U);
    EXPECT_EQ(restart_backoff_ms(6), 30000U);
    EXPECT_EQ(restart_backoff_ms(200), 30000U);
    EXPECT_EQ(restart_backoff_ms(0), 0U);   // a requested restart waits for nothing
}

AP_GTEST_MAIN()
