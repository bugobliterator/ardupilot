#include <AP_gtest.h>
#include <AP_Scripting/AP_ScriptingM4_NetStall.h>

using namespace ScriptingM4;

TEST(ScriptingM4NetStall, StillHeartbeatStallsAfterThreeSeconds)
{
    NetStall stall;
    stall.session_started();
    uint32_t heartbeat = 1;
    uint32_t now = 1000;
    // a status every 100 ms: never stalled
    for (; now < 10000; now += 100) {
        EXPECT_FALSE(stall.stalled(true, heartbeat++, now));
    }
    const uint32_t last_move = now - 100;
    // the net_port thread stops: the block still reads, with the same heartbeat
    EXPECT_FALSE(stall.stalled(true, heartbeat - 1, last_move + NetStall::STALL_MS - 1));
    EXPECT_TRUE(stall.stalled(true, heartbeat - 1, last_move + NetStall::STALL_MS));
    // moving again (a late write before the restart): not stalled
    EXPECT_FALSE(stall.stalled(true, heartbeat, last_move + NetStall::STALL_MS + 10));
}

TEST(ScriptingM4NetStall, TornBlockCountsAsStill)
{
    NetStall stall;
    stall.session_started();
    EXPECT_FALSE(stall.stalled(true, 7, 0));
    // the M4 stopped inside a write: every read is torn
    EXPECT_FALSE(stall.stalled(false, 0, NetStall::STALL_MS - 1));
    EXPECT_TRUE(stall.stalled(false, 0, NetStall::STALL_MS));
}

TEST(ScriptingM4NetStall, NoStatusNeverStalls)
{
    NetStall stall;
    stall.session_started();
    // no network configured (config magic 0): the session never writes a status
    for (uint32_t now = 0; now < 60000; now += 500) {
        EXPECT_FALSE(stall.stalled(false, 0, now));
    }
}

TEST(ScriptingM4NetStall, NewSessionStartsUnseen)
{
    NetStall stall;
    stall.session_started();
    EXPECT_FALSE(stall.stalled(true, 3, 100));
    EXPECT_TRUE(stall.stalled(true, 3, 100 + NetStall::STALL_MS));
    // a restart: the old session's block stays until the new one writes
    stall.session_started();
    EXPECT_FALSE(stall.stalled(false, 0, 100 + 2 * NetStall::STALL_MS));
    EXPECT_FALSE(stall.stalled(true, 3, 200 + 2 * NetStall::STALL_MS));   // the new session's first write
    EXPECT_TRUE(stall.stalled(true, 3, 200 + 3 * NetStall::STALL_MS));
}

TEST(ScriptingM4NetStall, SurvivesTheMillisWrap)
{
    NetStall stall;
    stall.session_started();
    EXPECT_FALSE(stall.stalled(true, 1, 0xFFFFFF00U));
    EXPECT_FALSE(stall.stalled(true, 1, 0x00000100U));
    EXPECT_TRUE(stall.stalled(true, 1, 0xFFFFFF00U + NetStall::STALL_MS));
}

AP_GTEST_MAIN()
