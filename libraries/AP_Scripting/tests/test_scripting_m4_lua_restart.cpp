#include <AP_gtest.h>
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
#include <AP_Scripting/AP_ScriptingM4_LuaRestart.h>

using namespace ScriptingM4;
using Outcome = LuaRestart::Outcome;
using State = LuaRestart::State;

static LuaRestart running()
{
    LuaRestart lua;
    lua.session_started(true);
    return lua;
}

TEST(ScriptingM4LuaRestart, FaultRestartsLuaAndCharges)
{
    LuaRestart lua = running();
    EXPECT_EQ(lua.control_word(), 0U);
    uint8_t used = 0;
    bool charged = false;
    EXPECT_EQ(lua.parked(0, 1, LuaParkCause::Fault, used, 3, charged), Outcome::Restarted);
    EXPECT_TRUE(charged);
    EXPECT_EQ(used, 1U);
    EXPECT_EQ(lua.control_word(), 1U);   // a new grant, no hold: the M4 restarts Lua
    EXPECT_EQ(lua.state(), State::Running);
}

TEST(ScriptingM4LuaRestart, RepeatedReportChargesOnce)
{
    LuaRestart lua = running();
    uint8_t used = 0;
    bool charged = false;
    lua.parked(0, 1, LuaParkCause::Fault, used, 3, charged);
    EXPECT_EQ(lua.parked(0, 1, LuaParkCause::Fault, used, 3, charged), Outcome::Ignored);
    EXPECT_FALSE(charged);
    EXPECT_EQ(used, 1U);
    EXPECT_EQ(lua.parked(7, 2, LuaParkCause::Fault, used, 3, charged), Outcome::Ignored);   // no such grant
}

TEST(ScriptingM4LuaRestart, StaleReportUnderTheNewGrantIsIgnored)
{
    // the resumed thread is still resetting and the service reads its old report under the new grant
    LuaRestart lua = running();
    uint8_t used = 0;
    bool charged = false;
    EXPECT_EQ(lua.parked(0, 1, LuaParkCause::Fault, used, 3, charged), Outcome::Restarted);
    EXPECT_EQ(lua.parked(1, 1, LuaParkCause::Fault, used, 3, charged), Outcome::Ignored);
    EXPECT_FALSE(charged);
    EXPECT_EQ(used, 1U);
    EXPECT_EQ(lua.grant(), 1U);   // no second grant: no second resume
    EXPECT_EQ(lua.parked(1, 2, LuaParkCause::Fault, used, 3, charged), Outcome::Restarted);   // its next park
    EXPECT_EQ(used, 2U);
}

TEST(ScriptingM4LuaRestart, LimitTurnsLuaOff)
{
    LuaRestart lua = running();
    uint8_t used = 0;
    bool charged = false;
    EXPECT_EQ(lua.parked(0, 1, LuaParkCause::Fault, used, 2, charged), Outcome::Restarted);
    EXPECT_EQ(lua.parked(1, 2, LuaParkCause::Stalled, used, 2, charged), Outcome::Restarted);   // a stall is charged
    EXPECT_EQ(used, 2U);
    EXPECT_EQ(lua.parked(2, 3, LuaParkCause::Fault, used, 2, charged), Outcome::Off);
    EXPECT_EQ(lua.control_word(), 3U | LUA_HOLD);   // answered, and held
    EXPECT_FALSE(lua.lua_live());
    EXPECT_FALSE(lua.request(true, 0));
    EXPECT_EQ(lua.parked(3, 4, LuaParkCause::Requested, used, 2, charged), Outcome::Ignored);
}

TEST(ScriptingM4LuaRestart, StopParksUnchargedAndStartResumes)
{
    LuaRestart lua = running();
    EXPECT_TRUE(lua.request(false, 1000));
    EXPECT_EQ(lua.control_word(), LUA_HOLD);   // the running grant, held
    EXPECT_TRUE(lua.lua_live());              // until it reports
    uint8_t used = 0;
    bool charged = true;
    EXPECT_EQ(lua.parked(0, 1, LuaParkCause::Requested, used, 3, charged), Outcome::Stopped);
    EXPECT_FALSE(charged);
    EXPECT_EQ(lua.control_word(), 1U | LUA_HOLD);
    EXPECT_FALSE(lua.request(false, 2000));   // stopped already
    EXPECT_TRUE(lua.request(true, 2000));
    EXPECT_EQ(lua.control_word(), 2U);
    EXPECT_EQ(lua.state(), State::Running);
    EXPECT_EQ(used, 0U);
}

TEST(ScriptingM4LuaRestart, RestartRequestParksThenRestarts)
{
    LuaRestart lua = running();
    lua.request(true, 0);
    uint8_t used = 0;
    bool charged = true;
    EXPECT_EQ(lua.parked(0, 1, LuaParkCause::Requested, used, 3, charged), Outcome::Restarted);
    EXPECT_FALSE(charged);
    EXPECT_EQ(lua.control_word(), 1U);
}

TEST(ScriptingM4LuaRestart, FaultDuringAStopStaysStoppedAndCharges)
{
    LuaRestart lua = running();
    lua.request(false, 0);
    uint8_t used = 0;
    bool charged = false;
    EXPECT_EQ(lua.parked(0, 1, LuaParkCause::Fault, used, 3, charged), Outcome::Stopped);
    EXPECT_TRUE(charged);
    EXPECT_EQ(used, 1U);
}

TEST(ScriptingM4LuaRestart, ParkThatNeverReportsIsOverdue)
{
    LuaRestart lua = running();
    lua.request(false, 10000);
    EXPECT_FALSE(lua.park_overdue(13000));
    EXPECT_TRUE(lua.park_overdue(13001));
}

TEST(ScriptingM4LuaRestart, ReportWhileHeldForTheRestartIsIgnored)
{
    LuaRestart lua = running();
    lua.hold_until_restart();
    EXPECT_EQ(lua.control_word(), LUA_HOLD);
    uint8_t used = 0;
    bool charged = false;
    EXPECT_EQ(lua.parked(0, 1, LuaParkCause::Requested, used, 3, charged), Outcome::Ignored);
    EXPECT_EQ(lua.next_session_word(true), 0U);   // the grant continues; the next session runs Lua
    lua.session_started(true);
    EXPECT_EQ(lua.control_word(), 0U);
}

TEST(ScriptingM4LuaRestart, StoppedSessionStartsHeldAndAnswersItsReport)
{
    LuaRestart lua;
    EXPECT_EQ(lua.next_session_word(false), LUA_HOLD);
    lua.session_started(false);
    uint8_t used = 0;
    bool charged = false;
    // its Lua thread parks at its first syscall and reports: answered, it stays parked
    EXPECT_EQ(lua.parked(0, 1, LuaParkCause::Requested, used, 3, charged), Outcome::Stopped);
    EXPECT_EQ(lua.control_word(), 1U | LUA_HOLD);
}

TEST(ScriptingM4LuaRestart, EveryAcceptedReportAdvancesTheGrant)
{
    LuaRestart lua = running();
    uint8_t used = 0;
    bool charged = false;
    for (uint32_t i = 0; i < 4; i++) {
        lua.parked(lua.grant(), uint16_t(i + 1), LuaParkCause::Requested, used, 3, charged);
    }
    EXPECT_EQ(lua.control_word() & LUA_HOLD, 0U);
    EXPECT_EQ(lua.grant(), 4U);
}
#endif

AP_GTEST_MAIN()
