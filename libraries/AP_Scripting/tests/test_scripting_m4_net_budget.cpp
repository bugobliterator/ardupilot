#include <AP_gtest.h>
#include <AP_Scripting/AP_ScriptingM4_NetBudget.h>

using namespace ScriptingM4;

TEST(ScriptingM4NetBudget, NetworkStallsNeverSpendLuasRestarts)
{
    uint8_t used = 0;
    const uint8_t limit = 3;
    for (int i = 0; i < 50; i++) {
        const bool charged = charges_lua_budget(NetFailure::NetworkStalled, true);
        EXPECT_FALSE(charged);
        EXPECT_FALSE(spend_lua_restart(used, limit, charged));   // Lua stays permitted
    }
    EXPECT_EQ(used, 0U);
}

TEST(ScriptingM4NetBudget, ProtocolErrorsWithoutLuaAreTheNetworks)
{
    uint8_t used = 1;
    for (int i = 0; i < 20; i++) {
        EXPECT_FALSE(spend_lua_restart(used, 3, charges_lua_budget(NetFailure::ProtocolErrors, false)));
    }
    EXPECT_EQ(used, 1U);
    // a session running Lua: its calls may be the bad frames
    EXPECT_TRUE(charges_lua_budget(NetFailure::ProtocolErrors, true));
}

TEST(ScriptingM4NetBudget, NoFailureOfASessionWithoutLuaCounts)
{
    // scripts stopped: the M4 runs the network alone, and lwIP faults restart it
    uint8_t used = 1;
    const uint8_t limit = 2;
    const NetFailure causes[] = {NetFailure::Fault, NetFailure::HeartbeatLost, NetFailure::ProtocolErrors,
                                 NetFailure::NetworkStalled};
    for (int i = 0; i < 10; i++) {
        for (NetFailure cause : causes) {
            EXPECT_FALSE(charges_lua_budget(cause, false));
            EXPECT_FALSE(spend_lua_restart(used, limit, charges_lua_budget(cause, false)));   // no "Lua off"
        }
    }
    EXPECT_EQ(used, 1U);
    // the scripts start again: a fault of that session counts
    EXPECT_FALSE(spend_lua_restart(used, limit, charges_lua_budget(NetFailure::Fault, true)));
    EXPECT_EQ(used, 2U);
}

TEST(ScriptingM4NetBudget, LuaFaultsStillCountToTheLimit)
{
    uint8_t used = 0;
    const uint8_t limit = 2;
    EXPECT_FALSE(spend_lua_restart(used, limit, charges_lua_budget(NetFailure::Fault, true)));
    EXPECT_EQ(used, 1U);
    // a stall between the faults changes nothing
    EXPECT_FALSE(spend_lua_restart(used, limit, charges_lua_budget(NetFailure::NetworkStalled, true)));
    EXPECT_EQ(used, 1U);
    EXPECT_FALSE(spend_lua_restart(used, limit, charges_lua_budget(NetFailure::HeartbeatLost, true)));
    EXPECT_EQ(used, 2U);
    // the next fault finds the budget spent: Lua off until reboot
    EXPECT_TRUE(spend_lua_restart(used, limit, charges_lua_budget(NetFailure::Fault, true)));
    EXPECT_EQ(used, 2U);
}

TEST(ScriptingM4NetBudget, NetworkFaultIsNeverCharged)
{
    EXPECT_FALSE(charges_lua_budget(NetFailure::NetworkFault, true));
    EXPECT_TRUE(charges_lua_budget(NetFailure::Fault, true));
    EXPECT_FALSE(charges_lua_budget(NetFailure::Fault, false));   // Lua was not live in that session
}

AP_GTEST_MAIN()
