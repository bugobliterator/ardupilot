#include <AP_gtest.h>
#include <AP_Scripting/AP_ScriptingM4_LuaPermit.h>

using namespace ScriptingM4;

TEST(ScriptingM4LuaPermit, SessionWithoutThePermitServesNoLuaCall)
{
    LuaPermit permit;
    permit.store_next(false);
    permit.session_granted();
    EXPECT_FALSE(permit.serves(MsgType::Call));      // a binding call: refused, a protocol error
    // Notify too: SCR_M4_INJECT 6 (a Noop flood) with Lua held ends in a protocol-error restart
    EXPECT_FALSE(permit.serves(MsgType::Notify));
    // the M4 service's own messages keep the network and the supervision running
    EXPECT_TRUE(permit.serves(MsgType::Stats));
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
    EXPECT_TRUE(permit.serves(MsgType::NetConfigRequest));
#endif
}

TEST(ScriptingM4LuaPermit, SessionWithThePermitServesLuaCalls)
{
    LuaPermit permit;
    permit.store_next(true);
    permit.session_granted();
    EXPECT_TRUE(permit.serves(MsgType::Call));
    EXPECT_TRUE(permit.serves(MsgType::Notify));
    EXPECT_TRUE(permit.serves(MsgType::Stats));
}

TEST(ScriptingM4LuaPermit, PermitChangesOnlyAtTheNextGrant)
{
    LuaPermit permit;
    EXPECT_FALSE(permit.serves(MsgType::Call));      // nothing granted yet
    permit.store_next(true);
    EXPECT_FALSE(permit.serves(MsgType::Call));
    permit.session_granted();
    EXPECT_TRUE(permit.serves(MsgType::Call));
    // a scripting stop stores 0 for the next start; the running session keeps its permit
    permit.store_next(false);
    EXPECT_TRUE(permit.serves(MsgType::Call));
    permit.session_granted();
    EXPECT_FALSE(permit.serves(MsgType::Call));
    EXPECT_FALSE(permit.session_permitted());
}

TEST(ScriptingM4LuaPermit, LuaCallWithoutThePermitIsDroppedNotRefusedByTheDutyCap)
{
    LuaPermit permit;
    permit.store_next(false);
    permit.session_granted();
    EXPECT_EQ(slow_call_route(MsgType::Call, true, true, permit), SlowCallRoute::ProtocolError);
    EXPECT_EQ(slow_call_route(MsgType::Notify, true, true, permit), SlowCallRoute::ProtocolError);
    EXPECT_EQ(slow_call_route(MsgType::Call, false, true, permit), SlowCallRoute::NotRefused);   // a system method
    EXPECT_EQ(slow_call_route(MsgType::Stats, false, true, permit), SlowCallRoute::NotRefused);
    EXPECT_EQ(slow_call_route(MsgType::Call, true, false, permit), SlowCallRoute::DutyCapError);  // NET_ENABLE 0
    permit.store_next(true);
    permit.session_granted();
    EXPECT_EQ(slow_call_route(MsgType::Call, true, true, permit), SlowCallRoute::DutyCapError);
}

AP_GTEST_MAIN()
