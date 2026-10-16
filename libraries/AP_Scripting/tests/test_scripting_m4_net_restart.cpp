#include <AP_gtest.h>
#include <AP_Scripting/AP_ScriptingM4_NetRestart.h>

using namespace ScriptingM4;

TEST(ScriptingM4NetRestart, BackoffDoublesToThirtySeconds)
{
    NetRestart restart;
    const uint32_t expected[] = {1000, 2000, 4000, 8000, 16000, 30000, 30000};
    uint32_t now = 5000;
    for (uint32_t delay : expected) {
        restart.session_started(now);
        now += 100;                       // each session faults at once
        EXPECT_EQ(restart.fault(now), delay);
        EXPECT_FALSE(restart.due(now + delay - 1));
        EXPECT_TRUE(restart.due(now + delay));
        now += delay;
    }
    EXPECT_EQ(restart.count(), 7U);
}

TEST(ScriptingM4NetRestart, AHealthySessionResetsTheBackoff)
{
    NetRestart restart;
    restart.session_started(0);
    restart.fault(10);
    restart.session_started(1010);
    restart.fault(1020);                  // 2 s
    restart.session_started(3020);
    EXPECT_EQ(restart.fault(3020 + 60000), 1000U);
}

TEST(ScriptingM4NetRestart, DueSurvivesTheMillisWrap)
{
    NetRestart restart;
    restart.session_started(0xFFFFFF00U);
    restart.fault(0xFFFFFF00U);
    EXPECT_FALSE(restart.due(0xFFFFFFFFU));
    EXPECT_TRUE(restart.due(0x000002E8U));   // 0xFFFFFF00 + 1000
}

AP_GTEST_MAIN()
