#include <AP_gtest.h>
#include <AP_Scripting/AP_ScriptingM4_Layout.h>

using namespace ScriptingM4;

#if !(defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED)
// c8a1a9f55a's layout: plain M4 Lua builds keep it (owner, 2026-10-15)
TEST(ScriptingM4Layout, PlainBuildKeepsTheM4LuaLayout)
{
    EXPECT_EQ(AP_SCRIPTING_M4_WINDOW_BASE, 0x30040000U);
    EXPECT_EQ(WINDOW_SIZE, 32768U);
    EXPECT_EQ(AP_SCRIPTING_M4_WINDOW_MPU_SIZE, 14U);
    EXPECT_EQ(SUPERVISOR_BLOCK_SIZE, 32U);
    EXPECT_EQ(CHANNEL_WINDOW_SIZE, 32736U);
    EXPECT_EQ(AP_SCRIPTING_M4_PRIV_RAM_SIZE, 32768U);
    EXPECT_EQ(AP_SCRIPTING_M4_LUA_STACK_SIZE, 16384U);
    EXPECT_EQ(AP_SCRIPTING_M4_LUA_HEAP_MIN, 196608U);
    EXPECT_EQ(RPC_PROTOCOL_VERSION, 3U);
}
#endif

#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
TEST(ScriptingM4Layout, NetworkingWindowSplitsIntoChannelBlockAndRings)
{
    EXPECT_EQ(CHANNEL_WINDOW_SIZE, 16352U);
    EXPECT_EQ(CHANNEL_WINDOW_SIZE + SUPERVISOR_BLOCK_SIZE, RING_REGION_OFFSET);
    EXPECT_EQ(AP_SCRIPTING_M4_WINDOW_BASE + RING_REGION_OFFSET, AP_NETWORKING_M4_RING_BASE);
    EXPECT_EQ(RING_REGION_OFFSET + AP_NETWORKING_M4_RING_SIZE, WINDOW_SIZE);
    EXPECT_EQ(AP_NETWORKING_M4_RING_BASE % AP_NETWORKING_M4_RING_SIZE, 0U);
    EXPECT_NE(AP_NETWORKING_M4_DOORBELL_GROUP, AP_SCRIPTING_M4_DOORBELL_GROUP);
    EXPECT_NE(AP_NETWORKING_M4_DOORBELL_GROUP, AP_SCRIPTING_M4_RESTART_GROUP);
    EXPECT_EQ(RPC_PROTOCOL_VERSION, 4U);
}

TEST(ScriptingM4Layout, NetworkingClearLeavesTheRingRegion)
{
    static uint8_t window[AP_SCRIPTING_M4_WINDOW_SIZE];
    memset(window, 0xA5, sizeof(window));
    clear_channel_region(window);
    for (uint32_t i = 0; i < RING_REGION_OFFSET; i++) {
        ASSERT_EQ(window[i], 0U) << i;
    }
    for (uint32_t i = RING_REGION_OFFSET; i < sizeof(window); i++) {
        ASSERT_EQ(window[i], 0xA5U) << i;
    }
}
#endif

TEST(ScriptingM4Layout, ClearCoversTheChannelAndBlock)
{
    static uint8_t window[AP_SCRIPTING_M4_WINDOW_SIZE];
    memset(window, 0xA5, sizeof(window));
    clear_channel_region(window);
    for (uint32_t i = 0; i < AP_SCRIPTING_M4_CHANNEL_REGION_SIZE; i++) {
        ASSERT_EQ(window[i], 0U) << i;
    }
}

AP_GTEST_MAIN()
