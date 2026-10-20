#include <AP_gtest.h>
#include <AP_ModuleM4/AP_ModuleM4_LoadLayout.h>

using namespace ModuleM4;

static const uint32_t AREA_BASE = 0x30008000U;
static const uint32_t AREA_SIZE = 229376U;

TEST(ModuleM4LoadLayout, ReferenceCaseMatchesTheGateTool)
{
    LoadLayout l {};
    const LoadRequest request { 100000, 90000, 8, 3000, 8, 32768, 32768 };
    ASSERT_EQ(load_layout(AREA_BASE, AREA_SIZE, request, l), LoadFit::Ok);
    EXPECT_EQ(l.window.rx_base, 0x30028000U);
    EXPECT_EQ(l.rw_base, 0x30027448U);
    EXPECT_EQ(l.staging_base, AREA_BASE);
    EXPECT_EQ(l.guard_base, AREA_BASE);
    EXPECT_EQ(l.stack_base, AREA_BASE + 32U);
    EXPECT_EQ(l.stack_top, AREA_BASE + 32U + 32768U);
    EXPECT_EQ(l.heap_base, l.stack_top);
    EXPECT_EQ(l.heap_end, l.heap_base + 32768U);
    EXPECT_EQ(l.load_peak, 201304U);
    EXPECT_EQ(l.run_need, 166872U);
}

TEST(ModuleM4LoadLayout, StagingThatReachesRwDoesNotFit)
{
    LoadLayout l {};
    EXPECT_EQ(load_layout(AREA_BASE, AREA_SIZE, LoadRequest { 130000, 110000, 8, 4000, 8, 32768, 32768 }, l),
              LoadFit::NoRoomToStage);
}

TEST(ModuleM4LoadLayout, HeapThatReachesRwDoesNotFit)
{
    LoadLayout l {};
    EXPECT_EQ(load_layout(AREA_BASE, AREA_SIZE, LoadRequest { 60000, 50000, 8, 2000, 8, 65536, 120000 }, l),
              LoadFit::NoRoomToRun);
}

TEST(ModuleM4LoadLayout, CodeAboveTheLargestWindow)
{
    LoadLayout l {};
    EXPECT_EQ(load_layout(AREA_BASE, AREA_SIZE, LoadRequest { 140000, 131073, 8, 100, 8, 1024, 1024 }, l),
              LoadFit::CodeTooLarge);
}

TEST(ModuleM4LoadLayout, RwAlignmentRoundsDown)
{
    LoadLayout l {};
    ASSERT_EQ(load_layout(AREA_BASE, AREA_SIZE, LoadRequest { 2000, 1000, 8, 100, 64, 4096, 4096 }, l), LoadFit::Ok);
    EXPECT_EQ(l.rw_base % 64U, 0U);
    EXPECT_LE(l.rw_base + 100U, l.window.rx_base);
}

TEST(ModuleM4LoadLayout, HostileRwSizeIsRefused)
{
    LoadLayout l {};
    EXPECT_EQ(load_layout(AREA_BASE, AREA_SIZE, LoadRequest { 1000, 1000, 8, 0xF0000000U, 8, 4096, 4096 }, l),
              LoadFit::NoRoomToStage);
    EXPECT_EQ(load_layout(AREA_BASE, AREA_SIZE, LoadRequest { 1000, 1000, 8, 0xFFFFFFFFU, 8, 4096, 4096 }, l),
              LoadFit::NoRoomToStage);
}

TEST(ModuleM4LoadLayout, StackAndHeapSizesAreBounded)
{
    LoadLayout l {};
    EXPECT_EQ(load_layout(AREA_BASE, AREA_SIZE, LoadRequest { 2000, 1000, 8, 100, 8, 0xFFFFFFF0U, 4096 }, l),
              LoadFit::NoRoomToRun);   // rounds up to 0
    EXPECT_EQ(load_layout(AREA_BASE, AREA_SIZE, LoadRequest { 2000, 1000, 8, 100, 8, 4096, 0xFFFFFFFFU }, l),
              LoadFit::NoRoomToRun);   // heap end wraps below RW
    EXPECT_EQ(load_layout(AREA_BASE, AREA_SIZE, LoadRequest { 2000, 1000, 8, 100, 8, 0, 4096 }, l),
              LoadFit::NoRoomToRun);
    EXPECT_EQ(load_layout(AREA_BASE, AREA_SIZE, LoadRequest { 2000, 1000, 8, 100, 8, 4095, 4096 }, l),
              LoadFit::NoRoomToRun);
    EXPECT_EQ(load_layout(AREA_BASE, AREA_SIZE, LoadRequest { 2000, 1000, 8, 100, 8, 4096, 0 }, l),
              LoadFit::NoRoomToRun);
    EXPECT_EQ(load_layout(AREA_BASE, AREA_SIZE, LoadRequest { 2000, 1000, 8, 100, 8, 4096, 15 }, l),
              LoadFit::NoRoomToRun);
    EXPECT_EQ(load_layout(AREA_BASE, AREA_SIZE, LoadRequest { 2000, 1000, 8, 100, 8, 4096, 16 }, l), LoadFit::Ok);
}

TEST(ModuleM4LoadLayout, RwAlignmentIsAPowerOfTwoUpTo4096)
{
    LoadLayout l {};
    EXPECT_EQ(load_layout(AREA_BASE, AREA_SIZE, LoadRequest { 2000, 1000, 8, 100, 24, 4096, 4096 }, l),
              LoadFit::NoRoomToStage);
    EXPECT_EQ(load_layout(AREA_BASE, AREA_SIZE, LoadRequest { 2000, 1000, 8, 100, 0, 4096, 4096 }, l),
              LoadFit::NoRoomToStage);
    EXPECT_EQ(load_layout(AREA_BASE, AREA_SIZE, LoadRequest { 2000, 1000, 8, 100, 0x40000000U, 4096, 4096 }, l),
              LoadFit::NoRoomToStage);
    EXPECT_EQ(load_layout(AREA_BASE, AREA_SIZE, LoadRequest { 2000, 1000, 8, 100, 8192, 4096, 4096 }, l),
              LoadFit::NoRoomToStage);
    ASSERT_EQ(load_layout(AREA_BASE, AREA_SIZE, LoadRequest { 2000, 1000, 8, 100, 4096, 4096, 4096 }, l),
              LoadFit::Ok);
    EXPECT_EQ(l.rw_base % 4096U, 0U);
}

TEST(ModuleM4LoadLayout, RwRoundedBelowTheAreaDoesNotFit)
{
    LoadLayout l {};
    const uint32_t base = AREA_BASE + 4U;
    const uint32_t rx_base = 0x3003FC00U;   // 1000 B of code in a 1 KiB window
    EXPECT_EQ(load_layout(base, AREA_BASE + AREA_SIZE - base, LoadRequest { 0, 1000, 8, rx_base - base, 8, 4096, 4096 }, l),
              LoadFit::NoRoomToStage);
}

TEST(ModuleM4LoadLayout, AreaSmallerThanTheCodeWindow)
{
    LoadLayout l {};
    EXPECT_EQ(load_layout(0x30038000U, 32768U, LoadRequest { 100, 65536, 8, 100, 8, 4096, 4096 }, l),
              LoadFit::CodeTooLarge);
}

AP_GTEST_MAIN()
