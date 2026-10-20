#include <AP_gtest.h>
#include <AP_RustM4/AP_RustM4_GranuleHeap.h>

using namespace RustM4;

struct HeapFixture {
    alignas(4096) uint8_t arena[8192];
    uint32_t used[16];
    uint32_t starts[16];
    GranuleHeap heap;
    HeapFixture(uint32_t size = sizeof(arena))
    {
        EXPECT_TRUE(heap.init(uintptr_t(arena), size, used, starts, 16));
    }
};

TEST(RustM4GranuleHeap, BitmapWordsCoverTheArena)
{
    EXPECT_EQ(GranuleHeap::bitmap_words(32768), 64U);   // 2048 granules: 256 B per bitmap, 512 B in all
    EXPECT_EQ(GranuleHeap::bitmap_words(16), 1U);
}

TEST(RustM4GranuleHeap, RefusesBadArguments)
{
    HeapFixture f;
    EXPECT_EQ(f.heap.allocate(0, 8), nullptr);
    EXPECT_EQ(f.heap.allocate(16, 0), nullptr);
    EXPECT_EQ(f.heap.allocate(16, 24), nullptr);
    EXPECT_EQ(f.heap.allocate(16, 8192), nullptr);   // above the PR's 4096
    GranuleHeap other;
    uint32_t words[1];
    EXPECT_FALSE(other.init(uintptr_t(f.arena) + 1, 64, words, words, 1));   // not granule-aligned
    EXPECT_FALSE(other.init(uintptr_t(f.arena), 8192, words, words, 1));      // bitmaps too small
}

TEST(RustM4GranuleHeap, AlignmentIsHonoured)
{
    HeapFixture f;
    void *small = f.heap.allocate(1, 1);
    ASSERT_NE(small, nullptr);
    for (uint32_t alignment = 16; alignment <= 4096; alignment <<= 1) {
        void *p = f.heap.allocate(8, alignment);
        ASSERT_NE(p, nullptr) << alignment;
        EXPECT_EQ(uintptr_t(p) % alignment, 0U) << alignment;
    }
}

TEST(RustM4GranuleHeap, ExhaustionAndReuse)
{
    HeapFixture f(1024);
    void *blocks[16];
    for (auto &b : blocks) {
        b = f.heap.allocate(64, 16);
        ASSERT_NE(b, nullptr);
    }
    EXPECT_EQ(f.heap.allocate(16, 16), nullptr);
    EXPECT_EQ(f.heap.bytes_in_use(), 1024U);
    EXPECT_EQ(f.heap.free(blocks[3]), GranuleHeap::FreeResult::Freed);
    EXPECT_EQ(f.heap.allocate(64, 16), blocks[3]);
}

TEST(RustM4GranuleHeap, FragmentationLeavesNoRoomForALargerBlock)
{
    HeapFixture f(1024);
    void *blocks[16];
    for (auto &b : blocks) {
        b = f.heap.allocate(64, 16);
    }
    for (uint32_t i = 0; i < 16; i += 2) {
        EXPECT_EQ(f.heap.free(blocks[i]), GranuleHeap::FreeResult::Freed);
    }
    EXPECT_EQ(f.heap.allocate(128, 16), nullptr);
    EXPECT_EQ(f.heap.allocate(64, 16), blocks[0]);
}

TEST(RustM4GranuleHeap, AdjacentBlocksStaySeparate)
{
    HeapFixture f(1024);
    void *a = f.heap.allocate(32, 16);
    void *b = f.heap.allocate(32, 16);
    ASSERT_EQ(uintptr_t(b), uintptr_t(a) + 32U);
    EXPECT_EQ(f.heap.free(a), GranuleHeap::FreeResult::Freed);
    EXPECT_EQ(f.heap.bytes_in_use(), 32U);
    EXPECT_EQ(f.heap.free(b), GranuleHeap::FreeResult::Freed);
    EXPECT_EQ(f.heap.bytes_in_use(), 0U);
}

TEST(RustM4GranuleHeap, DoubleForeignAndInteriorFreesAreRefused)
{
    HeapFixture f;
    uint8_t *p = static_cast<uint8_t *>(f.heap.allocate(64, 16));
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(f.heap.free(p + 16), GranuleHeap::FreeResult::NotABlock);
    EXPECT_EQ(f.heap.free(p + 1), GranuleHeap::FreeResult::NotABlock);
    static uint8_t outside[32];
    EXPECT_EQ(f.heap.free(outside), GranuleHeap::FreeResult::NotABlock);
    EXPECT_EQ(f.heap.free(f.arena + sizeof(f.arena)), GranuleHeap::FreeResult::NotABlock);
    EXPECT_EQ(f.heap.free(p), GranuleHeap::FreeResult::Freed);
    EXPECT_EQ(f.heap.free(p), GranuleHeap::FreeResult::NotABlock);
    EXPECT_EQ(f.heap.free(nullptr), GranuleHeap::FreeResult::Freed);   // PR Heap::deallocate(nullptr)
}

AP_GTEST_MAIN()
