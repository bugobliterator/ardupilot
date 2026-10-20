#include <AP_gtest.h>
#include <AP_Math/crc.h>
#include <AP_RustM4/AP_RustM4_Crc32.h>

using namespace RustM4;

// the SITL M4 process's platform feed, as m4/sitl/crc_port.cpp
void RustM4::crc32_feed(Crc32 &crc, const uint8_t *data, uint32_t length)
{
    crc.state = crc32_software_update(crc.state, data, length);
}

static uint32_t whole(const uint8_t *data, uint32_t length)
{
    Crc32 crc;
    crc32_begin(crc);
    crc32_feed(crc, data, length);
    return crc32_finish(crc);
}

static void fill_pattern(uint8_t *pattern, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++) {
        pattern[i] = uint8_t(i * 7 + 3);
    }
}

TEST(RustM4Crc32, CheckValueIsZlibs)
{
    EXPECT_EQ(whole(reinterpret_cast<const uint8_t *>("123456789"), 9), CRC32_CHECK);
    EXPECT_EQ(whole(nullptr, 0), 0U);
}

TEST(RustM4Crc32, PatternInPiecesEqualsWhole)
{
    uint8_t pattern[2048];
    fill_pattern(pattern, sizeof(pattern));
    EXPECT_EQ(whole(pattern, sizeof(pattern)), CRC32_PATTERN_CHECK);
    Crc32 crc;
    crc32_begin(crc);
    crc32_feed(crc, pattern, 1001);
    crc32_feed(crc, pattern + 1001, 1047);
    EXPECT_EQ(crc32_finish(crc), CRC32_PATTERN_CHECK);
}

TEST(RustM4Crc32, StateMatchesTheCrc32SmallImageCppUses)
{
    uint8_t buffer[333];
    for (uint32_t i = 0; i < sizeof(buffer); i++) {
        buffer[i] = uint8_t(i * 131 + 17);
    }
    for (uint32_t length : { 0U, 1U, 3U, 4U, 5U, 64U, 333U }) {
        EXPECT_EQ(crc32_software_update(0x12345678U, buffer, length), crc32_small(0x12345678U, buffer, length)) << length;
    }
}

// cubefw_pal_crc32's contract (CubeFramework PAL.h): zlib.crc32 with a seed that chains
TEST(RustM4Crc32, SeedChainsAsZlib)
{
    const uint8_t *digits = reinterpret_cast<const uint8_t *>("123456789");
    EXPECT_EQ(crc32_zlib(0, digits, 9), CRC32_CHECK);
    EXPECT_EQ(crc32_zlib(0x12345678U, digits, 9), 0x01F4807BU);   // zlib.crc32(b"123456789", 0x12345678)
    EXPECT_EQ(crc32_zlib(0x12345678U, nullptr, 0), 0x12345678U);
    uint8_t pattern[2048];
    fill_pattern(pattern, sizeof(pattern));
    EXPECT_EQ(crc32_zlib(crc32_zlib(0, pattern, 1001), pattern + 1001, 1047), CRC32_PATTERN_CHECK);
}

AP_GTEST_MAIN()
