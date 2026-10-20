#include <AP_gtest.h>
#include <string.h>
#include <AP_Math/crc.h>
#include <AP_ModuleM4/AP_ModuleM4_Crc32.h>

using namespace ModuleM4;

// the SITL M4 process's platform feed, as m4/sitl/crc_port.cpp
void ModuleM4::crc32_feed(Crc32 &crc, const uint8_t *data, uint32_t length)
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

TEST(ModuleM4Crc32, CheckValueIsZlibs)
{
    EXPECT_EQ(whole(reinterpret_cast<const uint8_t *>("123456789"), 9), CRC32_CHECK);
    EXPECT_EQ(whole(nullptr, 0), 0U);
}

TEST(ModuleM4Crc32, PatternInPiecesEqualsWhole)
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

TEST(ModuleM4Crc32, StateMatchesTheCrc32SmallImageCppUses)
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
TEST(ModuleM4Crc32, SeedChainsAsZlib)
{
    const uint8_t *digits = reinterpret_cast<const uint8_t *>("123456789");
    EXPECT_EQ(crc32_zlib(0, digits, 9), CRC32_CHECK);
    EXPECT_EQ(crc32_zlib(0x12345678U, digits, 9), 0x01F4807BU);   // zlib.crc32(b"123456789", 0x12345678)
    EXPECT_EQ(crc32_zlib(0x12345678U, nullptr, 0), 0x12345678U);
    uint8_t pattern[2048];
    fill_pattern(pattern, sizeof(pattern));
    EXPECT_EQ(crc32_zlib(crc32_zlib(0, pattern, 1001), pattern + 1001, 1047), CRC32_PATTERN_CHECK);
}

/*
  The unit as m4/chibios/crc_port.cpp configures it (RM0399 CRC): polynomial 0x04C11DB7,
  non-reflected, input bit-reversed by byte, output bit-reversed; a 32-bit DR write is processed
  from its top byte. feed_locked()'s sequence runs on it: INIT = bit reverse of the state, words as
  __REV(word), the tail as byte writes, then a DR read.
 */
class CrcUnitModel {
public:
    void reset(uint32_t init) { _register = init; }
    void write_word(uint32_t value)
    {
        for (int shift = 24; shift >= 0; shift -= 8) {
            process_byte(uint8_t(value >> shift));
        }
    }
    void write_byte(uint8_t value) { process_byte(value); }
    uint32_t read() const { return reverse_bits(_register); }

    static uint32_t reverse_bits(uint32_t value)
    {
        uint32_t result = 0;
        for (uint8_t bit = 0; bit < 32; bit++) {
            result = (result << 1) | ((value >> bit) & 1U);
        }
        return result;
    }

private:
    void process_byte(uint8_t value)
    {
        const uint8_t reversed = uint8_t(reverse_bits(value) >> 24);   // REV_IN by byte
        _register ^= uint32_t(reversed) << 24;
        for (uint8_t bit = 0; bit < 8; bit++) {
            _register = (_register << 1) ^ (0x04C11DB7U & (0U - (_register >> 31)));
        }
    }

    uint32_t _register;
};

static uint32_t unit_feed(uint32_t state, const uint8_t *data, uint32_t length)
{
    CrcUnitModel unit;
    unit.reset(CrcUnitModel::reverse_bits(state));
    uint32_t i = 0;
    for (; i + 4U <= length; i += 4U) {
        uint32_t word;
        memcpy(&word, data + i, sizeof(word));   // little-endian: data[i] is the low byte
        unit.write_word(__builtin_bswap32(word));
    }
    for (; i < length; i++) {
        unit.write_byte(data[i]);
    }
    return unit.read();
}

static uint32_t unit_zlib(uint32_t seed, const uint8_t *data, uint32_t length)
{
    return unit_feed(seed ^ 0xFFFFFFFFU, data, length) ^ 0xFFFFFFFFU;
}

// zlib.crc32 of the first n bytes of the pattern (i * 7 + 3) & 0xFF, n = 0 to 17
static const uint32_t ZLIB_PREFIX[18] = {
    0x00000000U, 0x4B0BBE37U, 0x8A21A822U, 0x6D58AF33U, 0x7ED34E62U, 0xFCC83142U,
    0x982182FDU, 0x54491CDBU, 0xE2E35978U, 0x3D351CFEU, 0x10E82536U, 0x12A86B05U,
    0xC911E29DU, 0xA97AD5C1U, 0xB21B1B8DU, 0x7C619EDCU, 0x191F3D9FU, 0x7BA75EE3U,
};

// every length across the word and tail paths, and every split point of a two-call chain
TEST(ModuleM4Crc32, LengthsAndSplitsEqualZlibOnBothPaths)
{
    uint8_t pattern[17];
    fill_pattern(pattern, sizeof(pattern));
    for (uint32_t length = 0; length <= 17U; length++) {
        EXPECT_EQ(crc32_zlib(0, pattern, length), ZLIB_PREFIX[length]) << length;
        EXPECT_EQ(unit_zlib(0, pattern, length), ZLIB_PREFIX[length]) << length;
        for (uint32_t split = 0; split <= length; split++) {
            EXPECT_EQ(crc32_zlib(crc32_zlib(0, pattern, split), pattern + split, length - split),
                      ZLIB_PREFIX[length]) << length << " split " << split;
            EXPECT_EQ(unit_zlib(unit_zlib(0, pattern, split), pattern + split, length - split),
                      ZLIB_PREFIX[length]) << length << " split " << split;
        }
    }
}

TEST(ModuleM4Crc32, UnitModelMatchesTheSelfCheckValues)
{
    EXPECT_EQ(unit_zlib(0, reinterpret_cast<const uint8_t *>("123456789"), 9), CRC32_CHECK);
    EXPECT_EQ(unit_zlib(0x12345678U, reinterpret_cast<const uint8_t *>("123456789"), 9), 0x01F4807BU);
    uint8_t pattern[2048];
    fill_pattern(pattern, sizeof(pattern));
    EXPECT_EQ(unit_zlib(unit_zlib(0, pattern, 1001), pattern + 1001, 1047), CRC32_PATTERN_CHECK);
}

// the defect the port avoids: the word path without __REV takes each word's bytes last first
TEST(ModuleM4Crc32, UnitModelWithoutTheByteSwapDisagrees)
{
    CrcUnitModel unit;
    unit.reset(0xFFFFFFFFU);
    const uint8_t *digits = reinterpret_cast<const uint8_t *>("123456789");
    for (uint32_t i = 0; i < 8U; i += 4U) {
        uint32_t word;
        memcpy(&word, digits + i, sizeof(word));
        unit.write_word(word);
    }
    unit.write_byte(digits[8]);
    EXPECT_NE(unit.read() ^ 0xFFFFFFFFU, CRC32_CHECK);
}

AP_GTEST_MAIN()
