#pragma once

/*
  CRC-32 as zlib.crc32 (IEEE 802.3, reflected polynomial 0xEDB88320, initial
  value and final xor 0xFFFFFFFF). The M4 image feeds the STM32H757 CRC unit
  (m4/chibios/crc_port.cpp) and links no software CRC; the SITL M4 process
  and host tests use crc32_software_update().
 */

#include <stdint.h>

namespace RustM4 {

struct Crc32 {
    uint32_t state;   // the reflected CRC register, before the final xor
};

static const uint32_t CRC32_CHECK = 0xCBF43926U;          // of "123456789"
static const uint32_t CRC32_PATTERN_CHECK = 0xB9D45861U;  // of 2048 bytes (i * 7 + 3) & 0xFF

inline void crc32_begin(Crc32 &crc)
{
    crc.state = 0xFFFFFFFFU;
}

// the platform's: the hardware unit on the M4 image, crc32_software_update() elsewhere
void crc32_feed(Crc32 &crc, const uint8_t *data, uint32_t length);

inline uint32_t crc32_finish(const Crc32 &crc)
{
    return crc.state ^ 0xFFFFFFFFU;
}

// zlib's crc32(seed, data, length): seed 0 starts a CRC, a result is the seed of the next call
inline uint32_t crc32_zlib(uint32_t seed, const uint8_t *data, uint32_t length)
{
    Crc32 crc { seed ^ 0xFFFFFFFFU };
    crc32_feed(crc, data, length);
    return crc32_finish(crc);
}

// bitwise, no table; never linked into the M4 image (Makefile check software_crc_absent.ok)
inline uint32_t crc32_software_update(uint32_t state, const uint8_t *data, uint32_t length)
{
    while (length--) {
        state ^= *data++;
        for (uint8_t bit = 0; bit < 8; bit++) {
            state = (state >> 1) ^ (0xEDB88320U & (0U - (state & 1U)));
        }
    }
    return state;
}

struct CrcSelfCheck {
    uint32_t short_check;     // of "123456789": CRC32_CHECK when the unit is right
    uint32_t pattern_check;   // of the 2048 B pattern in pieces: CRC32_PATTERN_CHECK
};

}  // namespace RustM4
