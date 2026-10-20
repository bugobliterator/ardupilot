/*
  CRC-32 on the STM32H757 CRC unit (AHB4, 0x58024C00): polynomial 0x04C11DB7
  (reset value), 32-bit, input bit-reversed by byte, output bit-reversed. The
  unit runs the non-reflected algorithm, so its register holds the bit reverse
  of zlib's reflected state: INIT takes __RBIT(state), which lets a feed
  continue any earlier one, and a zlib seed s starts from state ~s
  (INIT = __RBIT(~s); s = 0 gives 0xFFFFFFFF).

  The control register is written once per feed, with RESET, and never during
  it: whole words go in as 32-bit DR writes, the tail as 8-bit DR writes, both
  under the same byte reversal. A 32-bit write is processed from its top byte,
  so each word is written as __REV(word) to put its first byte there (one REV
  per word).

  This core enables the unit's clock in its own RCC_C2 set (AHB4ENR.CRCEN, at
  boot in main.cpp); the M7 never enables or uses the unit. On the M4 it is shared by the service
  thread (ring reads, the loader's checksum) and the module thread (ring
  writes inside its SVC handlers): every chunk runs with the kernel locked and
  reloads INIT from the caller's own state, so calls from the two threads
  never see each other's register contents. Callers are privileged; the unit
  lies outside every MPU region, reached through the default map (PRIVDEFENA)
  for one bounded chunk, as TIM5 in module_port.cpp.
 */
#include "ch.h"

#include <string.h>
#include <AP_Math/crc.h>
#include <CubePilotFW/PAL.h>
#include <AP_ModuleM4_Crc32.h>

using namespace ModuleM4;

static const uint32_t CR_RESET = 1U << 0;
static const uint32_t CR_REV_IN_BYTE = 1U << 5;   // POLYSIZE (4:3) 0: 32-bit
static const uint32_t CR_REV_OUT = 1U << 7;
// bounds the time with the kernel locked: 512 words, a few microseconds
static const uint32_t MAX_LOCKED_FEED = 2048U;

static uint32_t feed_locked(uint32_t state, const uint8_t *data, uint32_t length)
{
    // thread or ISR context: the ring may be read from an HSEM callback
    const syssts_t status = chSysGetStatusAndLockX();
    const uint32_t control = MPU->CTRL;
    MPU->CTRL = control | MPU_CTRL_PRIVDEFENA_Msk;
    __DSB();
    __ISB();
    CRC->INIT = __RBIT(state);
    CRC->CR = CR_REV_IN_BYTE | CR_REV_OUT | CR_RESET;
    uint32_t i = 0;
    for (; i + 4U <= length; i += 4U) {
        uint32_t word;
        memcpy(&word, data + i, sizeof(word));
        CRC->DR = __REV(word);
    }
    for (; i < length; i++) {
        *reinterpret_cast<volatile uint8_t *>(&CRC->DR) = data[i];
    }
    const uint32_t result = CRC->DR;
    MPU->CTRL = control;
    __DSB();
    __ISB();
    chSysRestoreStatusX(status);
    return result;
}

void ModuleM4::crc32_feed(Crc32 &crc, const uint8_t *data, uint32_t length)
{
    while (length != 0) {
        const uint32_t chunk = length < MAX_LOCKED_FEED ? length : MAX_LOCKED_FEED;
        crc.state = feed_locked(crc.state, data, chunk);
        data += chunk;
        length -= chunk;
    }
}

// what the PR's Image.cpp links against: the register update without initial value or final xor
uint32_t crc32_small(uint32_t crc, const uint8_t *buf, uint32_t size)
{
    Crc32 state { crc };
    crc32_feed(state, buf, size);
    return state.state;
}

// CubeFramework's ring CRC (PAL.h, zlib contract, built with CF_PAL_CRC32_EXTERNAL); frames are
// at most 2048 B, so the length fits the feed's uint32_t
extern "C" uint32_t cubefw_pal_crc32(uint32_t seed, const void *data, size_t len)
{
    return crc32_zlib(seed, static_cast<const uint8_t *>(data), uint32_t(len));
}
