#pragma once

/*
  MPU region 7 for a module's code: the smallest power-of-two window that
  holds it, ending at `top` (the shared window's base), with the
  subregions wholly below the code disabled so that those addresses fall
  back to region 1 (RW, execute-never) and can hold RW or heap.
 */

#include <stdint.h>

namespace RustM4 {

struct CodeWindow {
    uint32_t base;               // aligned to size
    uint32_t size;               // power of two, CODE_WINDOW_MIN to CODE_WINDOW_MAX
    uint32_t rx_base;            // where RX is loaded
    uint8_t subregion_disable;   // RASR SRD
    uint8_t size_field;          // RASR SIZE: size = 2^(size_field + 1)
};

// subregions need regions of 256 B or more (ARMv7-M)
static const uint32_t CODE_WINDOW_MIN = 256U;
// within SRAM2 below the window
static const uint32_t CODE_WINDOW_MAX = 128U * 1024U;

inline uint32_t round_up_pow2(uint32_t value, uint32_t alignment)
{
    return (value + alignment - 1U) & ~(alignment - 1U);
}

inline bool code_window(uint32_t top, uint32_t rx_size, uint32_t rx_alignment, CodeWindow &window)
{
    if (rx_size == 0 || rx_size > CODE_WINDOW_MAX || rx_alignment == 0 ||
        (rx_alignment & (rx_alignment - 1U)) != 0 || top % CODE_WINDOW_MAX != 0) {
        return false;
    }
    for (uint32_t size = CODE_WINDOW_MIN; size <= CODE_WINDOW_MAX; size <<= 1) {
        if (size < rx_size) {
            continue;
        }
        const uint32_t subregion = size / 8U;
        const uint32_t step = rx_alignment > subregion ? rx_alignment : subregion;
        const uint32_t span = round_up_pow2(rx_size, step);
        if (span > size) {
            continue;
        }
        window.base = top - size;
        window.size = size;
        window.rx_base = top - span;
        window.subregion_disable = uint8_t((1U << ((window.rx_base - window.base) / subregion)) - 1U);
        uint8_t field = 0;
        while ((2U << field) < size) {
            field++;
        }
        window.size_field = field;
        return true;
    }
    return false;
}

}  // namespace RustM4
