#pragma once

/*
  Placement of one module in the M4 module area by the staged loader
  (Tools/rust_modules/m4_fit.py computes the same): the file is staged at the
  bottom, RX top-aligned in region 7 below the shared window, RW directly
  below RX. Once staging is released, the stack (above a 32 B no-access
  guard) and the heap are carved from the bottom.
  The layout's fields are only valid when the result is LoadFit::Ok.
 */

#include <stdint.h>
#include "AP_RustM4_CodeWindow.h"

namespace RustM4 {

struct LoadRequest {
    uint32_t file_size;
    uint32_t rx_size;
    uint32_t rx_alignment;
    uint32_t rw_memory_size;   // RW and BSS
    uint32_t rw_alignment;
    uint32_t stack_size;       // RUST_STACK_SIZE
    uint32_t heap_size;        // RUST_HEAP_SIZE
};

enum class LoadFit : uint8_t { Ok, CodeTooLarge, NoRoomToStage, NoRoomToRun };

struct LoadLayout {
    CodeWindow window;
    uint32_t staging_base;
    uint32_t rw_base;
    uint32_t guard_base;
    uint32_t stack_base;
    uint32_t stack_top;
    uint32_t heap_base;
    uint32_t heap_end;
    uint32_t load_peak;   // staged file + RW + RX window
    uint32_t run_need;    // guard + stack + heap + RW + RX window
};

static const uint32_t MODULE_STACK_GUARD = 32U;
// room for the launch record and the host callback margin (AP_Rust Module.cpp)
static const uint32_t MODULE_STACK_MIN = 4096U;
// one allocator granule
static const uint32_t MODULE_HEAP_MIN = 16U;
// the .apmod header supplies it; the same limit as the module heap's alignment
static const uint32_t MODULE_RW_ALIGNMENT_MAX = 4096U;

inline LoadFit load_layout(uint32_t area_base, uint32_t area_size, const LoadRequest &r, LoadLayout &l)
{
    const uint32_t top = area_base + area_size;
    if (!code_window(top, r.rx_size, r.rx_alignment, l.window) || l.window.rx_base < area_base) {
        return LoadFit::CodeTooLarge;
    }
    if (r.rw_alignment == 0 || r.rw_alignment > MODULE_RW_ALIGNMENT_MAX ||
        (r.rw_alignment & (r.rw_alignment - 1U)) != 0 || r.rw_memory_size > l.window.rx_base - area_base) {
        return LoadFit::NoRoomToStage;
    }
    l.rw_base = (l.window.rx_base - r.rw_memory_size) & ~(r.rw_alignment - 1U);
    if (l.rw_base < area_base || r.file_size > l.rw_base - area_base) {
        return LoadFit::NoRoomToStage;
    }
    if (r.stack_size < MODULE_STACK_MIN || r.stack_size > area_size ||
        r.heap_size < MODULE_HEAP_MIN || r.heap_size > area_size) {
        return LoadFit::NoRoomToRun;
    }
    l.staging_base = area_base;
    l.guard_base = area_base;
    l.stack_base = area_base + MODULE_STACK_GUARD;
    l.stack_top = l.stack_base + round_up_pow2(r.stack_size, 32U);
    l.heap_base = round_up_pow2(l.stack_top, 16U);
    l.heap_end = l.heap_base + r.heap_size;
    l.load_peak = r.file_size + (top - l.rw_base);
    l.run_need = (l.heap_end - area_base) + (top - l.rw_base);
    if (l.heap_end > l.rw_base) {
        return LoadFit::NoRoomToRun;
    }
    return LoadFit::Ok;
}

// the largest RUST_CODE_SIZE (a module's RX and RW, a multiple of 4096) the staged loader can
// place in an area of area_size at stack_size and heap_size (m4_fit.py): the file, at least as
// large as the image, is staged below it, so the image is at most half the area; and it shares the
// area with the guard, the stack and the heap. 0 when the stack and heap alone fill the area
inline uint32_t staged_code_size_cap(uint32_t area_size, uint32_t stack_size, uint32_t heap_size)
{
    if (stack_size > area_size || heap_size > area_size) {
        return 0;
    }
    const uint64_t below = uint64_t(round_up_pow2(MODULE_STACK_GUARD + round_up_pow2(stack_size, 32U), 16U)) +
                           heap_size;
    if (below >= area_size) {
        return 0;
    }
    uint32_t cap = area_size - uint32_t(below);
    if (cap > area_size / 2U) {
        cap = area_size / 2U;
    }
    return cap & ~4095U;
}

}  // namespace RustM4
