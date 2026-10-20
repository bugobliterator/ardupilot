#pragma once

/*
  Where a module's buffers may lie, checked by the privileged SVC handlers
  before they copy through a module pointer: inputs in RX, RW, heap or stack,
  outputs in RW, heap or stack. A zero-length buffer is accepted at any
  address, so a callee that reads or writes a fixed size regardless of the
  length must have its exact sizes checked first.
 */

#include <stdint.h>

namespace ModuleM4 {

struct Range {
    uintptr_t base;
    uint32_t size;
    bool contains(uintptr_t address, uint32_t length) const
    {
        return address >= base && address - base <= size && length <= size - (address - base);
    }
};

struct ModuleRanges {
    Range rx;
    Range rw;
    Range heap;
    Range stack;

    bool input_ok(uintptr_t address, uint32_t length) const
    {
        return length == 0 || rx.contains(address, length) || output_ok(address, length);
    }
    bool output_ok(uintptr_t address, uint32_t length) const
    {
        return length == 0 || rw.contains(address, length) || heap.contains(address, length) ||
               stack.contains(address, length);
    }
};

// only for buffers that passed the range checks: their ends do not wrap
inline bool buffers_overlap(uintptr_t a, uint32_t a_length, uintptr_t b, uint32_t b_length)
{
    return a_length != 0 && b_length != 0 && a < b + b_length && b < a + a_length;
}

}  // namespace ModuleM4
