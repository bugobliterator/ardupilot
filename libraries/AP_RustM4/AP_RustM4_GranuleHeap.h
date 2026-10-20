#pragma once

/*
  The module heap's allocator. It runs privileged on an arena the module can
  write, so none of its metadata lives in the arena: one "used" bit and one
  "block start" bit per 16-byte granule, in arrays the caller keeps in
  privileged RAM. First fit; alignment up to 4096 (the PR's limit). Freeing
  anything but a live block's first byte is refused, which makes the PR's
  "invalid deallocation" panic exact.
 */

#include <stddef.h>
#include <stdint.h>

namespace RustM4 {

class GranuleHeap {
public:
    static const uint32_t GRANULE = 16;
    static const uint32_t MAX_ALIGNMENT = 4096;
    enum class FreeResult : uint8_t { Freed, NotABlock };

    static uint32_t bitmap_words(uint32_t arena_size)
    {
        return (arena_size / GRANULE + 31U) / 32U;
    }

    bool init(uintptr_t base, uint32_t size, uint32_t *used, uint32_t *starts, uint32_t words)
    {
        _count = 0;
        if (base % GRANULE != 0 || size < GRANULE || used == nullptr || starts == nullptr ||
            words < bitmap_words(size)) {
            return false;
        }
        for (uint32_t i = 0; i < words; i++) {
            used[i] = 0;
            starts[i] = 0;
        }
        _base = base;
        _count = size / GRANULE;
        _used = used;
        _starts = starts;
        _in_use = 0;
        return true;
    }

    void *allocate(uint32_t size, uint32_t alignment)
    {
        if (size == 0 || alignment == 0 || alignment > MAX_ALIGNMENT || (alignment & (alignment - 1U)) != 0 ||
            _count == 0 || size > _count * GRANULE) {
            return nullptr;
        }
        const uint32_t need = (size + GRANULE - 1U) / GRANULE;
        const uintptr_t align = alignment < GRANULE ? GRANULE : alignment;
        uint32_t first = 0;
        while (first + need <= _count) {
            const uintptr_t address = _base + uintptr_t(first) * GRANULE;
            if (address % align != 0) {
                first = uint32_t((((address + align - 1U) & ~(align - 1U)) - _base) / GRANULE);
                continue;
            }
            uint32_t run = 0;
            while (run < need && !test(_used, first + run)) {
                run++;
            }
            if (run == need) {
                for (uint32_t i = 0; i < need; i++) {
                    set(_used, first + i);
                }
                set(_starts, first);
                _in_use += need;
                return reinterpret_cast<void *>(address);
            }
            first += run + 1;   // past the used granule that ended the run
        }
        return nullptr;
    }

    FreeResult free(const void *block)
    {
        if (block == nullptr) {
            return FreeResult::Freed;
        }
        const uintptr_t address = reinterpret_cast<uintptr_t>(block);
        if (_count == 0 || address < _base || (address - _base) % GRANULE != 0 ||
            (address - _base) / GRANULE >= _count) {
            return FreeResult::NotABlock;
        }
        uint32_t i = uint32_t((address - _base) / GRANULE);
        if (!test(_starts, i) || !test(_used, i)) {
            return FreeResult::NotABlock;
        }
        clear(_starts, i);
        do {
            clear(_used, i);
            _in_use--;
            i++;
        } while (i < _count && test(_used, i) && !test(_starts, i));
        return FreeResult::Freed;
    }

    uint32_t bytes_in_use() const
    {
        return _in_use * GRANULE;
    }
    uint32_t capacity() const
    {
        return _count * GRANULE;
    }

private:
    static bool test(const uint32_t *bits, uint32_t i)
    {
        return ((bits[i >> 5] >> (i & 31U)) & 1U) != 0;
    }
    static void set(uint32_t *bits, uint32_t i)
    {
        bits[i >> 5] |= 1U << (i & 31U);
    }
    static void clear(uint32_t *bits, uint32_t i)
    {
        bits[i >> 5] &= ~(1U << (i & 31U));
    }

    uintptr_t _base;
    uint32_t _count;
    uint32_t *_used;
    uint32_t *_starts;
    uint32_t _in_use;
};

}  // namespace RustM4
