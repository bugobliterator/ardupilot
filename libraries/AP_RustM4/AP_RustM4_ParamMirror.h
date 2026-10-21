#pragma once

/*
  The M4's copy of its modules' Parameter values: written by the service thread from the M7's
  ParameterChanged frames, read by the module thread inside its binding call, both under the
  service lock. Parallel arrays sorted by (module, token): a binary search per get, O(n) inserts
  and removals at INIT and module end only. No ArduPilot dependencies.
 */

#include <stdint.h>
#include <string.h>

namespace RustM4 {

class ParamMirror {
public:
    // an enumerator: gtest binds it by reference, and gnu++11 has no inline variables
    enum : uint16_t { CAPACITY = 224 };   // AP_RustM4_Server::PARAMETER_HANDLES_MAX
    static const uint8_t FLAG_BOUND = 1;
    static const uint8_t FLAG_PENDING = 2;   // a writer of this handle is under way
    static const uint8_t FLAG_STALE = 4;     // forwarded until the next ParameterChanged for it

    void clear()
    {
        _count = 0;
        _overflow = false;
    }
    uint16_t count() const { return _count; }
    bool overflowed() const { return _overflow; }

    // a ParameterChanged entry: clears PENDING and STALE
    void upsert(uint8_t module, uintptr_t token, float value, bool bound)
    {
        uint16_t index;
        if (!find(module, token, index)) {
            if (_count == CAPACITY) {
                _overflow = true;   // a desync: misses are forwarded, answers stay exact
                return;
            }
            const uint16_t tail = uint16_t(_count - index);
            memmove(&_token[index + 1], &_token[index], tail * sizeof(_token[0]));
            memmove(&_value[index + 1], &_value[index], tail * sizeof(_value[0]));
            memmove(&_module[index + 1], &_module[index], tail * sizeof(_module[0]));
            memmove(&_flags[index + 1], &_flags[index], tail * sizeof(_flags[0]));
            _token[index] = token;
            _module[index] = module;
            _count++;
        }
        _value[index] = value;
        _flags[index] = bound ? FLAG_BOUND : 0;
    }

    // a forwarded Parameter.get answered OK: a STALE (not PENDING) entry takes the M7's answer and is
    // fresh again; an absent or fresh entry is left as it is (never inserted)
    bool refresh(uint8_t module, uintptr_t token, float value, bool bound)
    {
        uint16_t index;
        if (!find(module, token, index) || (_flags[index] & (FLAG_PENDING | FLAG_STALE)) != FLAG_STALE) {
            return false;
        }
        _value[index] = value;
        _flags[index] = bound ? FLAG_BOUND : 0;
        return true;
    }

    // the PR's Parameter.get output, [bound, value as f32 LE], for a fresh entry; nothing written on a miss
    bool get(uint8_t module, uintptr_t token, uint8_t *out5) const
    {
        uint16_t index;
        if (!find(module, token, index) || (_flags[index] & (FLAG_PENDING | FLAG_STALE)) != 0) {
            return false;
        }
        memset(out5, 0, 5);
        if ((_flags[index] & FLAG_BOUND) != 0) {
            out5[0] = 1;
            memcpy(out5 + 1, &_value[index], sizeof(float));
        }
        return true;
    }

    void mark_pending(uint8_t module, uintptr_t token)
    {
        uint16_t index;
        if (find(module, token, index)) {
            _flags[index] |= FLAG_PENDING;
        }
    }

    // the writer's reply arrived: still PENDING means its frame did not
    void settle(uint8_t module, uintptr_t token)
    {
        uint16_t index;
        if (find(module, token, index) && (_flags[index] & FLAG_PENDING) != 0) {
            _flags[index] = uint8_t((_flags[index] & ~FLAG_PENDING) | FLAG_STALE);
        }
    }

    void mark_module_stale(uint8_t module)
    {
        for (uint16_t i = 0; i < _count; i++) {
            if (_module[i] == module) {
                _flags[i] |= FLAG_STALE;
            }
        }
    }

    void remove_module(uint8_t module)
    {
        uint16_t kept = 0;
        for (uint16_t i = 0; i < _count; i++) {
            if (_module[i] == module) {
                continue;
            }
            _token[kept] = _token[i];
            _value[kept] = _value[i];
            _module[kept] = _module[i];
            _flags[kept] = _flags[i];
            kept++;
        }
        _count = kept;
    }

private:
    // the index of (module, token), or where it would be inserted
    bool find(uint8_t module, uintptr_t token, uint16_t &index) const
    {
        uint16_t low = 0, high = _count;
        while (low < high) {
            const uint16_t mid = uint16_t((low + high) / 2U);
            if (_module[mid] < module || (_module[mid] == module && _token[mid] < token)) {
                low = uint16_t(mid + 1U);
            } else {
                high = mid;
            }
        }
        index = low;
        return low < _count && _module[low] == module && _token[low] == token;
    }

    uintptr_t _token[CAPACITY];
    float _value[CAPACITY];
    uint8_t _module[CAPACITY];
    uint8_t _flags[CAPACITY];
    uint16_t _count;
    bool _overflow;
};

}  // namespace RustM4
