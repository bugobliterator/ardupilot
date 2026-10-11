#pragma once

/*
  M7 objects the M4 holds (ap_object userdata: RC_Channel, rangefinder and
  proximity backends, ...) as {type, index, generation}. The generation is the
  M4 session: reset() at every M4 start or restart makes every older handle
  stale. Entries are deduplicated by pointer, and every type served here
  points at an object the M7 keeps for its whole run, so nothing is released
  in between (release on garbage collection comes with the script-created
  objects of IO and Manual Bindings). Host thread only.
  C++11, no ArduPilot dependencies (host unit tests).
 */

#include <stdint.h>

namespace ScriptingM4 {

class HandleTable {
public:
    static const uint8_t CAPACITY = 64;

    void reset(uint32_t generation)
    {
        _count = 0;
        _generation = generation;
    }
    uint32_t generation() const { return _generation; }
    uint8_t count() const { return _count; }

    // the handle index of object, the existing one if it was handed out before; -1 if
    // object is null, type is 0 or the table is full
    int16_t insert(uint8_t type, void *object)
    {
        if (object == nullptr || type == 0) {
            return -1;
        }
        for (uint8_t i = 0; i < _count; i++) {
            if (_entries[i].object == object && _entries[i].type == type) {
                return i;
            }
        }
        if (_count >= CAPACITY) {
            return -1;
        }
        _entries[_count].object = object;
        _entries[_count].type = type;
        return _count++;
    }

    // the object of a handle of this session, nullptr unless type, index and generation match
    void *lookup(uint8_t type, uint16_t index, uint32_t generation) const
    {
        if (generation != _generation || index >= _count || _entries[index].type != type) {
            return nullptr;
        }
        return _entries[index].object;
    }

private:
    struct Entry {
        void *object;
        uint8_t type;
    };
    Entry _entries[CAPACITY] {};
    uint8_t _count = 0;
    uint32_t _generation = 0;
};

}  // namespace ScriptingM4
