#pragma once

/*
  The M7 objects M4 scripts own (spec §6): Parameter, serial ports, CRSF
  menus and parameters, DroneCAN handles. In-process each lives in its Lua
  userdata; here it lives in a slot of this pool, the M4 userdata holds an
  owned handle to it (HandleTable), and the object is destroyed when the M4
  collects the userdata (m4:release), when the call that made it fails, or
  when the M4 session ends. Host thread only.
  C++11, no ArduPilot dependencies (host unit tests).
 */

#include <stdint.h>
#include <string.h>
#include "AP_ScriptingM4_Handles.h"
#include "AP_ScriptingM4_Rpc.h"

namespace ScriptingM4 {

// the reply text of a call that found the object pool or the handle table full; the M4
// collects its garbage, sends its releases and tries such a call once more
static const char TOO_MANY_OBJECTS[] = "M4: too many objects";

class ObjectPool {
public:
    static const uint8_t SLOTS = 64;
    static const uint8_t SLOT_SIZE = 32;
    static const uint8_t SLOT_ALIGN = 8;
    typedef void (*Destroy)(void *object);

    // a zeroed slot, whose object destroy() ends; nullptr if every slot is taken
    void *allocate(Destroy destroy)
    {
        if (destroy == nullptr) {
            return nullptr;
        }
        for (uint8_t i = 0; i < SLOTS; i++) {
            if (_destroy[i] == nullptr) {
                _destroy[i] = destroy;
                memset(_slots[i].bytes, 0, SLOT_SIZE);
                return _slots[i].bytes;
            }
        }
        return nullptr;
    }

    // ends the object of a slot allocate() handed out and frees the slot; false for any other pointer
    bool destroy(void *object)
    {
        const int16_t i = slot_of(object);
        if (i < 0) {
            return false;
        }
        const Destroy destroy_function = _destroy[i];
        _destroy[i] = nullptr;   // first: the slot is free whatever the function does
        destroy_function(object);
        return true;
    }

    // every live object: the end of an M4 session
    void clear()
    {
        for (uint8_t i = 0; i < SLOTS; i++) {
            if (_destroy[i] != nullptr) {
                (void)destroy(_slots[i].bytes);
            }
        }
    }

    uint8_t used() const
    {
        uint8_t count = 0;
        for (const Destroy destroy_function : _destroy) {
            count += destroy_function != nullptr ? 1 : 0;
        }
        return count;
    }

private:
    struct Slot {
        alignas(SLOT_ALIGN) uint8_t bytes[SLOT_SIZE];
    };
    static_assert(sizeof(Slot) == SLOT_SIZE, "slots are contiguous");

    // the index of a live slot that starts at object, else -1
    int16_t slot_of(const void *object) const
    {
        const uintptr_t address = reinterpret_cast<uintptr_t>(object);
        const uintptr_t first = reinterpret_cast<uintptr_t>(&_slots[0]);
        if (address < first || address >= first + sizeof(_slots) || (address - first) % sizeof(Slot) != 0) {
            return -1;
        }
        const uint8_t i = uint8_t((address - first) / sizeof(Slot));
        return _destroy[i] != nullptr ? int16_t(i) : int16_t(-1);
    }

    Slot _slots[SLOTS];
    Destroy _destroy[SLOTS] {};   // nullptr: the slot is free
};

// the release of an owned type without one of its own: its destructor
template <typename T> void destroy_object(void *object)
{
    static_cast<T *>(object)->~T();
}

// m4:release(handle, ...): the owned objects the M4 collected are destroyed. Ok when every
// value named a live owned object of this session; any other value (stale, kept or session,
// another type, no handle) is skipped and makes the call Malformed, a counted protocol error
inline ReplyStatus release_objects(const RpcValue *values, uint8_t count, HandleTable &handles, ObjectPool &objects)
{
    bool all = count > 0;
    for (uint8_t i = 0; i < count; i++) {
        const RpcValue &value = values[i];
        void *object = value.tag == ValueTag::Handle ?
            handles.release_owned(value.type_id, value.handle_index, value.handle_generation) : nullptr;
        if (object == nullptr || !objects.destroy(object)) {
            all = false;
        }
    }
    return all ? ReplyStatus::Ok : ReplyStatus::Malformed;
}

}  // namespace ScriptingM4
