#pragma once

/*
  The M7 objects M4 scripts own (spec §6): Parameter, serial ports, CRSF
  menus and parameters, DroneCAN handles. In-process each lives in its Lua
  userdata; here it lives in a slot of this pool, the M4 userdata holds an
  owned handle to it (HandleTable), and the object is destroyed when the M4
  collects the userdata (m4:release), when the call that made it fails, or
  when the M4 session ends. The scripting host thread or its slow worker, one
  at a time; ReleaseQueue is the M4's end.
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
    void *allocate(Destroy destroy_function)
    {
        if (destroy_function == nullptr) {
            return nullptr;
        }
        for (uint8_t i = 0; i < SLOTS; i++) {
            if (_destroy[i] == nullptr) {
                _destroy[i] = destroy_function;
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

    // whether address lies in any slot, live or free: no kept or session handle may name such memory
    bool contains(const void *address) const
    {
        const uintptr_t value = reinterpret_cast<uintptr_t>(address);
        const uintptr_t first = reinterpret_cast<uintptr_t>(&_slots[0]);
        return value >= first && value < first + sizeof(_slots);
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

// the M4's side of m4:release (m4/m4_bindings.cpp): the owned handles its collector found, sent
// before the next remote call and after each script run's collection. A finalizer can run while
// the RPC client holds a reply its caller still reads, so __gc only queues. An M4 userdata of an
// owned type holds one of the pool's slots until its release is sent, so at most SLOTS wait
class ReleaseQueue {
public:
    struct Entry {
        uint8_t type_id;
        uint16_t index;
        uint32_t generation;
    };

    // queues the handle of a collected userdata and sets its generation to 0, which no live
    // handle has: a second __gc of the same userdata queues nothing, and a later use of it is
    // a stale handle on the M7. False if nothing was queued; a full queue counts the drop, and
    // the M7 frees that object at the session end
    bool push(uint8_t type_id, uint16_t index, uint32_t &generation)
    {
        if (generation == 0) {
            return false;
        }
        const uint32_t queued = generation;
        generation = 0;
        if (_count >= ObjectPool::SLOTS) {
            _dropped++;
            return false;
        }
        Entry &entry = _entries[_count++];
        entry.type_id = type_id;
        entry.index = index;
        entry.generation = queued;
        return true;
    }

    // send(entries, count) sends at most MAX_VALUES handles in one m4:release and returns false
    // if it could not: that batch and the rest stay queued for the next flush
    template <typename Send>
    void flush(Send send)
    {
        uint8_t sent = 0;
        while (sent < _count) {
            const uint8_t batch = uint8_t(_count - sent) < MAX_VALUES ? uint8_t(_count - sent) : MAX_VALUES;
            if (!send(&_entries[sent], batch)) {
                break;
            }
            sent = uint8_t(sent + batch);
        }
        memmove(&_entries[0], &_entries[sent], (_count - sent) * sizeof(Entry));
        _count = uint8_t(_count - sent);
    }

    uint8_t count() const { return _count; }
    uint32_t dropped() const { return _dropped; }

private:
    Entry _entries[ObjectPool::SLOTS] {};
    uint8_t _count = 0;
    uint32_t _dropped = 0;
};

// objects the HAL hands out but never takes back (AnalogIn::channel()): those earlier M4 sessions
// were handed are handed out again, in order, before make() is asked for a new one. Up to N are
// kept; past that a new object is still handed out, as in-process. The host thread or its slow
// worker, one at a time
template <typename T, uint8_t N> class SessionRecycler {
public:
    template <typename Make> T *take(Make make)
    {
        if (_handed < _count) {
            return _items[_handed++];
        }
        T *item = make();
        if (item != nullptr && _count < N) {
            _items[_count++] = item;
            _handed = _count;
        }
        return item;
    }
    // as take(), but at most N in a session: past them nullptr, before make() is asked, so at
    // most N objects are ever made (the scripting CAN buffers, which their sensor never frees)
    template <typename Make> T *take_at_most(Make make)
    {
        if (_handed >= N) {
            return nullptr;
        }
        return take(make);
    }
    // a new M4 session: every kept object can be handed out again
    void new_session() { _handed = 0; }
    uint8_t count() const { return _count; }

private:
    T *_items[N] {};
    uint8_t _count = 0;
    uint8_t _handed = 0;
};

}  // namespace ScriptingM4
