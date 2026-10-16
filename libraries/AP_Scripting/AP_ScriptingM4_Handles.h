#pragma once

/*
  M7 objects the M4 holds, as {type, index, generation}. The generation is
  session << 8 | serial: reset() at every M4 start or restart retires every
  entry, and retiring an entry bumps its serial, so a handle of an earlier
  session, or of a released object, never names an object again. Three
  classes of object share the table (spec §6):
  - kept: objects the M7 keeps for its whole run (RC_Channel, backends,
    analog sources); deduplicated by pointer, emptied at session reset;
  - session: objects a script makes that live until its session ends (I2C
    devices, PWM sources, CAN buffers, sockets); deduplicated, emptied at
    session reset; a socket's close() forgets its own entry;
  - owned: objects in the M7 object pool (AP_ScriptingM4_Objects.h), one
    entry per M4 userdata, retired when the M4 collects it (m4:release).
  The scripting host thread or its slow worker, one at a time. C++11, no
  ArduPilot dependencies (host unit tests).
 */

#include <stdint.h>

namespace ScriptingM4 {

class HandleTable {
public:
    static const uint8_t CAPACITY = 128;

    HandleTable()
    {
        for (Entry &entry : _entries) {
            entry.object = nullptr;
            entry.type = 0;
            entry.serial = 1;
            entry.owned = false;
        }
    }

    // a new M4 session: every entry is retired, even if the M4 repeats its session number
    void reset(uint32_t session)
    {
        for (Entry &entry : _entries) {
            retire(entry);
        }
        _session = session;
    }
    uint32_t session() const { return _session; }

    // live entries
    uint8_t count() const
    {
        uint8_t live = 0;
        for (const Entry &entry : _entries) {
            live += entry.object != nullptr ? 1 : 0;
        }
        return live;
    }

    // the generation a handle of entry index carries now
    uint32_t generation(uint16_t index) const
    {
        return index < CAPACITY ? ((_session << 8) | _entries[index].serial) : 0;
    }

    // a kept or session object: the entry it was handed out under before, else a new one;
    // -1 if object is null, type is 0 or the table is full
    int16_t insert(uint8_t type, void *object)
    {
        if (object == nullptr || type == 0) {
            return -1;
        }
        for (uint8_t i = 0; i < CAPACITY; i++) {
            const Entry &entry = _entries[i];
            if (entry.object == object && entry.type == type && !entry.owned) {
                return i;
            }
        }
        return add(type, object, false);
    }

    // an owned object: always a new entry, so releasing it never invalidates another userdata
    int16_t insert_owned(uint8_t type, void *object)
    {
        if (object == nullptr || type == 0) {
            return -1;
        }
        return add(type, object, true);
    }

    // the object of a live handle of this session; nullptr unless type, index and generation match
    void *lookup(uint8_t type, uint16_t index, uint32_t generation) const
    {
        const Entry *entry = live(type, index, generation);
        return entry != nullptr ? entry->object : nullptr;
    }

    // m4:release: the owned object of a live handle, whose entry is retired; nullptr for a stale
    // handle, a kept or session entry or another type, which the caller counts
    void *release_owned(uint8_t type, uint16_t index, uint32_t generation)
    {
        Entry *entry = const_cast<Entry *>(live(type, index, generation));
        if (entry == nullptr || !entry->owned) {
            return nullptr;
        }
        void *object = entry->object;
        retire(*entry);
        return object;
    }

    // the M7 deleted object itself (a closed socket), or a call that made it failed:
    // its entries are retired; false if none held it
    bool forget(const void *object)
    {
        bool found = false;
        for (Entry &entry : _entries) {
            if (object != nullptr && entry.object == object) {
                retire(entry);
                found = true;
            }
        }
        return found;
    }

private:
    struct Entry {
        void *object;     // nullptr: free
        uint8_t type;
        uint8_t serial;   // 1..255, bumped whenever the entry is retired
        bool owned;
    };

    int16_t add(uint8_t type, void *object, bool owned)
    {
        for (uint8_t i = 0; i < CAPACITY; i++) {
            Entry &entry = _entries[i];
            if (entry.object == nullptr) {
                entry.object = object;
                entry.type = type;
                entry.owned = owned;
                return i;
            }
        }
        return -1;
    }

    const Entry *live(uint8_t type, uint16_t index, uint32_t generation) const
    {
        if (type == 0 || index >= CAPACITY) {
            return nullptr;
        }
        const Entry &entry = _entries[index];
        if (entry.object == nullptr || entry.type != type || generation != this->generation(index)) {
            return nullptr;
        }
        return &entry;
    }

    static void retire(Entry &entry)
    {
        entry.object = nullptr;
        entry.type = 0;
        entry.owned = false;
        entry.serial = entry.serial == UINT8_MAX ? 1 : uint8_t(entry.serial + 1);
    }

    Entry _entries[CAPACITY];
    uint32_t _session = 0;
};

}  // namespace ScriptingM4
