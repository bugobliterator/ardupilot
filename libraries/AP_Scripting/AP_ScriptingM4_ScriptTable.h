#pragma once

/*
  The scripts the M7 serves to the M4 in one session, and the checksums of
  the scripts the M4 reports loaded and running (SCR_LD_CHECKSUM,
  SCR_RUN_CHECKSUM). The CRCs are the M7's own, of the bytes it serves. The
  M4 is untrusted: indexes are checked and each script changes state at
  most once per session. C++11, no ArduPilot dependencies.
 */

#include <stdint.h>
#include <string.h>
#include "AP_ScriptingM4_Rpc.h"

namespace ScriptingM4 {

class ScriptTable {
public:
    void clear() { memset(this, 0, sizeof(*this)); }

    // false when the table is full or the path does not fit
    bool add(const char *path, uint32_t crc)
    {
        const size_t length = strlen(path);
        if (_count >= MAX_SCRIPTS || length >= MAX_SCRIPT_PATH) {
            return false;
        }
        memcpy(_path[_count], path, length + 1);
        _crc[_count] = crc;
        _state[_count] = LISTED;
        _count++;
        return true;
    }

    uint8_t count() const { return _count; }
    const char *path(int32_t index) const { return valid(index) ? _path[index] : nullptr; }

    // the M7 has served the script's bytes to the M4 up to the end of the file
    bool mark_served(int32_t index)
    {
        if (!valid(index) || _state[index] != LISTED) {
            return false;
        }
        _state[index] = SERVED;
        return true;
    }

    // in-process rules: a loaded script adds its CRC to both checksums,
    // a stopped one takes it out of the running checksum; only a served
    // script can be loaded
    bool mark_loaded(int32_t index)
    {
        if (!valid(index) || _state[index] != SERVED) {
            return false;
        }
        _state[index] = RUNNING;
        _loaded ^= _crc[index];
        _running ^= _crc[index];
        return true;
    }

    bool mark_stopped(int32_t index)
    {
        if (!valid(index) || _state[index] != RUNNING) {
            return false;
        }
        _state[index] = STOPPED;
        _running ^= _crc[index];
        return true;
    }

    uint32_t loaded_checksum() const { return _loaded; }
    uint32_t running_checksum() const { return _running; }

private:
    static const uint8_t LISTED = 1;
    static const uint8_t SERVED = 2;
    static const uint8_t RUNNING = 3;
    static const uint8_t STOPPED = 4;

    bool valid(int32_t index) const { return index >= 0 && index < int32_t(_count); }

    char _path[MAX_SCRIPTS][MAX_SCRIPT_PATH];
    uint32_t _crc[MAX_SCRIPTS];
    uint8_t _state[MAX_SCRIPTS];
    uint8_t _count;
    uint32_t _loaded;
    uint32_t _running;
};

}  // namespace ScriptingM4
