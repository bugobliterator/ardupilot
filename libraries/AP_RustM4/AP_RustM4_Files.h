#pragma once

/*
  The M4's files on the M7 (AP_RustM4_FileServer): the per-session table of
  AP_Filesystem descriptors behind the ids the M4 holds, and the check of
  every path the M4 sends. The M4 only reads module files: rust/<name>.apmod
  in ROMFS or in the writable module directory, as the in-process runtime
  discovers them. An id carries a per-slot serial, so the id of a closed
  file, or of an earlier M4 session, never reaches a descriptor.
  C++11, no ArduPilot dependencies (host unit tests).
 */

#include <fcntl.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace RustM4 {

static const uint8_t FILE_SLOTS = 8;            // open files per M4 session; FatFs has 16 for everything
static const uint16_t FILE_CHUNK = 512;         // bytes per file read
static const uint8_t MAX_FS_PATH = 128;         // a path from the M4, terminator included
// a module file name, terminator included (AP_Rust::Runtime's ModuleFile name)
static const uint8_t MODULE_NAME_MAX = 64;

// the only flags the M4 opens a file with
static const int MODULE_OPEN_FLAGS = O_RDONLY;

// true if the M4 may read path: "rust/<name>.apmod", "@ROMFS/rust/<name>.apmod" or
// "/APM/rust/<name>.apmod", with <name> not empty and without a separator ('/', or '\', which
// FatFs also takes), and the file name shorter than MODULE_NAME_MAX. Nothing else
inline bool module_path_allowed(const char *path)
{
    static const char *const directories[] = { "rust/", "@ROMFS/rust/", "/APM/rust/" };
    static const char suffix[] = ".apmod";
    const size_t suffix_length = sizeof(suffix) - 1;
    if (path == nullptr) {
        return false;
    }
    for (const char *directory : directories) {
        const size_t directory_length = strlen(directory);
        if (strncmp(path, directory, directory_length) != 0) {
            continue;
        }
        const char *name = path + directory_length;
        const size_t length = strnlen(name, MODULE_NAME_MAX);
        if (length >= MODULE_NAME_MAX || length <= suffix_length ||
            strcmp(name + length - suffix_length, suffix) != 0) {
            return false;
        }
        return strchr(name, '/') == nullptr && strchr(name, '\\') == nullptr;
    }
    return false;
}

class FileTable {
public:
    FileTable()
    {
        for (Slot &slot : _slots) {
            slot.fd = -1;
            slot.serial = 1;
        }
    }

    bool full() const { return open_count() >= FILE_SLOTS; }

    uint8_t open_count() const
    {
        uint8_t count = 0;
        for (const Slot &slot : _slots) {
            count += slot.fd >= 0 ? 1 : 0;
        }
        return count;
    }

    // the id of a newly opened descriptor; -1 if the table is full or fd is no descriptor
    int32_t insert(int fd)
    {
        if (fd < 0) {
            return -1;
        }
        for (uint8_t i = 0; i < FILE_SLOTS; i++) {
            if (_slots[i].fd < 0) {
                _slots[i].fd = fd;
                return (int32_t(_slots[i].serial) << 8) | i;
            }
        }
        return -1;
    }

    // the descriptor of id; -1 unless id names a file open in this table now
    int lookup(int32_t id) const
    {
        const Slot *slot = slot_of(id);
        return slot != nullptr ? slot->fd : -1;
    }

    // forgets id and returns its descriptor for closing; -1 if id is not open
    int remove(int32_t id)
    {
        Slot *slot = const_cast<Slot *>(slot_of(id));
        if (slot == nullptr) {
            return -1;
        }
        const int fd = slot->fd;
        retire(*slot);
        return fd;
    }

    // calls close_fd(fd) for every open file and forgets them all: their ids go stale
    template <typename CloseFd> void close_all(CloseFd close_fd)
    {
        for (Slot &slot : _slots) {
            if (slot.fd >= 0) {
                close_fd(slot.fd);
                retire(slot);
            }
        }
    }

private:
    struct Slot {
        int fd;
        uint16_t serial;   // never 0: an id is at least 256
    };

    const Slot *slot_of(int32_t id) const
    {
        if (id < 256) {
            return nullptr;
        }
        const uint32_t index = uint32_t(id) & 0xFFU;
        const uint32_t serial = uint32_t(id) >> 8;
        if (index >= FILE_SLOTS || serial > UINT16_MAX) {
            return nullptr;
        }
        const Slot &slot = _slots[index];
        return (slot.fd >= 0 && slot.serial == serial) ? &slot : nullptr;
    }

    static void retire(Slot &slot)
    {
        slot.fd = -1;
        slot.serial = slot.serial == UINT16_MAX ? 1 : uint16_t(slot.serial + 1);
    }

    Slot _slots[FILE_SLOTS];
};

}  // namespace RustM4
