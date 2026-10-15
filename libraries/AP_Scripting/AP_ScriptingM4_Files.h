#pragma once

/*
  The M4's files on the M7 (m4:fs_* system methods, AP_ScriptingM4_FileServer):
  the per-session table of AP_Filesystem descriptors behind the ids the M4
  holds, and the check of every path the M4 sends: the reach of in-process
  scripting, less a ".." above the filesystem root, writes on the "@"
  virtual filesystems (also as "/@...": AP_Filesystem skips one leading '/'
  before it selects the filesystem) and @SYS/threads.txt. An id carries a per-slot serial, so the id of a closed
  file, or of an earlier M4 session, never reaches a descriptor. The check is
  lexical; '\' counts as a separator, as FatFs takes it.
  C++11, no ArduPilot dependencies (host unit tests).
 */

#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace ScriptingM4 {

static const uint8_t FILE_SLOTS = 8;            // open files per M4 session; FatFs has 16 for everything
static const uint16_t FILE_CHUNK = 512;         // bytes per m4:fs_read and m4:fs_write
static const uint8_t MAX_FS_PATH = 128;         // a path from the M4, terminator included
static const uint16_t DIR_REPLY_BYTES = 768;    // names per m4:fs_dir reply, by length; a name is at most 255

enum class PathUse : uint8_t {
    List,    // dirlist
    Read,    // open for reading
    Write,   // open for writing, remove: never on an "@" filesystem
};

// AP_Filesystem_Sys's threads.txt: "@SYS/" (after the one leading '/' AP_Filesystem skips), then
// the exact name; one more '/' before it is refused too, though AP_Filesystem_Sys does not find
// that name. Generating the file resets every thread's ChibiOS statistics, which the M7 host's
// duty cap charges
inline bool resets_thread_statistics(const char *selector, size_t length)
{
    static const char prefix[] = "@SYS/";
    static const char name[] = "threads.txt";
    const size_t prefix_length = sizeof(prefix) - 1;
    const size_t name_length = sizeof(name) - 1;
    if (length < prefix_length || memcmp(selector, prefix, prefix_length) != 0) {
        return false;
    }
    const char *rest = selector + prefix_length;
    size_t rest_length = length - prefix_length;
    if (rest_length > 0 && rest[0] == '/') {
        rest++;
        rest_length--;
    }
    return rest_length == name_length && memcmp(rest, name, name_length) == 0;
}

// 0 if the M4 may use path this way, else the errno its call fails with: the reach of
// in-process scripting, less a ".." above the start of the path, writes on "@" filesystems
// and @SYS/threads.txt
inline int check_script_path(const char *path, size_t length, PathUse use)
{
    if (length == 0) {
        // opendir("") lists the filesystem root in-process (AP_Filesystem::opendir)
        return use == PathUse::List ? 0 : ENOENT;
    }
    if (length >= MAX_FS_PATH) {
        return ENAMETOOLONG;
    }
    // a NUL would cut the checked text short of what is opened
    if (memchr(path, 0, length) != nullptr) {
        return EINVAL;
    }
    // @SYS, @PARAM, @ROMFS, @MISSION, ...: AP_Filesystem selects them by a leading '@' after
    // skipping one leading '/', so "/@PARAM/x" is the @PARAM filesystem
    const char *selector = path[0] == '/' ? path + 1 : path;
    if (selector[0] == '@' && use == PathUse::Write) {
        return EROFS;
    }
    if (resets_thread_statistics(selector, length - size_t(selector - path))) {
        return EACCES;
    }
    // components split at '/' and at '\' (FatFs takes both); "." and empty ones count nothing
    int32_t depth = 0;
    size_t start = 0;
    for (size_t i = 0; i <= length; i++) {
        if (i < length && path[i] != '/' && path[i] != '\\') {
            continue;
        }
        const size_t n = i - start;
        if (n == 2 && path[start] == '.' && path[start + 1] == '.') {
            if (--depth < 0) {
                return EACCES;   // above the root, or the working directory a relative path starts from
            }
        } else if (n > 0 && !(n == 1 && path[start] == '.')) {
            depth++;
        }
        start = i + 1;
    }
    return 0;
}

// open() flags of an fopen() mode, as AP_Filesystem/posix_compat.cpp maps them (r+
// truncates there, so here too); -1 for any other mode
inline int fopen_mode_flags(const char *mode, size_t length)
{
    struct Mode {
        const char *text;
        int flags;
    };
    static const Mode modes[] = {
        { "r", O_RDONLY }, { "rb", O_RDONLY },
        { "r+", O_RDWR | O_TRUNC }, { "r+b", O_RDWR | O_TRUNC }, { "rb+", O_RDWR | O_TRUNC },
        { "w", O_WRONLY | O_CREAT | O_TRUNC }, { "wb", O_WRONLY | O_CREAT | O_TRUNC },
        { "w+", O_RDWR | O_CREAT | O_TRUNC }, { "w+b", O_RDWR | O_CREAT | O_TRUNC }, { "wb+", O_RDWR | O_CREAT | O_TRUNC },
        { "a", O_WRONLY | O_CREAT | O_APPEND }, { "ab", O_WRONLY | O_CREAT | O_APPEND },
        { "a+", O_RDWR | O_CREAT | O_APPEND }, { "a+b", O_RDWR | O_CREAT | O_APPEND }, { "ab+", O_RDWR | O_CREAT | O_APPEND },
    };
    for (const Mode &m : modes) {
        if (strlen(m.text) == length && memcmp(m.text, mode, length) == 0) {
            return m.flags;
        }
    }
    return -1;
}

// the mode can change the file: the path must be writable and the open is quota'd
inline bool mode_writes(int flags)
{
    return (flags & (O_WRONLY | O_RDWR)) != 0;
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

}  // namespace ScriptingM4
