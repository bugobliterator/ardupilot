#pragma once

#include <AP_Rust/AP_Rust_config.h>

#if AP_SCRIPTING_RUST_ENABLED

#include <stdint.h>
#include <AP_Common/AP_Common.h>
#include "AP_RustM4_Files.h"

namespace RustM4 {

// the filesystem calls the file server makes; AP_Filesystem's on the vehicle, a fake in tests
struct FileOps {
    int (*open)(const char *path, int flags);   // a descriptor, or -1 with errno set
    int32_t (*read)(int fd, void *buffer, uint32_t count);
    int (*close)(int fd);
};

#if AP_RUST_M4_ENABLED
const FileOps &filesystem_file_ops();
#endif

}  // namespace RustM4

/*
  The M4's module files (FsOpen, FsRead, FsSeek, FsClose): read only, only
  module_path_allowed() paths, read in order from the start. Close returns
  zlib's CRC-32 of exactly the bytes served and their count, which the M4
  compares with what it received and parsed. Paths and ids come from the
  untrusted M4 and are checked here (module_path_allowed(), FileTable).
  Failures are -errno. Files belong to the M4 session: reset() closes them.
  One thread at a time.
 */
class AP_RustM4_FileServer {
public:
    explicit AP_RustM4_FileServer(const RustM4::FileOps &ops) : _ops(ops) {}
    CLASS_NO_COPY(AP_RustM4_FileServer);

    struct Closed {
        int32_t result;   // 0, or -errno
        uint32_t crc;
        uint32_t served;
    };

    // path terminated; flags must be RustM4::MODULE_OPEN_FLAGS. The id, or -errno
    int32_t open(const char *path, int flags);
    // up to count bytes (at most RustM4::FS_READ_MAX) at the file's offset; the count read
    // (0 at the end), or -errno
    int32_t read(int32_t id, uint8_t *buffer, uint16_t count);
    // offset, if it is the file's current offset (the bytes served so far); otherwise -EINVAL
    int32_t seek(int32_t id, int32_t offset);
    // the id is stale afterwards
    Closed close(int32_t id);
    // closes every file the M4 left open; their ids go stale
    void reset();

private:
    struct Served {
        uint32_t crc;      // crc_crc32() register: no final xor yet
        uint32_t count;
    };

    const RustM4::FileOps &_ops;
    RustM4::FileTable _table;
    Served _served[RustM4::FILE_SLOTS];
};

#endif  // AP_SCRIPTING_RUST_ENABLED
