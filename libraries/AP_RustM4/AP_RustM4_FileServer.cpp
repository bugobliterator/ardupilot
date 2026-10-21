#include "AP_RustM4_FileServer.h"

#if AP_SCRIPTING_RUST_ENABLED

#include <errno.h>
#include <string.h>
#include <AP_Math/crc.h>
#include "AP_RustM4_Protocol.h"

using namespace RustM4;

static int32_t errno_or_io()
{
    return errno > 0 ? -errno : -EIO;
}

int32_t AP_RustM4_FileServer::open(const char *path, int flags)
{
    if (flags != MODULE_OPEN_FLAGS) {
        return -EACCES;
    }
    if (path == nullptr) {
        return -EINVAL;
    }
    if (strnlen(path, MAX_FS_PATH) >= MAX_FS_PATH) {
        return -ENAMETOOLONG;
    }
    if (!module_path_allowed(path)) {
        return -EACCES;
    }
    if (_table.full()) {
        return -EMFILE;
    }
    errno = 0;
    const int fd = _ops.open(path, flags);
    if (fd < 0) {
        return errno_or_io();
    }
    const int32_t id = _table.insert(fd);
    if (id < 0) {
        _ops.close(fd);
        return -EMFILE;
    }
    // AP_Math's crc_crc32() has no initial value or final xor: zlib's CRC starts at all ones
    _served[id & 0xFF] = Served { 0xFFFFFFFFU, 0 };
    return id;
}

int32_t AP_RustM4_FileServer::read(int32_t id, uint8_t *buffer, uint16_t count)
{
    const int fd = _table.lookup(id);
    if (fd < 0) {
        return -EBADF;
    }
    if (count > FS_READ_MAX || buffer == nullptr) {
        return -EINVAL;
    }
    errno = 0;
    const int32_t n = _ops.read(fd, buffer, count);
    if (n < 0) {
        return errno_or_io();
    }
    if (n > int32_t(count)) {
        return -EIO;
    }
    Served &served = _served[id & 0xFF];
    served.crc = crc_crc32(served.crc, buffer, uint32_t(n));
    served.count += uint32_t(n);
    return n;
}

int32_t AP_RustM4_FileServer::seek(int32_t id, int32_t offset)
{
    if (_table.lookup(id) < 0) {
        return -EBADF;
    }
    if (offset < 0 || uint32_t(offset) != _served[id & 0xFF].count) {
        return -EINVAL;
    }
    return offset;
}

AP_RustM4_FileServer::Closed AP_RustM4_FileServer::close(int32_t id)
{
    Closed closed {};
    if (_table.lookup(id) < 0) {
        closed.result = -EBADF;
        return closed;
    }
    const Served served = _served[id & 0xFF];
    const int fd = _table.remove(id);
    closed.crc = ~served.crc;
    closed.served = served.count;
    errno = 0;
    closed.result = _ops.close(fd) == 0 ? 0 : errno_or_io();
    return closed;
}

void AP_RustM4_FileServer::reset()
{
    _table.close_all([this](int fd) { _ops.close(fd); });
}

#if AP_RUST_M4_ENABLED

#include <AP_Filesystem/AP_Filesystem.h>

namespace {

int filesystem_open(const char *path, int flags)
{
    return AP::FS().open(path, flags);
}

int32_t filesystem_read(int fd, void *buffer, uint32_t count)
{
    return AP::FS().read(fd, buffer, count);
}

int filesystem_close(int fd)
{
    return AP::FS().close(fd);
}

}  // namespace

const FileOps &RustM4::filesystem_file_ops()
{
    static const FileOps ops { filesystem_open, filesystem_read, filesystem_close };
    return ops;
}

#endif  // AP_RUST_M4_ENABLED

#endif  // AP_SCRIPTING_RUST_ENABLED
