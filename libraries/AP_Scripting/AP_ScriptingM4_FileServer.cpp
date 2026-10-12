#include "AP_ScriptingM4_FileServer.h"

#if AP_SCRIPTING_M4_ENABLED

#include <errno.h>
#include <string.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/AP_HAL.h>

using namespace ScriptingM4;

static_assert(DIR_REPLY_BYTES >= 255 + 3, "one name always fits a m4:fs_dir reply");

static ReplyStatus arg_error(const RpcArgs &args, RpcWriter &results, uint8_t &error_arg)
{
    results.string(args.message(), strlen(args.message()));
    error_arg = args.error_arg();
    return ReplyStatus::ArgError;
}

static ReplyStatus fail(RpcWriter &results, int error)
{
    results.integer(-error);
    return ReplyStatus::Ok;
}

// an AP_Filesystem call failed: its errno for the M4
static ReplyStatus fail_errno(RpcWriter &results)
{
    const int error = errno;
    return fail(results, error > 0 ? error : EIO);
}

// the M4's path text copied and terminated here (it is not terminated on the wire); 0, or the
// errno the call fails with. check_script_path() later checks this buffer, which is the one opened
static int copy_path(const char *text, uint16_t length, char (&path)[MAX_FS_PATH])
{
    if (length >= sizeof(path)) {
        return ENAMETOOLONG;
    }
    memcpy(path, text, length);
    path[length] = 0;
    return 0;
}

ReplyStatus AP_ScriptingM4_FileServer::handle(SystemMethod method, RpcArgs &args, RpcWriter &results,
                                              uint8_t &error_arg, Quota &quota)
{
    switch (method) {
    case SystemMethod::FsOpen:
        return open(args, results, error_arg, quota);
    case SystemMethod::FsRead:
        return read(args, results, error_arg);
    case SystemMethod::FsWrite:
        return write(args, results, error_arg, quota);
    case SystemMethod::FsSeek:
        return seek(args, results, error_arg);
    case SystemMethod::FsFlush:
        return flush(args, results, error_arg);
    case SystemMethod::FsClose:
        return close(args, results, error_arg);
    case SystemMethod::FsRemove:
        return remove(args, results, error_arg, quota);
    case SystemMethod::FsDir:
        return dir(args, results, error_arg);
    default:
        break;
    }
    return ReplyStatus::UnknownMethod;
}

void AP_ScriptingM4_FileServer::reset()
{
    _table.close_all([](int fd) { AP::FS().close(fd); });
}

// m4:fs_open(path, mode) -> id, or -errno
ReplyStatus AP_ScriptingM4_FileServer::open(RpcArgs &args, RpcWriter &results, uint8_t &error_arg, Quota &quota)
{
    const char *text = nullptr;
    uint16_t length = 0;
    if (!args.count_is(2) || !args.string(1, text, length)) {
        return arg_error(args, results, error_arg);
    }
    // the next string() may reuse args' conversion buffer: keep the path first
    char path[MAX_FS_PATH];
    const int copy_error = copy_path(text, length, path);
    const char *mode = nullptr;
    uint16_t mode_length = 0;
    if (!args.string(2, mode, mode_length)) {
        return arg_error(args, results, error_arg);
    }
    if (copy_error != 0) {
        return fail(results, copy_error);
    }
    const int flags = fopen_mode_flags(mode, mode_length);
    if (flags < 0) {
        return fail(results, EINVAL);
    }
    const bool writes = mode_writes(flags);
    const int error = check_script_path(path, length, writes ? PathUse::Write : PathUse::Read);
    if (error != 0) {
        return fail(results, error);
    }
    if (_table.full()) {
        return fail(results, EMFILE);
    }
    // writing costs SD card time: write opens share the FileIo quota; reads (require, io) do not
    if (writes && !quota.allow(QuotaClass::FileIo, AP_HAL::millis())) {
        return fail(results, EAGAIN);
    }
    errno = 0;
    const int fd = AP::FS().open(path, flags);
    if (fd < 0) {
        return fail_errno(results);
    }
    const int32_t id = _table.insert(fd);
    if (id < 0) {
        AP::FS().close(fd);
        return fail(results, EMFILE);
    }
    results.integer(id);
    return ReplyStatus::Ok;
}

// m4:fs_read(id, count) -> up to count bytes ("" at the end), or -errno
ReplyStatus AP_ScriptingM4_FileServer::read(RpcArgs &args, RpcWriter &results, uint8_t &error_arg)
{
    int32_t id = 0;
    int32_t count = 0;
    if (!args.count_is(2) || !args.integer(1, id) || !args.integer_in(2, 0, FILE_CHUNK, count)) {
        return arg_error(args, results, error_arg);
    }
    const int fd = _table.lookup(id);
    if (fd < 0) {
        return fail(results, EBADF);
    }
    uint8_t chunk[FILE_CHUNK];
    errno = 0;
    const int32_t n = AP::FS().read(fd, chunk, uint32_t(count));
    if (n < 0) {
        return fail_errno(results);
    }
    results.string(reinterpret_cast<const char *>(chunk), size_t(n));
    return ReplyStatus::Ok;
}

// m4:fs_write(id, bytes) -> bytes written, or -errno
ReplyStatus AP_ScriptingM4_FileServer::write(RpcArgs &args, RpcWriter &results, uint8_t &error_arg, Quota &quota)
{
    int32_t id = 0;
    const char *data = nullptr;
    uint16_t length = 0;
    if (!args.count_is(2) || !args.integer(1, id) || !args.string(2, data, length)) {
        return arg_error(args, results, error_arg);
    }
    if (length > FILE_CHUNK) {
        return fail(results, EINVAL);
    }
    const int fd = _table.lookup(id);
    if (fd < 0) {
        return fail(results, EBADF);
    }
    if (!quota.allow(QuotaClass::FileIo, AP_HAL::millis())) {
        return fail(results, EAGAIN);
    }
    errno = 0;
    const int32_t n = AP::FS().write(fd, data, length);
    if (n < 0) {
        return fail_errno(results);
    }
    results.integer(n);
    return ReplyStatus::Ok;
}

// m4:fs_seek(id, offset, whence) -> the new position, or -errno
ReplyStatus AP_ScriptingM4_FileServer::seek(RpcArgs &args, RpcWriter &results, uint8_t &error_arg)
{
    int32_t id = 0;
    int32_t offset = 0;
    int32_t whence = 0;
    if (!args.count_is(3) || !args.integer(1, id) || !args.integer(2, offset) ||
        !args.integer_in(3, SEEK_SET, SEEK_END, whence)) {
        return arg_error(args, results, error_arg);
    }
    const int fd = _table.lookup(id);
    if (fd < 0) {
        return fail(results, EBADF);
    }
    if (whence == SEEK_SET && offset < 0) {
        return fail(results, EINVAL);
    }
    errno = 0;
    const int32_t position = AP::FS().lseek(fd, offset, int(whence));
    if (position < 0) {
        return fail_errno(results);
    }
    results.integer(position);
    return ReplyStatus::Ok;
}

// m4:fs_flush(id) -> 0, or -errno
ReplyStatus AP_ScriptingM4_FileServer::flush(RpcArgs &args, RpcWriter &results, uint8_t &error_arg)
{
    int32_t id = 0;
    if (!args.count_is(1) || !args.integer(1, id)) {
        return arg_error(args, results, error_arg);
    }
    const int fd = _table.lookup(id);
    if (fd < 0) {
        return fail(results, EBADF);
    }
    errno = 0;
    if (AP::FS().fsync(fd) != 0) {
        return fail_errno(results);
    }
    results.integer(0);
    return ReplyStatus::Ok;
}

// m4:fs_close(id) -> 0, or -errno; the id is stale afterwards either way
ReplyStatus AP_ScriptingM4_FileServer::close(RpcArgs &args, RpcWriter &results, uint8_t &error_arg)
{
    int32_t id = 0;
    if (!args.count_is(1) || !args.integer(1, id)) {
        return arg_error(args, results, error_arg);
    }
    const int fd = _table.remove(id);
    if (fd < 0) {
        return fail(results, EBADF);
    }
    errno = 0;
    if (AP::FS().close(fd) != 0) {
        return fail_errno(results);
    }
    results.integer(0);
    return ReplyStatus::Ok;
}

// m4:fs_remove(path) -> 0, or -errno
ReplyStatus AP_ScriptingM4_FileServer::remove(RpcArgs &args, RpcWriter &results, uint8_t &error_arg, Quota &quota)
{
    const char *text = nullptr;
    uint16_t length = 0;
    if (!args.count_is(1) || !args.string(1, text, length)) {
        return arg_error(args, results, error_arg);
    }
    char path[MAX_FS_PATH];
    int error = copy_path(text, length, path);
    if (error == 0) {
        error = check_script_path(path, length, PathUse::Write);
    }
    if (error != 0) {
        return fail(results, error);
    }
    if (!quota.allow(QuotaClass::FileIo, AP_HAL::millis())) {
        return fail(results, EAGAIN);
    }
    errno = 0;
    if (AP::FS().unlink(path) != 0) {
        return fail_errno(results);
    }
    results.integer(0);
    return ReplyStatus::Ok;
}

// m4:fs_dir(path, start) -> the names from entry start on, then the start of the next call
// (0 once the listing is complete); or one -errno. Each call opens the directory again
ReplyStatus AP_ScriptingM4_FileServer::dir(RpcArgs &args, RpcWriter &results, uint8_t &error_arg)
{
    const char *text = nullptr;
    uint16_t length = 0;
    int32_t start = 0;
    if (!args.count_is(2) || !args.string(1, text, length)) {
        return arg_error(args, results, error_arg);
    }
    // copied before integer_in(), which may reuse args' conversion buffer
    char path[MAX_FS_PATH];
    int error = copy_path(text, length, path);
    if (!args.integer_in(2, 0, INT32_MAX, start)) {
        return arg_error(args, results, error_arg);
    }
    if (error == 0) {
        error = check_script_path(path, length, PathUse::List);
    }
    if (error != 0) {
        return fail(results, error);
    }
    errno = 0;
    auto *d = AP::FS().opendir(path);
    if (d == nullptr) {
        return fail_errno(results);
    }
    int32_t index = 0;
    int32_t next = 0;
    uint16_t bytes = 0;
    for (struct dirent *de = AP::FS().readdir(d); de != nullptr; de = AP::FS().readdir(d)) {
        if (index++ < start) {
            continue;
        }
        const size_t name_length = strnlen(de->d_name, sizeof(de->d_name));
        // one value stays for next
        if (results.count() + 1 >= MAX_VALUES || bytes + name_length + 3 > DIR_REPLY_BYTES) {
            next = index - 1;   // this entry opens the next reply
            break;
        }
        results.string(de->d_name, name_length);
        bytes += uint16_t(name_length + 3);
    }
    AP::FS().closedir(d);
    results.integer(next);
    return ReplyStatus::Ok;
}

#endif  // AP_SCRIPTING_M4_ENABLED
