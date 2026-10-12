/*
  The C file calls of the Lua core on the M4 (m4/shim/AP_Filesystem/posix_compat.h),
  on files the M7 opens for it (m4:fs_* system methods,
  AP_ScriptingM4_FileServer). A FILE is a stream of this pool: a pointer the
  Lua core passes is checked against the pool before use, so the standard
  streams (1, 2, 3) fail with EBADF instead of being read. Lua thread only.
 */
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <AP_Filesystem/posix_compat.h>
#include <AP_ScriptingM4_Rpc.h>
extern "C" {
#include <lua/src/lauxlib.h>
}
#include "m4_bindings.h"
#include "m4_file_proxy.h"
#include "m4_file_stream.h"
#include "m4_rpc_client.h"

using namespace ScriptingM4;

struct apfs_file {
    FileStream stream;
};

namespace {

// the reply of an m4:fs_* call that answers one integer: >= 0, or -errno
int32_t integer_reply()
{
    const int ret = rpc.invoke();
    if (ret != 0) {
        return ret < 0 ? ret : -EIO;   // -ETIMEDOUT, -EMSGSIZE: the call did not get through
    }
    if (rpc.status() != ReplyStatus::Ok || rpc.count() != 1 || rpc.value(0).tag != ValueTag::Integer) {
        return -EIO;
    }
    return rpc.value(0).integer;
}

// FileStream's backend: the M7's file server
struct RpcFileBackend {
    int32_t open(const char *path, const char *mode)
    {
        RpcWriter &args = rpc.begin(SystemMethod::FsOpen);
        args.string(path, strlen(path));
        args.string(mode, strlen(mode));
        return integer_reply();
    }
    int32_t read(int32_t id, uint8_t *buf, uint16_t count)
    {
        RpcWriter &args = rpc.begin(SystemMethod::FsRead);
        args.integer(id);
        args.integer(count);
        const int ret = rpc.invoke();
        if (ret != 0) {
            return ret < 0 ? ret : -EIO;
        }
        if (rpc.status() != ReplyStatus::Ok || rpc.count() != 1) {
            return -EIO;
        }
        const RpcValue &value = rpc.value(0);
        if (value.tag == ValueTag::Integer) {
            return value.integer < 0 ? value.integer : -EIO;
        }
        if (value.tag != ValueTag::String || value.length > count) {
            return -EIO;   // never more than asked for
        }
        memcpy(buf, value.string, value.length);
        return value.length;
    }
    int32_t write(int32_t id, const uint8_t *buf, uint16_t count)
    {
        RpcWriter &args = rpc.begin(SystemMethod::FsWrite);
        args.integer(id);
        args.string(reinterpret_cast<const char *>(buf), count);
        return integer_reply();
    }
    int32_t seek(int32_t id, int32_t offset, int whence)
    {
        RpcWriter &args = rpc.begin(SystemMethod::FsSeek);
        args.integer(id);
        args.integer(offset);
        args.integer(whence);
        return integer_reply();
    }
    int32_t flush(int32_t id)
    {
        RpcWriter &args = rpc.begin(SystemMethod::FsFlush);
        args.integer(id);
        return integer_reply();
    }
    int32_t close(int32_t id)
    {
        RpcWriter &args = rpc.begin(SystemMethod::FsClose);
        args.integer(id);
        return integer_reply();
    }
};

RpcFileBackend backend;
apfs_file files[FILE_SLOTS];

// the stream of a FILE the Lua core passes; nullptr, with errno EBADF, for anything else
FileStream *stream_of(APFS_FILE *file)
{
    for (apfs_file &candidate : files) {
        if (&candidate == file && candidate.stream.is_open()) {
            return &candidate.stream;
        }
    }
    errno = EBADF;
    return nullptr;
}

// m4:fs_remove(path): 0, or -errno
int32_t remove_file(const char *path)
{
    RpcWriter &args = rpc.begin(SystemMethod::FsRemove);
    args.string(path, strlen(path));
    return integer_reply();
}

// m4:fs_dir replies per dirlist(): about 15 names each, so thousands of entries
const uint16_t DIR_REPLIES_MAX = 256;

// dirlist(path): lua_dirlist() of lua_bindings.cpp, a page of names per m4:fs_dir
int lua_dirlist(lua_State *L)
{
    binding_argcheck(L, 1);
    const char *path = luaL_checkstring(L, 1);
    int32_t start = 0;
    lua_Number key = 1;
    bool listed = false;
    for (uint16_t replies = 0; replies < DIR_REPLIES_MAX; replies++) {
        RpcWriter &args = rpc.begin(SystemMethod::FsDir);
        args.string(path, strlen(path));
        args.integer(start);
        // pushing names allocates, which may run a finalizer that calls the M7 and
        // reuses the client: work from a copy of the reply
        uint8_t reply[MAX_FRAME];
        RpcValue values[MAX_VALUES];
        uint8_t count = 0;
        const int ret = rpc.invoke();
        int32_t next = ret < 0 ? ret : -EIO;
        if (ret == 0 && rpc.status() == ReplyStatus::Ok && rpc.copy_reply(reply, sizeof(reply), values, count) &&
            count >= 1 && values[count - 1].tag == ValueTag::Integer) {
            next = values[count - 1].integer;
        }
        if (next < 0) {
            if (listed) {
                return 1;   // the directory went away part way: what was listed
            }
            const int error = int(-next);
            lua_pushnil(L);
            lua_pushstring(L, strerror(error));
            return 2;
        }
        if (!listed) {
            lua_newtable(L);
            listed = true;
        }
        for (uint8_t i = 0; i + 1 < count; i++) {
            if (values[i].tag != ValueTag::String) {
                continue;
            }
            lua_pushnumber(L, key++);
            lua_pushlstring(L, values[i].string, values[i].length);
            lua_settable(L, -3);
        }
        if (next == 0 || next <= start) {
            return 1;       // complete; or no progress, which a well-formed reply never makes
        }
        start = next;
    }
    return 1;
}

// remove(path): lua_removefile() of lua_bindings.cpp
int lua_removefile(lua_State *L)
{
    binding_argcheck(L, 1);
    const char *filename = luaL_checkstring(L, 1);
    const int32_t ret = remove_file(filename);
    if (ret < 0) {
        errno = int(-ret);
    }
    return luaL_fileresult(L, ret == 0, filename);
}

}  // namespace

void ScriptingM4::open_file_functions(lua_State *L)
{
    lua_pushcfunction(L, lua_dirlist);
    lua_setfield(L, -2, "dirlist");
    lua_pushcfunction(L, lua_removefile);
    lua_setfield(L, -2, "remove");
}

extern "C" {

APFS_FILE *apfs_fopen(const char *pathname, const char *mode)
{
    for (apfs_file &file : files) {
        if (!file.stream.is_open()) {
            return file.stream.open(backend, pathname, mode) ? &file : nullptr;
        }
    }
    errno = EMFILE;
    return nullptr;
}

int apfs_fprintf(APFS_FILE *stream, const char *format, ...)
{
    FileStream *file = stream_of(stream);
    if (file == nullptr) {
        return -1;
    }
    // the io library prints numbers (LUA_NUMBER_FMT, LUA_INTEGER_FMT): short texts
    char text[64];
    va_list ap;
    va_start(ap, format);
    const int length = vsnprintf(text, sizeof(text), format, ap);
    va_end(ap);
    if (length < 0 || size_t(length) >= sizeof(text)) {
        errno = EOVERFLOW;
        return -1;
    }
    return int(file->write(backend, text, 1, size_t(length)));
}

int apfs_fflush(APFS_FILE *stream)
{
    FileStream *file = stream_of(stream);
    return file != nullptr ? file->flush(backend) : EOF;
}

size_t apfs_fread(void *ptr, size_t size, size_t nmemb, APFS_FILE *stream)
{
    FileStream *file = stream_of(stream);
    return file != nullptr ? file->read(backend, ptr, size, nmemb) : 0;
}

size_t apfs_fwrite(const void *ptr, size_t size, size_t nmemb, APFS_FILE *stream)
{
    FileStream *file = stream_of(stream);
    return file != nullptr ? file->write(backend, ptr, size, nmemb) : 0;
}

int apfs_fputs(const char *s, APFS_FILE *stream)
{
    FileStream *file = stream_of(stream);
    if (file == nullptr) {
        return EOF;
    }
    const size_t length = strlen(s);
    return file->write(backend, s, 1, length) == length ? int(length) : EOF;
}

char *apfs_fgets(char *s, int size, APFS_FILE *stream)
{
    FileStream *file = stream_of(stream);
    if (file == nullptr || size <= 0) {
        return nullptr;
    }
    int n = 0;
    while (n < size - 1) {
        const int c = file->getc(backend);
        if (c == EOF) {
            break;
        }
        s[n++] = char(c);
        if (c == '\n') {
            break;
        }
    }
    if (n == 0) {
        return nullptr;
    }
    s[n] = 0;
    return s;
}

void apfs_clearerr(APFS_FILE *stream)
{
    FileStream *file = stream_of(stream);
    if (file != nullptr) {
        file->clear_error();
    }
}

int apfs_fseek(APFS_FILE *stream, long offset, int whence)
{
    FileStream *file = stream_of(stream);
    return file != nullptr ? file->seek(backend, offset, whence) : EOF;
}

// as posix_compat.cpp, a stream that is no file reads as an error (EOF)
int apfs_ferror(APFS_FILE *stream)
{
    FileStream *file = stream_of(stream);
    return file != nullptr ? int(file->error()) : EOF;
}

int apfs_fclose(APFS_FILE *stream)
{
    FileStream *file = stream_of(stream);
    return file != nullptr ? file->close(backend) : EOF;
}

APFS_FILE *apfs_tmpfile(void)
{
    errno = ENOSYS;   // io.tmpfile is not registered (liolib.c)
    return nullptr;
}

int apfs_getc(APFS_FILE *stream)
{
    FileStream *file = stream_of(stream);
    return file != nullptr ? file->getc(backend) : EOF;
}

int apfs_ungetc(int c, APFS_FILE *stream)
{
    FileStream *file = stream_of(stream);
    return file != nullptr ? file->ungetc(c) : EOF;
}

int apfs_feof(APFS_FILE *stream)
{
    FileStream *file = stream_of(stream);
    return file != nullptr ? int(file->eof()) : EOF;
}

long apfs_ftell(APFS_FILE *stream)
{
    FileStream *file = stream_of(stream);
    return file != nullptr ? file->tell(backend) : -1;
}

APFS_FILE *apfs_freopen(const char *pathname, const char *mode, APFS_FILE *stream)
{
    (void)pathname;
    (void)mode;
    // only luaL_loadfilex with LUA_SUPPORT_LOAD_BINARY (0) reopens: close, so nothing leaks
    FileStream *file = stream_of(stream);
    if (file != nullptr) {
        file->close(backend);
    }
    errno = ENOSYS;
    return nullptr;
}

int apfs_remove(const char *pathname)
{
    const int32_t ret = remove_file(pathname);
    if (ret < 0) {
        errno = int(-ret);
        return -1;
    }
    return 0;
}

}  // extern "C"
