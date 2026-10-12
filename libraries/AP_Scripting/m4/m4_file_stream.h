#pragma once

/*
  The M4 side of a file the M7 holds open (m4_file_proxy.cpp: Lua io,
  require, luaL_loadfile): the stdio behaviour of AP_Filesystem/posix_compat.cpp
  over a backend, the m4:fs_* calls on the M4 and a memory file in the host
  tests. A Backend has open/read/write/seek/flush/close returning >= 0 or
  -errno. A file opened "r"/"rb" reads ahead FILE_READ_AHEAD bytes, so Lua's
  line and number reads (getc) make one call per FILE_READ_AHEAD bytes; other
  modes go straight to the M7, so a write never lands behind bytes read ahead.
  A stream that is not open fails with EBADF and calls nothing.
  C++11, no ArduPilot or Lua dependencies (host unit tests).
 */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../AP_ScriptingM4_Files.h"

namespace ScriptingM4 {

static const uint16_t FILE_READ_AHEAD = 256;
static_assert(FILE_READ_AHEAD <= FILE_CHUNK, "a read-ahead is one m4:fs_read");

class FileStream {
public:
    bool is_open() const { return _id >= 0; }
    bool error() const { return _error; }
    bool eof() const { return _eof; }
    void clear_error() { _error = false; }

    // fopen(): false, with errno set, if the M7 refused
    template <typename Backend> bool open(Backend &backend, const char *path, const char *mode)
    {
        forget();
        const int32_t id = backend.open(path, mode);
        if (id < 0) {
            errno = int(-id);
            return false;
        }
        _id = id;
        _read_ahead = strcmp(mode, "r") == 0 || strcmp(mode, "rb") == 0;
        return true;
    }

    // fread(): whole items; reads on until count items are in or the file ends, as one
    // AP::FS().read() in-process fills the request
    template <typename Backend> size_t read(Backend &backend, void *ptr, size_t size, size_t count)
    {
        if (!usable() || size == 0 || count == 0) {
            return 0;
        }
        if (count > SIZE_MAX / size) {
            errno = EOVERFLOW;
            _error = true;
            return 0;
        }
        uint8_t *out = static_cast<uint8_t *>(ptr);
        const size_t wanted = size * count;
        size_t got = 0;
        if (_unget >= 0) {
            out[got++] = uint8_t(_unget);
            _unget = -1;
        }
        while (got < wanted) {
            if (_pos < _len) {
                const size_t n = smaller(size_t(_len - _pos), wanted - got);
                memcpy(&out[got], &_buf[_pos], n);
                _pos = uint16_t(_pos + n);
                got += n;
                continue;
            }
            const size_t left = wanted - got;
            if (_read_ahead && left < FILE_READ_AHEAD) {
                if (!refill(backend)) {
                    break;
                }
                continue;
            }
            const int32_t n = backend.read(_id, &out[got], uint16_t(smaller(left, size_t(FILE_CHUNK))));
            if (n <= 0) {
                ended(n);
                break;
            }
            got += size_t(n);
        }
        return got / size;
    }

    template <typename Backend> int getc(Backend &backend)
    {
        if (!usable()) {
            return EOF;
        }
        if (_unget >= 0) {
            const int c = _unget;
            _unget = -1;
            return c;
        }
        if (_read_ahead) {
            if (_pos >= _len && !refill(backend)) {
                return EOF;
            }
            return _buf[_pos++];
        }
        uint8_t c = 0;
        const int32_t n = backend.read(_id, &c, 1);
        if (n <= 0) {
            ended(n);
            return EOF;
        }
        return c;
    }

    int ungetc(int c)
    {
        if (!usable() || c == EOF) {
            return EOF;
        }
        _unget = int16_t(uint8_t(c));
        _eof = false;
        return c;
    }

    // fwrite(): FILE_CHUNK bytes per call; a failed or short write sets the error
    template <typename Backend> size_t write(Backend &backend, const void *ptr, size_t size, size_t count)
    {
        if (!usable() || size == 0 || count == 0) {
            return 0;
        }
        if (count > SIZE_MAX / size) {
            errno = EOVERFLOW;
            _error = true;
            return 0;
        }
        const uint8_t *in = static_cast<const uint8_t *>(ptr);
        const size_t total = size * count;
        size_t done = 0;
        while (done < total) {
            const uint16_t n = uint16_t(smaller(total - done, size_t(FILE_CHUNK)));
            const int32_t written = backend.write(_id, &in[done], n);
            if (written <= 0) {
                if (written < 0) {
                    errno = int(-written);
                }
                _error = true;
                break;
            }
            done += size_t(written);
            if (written < n) {
                _error = true;   // the card is full
                break;
            }
        }
        return done / size;
    }

    // fseek(): 0 even when the M7 refuses, as posix_compat.cpp; the next tell() shows where the file is
    template <typename Backend> int seek(Backend &backend, long offset, int whence)
    {
        if (!usable()) {
            return EOF;
        }
        if (whence == SEEK_CUR) {
            offset -= long(_len - _pos);   // the M7 is past the bytes read ahead
        }
        _pos = 0;
        _len = 0;
        _unget = -1;
        _eof = false;
        const int32_t position = backend.seek(_id, int32_t(offset), whence);
        if (position < 0) {
            errno = int(-position);
        }
        return 0;
    }

    // ftell(): the M7's position less the bytes read ahead; an ungetc'd byte counts as read,
    // as in posix_compat.cpp
    template <typename Backend> long tell(Backend &backend)
    {
        if (!usable()) {
            return -1;
        }
        const int32_t position = backend.seek(_id, 0, SEEK_CUR);
        if (position < 0) {
            errno = int(-position);
            return -1;
        }
        return long(position) - long(_len - _pos);
    }

    template <typename Backend> int flush(Backend &backend)
    {
        if (!usable()) {
            return EOF;
        }
        const int32_t ret = backend.flush(_id);
        if (ret < 0) {
            errno = int(-ret);
            return EOF;
        }
        return 0;
    }

    // fclose(): the stream is free afterwards whatever the M7 answers
    template <typename Backend> int close(Backend &backend)
    {
        if (!usable()) {
            return EOF;
        }
        const int32_t ret = backend.close(_id);
        forget();
        if (ret < 0) {
            errno = int(-ret);
            return EOF;
        }
        return 0;
    }

private:
    static size_t smaller(size_t a, size_t b) { return a < b ? a : b; }

    bool usable() const
    {
        if (_id < 0) {
            errno = EBADF;
            return false;
        }
        return true;
    }

    void forget()
    {
        _id = -1;
        _read_ahead = false;
        _error = false;
        _eof = false;
        _unget = -1;
        _pos = 0;
        _len = 0;
    }

    // a read found the end (0) or failed (-errno)
    void ended(int32_t n)
    {
        if (n < 0) {
            errno = int(-n);
            _error = true;
        }
        _eof = true;
    }

    template <typename Backend> bool refill(Backend &backend)
    {
        const int32_t n = backend.read(_id, _buf, FILE_READ_AHEAD);
        if (n <= 0) {
            ended(n);
            return false;
        }
        _pos = 0;
        _len = uint16_t(n);
        return true;
    }

    int32_t _id = -1;           // the M7's file id
    bool _read_ahead = false;   // opened "r" or "rb"
    bool _error = false;
    bool _eof = false;
    int16_t _unget = -1;
    uint16_t _pos = 0;          // the bytes read ahead and not read yet: _buf[_pos, _len)
    uint16_t _len = 0;
    uint8_t _buf[FILE_READ_AHEAD] {};   // initialised: the stream pool is constant-initialised, no constructor
};

}  // namespace ScriptingM4
