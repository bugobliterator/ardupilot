#pragma once

/*
  The wire form of a pure-data userdata (a blob value): its described fields
  in the generator's order, little-endian, at their declared width. The M7's
  codecs (lua_remote_dispatch.cpp) and the M4's local types
  (lua_remote_local.cpp) read and write it; it needs no AP_HAL, so both
  images build it. The writer fills a buffer its caller owns.
 */

#include <stdint.h>
#include <string.h>
#include <type_traits>

namespace ScriptingM4 {

class WireWriter {
public:
    WireWriter(uint8_t *buf, uint16_t capacity) : _buf(buf), _capacity(capacity) {}

    template <typename T> void write(T value)
    {
        static_assert(std::is_arithmetic<T>::value && !std::is_same<T, bool>::value, "write the wire kind");
        if (!_ok || uint32_t(_length) + sizeof(value) > _capacity) {
            _ok = false;
            return;
        }
        memcpy(&_buf[_length], &value, sizeof(value));
        _length += sizeof(value);
    }
    bool ok() const { return _ok; }
    uint16_t length() const { return _length; }
    const uint8_t *data() const { return _buf; }

private:
    uint8_t *_buf;
    uint16_t _capacity;
    uint16_t _length = 0;
    bool _ok = true;
};

class WireReader {
public:
    WireReader(const uint8_t *data, uint16_t length) : _data(data), _length(length) {}

    template <typename T> bool read(T &value)
    {
        // a bool or enum read from arbitrary bytes is undefined: read the wire kind, then check and convert
        static_assert(std::is_arithmetic<T>::value && !std::is_same<T, bool>::value, "read the wire kind, then convert");
        if (uint32_t(_pos) + sizeof(value) > _length) {
            return false;
        }
        memcpy(&value, &_data[_pos], sizeof(value));
        _pos += sizeof(value);
        return true;
    }
    bool done() const { return _pos == _length; }

private:
    const uint8_t *_data;
    uint16_t _length;
    uint16_t _pos = 0;
};

}  // namespace ScriptingM4
