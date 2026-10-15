#pragma once

/*
  Lua calls between the M4 Lua runtime and the M7 scripting host: the tagged
  value codec, the argument checks the M7 applies (the texts lauxlib and
  binding_argcheck() give in-process), and the system method table whose
  hash is the contract both images publish in their peer headers.
  C++11, no ArduPilot or ChibiOS dependencies: the M4 image, the SITL M4
  process and the host unit tests include it.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "AP_ScriptingM4_Placement.h"
#include "AP_ScriptingM4_Protocol.h"

namespace ScriptingM4 {

#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
static const uint32_t RPC_PROTOCOL_VERSION = 5;   // Channel 16 KB, network rings; NetConfigRequest/NetConfig
#else
static const uint32_t RPC_PROTOCOL_VERSION = 3;
#endif

enum class ValueTag : uint8_t {
    Nil = 0,
    False = 1,
    True = 2,
    Integer = 3,   // int32 (LUA_32BITS)
    Number = 4,    // float (LUA_32BITS)
    String = 5,    // uint16 length, then the bytes, no terminator
    Blob = 6,      // uint8 type id, uint16 length, the bytes: a pure-data userdata in its wire form
    Handle = 7,    // uint8 type id, uint16 index, uint32 generation: an M7 object of this M4 session
    Opaque = 8,    // a value that cannot cross (table, function, ...): uint8 Lua type, for error texts
};

// the most values one call or reply carries: logger:write with units, multipliers and 15 fields is
// the self, five strings and 15 values (AP_Logger: LS_FORMAT_SIZE 17 leaves 15 format characters)
static const uint8_t MAX_VALUES = 21;
// a string value fills a reply frame at most: header, tag and length
static const uint16_t MAX_STRING = MAX_FRAME - sizeof(ReplyHeader) - 3;
// the largest blob: AP_MotorsMatrix_Scripting_Dynamic::factor_table, 512 bytes
static const uint16_t MAX_BLOB = 512;
// scripts per M4 session and their path length, terminator included
static const uint8_t MAX_SCRIPTS = 32;
static const uint8_t MAX_SCRIPT_PATH = 64;
// the M4's buffer for require's search path (m4:config), terminator included
static const uint16_t MAX_MODULES_PATH = 192;

struct RpcValue {
    ValueTag tag;
    uint8_t lua_type;             // Opaque
    uint8_t type_id;              // Blob, Handle: generated type id (lua_remote_contract.h)
    uint16_t length;              // String, Blob
    uint16_t handle_index;        // Handle
    int32_t integer;              // Integer
    float number;                 // Number
    uint32_t handle_generation;   // Handle
    const char *string;           // String: points into the frame, not terminated
    const uint8_t *blob;          // Blob: points into the frame
};

// appends encoded values to a frame; once a value does not fit, ok() stays false
class RpcWriter {
public:
    void begin(uint8_t *buf, uint16_t capacity, uint16_t start)
    {
        _buf = buf;
        _capacity = capacity;
        _start = start;
        restart();
    }
    // empty again, from the start position
    void restart()
    {
        _length = _start;
        _count = 0;
        _ok = _buf != nullptr && _start <= _capacity;
    }
    void nil() { tag(ValueTag::Nil); }
    void boolean(bool value) { tag(value ? ValueTag::True : ValueTag::False); }
    void integer(int32_t value)
    {
        if (tag(ValueTag::Integer)) {
            put(&value, sizeof(value));
        }
    }
    void number(float value)
    {
        if (tag(ValueTag::Number)) {
            put(&value, sizeof(value));
        }
    }
    void string(const char *text, size_t length)
    {
        if (length > MAX_STRING) {
            _ok = false;
            return;
        }
        const uint16_t length16 = uint16_t(length);
        if (tag(ValueTag::String) && put(&length16, sizeof(length16)) && length16 > 0) {
            put(text, length16);
        }
    }
    void opaque(uint8_t lua_type)
    {
        if (tag(ValueTag::Opaque)) {
            put(&lua_type, sizeof(lua_type));
        }
    }
    void blob(uint8_t type_id, const void *data, uint16_t length)
    {
        if (type_id == 0 || length > MAX_BLOB || (data == nullptr && length > 0)) {
            _ok = false;
            return;
        }
        if (tag(ValueTag::Blob) && put(&type_id, sizeof(type_id)) && put(&length, sizeof(length)) && length > 0) {
            put(data, length);
        }
    }
    void handle(uint8_t type_id, uint16_t index, uint32_t generation)
    {
        if (type_id == 0) {
            _ok = false;
            return;
        }
        if (tag(ValueTag::Handle) && put(&type_id, sizeof(type_id)) && put(&index, sizeof(index))) {
            put(&generation, sizeof(generation));
        }
    }
    bool ok() const { return _ok; }
    uint16_t length() const { return _length; }
    uint8_t count() const { return _count; }

private:
    bool tag(ValueTag value_tag)
    {
        if (_count >= MAX_VALUES) {
            _ok = false;
            return false;
        }
        const uint8_t byte = uint8_t(value_tag);
        if (!put(&byte, sizeof(byte))) {
            return false;
        }
        _count++;
        return true;
    }
    bool put(const void *data, uint16_t length)
    {
        if (!_ok || uint32_t(_length) + length > _capacity) {
            _ok = false;
            return false;
        }
        memcpy(&_buf[_length], data, length);
        _length += length;
        return true;
    }

    uint8_t *_buf = nullptr;
    uint16_t _capacity = 0;
    uint16_t _start = 0;
    uint16_t _length = 0;
    uint8_t _count = 0;
    bool _ok = false;
};

// decodes exactly count values from data[0, length); false on truncation, an unknown tag or trailing bytes
inline bool decode_values(const uint8_t *data, uint16_t length, uint8_t count, RpcValue *values)
{
    if (count > MAX_VALUES) {
        return false;
    }
    uint16_t pos = 0;
    for (uint8_t i = 0; i < count; i++) {
        if (pos >= length) {
            return false;
        }
        RpcValue &value = values[i];
        memset(&value, 0, sizeof(value));
        value.tag = ValueTag(data[pos++]);
        switch (value.tag) {
        case ValueTag::Nil:
        case ValueTag::False:
        case ValueTag::True:
            break;
        case ValueTag::Integer:
            if (length - pos < 4) {
                return false;
            }
            memcpy(&value.integer, &data[pos], 4);
            pos += 4;
            break;
        case ValueTag::Number:
            if (length - pos < 4) {
                return false;
            }
            memcpy(&value.number, &data[pos], 4);
            pos += 4;
            break;
        case ValueTag::String: {
            if (length - pos < 2) {
                return false;
            }
            uint16_t string_length;
            memcpy(&string_length, &data[pos], 2);
            pos += 2;
            if (string_length > MAX_STRING || length - pos < string_length) {
                return false;
            }
            value.length = string_length;
            value.string = reinterpret_cast<const char *>(&data[pos]);
            pos += string_length;
            break;
        }
        case ValueTag::Blob: {
            if (length - pos < 3) {
                return false;
            }
            uint16_t blob_length;
            value.type_id = data[pos];
            memcpy(&blob_length, &data[pos + 1], 2);
            pos += 3;
            if (value.type_id == 0 || blob_length > MAX_BLOB || length - pos < blob_length) {
                return false;
            }
            value.length = blob_length;
            value.blob = &data[pos];
            pos += blob_length;
            break;
        }
        case ValueTag::Handle:
            if (length - pos < 7) {
                return false;
            }
            value.type_id = data[pos];
            memcpy(&value.handle_index, &data[pos + 1], 2);
            memcpy(&value.handle_generation, &data[pos + 3], 4);
            pos += 7;
            if (value.type_id == 0) {
                return false;
            }
            break;
        case ValueTag::Opaque:
            if (length - pos < 1) {
                return false;
            }
            value.lua_type = data[pos++];
            break;
        default:
            return false;
        }
    }
    return pos == length;
}

// lua_typename() of a value; a missing argument is "no value" (LUA_TNONE)
inline const char *value_type_name(const RpcValue *value)
{
    if (value == nullptr) {
        return "no value";
    }
    switch (value->tag) {
    case ValueTag::Nil:
        return "nil";
    case ValueTag::False:
    case ValueTag::True:
        return "boolean";
    case ValueTag::Integer:
    case ValueTag::Number:
        return "number";
    case ValueTag::String:
        return "string";
    case ValueTag::Blob:
    case ValueTag::Handle:
        return "userdata";   // RpcArgs names them through its TypeNamer
    case ValueTag::Opaque: {
        static const char *const names[] = { "nil", "boolean", "light userdata", "number", "string",
                                              "table", "function", "userdata", "thread" };
        return value->lua_type < sizeof(names) / sizeof(names[0]) ? names[value->lua_type] : "userdata";
    }
    }
    return "userdata";
}

/*
  Argument checks for M7 handlers, with the texts luaL_check*, get_integer()
  and binding_argcheck() give in-process; index is the Lua stack index (the
  self of a method is 1). On failure error_arg() and message() say what the
  M4 raises with luaL_argerror(). The text of a converted argument lives
  until the next check.
 */
// the __name of a blob or handle type (luaL_typeerror() names a userdata by its metatable)
typedef const char *(*TypeNamer)(uint8_t type_id);

class RpcArgs {
public:
    RpcArgs(const RpcValue *values, uint8_t count, TypeNamer namer = nullptr) :
        _values(values),
        _count(count),
        _error_arg(0),
        _namer(namer)
    {
        _message[0] = 0;
        _text[0] = 0;
    }

    // binding_argcheck(): the argument count, self included; the error names the last argument
    bool count_is(uint8_t expected)
    {
        if (_count > expected) {
            return fail(_count, "too many arguments");
        }
        if (_count < expected) {
            return fail(_count, "too few arguments");
        }
        return true;
    }

    // luaL_checknumber(): numbers and numeric strings
    bool number(uint8_t index, float &out)
    {
        bool is_integer = false;
        int32_t as_integer = 0;
        float as_number = 0;
        if (!to_number(value(index), is_integer, as_integer, as_number)) {
            return type_error(index, "number");
        }
        out = is_integer ? float(as_integer) : as_number;
        return true;
    }

    // lua_typename(), or the __name of a blob or handle type
    const char *type_name(uint8_t index) const
    {
        const RpcValue *v = value(index);
        if (v != nullptr && (v->tag == ValueTag::Blob || v->tag == ValueTag::Handle) && _namer != nullptr) {
            const char *name = _namer(v->type_id);
            return name != nullptr ? name : "userdata";
        }
        return value_type_name(v);
    }

    // luaL_checkinteger()
    bool integer(uint8_t index, int32_t &out)
    {
        bool is_integer = false;
        int32_t as_integer = 0;
        float as_number = 0;
        if (!to_number(value(index), is_integer, as_integer, as_number)) {
            return type_error(index, "number");
        }
        if (is_integer) {
            out = as_integer;
            return true;
        }
        // a float converts only when integral and in range, as lua_tointegerx()
        if (!(as_number >= -2147483648.0f && as_number < 2147483648.0f)) {
            return fail(index, "number has no integer representation");
        }
        const float truncated = float(int32_t(as_number));
        if (as_number < truncated || as_number > truncated) {
            return fail(index, "number has no integer representation");
        }
        out = int32_t(as_number);
        return true;
    }

    // get_integer(): luaL_checkinteger() and a range check
    bool integer_in(uint8_t index, int32_t low, int32_t high, int32_t &out)
    {
        if (!integer(index, out)) {
            return false;
        }
        if (out < low || out > high) {
            return fail(index, "out of range");
        }
        return true;
    }

    // luaL_checklstring(): numbers become their Lua text ("%.7g", ".0" added to integral floats)
    bool string(uint8_t index, const char *&out, uint16_t &length)
    {
        const RpcValue *v = value(index);
        if (v != nullptr && v->tag == ValueTag::String) {
            out = v->string;
            length = v->length;
            return true;
        }
        int written = -1;
        if (v != nullptr && v->tag == ValueTag::Integer) {
            written = snprintf(_text, sizeof(_text), "%ld", long(v->integer));
        } else if (v != nullptr && v->tag == ValueTag::Number) {
            written = snprintf(_text, sizeof(_text), "%.7g", double(v->number));
            if (written > 0 && strspn(_text, "-0123456789") == size_t(written) &&
                size_t(written) + 2 < sizeof(_text)) {
                _text[written++] = '.';
                _text[written++] = '0';
                _text[written] = 0;
            }
        }
        if (written < 0) {
            return type_error(index, "string");
        }
        out = _text;
        length = uint16_t(written);
        return true;
    }

    uint8_t error_arg() const { return _error_arg; }
    const char *message() const { return _message; }

private:
    const RpcValue *value(uint8_t index) const
    {
        return (index >= 1 && index <= _count) ? &_values[index - 1] : nullptr;
    }

    static bool only_spaces(const char *text)
    {
        while (*text == ' ' || *text == '\t' || *text == '\n' || *text == '\r') {
            text++;
        }
        return *text == 0;
    }

    // Lua's coercion to a number: numbers, and strings that read fully as a decimal
    // number (hexadecimal strings convert through strtof(), without Lua's integer wrap-around)
    bool to_number(const RpcValue *v, bool &is_integer, int32_t &integer, float &number)
    {
        if (v == nullptr) {
            return false;
        }
        if (v->tag == ValueTag::Integer) {
            is_integer = true;
            integer = v->integer;
            return true;
        }
        if (v->tag == ValueTag::Number) {
            is_integer = false;
            number = v->number;
            return true;
        }
        if (v->tag != ValueTag::String || v->length == 0 || v->length >= sizeof(_text)) {
            return false;
        }
        memcpy(_text, v->string, v->length);
        _text[v->length] = 0;
        char *end = nullptr;
        const long long as_long = strtoll(_text, &end, 10);
        if (end != _text && only_spaces(end) && as_long >= INT32_MIN && as_long <= INT32_MAX) {
            is_integer = true;
            integer = int32_t(as_long);
            return true;
        }
        number = strtof(_text, &end);
        if (end != _text && only_spaces(end)) {
            is_integer = false;
            return true;
        }
        return false;
    }

    bool type_error(uint8_t index, const char *expected)
    {
        snprintf(_message, sizeof(_message), "%s expected, got %s", expected, type_name(index));
        _error_arg = index;
        return false;
    }

    bool fail(uint8_t index, const char *message)
    {
        snprintf(_message, sizeof(_message), "%s", message);
        _error_arg = index;
        return false;
    }

    const RpcValue *_values;
    uint8_t _count;
    uint8_t _error_arg;
    TypeNamer _namer;
    char _message[64];
    char _text[32];
};

enum class MethodKind : uint8_t {
    Call = 0,     // a Reply follows
    Notify = 1,   // no reply
};

/*
  System methods: id, symbol, name (SRPC rows, the contract hash), kind.
  Ids below FIRST_BINDING_METHOD are theirs; generated bindings start there.
  m4:send_text carries the runtime's own texts; global:print is the manual
  binding for print.
  m4:fs_* are the M4's file calls (Lua io, require, dirlist, remove), served
  by AP_ScriptingM4_FileServer with the reach of in-process scripting.
  m4:release names the M4's collected owned objects (AP_ScriptingM4_Objects.h).
 */
#define SCRIPTING_M4_SYSTEM_METHODS(X) \
    X(1, Config, "m4:config", Call) \
    X(2, ListScript, "m4:list_script", Call) \
    X(3, ReadScript, "m4:read_script", Call) \
    X(4, ScriptLoaded, "m4:script_loaded", Notify) \
    X(5, ScriptStopped, "m4:script_stopped", Notify) \
    X(6, ScriptError, "m4:script_error", Notify) \
    X(7, RunStats, "m4:run_stats", Notify) \
    X(8, Started, "m4:started", Notify) \
    X(9, Noop, "m4:noop", Notify) \
    X(10, RuntimeText, "m4:send_text", Call) \
    X(11, Print, "global:print", Call) \
    X(12, Bindings, "m4:bindings", Call) \
    X(13, Other, "m4:other", Notify) \
    X(14, SystemOther, "m4:sysother", Notify) \
    X(15, FsOpen, "m4:fs_open", Call) \
    X(16, FsRead, "m4:fs_read", Call) \
    X(17, FsWrite, "m4:fs_write", Call) \
    X(18, FsSeek, "m4:fs_seek", Call) \
    X(19, FsFlush, "m4:fs_flush", Call) \
    X(20, FsClose, "m4:fs_close", Call) \
    X(21, FsRemove, "m4:fs_remove", Call) \
    X(22, FsDir, "m4:fs_dir", Call) \
    X(23, Release, "m4:release", Notify)

#define SCRIPTING_M4_METHOD_ENUM(id, symbol, name, kind) symbol = id,
enum class SystemMethod : uint16_t { SCRIPTING_M4_SYSTEM_METHODS(SCRIPTING_M4_METHOD_ENUM) };
#undef SCRIPTING_M4_METHOD_ENUM
static const uint16_t FIRST_BINDING_METHOD = 64;

inline const char *method_name(uint16_t method)
{
    switch (method) {
#define SCRIPTING_M4_METHOD_NAME(id, symbol, name, kind) case id: return name;
    SCRIPTING_M4_SYSTEM_METHODS(SCRIPTING_M4_METHOD_NAME)
#undef SCRIPTING_M4_METHOD_NAME
    }
    return "m4:unknown";
}

inline MethodKind method_kind(uint16_t method)
{
    switch (method) {
#define SCRIPTING_M4_METHOD_KIND(id, symbol, name, kind) case id: return MethodKind::kind;
    SCRIPTING_M4_SYSTEM_METHODS(SCRIPTING_M4_METHOD_KIND)
#undef SCRIPTING_M4_METHOD_KIND
    }
    return MethodKind::Call;
}

// FNV-1a, C++11 constexpr (single return statements)
static const uint32_t FNV_OFFSET = 2166136261U;
static const uint32_t FNV_PRIME = 16777619U;
constexpr uint32_t fnv1a_byte(uint32_t hash, uint8_t byte)
{
    return (hash ^ byte) * FNV_PRIME;
}
constexpr uint32_t fnv1a_word(uint32_t word, uint32_t hash)
{
    return fnv1a_byte(fnv1a_byte(fnv1a_byte(fnv1a_byte(hash, uint8_t(word)), uint8_t(word >> 8)),
                                 uint8_t(word >> 16)), uint8_t(word >> 24));
}
constexpr uint32_t fnv1a_string(const char *text, uint32_t hash)
{
    return *text == 0 ? hash : fnv1a_string(text + 1, fnv1a_byte(hash, uint8_t(*text)));
}

// the contract of both images: protocol version, frame layout and the method table; on
// networking builds the Channel's size too
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
#define SCRIPTING_M4_WINDOW_TERM(inner) fnv1a_word(AP_SCRIPTING_M4_CHANNEL_REGION_SIZE, inner)
#else
#define SCRIPTING_M4_WINDOW_TERM(inner) inner
#endif
#define SCRIPTING_M4_HASH_OPEN(id, symbol, name, kind) \
    fnv1a_string(name, fnv1a_word((uint32_t(id) << 1) | uint32_t(MethodKind::kind),
#define SCRIPTING_M4_HASH_CLOSE(id, symbol, name, kind) ))
static constexpr uint32_t SYSTEM_CONTRACT_HASH =
    SCRIPTING_M4_SYSTEM_METHODS(SCRIPTING_M4_HASH_OPEN)
    fnv1a_word(MAX_VALUES, fnv1a_word(MAX_BLOB, fnv1a_word(RPC_PROTOCOL_VERSION,
               fnv1a_word(MAX_FRAME, fnv1a_word(sizeof(CallHeader), fnv1a_word(sizeof(ReplyHeader),
               SCRIPTING_M4_WINDOW_TERM(FNV_OFFSET)))))))
    SCRIPTING_M4_SYSTEM_METHODS(SCRIPTING_M4_HASH_CLOSE);
#undef SCRIPTING_M4_HASH_OPEN
#undef SCRIPTING_M4_HASH_CLOSE
#undef SCRIPTING_M4_WINDOW_TERM

}  // namespace ScriptingM4
