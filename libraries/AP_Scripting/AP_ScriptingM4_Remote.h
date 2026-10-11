#pragma once

/*
  The M7 end of the generated bindings. gen-bindings -r writes
  lua_remote_dispatch.cpp: the in-process binding functions, as
  emit_userdata_method() and emit_field() write them, compiled in this
  namespace against this header in place of Lua. A lua_State is one call
  from the M4: its arguments are the decoded values, its results are
  collected in order and encoded when the function returns. Argument errors
  leave the function with ap_longjmp, as luaL_error() does in-process; the
  generated code checks every argument before it takes a semaphore.
  Userdata arrive as blobs (the described fields of a pure-data type, in
  their wire form) or handles (HandleTable). Scripting host thread only.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>
// ap_setjmp.h includes the board header inside extern "C"; C++ needs it included first
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/ap_setjmp.h>
#include "AP_ScriptingM4_Rpc.h"
#include "AP_ScriptingM4_Handles.h"

namespace ScriptingM4Remote {

typedef int32_t lua_Integer;   // LUA_32BITS, as on the M4
typedef float lua_Number;
struct RemoteCall;
typedef RemoteCall lua_State;
typedef int (*lua_CFunction)(lua_State *L);

// the wire form of a blob: the described fields in the generator's order, little-endian
class WireWriter {
public:
    template <typename T> void write(T value)
    {
        if (!_ok || uint32_t(_length) + sizeof(value) > sizeof(_buf)) {
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
    uint8_t _buf[ScriptingM4::MAX_BLOB];
    uint16_t _length = 0;
    bool _ok = true;
};

class WireReader {
public:
    WireReader(const uint8_t *data, uint16_t length) : _data(data), _length(length) {}
    template <typename T> bool read(T &value)
    {
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

// one generated type, by type id - 1; decode == nullptr: no blob (another kind, or not on this firmware)
struct BlobCodec {
    const char *name;                                       // the __name in type error texts
    uint16_t size;                                          // sizeof the C++ type
    bool (*decode)(WireReader &reader, void *storage);      // constructs the object in storage
    void (*encode)(WireWriter &writer, const void *object);
};

// method flags (gen-bindings -r); the high nibble is the ScriptingM4::QuotaClass
static const uint8_t METHOD_WRITES_SELF = 0x01;   // the reply ends with the updated self blob
static const uint8_t METHOD_QUOTA_FALSE = 0x02;   // over its quota the call returns false, else it is dropped

struct RemoteCall {
    // set once by the server
    ScriptingM4::HandleTable *handles;
    void (*warn)(void *context, uint8_t severity, const char *text);   // set_and_print_new_error_message()
    void *warn_context;
    // set by dispatch()
    const BlobCodec *codecs;
    uint8_t codec_count;
    uint8_t uint32_type;          // the boxed numerics' type ids: 4 and 8 raw bytes
    uint8_t uint64_type;

    // per call (run())
    const ScriptingM4::RpcValue *args;
    uint8_t argc;
    struct Result {
        enum Kind : uint8_t { Value, Object, Handle } kind;
        uint8_t type_id;
        ScriptingM4::RpcValue value;   // Value; a string points into the arena
        void *object;                  // Object: in the arena; Handle: an arena slot holding the pointer
    } results[ScriptingM4::MAX_VALUES];
    uint8_t result_count;
    void *self;                        // argument 1 decoded as a blob: written back
    uint8_t self_type;
    uint16_t arena_used;
    alignas(8) uint8_t arena[2048];    // decoded blobs, results, terminated strings
    ScriptingM4::ReplyStatus status;
    uint8_t error_arg;
    char message[96];
    ap_jmp_buf jump;
};

// runs one binding function on decoded arguments: Ok with its results (and, with
// METHOD_WRITES_SELF, the updated self) in results, or the error (text first)
ScriptingM4::ReplyStatus run(RemoteCall &call, const ScriptingM4::RpcValue *args, uint8_t argc,
                             lua_CFunction function, uint8_t flags,
                             ScriptingM4::RpcWriter &results, uint8_t &error_arg);

// the in-process API the generated functions call (generated_cpp_deps.h, lauxlib, lua_boxed_numerics)
int lua_gettop(lua_State *L);
int binding_argcheck(lua_State *L, int expected_arg_count);
int field_argerror(lua_State *L);
bool userdata_zero_arg_check(lua_State *L);
lua_Integer luaL_checkinteger(lua_State *L, int arg);
lua_Number luaL_checknumber(lua_State *L, int arg);
const char *luaL_checklstring(lua_State *L, int arg, size_t *length);
#ifndef luaL_checkstring
inline const char *luaL_checkstring(lua_State *L, int arg) { return luaL_checklstring(L, arg, nullptr); }
#endif
int lua_toboolean(lua_State *L, int arg);
lua_Integer get_integer(lua_State *L, int arg, lua_Integer min_value, lua_Integer max_value);
int8_t get_int8_t(lua_State *L, int arg);
int16_t get_int16_t(lua_State *L, int arg);
uint8_t get_uint8_t(lua_State *L, int arg);
uint16_t get_uint16_t(lua_State *L, int arg);
float get_number(lua_State *L, int arg, float min_value, float max_value);
uint32_t get_uint32(lua_State *L, int arg, uint32_t min_value, uint32_t max_value);
uint32_t coerce_to_uint32_t(lua_State *L, int arg);
// these leave the binding function (ap_longjmp), as in-process
__attribute__((noreturn)) int luaL_argerror(lua_State *L, int arg, const char *message);
__attribute__((noreturn, format(printf, 2, 3))) int luaL_error(lua_State *L, const char *fmt, ...);
void luaL_checkstack(lua_State *L, int size, const char *message);
void lua_pushboolean(lua_State *L, int value);
void lua_pushinteger(lua_State *L, lua_Integer value);
void lua_pushnumber(lua_State *L, lua_Number value);
void lua_pushstring(lua_State *L, const char *text);
uint32_t *new_uint32_t(lua_State *L);
uint64_t *new_uint64_t(lua_State *L);   // gps:time_epoch_usec()
__attribute__((noreturn)) int not_supported_error(lua_State *L, int arg, const char *name);
namespace lua_scripts {
void set_and_print_new_error_message(uint8_t severity, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
}
#ifndef LUA_MINSTACK
#define LUA_MINSTACK 20
#endif

// storage of the generated check_<type>()/new_<type>()
void *remote_check_blob(lua_State *L, int arg, uint8_t type_id);
void *remote_new_blob(lua_State *L, uint8_t type_id, size_t size);
void **remote_check_handle(lua_State *L, int arg, uint8_t type_id);
void **remote_new_handle(lua_State *L, uint8_t type_id);

// manual bindings ported to the adapter (AP_ScriptingM4_Manual.cpp; gen-bindings -r remote_ready_manual[])
int lua_millis(lua_State *L);
int lua_micros(lua_State *L);

// generated (lua_remote_dispatch.cpp, M4 builds only)
bool method_available(uint16_t method);
uint8_t method_flags(uint16_t method);   // 0 for an unavailable method
ScriptingM4::ReplyStatus dispatch(uint16_t method, const ScriptingM4::RpcValue *args, uint8_t argc,
                                  RemoteCall &call, ScriptingM4::RpcWriter &results, uint8_t &error_arg);
void binding_info(uint8_t *info);        // BINDING_INFO_SIZE bytes (AP_ScriptingM4_Bindings.h)

}  // namespace ScriptingM4Remote
