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
#include <type_traits>
// ap_setjmp.h includes the board header inside extern "C"; C++ needs it included first
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/ap_setjmp.h>
#include "AP_ScriptingM4_Rpc.h"
#include "AP_ScriptingM4_Handles.h"
#include "AP_ScriptingM4_Objects.h"
#include "AP_ScriptingM4_Wire.h"

namespace ScriptingM4Remote {

// bytes per reply of a manual function that reads "up to" a count the script gives
// (readstring, recv): what fits one reply frame with the values that follow it
static const uint16_t IO_CHUNK = 768;
static_assert(IO_CHUNK + 3 + 8 + 5 <= ScriptingM4::MAX_FRAME - sizeof(ScriptingM4::ReplyHeader),
              "a chunk, a uint32_t blob and an integer fit one reply");

typedef int32_t lua_Integer;   // LUA_32BITS, as on the M4
typedef float lua_Number;
struct RemoteCall;
typedef RemoteCall lua_State;
typedef int (*lua_CFunction)(lua_State *L);

// the blob wire form (AP_ScriptingM4_Wire.h); the generated codecs name them unqualified
using ScriptingM4::WireReader;
using ScriptingM4::WireWriter;

// one generated type, by type id - 1; decode == nullptr: no blob (another kind, or not on this firmware)
struct BlobCodec {
    const char *name;                                       // the __name in type error texts
    uint16_t size;                                          // sizeof the C++ type
    bool (*decode)(WireReader &reader, void *storage);      // constructs the object in storage
    void (*encode)(WireWriter &writer, const void *object);
};

// I2C limits on the M7 host thread (AP_ScriptingM4_ManualI2C.cpp, and the generated methods of
// AP_HAL::I2CDevice, which take the semaphore for at most I2C_SEMAPHORE_WAIT_MS). ChibiOS times
// a transfer out after 1 + 2 * (8e6 / busclock * bytes) / 1000 ms per attempt and makes
// retries + 1 attempts, clearing the bus between them (20 x 10 us, under 1 ms). At the clock floor
// 256 bytes time out after 41 ms; with at most 18 retries (19 attempts) the nominal worst case is
// 30 + 19 * (41 + 1) = 828 ms, below the 850 ms budget of the 900 ms rule. In-process passes any
// clock, length and retry count through (recorded deviation). A request at the floor still sets
// the shared bus to 100 kHz until reboot, as in-process: the floor does not prevent that
static const uint32_t I2C_CLOCK_MIN_HZ = 100000;
static const uint32_t I2C_CLOCK_MAX_HZ = 1000000;
static const uint32_t I2C_TRANSFER_BYTES_MAX = 256;
static const uint32_t I2C_SEMAPHORE_WAIT_MS = 30;
static const uint32_t I2C_RETRIES_MAX = 18;
static const uint32_t I2C_BUS_CLEAR_MS = 1;
static_assert(I2C_SEMAPHORE_WAIT_MS + (I2C_RETRIES_MAX + 1) *
              (1 + 2 * ((8 * 1000000UL / I2C_CLOCK_MIN_HZ) * I2C_TRANSFER_BYTES_MAX) / 1000 + I2C_BUS_CLEAR_MS) <= 850,
              "the worst I2C transfer fits the host thread's budget");

inline uint32_t i2c_clamp_bus_clock(uint32_t hertz)
{
    return hertz < I2C_CLOCK_MIN_HZ ? I2C_CLOCK_MIN_HZ : (hertz > I2C_CLOCK_MAX_HZ ? I2C_CLOCK_MAX_HZ : hertz);
}

// the bus clock argument of i2c:get_device, clamped
uint32_t i2c_bus_clock_argument(lua_State *L, int arg);

inline bool i2c_transfer_fits(size_t send_bytes, size_t receive_bytes)
{
    return send_bytes <= I2C_TRANSFER_BYTES_MAX && receive_bytes <= I2C_TRANSFER_BYTES_MAX &&
           send_bytes + receive_bytes <= I2C_TRANSFER_BYTES_MAX;
}

// method flags (gen-bindings -r); the high nibble is the ScriptingM4::QuotaClass
static const uint8_t METHOD_WRITES_SELF = 0x01;   // the reply ends with the updated self blob
static const uint8_t METHOD_QUOTA_FALSE = 0x02;   // over its quota the call returns false, else it is dropped

struct RemoteCall {
    // set once by the server
    ScriptingM4::HandleTable *handles;
    ScriptingM4::ObjectPool *objects;    // the owned objects (Owned results)
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
        enum Kind : uint8_t { Value, Object, Handle, Owned };
        Kind kind;
        uint8_t type_id;
        ScriptingM4::RpcValue value;   // Value; a string points into the arena
        void *object;                  // Object: in the arena; Handle: an arena slot holding the pointer;
                                       // Owned: the pool slot
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
void lua_pushlstring(lua_State *L, const char *text, size_t length);   // binary, NULs included
int lua_type(lua_State *L, int arg);   // lua.h's LUA_TNONE (-1), LUA_TNIL (0), ... of an argument
const char *lua_tolstring(lua_State *L, int arg, size_t *length);   // nullptr for a non-string
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
// storage of the generated check_<type>()/new_<type>() of an owned type: the object itself, in
// the pool; a failed call destroys the objects it made (run())
void *remote_check_owned(lua_State *L, int arg, uint8_t type_id);
void *remote_new_owned(lua_State *L, uint8_t type_id, ScriptingM4::ObjectPool::Destroy destroy);

// manual bindings ported to the adapter (AP_ScriptingM4_Manual.cpp; gen-bindings -r remote_ready_manual[])
int lua_millis(lua_State *L);
int lua_micros(lua_State *L);
int lua_new_Parameter(lua_State *L);
// AP_ScriptingM4_ManualSerial.cpp
int lua_serial_find_serial(lua_State *L);
int lua_serial_find_simulated_device(lua_State *L);
int lua_serial_begin(lua_State *L);
int lua_serial_writestring(lua_State *L);
int lua_serial_readstring(lua_State *L);
// AP_ScriptingM4_ManualI2C.cpp
int lua_get_i2c_device(lua_State *L);
int AP_HAL__I2CDevice_read_registers(lua_State *L);
int AP_HAL__I2CDevice_transfer(lua_State *L);
int lua_get_PWMSource(lua_State *L);
int lua_m4_I2CDevice_set_retries(lua_State *L);
// M7 replacements of generated methods (gen-bindings -r remote_replaced_methods[])
int lua_m4_Parameter_init_by_info(lua_State *L);
int lua_m4_analog_channel(lua_State *L);
// the analog sources earlier sessions were handed may be handed out again (AP_ScriptingM4_Server::reset())
void analog_new_session();

// generated (lua_remote_dispatch.cpp, M4 builds only)
bool method_available(uint16_t method);
uint8_t method_flags(uint16_t method);   // 0 for an unavailable method
ScriptingM4::ReplyStatus dispatch(uint16_t method, const ScriptingM4::RpcValue *args, uint8_t argc,
                                  RemoteCall &call, ScriptingM4::RpcWriter &results, uint8_t &error_arg);
void binding_info(uint8_t *info);        // BINDING_INFO_SIZE bytes (AP_ScriptingM4_Bindings.h)

}  // namespace ScriptingM4Remote
