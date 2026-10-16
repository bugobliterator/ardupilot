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
  their wire form) or handles (HandleTable). The scripting host thread or its
  slow worker, one at a time (the host waits for the worker).
 */

#include <errno.h>
#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <type_traits>
// ap_setjmp.h includes the board header inside extern "C"; C++ needs it included first
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/ap_setjmp.h>
#include "AP_ScriptingM4_Rpc.h"
#include "AP_ScriptingM4_Files.h"
#include "AP_ScriptingM4_Handles.h"
#include "AP_ScriptingM4_Objects.h"
#include "AP_ScriptingM4_Quota.h"
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
              "the worst I2C transfer fits the serving thread's budget");

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

// M7 I2C bus time an M4 session may use. Each call that uses the bus (transfer, read_registers,
// write_register) is charged the time it held the bus's semaphore; while more than
// I2C_BUS_BUDGET_US were used in the current window the call is refused with its failure value.
// Back-to-back calls of up to 798 ms each would otherwise keep a compass or barometer on a shared
// bus waiting for most of every second. Time used past the budget carries into the next windows,
// so a long call is paid for before the next one runs (recorded deviation: in-process has no bound)
static const uint32_t I2C_BUS_WINDOW_US = 1000000;
static const uint32_t I2C_BUS_BUDGET_US = 100000;
class I2CBusBudget {
public:
    // true if a call may use the bus at now_us
    bool allow(uint64_t now_us)
    {
        roll(now_us);
        return _used_us <= I2C_BUS_BUDGET_US;
    }
    // a call that held the bus for elapsed_us, until now_us
    void charge(uint64_t now_us, uint32_t elapsed_us)
    {
        roll(now_us);
        _used_us += elapsed_us;
    }
    void new_session()
    {
        _window_start_us = 0;
        _used_us = 0;
    }
    uint64_t used_us() const { return _used_us; }

private:
    // each window that ended pays the budget off what was used
    void roll(uint64_t now_us)
    {
        if (now_us < _window_start_us + I2C_BUS_WINDOW_US) {
            return;
        }
        const uint64_t windows = (now_us - _window_start_us) / I2C_BUS_WINDOW_US;
        _window_start_us += windows * I2C_BUS_WINDOW_US;
        const uint64_t paid = windows * I2C_BUS_BUDGET_US;
        _used_us = _used_us > paid ? _used_us - paid : 0;
    }
    uint64_t _window_start_us = 0;
    uint64_t _used_us = 0;
};

// one call of an M4 I2C device that uses the bus: run() with the bus semaphore taken for at most
// I2C_SEMAPHORE_WAIT_MS, its time charged to budget. False without run() when the budget is spent
// or the semaphore is busy, else what run() returns
template <typename Semaphore, typename Clock, typename Run>
bool i2c_bus_call(I2CBusBudget &budget, Semaphore &sem, Clock now_us, Run run)
{
    if (!budget.allow(now_us()) || !sem.take(I2C_SEMAPHORE_WAIT_MS)) {
        return false;
    }
    const uint64_t start_us = now_us();
    const bool done = run();
    const uint64_t end_us = now_us();
    sem.give();
    budget.charge(end_us, uint32_t(end_us - start_us));
    return done;
}

// CAN limits on the M7 host thread (AP_ScriptingM4_ManualCAN.cpp). write_frame() waits in
// CANSensor::write_frame()'s select() for as long as the script asks; the port clamps the wait
// (recorded deviation: in-process passes any timeout through)
static const uint32_t CAN_WRITE_TIMEOUT_MAX_US = 900000;
// the timeout argument of ScriptingCANBuffer:write_frame, clamped
uint32_t can_write_timeout_argument(lua_State *L, int arg);
// scripting CAN buffers an M4 session may hold per sensor: the sensor never frees one, and its
// CAN thread walks the buffer list recursively. Buffers of earlier sessions are handed out again
static const uint8_t CAN_BUFFERS_PER_SESSION = 8;
static const uint32_t CAN_BUFFER_WAIT_MS = 30;

// method flags (gen-bindings -r); the high nibble is the ScriptingM4::QuotaClass
static const uint8_t METHOD_WRITES_SELF = 0x01;   // the reply ends with the updated self blob
static const uint8_t METHOD_QUOTA_FALSE = 0x02;   // over its quota the call returns false, else it is dropped
static const uint8_t METHOD_FAST_LANE = 0x04;     // the host serves it itself (AP_ScriptingM4_ServeWait.h)
static const uint8_t METHOD_PATH_ARGS = 0x08;     // its string arguments are paths, checked as m4:fs_open's

struct RemoteCall {
    // set once by the server
    ScriptingM4::HandleTable *handles;
    ScriptingM4::ObjectPool *objects;    // the owned objects (Owned results)
    ScriptingM4::FileTable *files;    // the M4 session's open files (sock:sendfile)
    void (*warn)(void *context, uint8_t severity, const char *text);   // set_and_print_new_error_message()
    void *warn_context;
    ScriptingM4::Quota *quota;           // the quotas a port charges itself (CAN frames)
    // set by dispatch()
    const BlobCodec *codecs;
    uint8_t codec_count;
    uint8_t uint32_type;          // the boxed numerics' type ids: 4 and 8 raw bytes
    uint8_t uint64_type;

    // per call (run())
    uint32_t now_ms;                   // set by the server before dispatch()
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
uint64_t *check_uint64_t(lua_State *L, int arg);   // DroneCAN_Handle(): a boxed uint64_t argument
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
int lua_m4_I2CDevice_write_register(lua_State *L);
// the session's I2C bus time starts unused (AP_ScriptingM4_Server::reset())
void i2c_new_session();
// AP_ScriptingM4_ManualCAN.cpp
int lua_get_CAN_device(lua_State *L);
int lua_get_CAN_device2(lua_State *L);
int lua_m4_ScriptingCANBuffer_write_frame(lua_State *L);
int DroneCAN_Handle__new_handle(lua_State *L);
int DroneCAN_Handle__broadcast(lua_State *L);
int DroneCAN_Handle__request(lua_State *L);
int DroneCAN_Handle__check_message(lua_State *L);
int lua_m4_DroneCAN_Handle_subscribe(lua_State *L);
// AP_ScriptingM4_ManualSocket.cpp
int lua_get_SocketAPM(lua_State *L);
int SocketAPM_close(lua_State *L);
int SocketAPM_sendfile(lua_State *L);
int SocketAPM_recv(lua_State *L);
int SocketAPM_accept(lua_State *L);
int SocketAPM_ipv4_addr_to_string(lua_State *L);
int SocketAPM_string_to_ipv4_addr(lua_State *L);
int SocketAPM_send_checked(lua_State *L);
int SocketAPM_sendto_checked(lua_State *L);
int lua_m4_SocketAPM_pollin(lua_State *L);
int lua_m4_SocketAPM_pollout(lua_State *L);
int lua_m4_SocketAPM_set_blocking(lua_State *L);
// the session's sockets are deleted and its sendfile hand-overs counted afresh
// (AP_ScriptingM4_Server::end_session())
void sockets_end_session();
// AP_ScriptingM4_ManualCRSF.cpp
int lua_CRSF_new_menu(lua_State *L);
int lua_CRSF_add_parameter(lua_State *L);
int lua_CRSF_add_menu(lua_State *L);
int lua_CRSF_param_data(lua_State *L);
int lua_CRSF_add_root_menu(lua_State *L);
int lua_CRSF_get_menu_event(lua_State *L);
int lua_CRSF_peek_menu_event(lua_State *L);
int lua_CRSF_send_response(lua_State *L);
// the session's scripted CRSF menus go, if it made any (AP_ScriptingM4_Server::end_session())
void crsf_end_session();
// AP_ScriptingM4_ManualMAVLink.cpp
int lua_mavlink_init(lua_State *L);
int lua_mavlink_register_rx_msgid(lua_State *L);
int lua_mavlink_send_chan(lua_State *L);
int lua_mavlink_receive_chan(lua_State *L);
int lua_mavlink_block_command(lua_State *L);
int lua_mission_receive(lua_State *L);
// the session's command blocks are counted afresh and its unread mission items dropped
// (AP_ScriptingM4_Server::end_session())
void mavlink_end_session();

// gcs:run_command_int() refuses with MAV_RESULT_DENIED the commands whose handler holds the
// calling thread for longer than the serving thread's 900 ms (the scheduler semaphore is held too, so
// the main loop stalls with it), or never returns, or takes the M7 down. In-process the scripting
// thread and the main loop wait, or the vehicle crashes (recorded deviation):
// - MAV_CMD_PREFLIGHT_CALIBRATION: a gyro/accel/baro calibration of seconds
//   (GCS_MAVLINK::_handle_command_preflight_calibration, EXPECT_DELAY_MS(30000));
// - MAV_CMD_FLASH_BOOTLOADER with its magic x: hal.util->flash_bootloader(), EXPECT_DELAY_MS(11000);
// - MAV_CMD_DEBUG_TRAP with its magic p1: hal.util->trap();
// - MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN with p1/p2/p3 42/24/71, whatever p4: the crash triggers
//   (lockups, a deadlock, faults, a parameter-header wipe; AP_MAVLINK_FAILURE_CREATION_ENABLED) and
//   DFU boot (p4 99). A plain reboot stays allowed;
// - MAV_CMD_PREFLIGHT_STORAGE with p1 2 or 4 (erase all parameters) and MAV_CMD_STORAGE_FORMAT with
//   p1 = p2 = 1 (format the SD card): destructive, and long on the host thread. The ground
//   station keeps both
static const uint16_t COMMAND_INT_PREFLIGHT_CALIBRATION = 241;   // MAV_CMD_PREFLIGHT_CALIBRATION
static const uint16_t COMMAND_INT_PREFLIGHT_REBOOT_SHUTDOWN = 246;   // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
static const uint16_t COMMAND_INT_FLASH_BOOTLOADER = 42650;      // MAV_CMD_FLASH_BOOTLOADER
static const int32_t COMMAND_INT_FLASH_BOOTLOADER_MAGIC = 290876;   // its x
static const uint16_t COMMAND_INT_DEBUG_TRAP = 42700;            // MAV_CMD_DEBUG_TRAP
static const uint32_t COMMAND_INT_DEBUG_TRAP_MAGIC = 32451;      // its p1
static const uint16_t COMMAND_INT_PREFLIGHT_STORAGE = 245;       // MAV_CMD_PREFLIGHT_STORAGE
static const uint16_t COMMAND_INT_STORAGE_FORMAT = 526;          // MAV_CMD_STORAGE_FORMAT
static const uint8_t COMMAND_INT_PARAM_RESET_FACTORY_DEFAULT = 2;   // PARAM_RESET_FACTORY_DEFAULT
static const uint8_t COMMAND_INT_PARAM_RESET_ALL_DEFAULT = 4;       // PARAM_RESET_ALL_DEFAULT
static const uint8_t COMMAND_INT_DENIED = 2;                      // MAV_RESULT_DENIED
// equal as AP_Math's is_equal() for floats, but within <= FLT_EPSILON: a superset, so a refusal
// never misses a value the handler takes for the match
inline bool command_int_param_is(float param, float value)
{
    return fabsf(param - value) <= FLT_EPSILON;
}
// the handler's uint8_t(param1), without the undefined float-to-integer conversion: its low byte
// for a value in int32_t's range (where a conversion that wraps would land), else 0xFFFF
inline uint16_t command_int_param_byte(float param)
{
    if (!(param > -2147483648.0f && param < 2147483648.0f)) {   // also NaN
        return 0xFFFF;
    }
    return uint16_t(uint32_t(int32_t(param)) & 0xFF);
}
// Packet is mavlink_command_int_t
template <typename Packet> bool command_int_runs_on_the_host(const Packet &packet)
{
    switch (packet.command) {
    case COMMAND_INT_PREFLIGHT_CALIBRATION:
        return false;
    case COMMAND_INT_FLASH_BOOTLOADER:
        return packet.x != COMMAND_INT_FLASH_BOOTLOADER_MAGIC;
    case COMMAND_INT_DEBUG_TRAP:
        // the handler casts p1 to uint32_t: every value in [32451, 32452) is the magic
        return !(packet.param1 >= float(COMMAND_INT_DEBUG_TRAP_MAGIC) &&
                 packet.param1 < float(COMMAND_INT_DEBUG_TRAP_MAGIC + 1));
    case COMMAND_INT_PREFLIGHT_STORAGE: {
        const uint16_t action = command_int_param_byte(packet.param1);
        return action != COMMAND_INT_PARAM_RESET_FACTORY_DEFAULT && action != COMMAND_INT_PARAM_RESET_ALL_DEFAULT;
    }
    case COMMAND_INT_STORAGE_FORMAT:
        return !(command_int_param_is(packet.param1, 1.0f) && command_int_param_is(packet.param2, 1.0f));
    case COMMAND_INT_PREFLIGHT_REBOOT_SHUTDOWN:
        return !(command_int_param_is(packet.param1, 42.0f) && command_int_param_is(packet.param2, 24.0f) &&
                 command_int_param_is(packet.param3, 71.0f));
    default:
        return true;
    }
}

// the result of gcs:run_command_int(): COMMAND_INT_DENIED, with run not called, for a command that
// must not run on the host thread, else run()'s MAV_RESULT. The port calls this, so a test sees
// the refusal and the call it guards
template <typename Packet, typename Run> int command_int_result(const Packet &packet, Run run)
{
    if (!command_int_runs_on_the_host(packet)) {
        return COMMAND_INT_DENIED;
    }
    return run();
}

// the packet of gcs:run_command_int(command, table): values 3..10 are p1..p4, x, y, z and frame,
// each nil where the in-process code leaves the field 0 (the M4 wrapper, m4_wrappers.cpp). False
// for a call without a table (2 values), for which in-process returns nothing. Any other count is
// an argument error: the wrapper never sends it. Packet is mavlink_command_int_t
template <typename Packet> bool command_int_packet(lua_State *L, Packet &packet)
{
    const int args = lua_gettop(L);
    if (args != 2 && args != 10) {
        luaL_argerror(L, args, "M4: malformed run_command_int");
    }
    packet.command = get_uint16_t(L, 2);
    if (args == 2) {
        return false;
    }
    float *const params[] = { &packet.param1, &packet.param2, &packet.param3, &packet.param4 };
    for (uint8_t i = 0; i < 4; i++) {
        if (lua_type(L, 3 + i) > 0) {
            *params[i] = luaL_checknumber(L, 3 + i);
        }
    }
    if (lua_type(L, 7) > 0) {
        packet.x = luaL_checkinteger(L, 7);
    }
    if (lua_type(L, 8) > 0) {
        packet.y = luaL_checkinteger(L, 8);
    }
    if (lua_type(L, 9) > 0) {
        packet.z = luaL_checknumber(L, 9);
    }
    if (lua_type(L, 10) > 0) {
        packet.frame = uint8_t(luaL_checkinteger(L, 10));
    }
    return true;
}

// gps:inject_data() holds the GPS semaphore, which AP_GPS::update() holds while it deletes a
// driver, for the driver call; it waits at most this long for it
static const uint32_t GPS_INJECT_WAIT_MS = 30;
// runs action with sem held: false, action not run, if sem is not taken within wait_ms. The
// action must not leave the binding function (no luaL_* under the semaphore)
template <typename Semaphore, typename Action> bool run_with_bounded_take(Semaphore &sem, uint32_t wait_ms, Action action)
{
    if (!sem.take(wait_ms)) {
        return false;
    }
    action();
    sem.give();
    return true;
}

// AP_ScriptingM4_ManualVehicle.cpp
int lua_GCS_command_int(lua_State *L);
int lua_AP_AHRS_wind_estimate(lua_State *L);
int lua_gps_inject_data(lua_State *L);
int lua_AP_Vehicle_set_target_velocity_NED(lua_State *L);
int SRV_Channels_get_safety_state(lua_State *L);
int lua_range_finder_handle_script_msg(lua_State *L);
int lua_DroneCAN_get_FlexDebug(lua_State *L);

// logger:write() (AP_ScriptingM4_ManualLogger.cpp) checks what it is given before it asks the
// logger for a message type, which takes a type and M7 memory for good; SITL's logger stops the
// host thread on what these refuse (Write_calc_msg_len(): panic; validate_structure(): config_error)

// the first character of text that allowed does not list, or nullptr
inline const char *logger_ids_unsupported(const char *text, const char *allowed)
{
    for (const char *c = text; *c != 0; c++) {
        if (strchr(allowed, *c) == nullptr) {
            return c;
        }
    }
    return nullptr;
}

// the format characters AP_Logger_Write() writes (its switch): the first other one, or nullptr
inline const char *logger_format_unsupported(const char *format)
{
    return logger_ids_unsupported(format, "bhcHCiLefnMBIEQNZ");
}

// the IDs of LogStructure.h's log_Units[] and log_Multipliers[], in their order
static const char LOGGER_UNIT_IDS[] = "-?AadbBkDeEGhiJlLmnoO%SstqrRUuvPwWXyYz#";
static const char LOGGER_MULTIPLIER_IDS[] = "-?210ABCDEFGI!/";

// a message name of letters, digits and '_' (recorded deviation: in-process takes any bytes)
inline bool logger_name_valid(const char *name)
{
    if (*name == 0) {
        return false;
    }
    for (const char *c = name; *c != 0; c++) {
        if (!((*c >= 'A' && *c <= 'Z') || (*c >= 'a' && *c <= 'z') || (*c >= '0' && *c <= '9') || *c == '_')) {
            return false;
        }
    }
    return true;
}

// the end of the label at label: its comma or the terminator
inline const char *logger_label_end(const char *label)
{
    const char *end = strchr(label, ',');
    return end != nullptr ? end : label + strlen(label);
}

// labels as the logger takes them ("TimeUS," prepended): none empty, none twice
inline bool logger_labels_valid(const char *labels)
{
    for (const char *label = labels; ; ) {
        const char *end = logger_label_end(label);
        if (end == label) {
            return false;
        }
        if (*end == 0) {
            return true;
        }
        for (const char *other = end + 1; ; ) {
            const char *other_end = logger_label_end(other);
            if (other_end - other == end - label && memcmp(other, label, size_t(end - label)) == 0) {
                return false;
            }
            if (*other_end == 0) {
                break;
            }
            other = other_end + 1;
        }
        label = end + 1;
    }
}

// logger:log_file_content(path) queues a file while fewer than this many wait: each is an M7
// allocation the logger frees once it has logged the whole file, at about 640 bytes a second.
// The M7 server's own script logging (m4:script_loaded) takes the same bound
static const uint8_t FILE_CONTENT_QUEUE_MAX = ScriptingM4::MAX_SCRIPTS + 8;

// a path the M4's file calls may read (check_script_path(), PathUse::Read) and room in the
// logger's queue (recorded deviation: in-process queues any path, any number)
inline bool file_content_allowed(const char *path, size_t length, uint8_t queued)
{
    return queued < FILE_CONTENT_QUEUE_MAX &&
           ScriptingM4::check_script_path(path, length, ScriptingM4::PathUse::Read) == 0;
}

// M4 scripts may take this many of the logger's message types per boot (one per new logger:write
// name; a type is never freed). The vehicle registers its own lazily (AP::logger().Write("NAME", ...))
// and raises an internal error, which blocks arming, when none is left (recorded deviation)
static const uint8_t LOGGER_SCRIPT_TYPES_MAX = 32;

// a path component as FatFs looks it up: its trailing spaces and dots dropped ("." and ".."
// excepted), ASCII letters compared without case (path_in_directory() takes no other byte)
inline size_t path_component_length(const char *start, size_t length)
{
    if ((length == 1 && start[0] == '.') || (length == 2 && start[0] == '.' && start[1] == '.')) {
        return length;
    }
    while (length > 0 && (start[length - 1] == ' ' || start[length - 1] == '.')) {
        length--;
    }
    return length;
}

inline bool path_components_equal(const char *a, size_t a_length, const char *b, size_t b_length)
{
    if (a_length != b_length) {
        return false;
    }
    for (size_t i = 0; i < a_length; i++) {
        const char upper_a = (a[i] >= 'a' && a[i] <= 'z') ? char(a[i] - 'a' + 'A') : a[i];
        const char upper_b = (b[i] >= 'a' && b[i] <= 'z') ? char(b[i] - 'a' + 'A') : b[i];
        if (upper_a != upper_b) {
            return false;
        }
    }
    return true;
}

// true if path names something inside directory, both read as FatFs reads them below the
// filesystem root: '/' and '\' separate, leading ones optional, "." skipped, ".." taken back,
// path_component_length() names. A path FatFs reads otherwise counts as inside: a ':' drive
// prefix, a control character (which ends it there), or a byte of 0x80 or above (a CP850
// character, which its short-name lookup folds, 0x83 to 'A', 0x93 to 'O', ...). The logger's
// directory holds the log it is writing, which grows faster than FILE logging reads it: queued,
// it would hold the queue's head for the rest of the boot
inline bool path_in_directory(const char *path, const char *directory)
{
    for (const char *c = path; *c != 0; c++) {
        if ((unsigned char)*c < ' ' || (unsigned char)*c >= 0x80 || *c == ':') {
            return true;
        }
    }
    struct Components {
        const char *start[64];
        uint8_t length[64];
        uint8_t count;
        bool read(const char *text)
        {
            count = 0;
            while (*text != 0) {
                const size_t raw_length = strcspn(text, "/\\");
                const char *end = text + raw_length;
                if (raw_length == 2 && text[0] == '.' && text[1] == '.') {
                    if (count > 0) {
                        count--;
                    }
                } else if (raw_length != 0 && !(raw_length == 1 && text[0] == '.')) {
                    const size_t length_here = path_component_length(text, raw_length);
                    if (count >= 64 || length_here > UINT8_MAX) {
                        return false;
                    }
                    start[count] = text;
                    length[count] = uint8_t(length_here);
                    count++;
                }
                text = *end == 0 ? end : end + 1;
            }
            return true;
        }
    };
    Components file, dir;
    if (!file.read(path) || !dir.read(directory)) {
        return true;
    }
    if (file.count <= dir.count) {
        return false;
    }
    for (uint8_t i = 0; i < dir.count; i++) {
        if (!path_components_equal(file.start[i], file.length[i], dir.start[i], dir.length[i])) {
            return false;
        }
    }
    return true;
}

// AP_Filesystem's alias of the logger's directory (AP_FILESYSTEM_MAVLOG_ENABLED)
static const char LOGGER_DIRECTORY_ALIAS[] = "@MAV_LOG";

// path_in_directory(), or a path AP_Filesystem gives to its alias of the directory: one whose
// first component is "@MAV_LOG" (as AP_Filesystem selects it, and also with '\', other case or
// trailing dots, as other filesystems would read it)
inline bool path_in_log_directory(const char *path, const char *log_directory)
{
    const char *first = path + strspn(path, "/\\");
    const size_t first_length = path_component_length(first, strcspn(first, "/\\"));
    if (path_components_equal(first, first_length, LOGGER_DIRECTORY_ALIAS, strlen(LOGGER_DIRECTORY_ALIAS))) {
        return true;
    }
    return path_in_directory(path, log_directory);
}

// coerce_to_uint64_t() of lua_boxed_numerics.cpp: a boxed uint64_t or uint32_t, a non-negative
// integer or number below 2^64; else "Unable to coerce to uint64_t"
uint64_t coerce_to_uint64_t(lua_State *L, int arg);

// AP_ScriptingM4_ManualLogger.cpp
int AP_Logger_Write(lua_State *L);
int lua_m4_AP_Logger_log_file_content(lua_State *L);
// an owned DroneCAN_Handle's release (the in-process __gc): its subscriber goes
void DroneCAN_Handle__release(void *object);
// M7 replacements of generated methods (gen-bindings -r remote_replaced_methods[])
int lua_m4_Parameter_init_by_info(lua_State *L);
int lua_m4_analog_channel(lua_State *L);
// the analog sources earlier sessions were handed may be handed out again (AP_ScriptingM4_Server::reset())
void analog_new_session();
// the CAN buffers earlier sessions were handed may be handed out again (AP_ScriptingM4_Server::reset())
void can_new_session();

// the members of ScriptingCANBuffer a buffer handed out again resets (a friend of the class)
struct CANBufferAccess {
    // empty, unfiltered and of the depth the new script asked for; false if the CAN thread held
    // the buffer past the wait or the new depth could not be allocated (it then holds no frame).
    // A frame the CAN thread accepted under the old filters just before the reset can still be
    // pushed after it: the new session accepts every frame, so it is a new frame, not an old one
    template <typename Buffer> static bool reuse(Buffer &buffer, uint32_t depth)
    {
        if (!buffer.sem.take(CAN_BUFFER_WAIT_MS)) {
            return false;
        }
        buffer.num_filters = 0;
        const bool resized = buffer.buffer.set_size(depth);
        buffer.sem.give();
        return resized;
    }
};

// CAN:get_device*()'s buffer: one an earlier session was handed, else make()'s, reset for this
// session; nullptr past CAN_BUFFERS_PER_SESSION (before make()) or when the reset fails. A
// failed reset still spends the session's slot; the buffer is handed out again next session
template <typename Buffer, typename Make>
Buffer *take_CAN_buffer(ScriptingM4::SessionRecycler<Buffer, CAN_BUFFERS_PER_SESSION> &buffers, Make make,
                        uint32_t depth)
{
    Buffer *buffer = buffers.take_at_most(make);
    if (buffer == nullptr || !CANBufferAccess::reuse(*buffer, depth)) {
        return nullptr;
    }
    return buffer;
}

// buffer:write_frame(frame, timeout_us)'s call, the timeout clamped. Back-to-back calls on a bus
// that takes no frame keep the host thread busy up to 900 ms each (Bench Validation)
template <typename Buffer, typename Frame> int CAN_write_frame(lua_State *L, Buffer &buffer, Frame &frame, int timeout_arg)
{
    lua_pushboolean(L, buffer.write_frame(frame, can_write_timeout_argument(L, timeout_arg)));
    return 1;
}

// DroneCAN_Handle:check_message()'s results are the payload, the node id (integer, 5 bytes), the
// uint64_t timestamp (blob, 12 bytes) and canfd (1 byte): a longer payload cannot cross in one reply
static const uint16_t DRONECAN_PAYLOAD_MAX = ScriptingM4::MAX_STRING - (5 + 12 + 1);
// the next payload of a subscriber's queue, peeked: the caller pops it once copied. One that
// cannot cross is popped and freed, and false returned, so it never blocks the queue
template <typename Payloads, typename Payload> bool dronecan_peek_payload(Payloads &payloads, Payload &payload)
{
    if (!payloads.peek(payload)) {
        return false;
    }
    if (payload.length > DRONECAN_PAYLOAD_MAX) {
        payloads.pop();
        free(payload.data);
        return false;
    }
    return true;
}

// check_message()'s payload, node id and timestamp pushed and the payload popped and freed, as
// in-process (peek first: a payload stays queued if a push faults); false for none (nil)
template <typename Payloads, typename Payload> bool dronecan_push_payload(lua_State *L, Payloads &payloads, Payload &payload)
{
    if (!dronecan_peek_payload(payloads, payload)) {
        return false;
    }
    lua_pushlstring(L, reinterpret_cast<const char *>(payload.data), payload.length);
    payloads.pop();
    free(payload.data);
    lua_pushinteger(L, payload.node_id);
    *new_uint64_t(L) = payload.timestamp;
    return true;
}

// the CAN frames of a DroneCAN transfer of length payload bytes: one for a single-frame transfer,
// else 7 bytes per frame after the 2-byte transfer CRC. The Can quota counts frames, as a long
// payload is up to 145 frames on the bus
inline uint16_t dronecan_transfer_frames(size_t length)
{
    return length <= 7 ? 1 : uint16_t((length + 2 + 6) / 7);
}
// broadcast()/request(): takes the transfer's frames from the Can quota; false over it
bool can_transfer_allowed(lua_State *L, size_t length);
// request()'s node: an integer 0..127, else an argument error (in-process a float conversion)
uint8_t dronecan_target_node(lua_State *L, int arg);

// a port's charge to its quota class (send_chan, logger:write) at the call's time: false over it.
// Generated methods are charged by the server (remote_quotas[]); manual rows carry no class
bool remote_quota_allow(lua_State *L, ScriptingM4::QuotaClass quota, uint16_t cost = 1);

// commands one M4 session may block (mavlink:block_command): each is an M7 allocation the session
// holds, and every COMMAND_LONG and COMMAND_INT the vehicle receives walks the list under its
// semaphore. MAV_CMD_SCRIPTING is never blocked, so the GCS keeps its stop and restart (recorded
// deviations: in-process has no cap and blocks any command). Past either the call returns false
static const uint8_t MAVLINK_BLOCKED_COMMANDS_PER_SESSION = 32;
static const uint16_t MAVLINK_SCRIPTING_COMMAND = 42701;   // MAV_CMD_SCRIPTING
class CommandBlocks {
public:
    bool allow(uint16_t id) const
    {
        return id != MAVLINK_SCRIPTING_COMMAND && _blocked < MAVLINK_BLOCKED_COMMANDS_PER_SESSION;
    }
    void take() { _blocked++; }
    void new_session() { _blocked = 0; }
    uint8_t blocked() const { return _blocked; }

private:
    uint8_t _blocked = 0;
};

// a received mavlink_message_t (mavlink:receive_chan) holds bytes of earlier messages on its
// channel past its payload: the parser zero-fills only up to the message's own length, and the
// signature block stays from the last signed message (a SETUP_SIGNING key, for one). Zeroes the
// payload from len and an unsigned message's signature, so the M4 gets this message only. Zeros
// past len are what MAVLink 2 trimming means, so decoding is unchanged
static const uint8_t MAVLINK_SIGNED_FLAG = 0x01;   // MAVLINK_IFLAG_SIGNED
template <typename Message> void mavlink_clear_stale(Message &msg)
{
    uint8_t *bytes = reinterpret_cast<uint8_t *>(&msg);
    const size_t payload_size = sizeof(msg.payload64);
    if (msg.len < payload_size) {
        memset(bytes + offsetof(Message, payload64) + msg.len, 0, payload_size - msg.len);
    }
    if ((msg.incompat_flags & MAVLINK_SIGNED_FLAG) == 0) {
        memset(bytes + offsetof(Message, signature), 0, sizeof(msg.signature));
    }
}

// mavlink:send_chan sends only while the channel keeps room for one more full MAVLink 2 frame
// after it, so the vehicle's own streams are never left without txspace by a script (recorded
// deviation: in-process sends while the message alone fits)
static const uint16_t MAVLINK_TX_RESERVE = 280;   // 12 framing + 255 payload + 13 signature bytes
inline bool mavlink_send_has_room(uint32_t txspace, uint32_t packet_length)
{
    return txspace >= packet_length + MAVLINK_TX_RESERVE;
}

// empties a queue another thread fills, from its consumer side: pops at most the items it held
// when the drain started, so a producer that keeps pushing cannot hold the host thread. The items
// dropped; a null queue holds none
template <typename Queue> uint32_t drain_queue(Queue *queue)
{
    if (queue == nullptr) {
        return 0;
    }
    uint32_t dropped = 0;
    for (uint32_t left = queue->available(); left > 0 && queue->pop(); left--) {
        dropped++;
    }
    return dropped;
}

// the priority of every M4 DroneCAN transfer: CANARD_TRANSFER_PRIORITY_MEDIUM, below ESC and
// actuator traffic (recorded deviation: in-process sends at CANARD_TRANSFER_PRIORITY_HIGHEST)
static const uint8_t DRONECAN_M4_TRANSFER_PRIORITY = 16;
// the fields of a broadcast's or request's Canard::Transfer that come from the handle and payload
template <typename Transfer, typename Handle>
void dronecan_fill_transfer(Transfer &transfer, Handle &handle, const void *data, size_t length)
{
    transfer.data_type_signature = handle.signature;
    transfer.data_type_id = handle.data_type;
    transfer.inout_transfer_id = &handle.transfer_id;
    transfer.priority = DRONECAN_M4_TRANSFER_PRIORITY;
    transfer.payload = data;
    transfer.payload_len = length;
    transfer.timeout_ms = 10;
}

// M4 DroneCAN subscribers live at once: each holds up to 8 payloads of M7 heap and runs on the
// DroneCAN thread for every matching transfer (recorded deviation: in-process has no cap)
static const uint8_t DRONECAN_SUBSCRIBERS_PER_SESSION = 8;
// the live subscribers of this session's handles; the caller holds the canard rx semaphore
class DroneCANSubscribers {
public:
    // slot's subscriber, if any, replaced by make()'s; false (no subscriber) past the cap
    template <typename Subscriber, typename Make> bool replace(Subscriber *&slot, Make make)
    {
        drop(slot);
        if (_live >= DRONECAN_SUBSCRIBERS_PER_SESSION) {
            return false;
        }
        slot = make();
        if (slot == nullptr) {
            return false;
        }
        _live++;
        return true;
    }
    template <typename Subscriber> void drop(Subscriber *&slot)
    {
        if (slot != nullptr) {
            delete slot;
            slot = nullptr;
            _live--;
        }
    }
    // the release of every handle ran at the session end; a new session starts from none
    void new_session() { _live = 0; }
    uint8_t live() const { return _live; }

private:
    uint8_t _live = 0;
};

// Sockets (AP_ScriptingM4_ManualSocket.cpp). The M7 issues every socket call non-blocking, so
// its serving thread never waits on the network: a script's blocking socket (the in-process default,
// or set_blocking(true)) is the M4's, which tries a connect or send again until it completes
// (m4/m4_socket_wait.h). pollin()/pollout() wait in select() for at most this long (recorded
// deviation: in-process waits as long as the script asks)
static const uint32_t SOCKET_POLL_TIMEOUT_MAX_MS = 900;
// the timeout argument of SocketAPM:pollin/pollout, clamped
uint32_t socket_poll_timeout_argument(lua_State *L, int arg);
// sock:sendfile() hand-overs per M4 session: each leaves an M7 descriptor and a socket with
// AP_Networking until the file is sent, past the session's end (recorded deviation)
static const uint8_t SOCKET_SENDFILES_PER_SESSION = 2;

// whether Socket() may make a socket: AP_Networking runs (NET_ENABLE, and a backend that
// initialised) and its stack's core is made (lwIP: tcpip_init() ran on the networking thread, which
// AP_Networking reports healthy before). Without a stack lwIP's first call locks a null mutex: a
// fault of the M7. Socket() gives nil while it is not (recorded deviation: in-process faults)
inline bool socket_stack_ready(bool networking_healthy, bool stack_core_made)
{
    return networking_healthy && stack_core_made;
}

// a socket the M7 hands out (Socket(), sock:accept()), made non-blocking; nullptr stays nullptr
template <typename Socket> Socket *socket_nonblocking(Socket *sock)
{
    if (sock != nullptr) {
        (void)sock->set_blocking(false);
    }
    return sock;
}

// sock:set_blocking(blocking)'s call: the socket stays non-blocking whatever the script asks
// (the M4 keeps the script's mode); true if the socket is open, as in-process
template <typename Socket> int socket_set_blocking(lua_State *L, Socket &sock)
{
    (void)lua_toboolean(L, 2);
    lua_pushboolean(L, sock.set_blocking(false));
    return 1;
}

// sock:pollin(timeout_ms)/pollout(timeout_ms)'s call, the timeout clamped
template <typename Socket> int socket_poll(lua_State *L, Socket &sock, bool out)
{
    const uint32_t timeout_ms = socket_poll_timeout_argument(L, 2);
    lua_pushboolean(L, out ? sock.pollout(timeout_ms) : sock.pollin(timeout_ms));
    return 1;
}

// a send's results: the count, as in-process, and whether the call would have blocked, which the
// M4 drops after it tried again (m4_wrappers.cpp). errno is one for all threads on ChibiOS: another
// thread's socket call in between can change it, so a blocking send may end early or try again
// for up to the idle limit; Socket.cpp's own errno checks share this
inline int socket_send_results(lua_State *L, int32_t count)
{
    lua_pushinteger(L, count);
    lua_pushboolean(L, count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK));
    return 2;
}

// sock:send(data, len)'s call, the length bounded by the string: in-process SocketAPM reads len
// bytes from the Lua string, which here would read past the call's copy into M7 memory
template <typename Socket> int socket_send(lua_State *L, Socket &sock)
{
    size_t length = 0;
    const char *data = luaL_checklstring(L, 2, &length);
    const uint32_t requested = coerce_to_uint32_t(L, 3);
    errno = 0;   // a failure that sets none (a closed socket) is no earlier call's EAGAIN
    const auto count = sock.send(data, requested < length ? size_t(requested) : length);
    return socket_send_results(L, int32_t(count));
}

// sock:sendto(data, len, addr, port)'s call, the length bounded as send()'s
template <typename Socket> int socket_sendto(lua_State *L, Socket &sock)
{
    size_t length = 0;
    const char *data = luaL_checklstring(L, 2, &length);
    const uint32_t requested = coerce_to_uint32_t(L, 3);
    const uint32_t address = coerce_to_uint32_t(L, 4);
    const uint16_t port = get_uint16_t(L, 5);
    errno = 0;
    const auto count = sock.sendto(data, requested < length ? size_t(requested) : length, address, port);
    return socket_send_results(L, int32_t(count));
}

// the session's sock:sendfile() hand-overs
class SocketSendfiles {
public:
    bool available() const { return _handed_over < SOCKET_SENDFILES_PER_SESSION; }
    void take() { _handed_over++; }
    void new_session() { _handed_over = 0; }

private:
    uint8_t _handed_over = 0;
};

// AP_Networking::sendfile() waits for the networking semaphore, which its sendfile thread holds
// over a pass of file reads (up to 32 KB each, on the SD card the logger shares): the port takes
// it first for at most this long (recorded deviation: false if it is busy, in-process waits)
static const uint32_t SOCKET_SENDFILE_WAIT_MS = 30;

// sock:sendfile()'s hand-over of descriptor fd: hand_over(fd) only while the session has
// hand-overs left and the semaphore was taken within SOCKET_SENDFILE_WAIT_MS; hand_over takes the
// held, recursive semaphore again at once. One refused spends none. True if handed over
template <typename Semaphore, typename HandOver>
bool socket_sendfile(SocketSendfiles &sendfiles, int fd, Semaphore &sem, HandOver hand_over)
{
    if (fd == -1 || !sendfiles.available() || !sem.take(SOCKET_SENDFILE_WAIT_MS)) {
        return false;
    }
    const bool handed_over = hand_over(fd);
    sem.give();
    if (!handed_over) {
        return false;
    }
    sendfiles.take();
    return true;
}

// what a socket is at the session end, as its descriptor reports it (SO_TYPE, SO_ACCEPTCONN)
enum class SocketKind : uint8_t {
    Stream,      // a TCP socket that is not listening: connected, connecting or never connected
    Listening,   // a TCP socket after listen()
    Other,       // a datagram socket, or one whose kind could not be read
};

// only a non-listening stream socket is reset at the session end. lwIP closes a listening socket
// without waiting, and its close path reads unsent/unacked from the smaller listen PCB, so a
// linger of 0 there would abort a listen PCB; a datagram socket has nothing to linger for
inline bool socket_end_resets(SocketKind kind)
{
    return kind == SocketKind::Stream;
}

// a session's socket at the session end: a stream socket set to close abortively (reset(fd):
// SO_LINGER {1, 0}), then every socket deleted. A graceful close of a stream socket can hold the
// host thread while the network stack has no segment for the FIN (lwIP: up to 20 s and more); the
// dying session's unsent data is dropped (recorded deviation). A script's own sock:close() stays
// graceful, as in-process
template <typename Socket, typename KindOf, typename Reset> void socket_end(Socket *&slot, KindOf kind_of, Reset reset)
{
    if (slot == nullptr) {
        return;
    }
    const int fd = slot->get_read_fd();
    if (fd != -1 && socket_end_resets(kind_of(fd))) {
        reset(fd);
    }
    delete slot;
    slot = nullptr;
}

// CRSF scripted menus. AP_CRSF_Telem keeps a menu's parameters in an array of
// MAX_SCRIPTED_MENU_SIZE ids and numbers menus up to MAX_SCRIPTED_MENUS, but add_parameter()
// and add_menu() check neither: 256 add_parameter() calls wrap the uint8_t num_params and write
// params[-1]. The M7 stops at the limits (ManualCRSF.cpp static_asserts them) before AP code
// runs: past one the call returns its refusal value (recorded deviation)
static const uint8_t CRSF_MENUS_PER_SESSION = 10;
static const uint8_t CRSF_PARAMETERS_PER_MENU = 20;   // a sub-menu is one of its parent's
// AP_CRSF_Telem stores names in 17 bytes and copies 16 from the string it is given, so a longer
// name is stored unterminated: the M7 gives it 15 characters and a NUL
static const uint8_t CRSF_NAME_LENGTH = 15;

inline void crsf_cut_name(char (&cut)[CRSF_NAME_LENGTH + 1], const char *name)
{
    strncpy(cut, name, CRSF_NAME_LENGTH);
    cut[CRSF_NAME_LENGTH] = 0;
}

// AP::crsf_telem() for a call: null while armed if CRSF telemetry was never made. The in-process
// bindings dereference it; from an M4 script that would fault the M7
template <typename Telem> Telem *crsf_required(Telem *telem, lua_State *L)
{
    if (telem == nullptr) {
        luaL_error(L, "internal error: %s is null", "crsf");
    }
    return telem;
}

// the menus this session made, roots and sub-menus: they live until the session ends
class CRSFMenuBudget {
public:
    bool available() const { return _made < CRSF_MENUS_PER_SESSION; }
    void take() { _made++; }
    bool any() const { return _made != 0; }
    void new_session() { _made = 0; }

private:
    uint8_t _made = 0;
};

// a menu for the script, a root or a sub-menu (parent_has_room: the parent can take the
// parameter that stands for it). make(name) makes the script's object and the menu, with the name
// cut, and returns whether it did. Nothing is made past the session's menus or in a full parent
template <typename MakeMenu>
bool crsf_add_menu(CRSFMenuBudget &budget, bool parent_has_room, const char *name, MakeMenu make)
{
    if (!budget.available() || !parent_has_room) {
        return false;
    }
    char cut[CRSF_NAME_LENGTH + 1];
    crsf_cut_name(cut, name);
    if (!make(cut)) {
        return false;
    }
    budget.take();
    return true;
}

// room for another parameter in a menu holding num_params
inline bool crsf_menu_has_room(uint8_t num_params)
{
    return num_params < CRSF_PARAMETERS_PER_MENU;
}

// a parameter for the script in a menu holding num_params: make() makes the object and the
// parameter and returns whether it did. Nothing is made in a full menu
template <typename MakeParameter> bool crsf_add_parameter(uint8_t num_params, MakeParameter make)
{
    return crsf_menu_has_room(num_params) && make();
}

// generated (lua_remote_dispatch.cpp, M4 builds only)
bool method_available(uint16_t method);
uint8_t method_flags(uint16_t method);   // 0 for an unavailable method
ScriptingM4::ReplyStatus dispatch(uint16_t method, const ScriptingM4::RpcValue *args, uint8_t argc,
                                  RemoteCall &call, ScriptingM4::RpcWriter &results, uint8_t &error_arg);
void binding_info(uint8_t *info);        // BINDING_INFO_SIZE bytes (AP_ScriptingM4_Bindings.h)

}  // namespace ScriptingM4Remote
