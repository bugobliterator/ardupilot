/*
  The singleton, ap_object and global manual bindings of lua_bindings.cpp that are no IO object,
  ported to the remote adapter (gen-bindings -r remote_ready_manual[]). A singleton's self arrives
  as nil; the generated checkers (check_GCS(), ...) find the singleton, as in-process.

  Waits on the host thread (the 900 ms rule): the scheduler semaphore (run_command_int,
  set_target_velocity_NED), held by the main loop for at most one loop, the wait every generated
  scheduler-semaphore method takes; the AHRS semaphore (wind_estimate), held for one EKF update;
  the GPS semaphore (inject_data), held by AP_GPS::update(), taken for at most GPS_INJECT_WAIT_MS.
  run_command_int refuses the commands whose handler holds the thread for seconds or crashes the
  vehicle (command_int_runs_on_the_host()).
 */
#include "AP_Scripting_config.h"

#if AP_SCRIPTING_M4_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_RangeFinder/AP_RangeFinder_Backend.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_ScriptingM4_Remote.h"

extern const AP_HAL::HAL& hal;

namespace ScriptingM4Remote {

// lua.h's LUA_TUSERDATA, as the adapter's lua_type() reports a blob or a handle
static const int LUA_TYPE_USERDATA = 7;

#if (HAL_GCS_ENABLED && !defined(HAL_BUILD_AP_PERIPH))
// generated (lua_remote_dispatch.cpp)
GCS *check_GCS(lua_State *L);

static_assert(COMMAND_INT_PREFLIGHT_CALIBRATION == MAV_CMD_PREFLIGHT_CALIBRATION, "the refused command");
static_assert(COMMAND_INT_PREFLIGHT_REBOOT_SHUTDOWN == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, "the refused command");
static_assert(COMMAND_INT_FLASH_BOOTLOADER == MAV_CMD_FLASH_BOOTLOADER, "the refused command");
static_assert(COMMAND_INT_PREFLIGHT_STORAGE == MAV_CMD_PREFLIGHT_STORAGE, "the refused command");
static_assert(COMMAND_INT_STORAGE_FORMAT == MAV_CMD_STORAGE_FORMAT, "the refused command");
static_assert(COMMAND_INT_PARAM_RESET_FACTORY_DEFAULT == PARAM_RESET_FACTORY_DEFAULT, "the refused action");
static_assert(COMMAND_INT_PARAM_RESET_ALL_DEFAULT == PARAM_RESET_ALL_DEFAULT, "the refused action");
static_assert(COMMAND_INT_DEBUG_TRAP == MAV_CMD_DEBUG_TRAP, "the refused command");
static_assert(COMMAND_INT_DENIED == MAV_RESULT_DENIED, "the refusal's result");

// gcs:run_command_int(command, table): lua_GCS_command_int(). The M4 wrapper sends the table's
// fields as values (command_int_packet()); the command runs on GCS channel 0 with the scheduler
// lock, as in-process
int lua_GCS_command_int(lua_State *L)
{
    GCS *_gcs = check_GCS(L);

    mavlink_command_int_t pkt {};
    if (!command_int_packet(L, pkt)) {
        // must have parameter table
        return 0;
    }
    // M4: a command whose handler holds the calling thread for seconds, crashes the vehicle or
    // destroys its storage is refused (command_int_result()); the others run with the scheduler lock
    const int result = command_int_result(pkt, [&]() {
        WITH_SEMAPHORE(AP::scheduler().get_semaphore());
        return int(_gcs->lua_command_int_packet(pkt));
    });

    // Return the resulting MAV_RESULT
    lua_pushinteger(L, result);

    return 1;
}
#endif

#if HAL_ENABLE_DRONECAN_DRIVERS
// DroneCAN_get_FlexDebug(bus, node, id, last_us): lua_DroneCAN_get_FlexDebug(). The payload is at
// most 255 bytes (dronecan_protocol_FlexDebug's u8), within one reply
int lua_DroneCAN_get_FlexDebug(lua_State *L)
{
    binding_argcheck(L, 4);

    const uint8_t bus = get_uint8_t(L, 1);
    const uint8_t node_id = get_uint8_t(L, 2);
    const uint16_t msg_id = get_uint16_t(L, 3);
    uint32_t tstamp_us = get_uint32(L, 4, 0, UINT32_MAX);

    const auto *dc = AP_DroneCAN::get_dronecan(bus);
    if (dc == nullptr) {
        return 0;
    }
    dronecan_protocol_FlexDebug msg;

    if (!dc->get_FlexDebug(node_id, msg_id, tstamp_us, msg)) {
        return 0;
    }

    *new_uint32_t(L) = tstamp_us;
    lua_pushlstring(L, (const char *)msg.u8.data, msg.u8.len);

    return 2;
}
#endif // HAL_ENABLE_DRONECAN_DRIVERS

#if AP_GPS_ENABLED
// gps:inject_data(data): lua_gps_inject_data(). The data is the call's copy, at most MAX_STRING
// bytes (in-process up to UINT16_MAX: a longer injection takes more calls)
int lua_gps_inject_data(lua_State *L)
{
    binding_argcheck(L, 2);
    // M4: no luaL_checkudata(L, 1, "gps"): a singleton's self arrives as nil

    size_t len = 0;
    const char *data = luaL_checklstring(L, 2, &len);

    if (len > 0 && len <= UINT16_MAX)
    {
        // M4: the GPS semaphore, which AP_GPS::update() holds while it deletes a driver that timed
        // out, so the main loop cannot free it mid-call. If it is not taken in time the data is
        // dropped (the stream is for a receiver that tolerates loss; the call returns nothing)
        (void)run_with_bounded_take(AP::gps().get_semaphore(), GPS_INJECT_WAIT_MS, [&]() {
            AP::gps().inject_data((const uint8_t *)data, (uint16_t)len);
        });
    }

    return 0;
}
#endif  // AP_GPS_ENABLED

#if AP_AHRS_ENABLED
// generated (lua_remote_dispatch.cpp)
AP_AHRS *check_AP_AHRS(lua_State *L);
Vector3f *new_Vector3f(lua_State *L);

// ahrs:wind_estimate(): lua_AP_AHRS_wind_estimate(), the deprecated binding; its warning is
// printed once per M7 boot, as once per boot in-process
int lua_AP_AHRS_wind_estimate(lua_State *L)
{
    binding_argcheck(L, 1);
    AP_AHRS *ahrs = check_AP_AHRS(L);

    static bool warned;
    if (!warned) {
        lua_scripts::set_and_print_new_error_message(MAV_SEVERITY_WARNING, "ahrs:wind_estimate Use get_wind");
        warned = true;
    }

    Vector3f wind;
    {
        WITH_SEMAPHORE(ahrs->get_semaphore());
        IGNORE_RETURN(ahrs->get_wind(wind));
    }

    *new_Vector3f(L) = wind;

    return 1;
}
#endif  // AP_AHRS_ENABLED

#if AP_SCRIPTING_BINDING_VEHICLE_ENABLED
// generated (lua_remote_dispatch.cpp)
AP_Vehicle *check_AP_Vehicle(lua_State *L);
Vector3f *check_Vector3f(lua_State *L, int arg);

// vehicle:set_target_velocity_NED(vel, [yaw_to_target]): lua_AP_Vehicle_set_target_velocity_NED()
int lua_AP_Vehicle_set_target_velocity_NED(lua_State *L)
{
    const int args = lua_gettop(L);

    if (args > 3) {
        return luaL_argerror(L, args, "too many arguments");
    } else if (args < 2) {
        return luaL_argerror(L, args, "too few arguments");
    }

    AP_Vehicle * ud = check_AP_Vehicle(L);
    Vector3f & data_2 = *check_Vector3f(L, 2);

    bool yaw_to_target = false;

    if (args == 3) {
        yaw_to_target = static_cast<bool>(lua_toboolean(L, 3));
    }
    // M4: WITH_SEMAPHORE in place of take_blocking()/give(): the same wait, released on every path
    WITH_SEMAPHORE(AP::scheduler().get_semaphore());
    const bool data = static_cast<bool>(ud->set_target_velocity_NED(
            data_2,
            yaw_to_target));

    lua_pushboolean(L, data);
    return 1;
}
#endif // AP_SCRIPTING_BINDING_VEHICLE_ENABLED

// SRV_Channels:get_safety_state(): SRV_Channels_get_safety_state()
int SRV_Channels_get_safety_state(lua_State *L)
{
    binding_argcheck(L, 1);
    const bool data = hal.util->safety_switch_state() !=  AP_HAL::Util::SAFETY_ARMED;
    lua_pushboolean(L, data);
    return 1;
}

#if AP_RANGEFINDER_ENABLED
// generated (lua_remote_dispatch.cpp)
AP_RangeFinder_Backend **check_AP_RangeFinder_Backend(lua_State *L, int arg);
RangeFinder::RangeFinder_State *check_RangeFinder__RangeFinder_State(lua_State *L, int arg);

// backend:handle_script_msg(distance or state): lua_range_finder_handle_script_msg()
int lua_range_finder_handle_script_msg(lua_State *L)
{
    // Arg 1 => self (an instance of rangefinder_backend)
    // Arg 2 => a float distance or a RangeFinder_State user data
    binding_argcheck(L, 2);

    // check_AP_RangeFinder_Backend aborts if not found. No need to check for null
    AP_RangeFinder_Backend * ud = *check_AP_RangeFinder_Backend(L, 1);

    bool result = false;

    // M4: luaL_testudata(L, 2, "RangeFinder_State"): a userdata argument is checked as the state,
    // so another userdata's text names RangeFinder_State, not number (recorded)
    if (lua_type(L, 2) == LUA_TYPE_USERDATA) {
        result = ud->handle_script_msg(*check_RangeFinder__RangeFinder_State(L, 2));

    } else {
        // Otherwise assume the argument is a number and set the measurement.
        result = ud->handle_script_msg(luaL_checknumber(L, 2));
    }

    lua_pushboolean(L, result);
    return 1;
}
#endif  // AP_RANGEFINDER_ENABLED

}  // namespace ScriptingM4Remote

#endif  // AP_SCRIPTING_M4_ENABLED
