/*
  The I2C and PWM-source manual bindings of lua_bindings.cpp, ported to the
  remote adapter (gen-bindings -r remote_ready_manual[]). Devices and sources
  are session objects: AP_Scripting's arrays hold them, as in-process, and
  AP_ScriptingM4_Server::end_session() deletes them when the M4 session ends.
  A dot call (i2c.get_device) arrives with a nil self (ROW_DOT_ACCESS).
 */
#include "AP_Scripting_config.h"

#if AP_SCRIPTING_M4_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/GPIO.h>
#include "AP_Scripting.h"
#include "AP_ScriptingM4_Remote.h"

extern const AP_HAL::HAL& hal;

namespace ScriptingM4Remote {

// generated (lua_remote_dispatch.cpp)
AP_HAL::I2CDevice **check_AP_HAL__I2CDevice(lua_State *L, int arg);
AP_HAL::I2CDevice **new_AP_HAL__I2CDevice(lua_State *L);
AP_HAL::PWMSource **new_AP_HAL__PWMSource(lua_State *L);

// the session's I2C bus time (I2CBusBudget)
static I2CBusBudget bus_budget;

void i2c_new_session()
{
    bus_budget.new_session();
}

// run() on dev's bus: within the session's bus time, the semaphore taken for at most
// I2C_SEMAPHORE_WAIT_MS (i2c_bus_call())
template <typename Run> static bool bus_call(AP_HAL::I2CDevice &dev, Run run)
{
    return i2c_bus_call(bus_budget, *dev.get_semaphore(), []() { return AP_HAL::micros64(); }, run);
}

// i2c:get_device(bus, address [, clock [, smbus]]): lua_get_i2c_device()
int lua_get_i2c_device(lua_State *L)
{
    const int args = lua_gettop(L);
    if (args < 3) {
        return luaL_argerror(L, args, "require i2c bus and address");
    }
    if (args > 5) {
        return luaL_argerror(L, args, "too many arguments");
    }

    const lua_Integer bus_in = get_integer(L, 2, 0, 4);
    const uint8_t bus = static_cast<uint8_t>(bus_in);

    const lua_Integer address_in = get_integer(L, 3, 0, 128);
    const uint8_t address = static_cast<uint8_t>(address_in);

    // optional arguments, use the same defaults as the hal get_device function
    uint32_t bus_clock = 400000;
    bool use_smbus = false;

    if (args > 3) {
        // recorded deviation: in-process passes any clock through, and a low one persists on the bus
        bus_clock = i2c_bus_clock_argument(L, 4);

        if (args > 4) {
            use_smbus = static_cast<bool>(lua_toboolean(L, 5));
        }
    }

    auto *scripting = AP::scripting();

    if (scripting->num_i2c_devices >= SCRIPTING_MAX_NUM_I2C_DEVICE) {
        return luaL_argerror(L, 1, "no i2c devices available");
    }

    scripting->_i2c_dev[scripting->num_i2c_devices] = hal.i2c_mgr->get_device_ptr(bus, address, bus_clock, use_smbus);

    if (scripting->_i2c_dev[scripting->num_i2c_devices] == nullptr) {
        return luaL_argerror(L, 1, "i2c device nullptr");
    }

    // a handle refused after the device was registered repeats the registration when the
    // M4 retries; the extra device is deleted at the session end
    *new_AP_HAL__I2CDevice(L) = scripting->_i2c_dev[scripting->num_i2c_devices];

    scripting->num_i2c_devices++;

    return 1;
}

// dev:read_registers(first [, count]): AP_HAL__I2CDevice_read_registers(). With a count the
// in-process binding returns a table of the bytes; this returns them as a string, from which the
// M4's wrapper (wrap_I2CDevice_read_registers) builds that table
int AP_HAL__I2CDevice_read_registers(lua_State *L)
{
    const int args = lua_gettop(L);
    bool multi_register;
    if (args == 2) {
        multi_register = false;
    } else if (args == 3) {
        multi_register = true;
    } else {
        return luaL_argerror(L, args, "expected 1 or 2 arguments");
    }

    AP_HAL::I2CDevice *ud = *check_AP_HAL__I2CDevice(L, 1);

    const uint8_t first_reg = get_uint8_t(L, 2);

    uint8_t recv_length = 1;
    if (multi_register) {
        recv_length = get_uint8_t(L, 3);
    }

    uint8_t data[UINT8_MAX];   // in-process a variable-length array of recv_length

    // byte cap, bus time and bounded take: the host thread never waits past 900 ms (I2C_* in Remote.h)
    if (!i2c_transfer_fits(1, recv_length)) {
        return 0;
    }
    const bool success = bus_call(*ud, [&]() { return ud->read_registers(first_reg, data, recv_length); });

    if (success) {
        if (!multi_register) {
            lua_pushinteger(L, data[0]);
        } else {
            lua_pushlstring(L, reinterpret_cast<const char *>(data), recv_length);
        }
    }
    return success;
}

// dev:transfer(send, count): AP_HAL__I2CDevice_transfer(); the bytes sent are the call's arena copy
int AP_HAL__I2CDevice_transfer(lua_State *L)
{
    binding_argcheck(L, 3);

    AP_HAL::I2CDevice *ud = *check_AP_HAL__I2CDevice(L, 1);

    // Parse string of bytes to send
    size_t send_len;
    const uint8_t *send_data = reinterpret_cast<const uint8_t *>(lua_tolstring(L, 2, &send_len));

    // Parse and setup rx buffer
    uint32_t rx_len = get_uint8_t(L, 3);
    uint8_t rx_data[UINT8_MAX];   // in-process a variable-length array of rx_len

    // Transfer; refused beyond the byte cap, the session's bus time and a busy bus (I2C_* in Remote.h)
    if (!i2c_transfer_fits(send_len, rx_len)) {
        return luaL_argerror(L, 2, "transfer longer than 256 bytes");
    }
    const bool success = bus_call(*ud, [&]() { return ud->transfer(send_data, send_len, rx_data, rx_len); });

    if (!success) {
        return 0;
    }

    // Return a string
    lua_pushlstring(L, reinterpret_cast<const char *>(rx_data), rx_len);
    return 1;
}

// dev:set_retries(count): the generated method with the count capped at I2C_RETRIES_MAX, which
// bounds the attempts of a transfer on the host thread (recorded deviation: in-process allows 20),
// and the bounded semaphore take
int lua_m4_I2CDevice_set_retries(lua_State *L)
{
    binding_argcheck(L, 2);
    AP_HAL::I2CDevice *ud = *check_AP_HAL__I2CDevice(L, 1);
    const uint8_t retries = static_cast<uint8_t>(get_integer(L, 2, 0, I2C_RETRIES_MAX));
    if (!ud->get_semaphore()->take(I2C_SEMAPHORE_WAIT_MS)) {
        return 0;
    }
    ud->set_retries(retries);
    ud->get_semaphore()->give();
    return 0;
}

// dev:write_register(register, value): the generated method within the session's bus time
// (bus_call()); false when refused, as when the semaphore is busy
int lua_m4_I2CDevice_write_register(lua_State *L)
{
    binding_argcheck(L, 3);
    AP_HAL::I2CDevice *ud = *check_AP_HAL__I2CDevice(L, 1);
    const uint8_t reg = get_uint8_t(L, 2);
    const uint8_t value = get_uint8_t(L, 3);
    lua_pushboolean(L, bus_call(*ud, [&]() { return ud->write_register(reg, value); }));
    return 1;
}

// PWMSource(): lua_get_PWMSource()
int lua_get_PWMSource(lua_State *L)
{
    binding_argcheck(L, 0);

    auto *scripting = AP::scripting();

    if (scripting->num_pwm_source >= SCRIPTING_MAX_NUM_PWM_SOURCE) {
        return luaL_argerror(L, 1, "no PWMSources available");
    }

    scripting->_pwm_source[scripting->num_pwm_source] = NEW_NOTHROW AP_HAL::PWMSource;
    if (scripting->_pwm_source[scripting->num_pwm_source] == nullptr) {
        return luaL_argerror(L, 1, "PWMSources device nullptr");
    }

    // a handle refused after the source was registered repeats the registration when the M4
    // retries; the extra source is deleted at the session end
    *new_AP_HAL__PWMSource(L) = scripting->_pwm_source[scripting->num_pwm_source];

    scripting->num_pwm_source++;

    return 1;
}

}  // namespace ScriptingM4Remote

#endif  // AP_SCRIPTING_M4_ENABLED
