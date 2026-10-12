/*
  The serial-port manual bindings of lua_bindings.cpp, ported to the remote
  adapter (gen-bindings -r remote_ready_manual[]). A port is an owned object:
  an AP_Scripting_SerialAccess in the M7's object pool. Dot calls
  (serial.find_serial(0)) arrive with a nil self, as
  fix_dot_access_never_add_another_call() leaves them in-process (ROW_DOT_ACCESS).
 */
#include "AP_Scripting_config.h"

#if AP_SCRIPTING_M4_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include "AP_Scripting.h"
#include "AP_Scripting_SerialAccess.h"
#include "AP_ScriptingM4_Remote.h"

namespace ScriptingM4Remote {

// generated (lua_remote_dispatch.cpp)
AP_Scripting_SerialAccess *check_AP_Scripting_SerialAccess(lua_State *L, int arg);
AP_Scripting_SerialAccess *new_AP_Scripting_SerialAccess(lua_State *L);

#if AP_SERIALMANAGER_ENABLED
// serial:find_serial(instance): lua_serial_find_serial()
int lua_serial_find_serial(lua_State *L)
{
    binding_argcheck(L, 2);

    uint8_t instance = get_uint8_t(L, 2);

    AP_SerialManager *mgr = &AP::serialmanager();
    AP_HAL::UARTDriver *driver_stream = mgr->find_serial(
        AP_SerialManager::SerialProtocol_Scripting, instance);

    if (driver_stream == nullptr) { // not found
        return 0;
    }

    AP_Scripting_SerialAccess *port = new_AP_Scripting_SerialAccess(L);
    port->stream = driver_stream;
#if AP_SCRIPTING_SERIALDEVICE_ENABLED
    port->is_device_port = false;
#endif

    return 1;
}
#endif // AP_SERIALMANAGER_ENABLED

#if AP_SCRIPTING_SERIALDEVICE_ENABLED
// serial:find_simulated_device(protocol, instance): lua_serial_find_simulated_device()
int lua_serial_find_simulated_device(lua_State *L)
{
    binding_argcheck(L, 3);

    const int8_t protocol = (int8_t)get_uint32(L, 2, 0, INT8_MAX);
    uint32_t instance = get_uint16_t(L, 3);

    auto *scripting = AP::scripting();
    AP_Scripting_SerialDevice::Port *device_stream = nullptr;

    for (auto &port : scripting->_serialdevice.ports) {
        if (port.state.protocol == protocol) {
            if (instance-- == 0) {
                device_stream = &port;
                break;
            }
        }
    }

    if (!scripting->_serialdevice.enable || device_stream == nullptr) {
        // serial devices as a whole are disabled, or port not found
        return 0;
    }

    AP_Scripting_SerialAccess *port = new_AP_Scripting_SerialAccess(L);
    port->stream = device_stream;
    port->is_device_port = true;

    return 1;
}
#endif // AP_SCRIPTING_SERIALDEVICE_ENABLED

// port:writestring(data): lua_serial_writestring(); the data is the call's terminated arena copy
int lua_serial_writestring(lua_State *L)
{
    binding_argcheck(L, 2);

    AP_Scripting_SerialAccess *port = check_AP_Scripting_SerialAccess(L, 1);

    // get the bytes the user wants to write, along with their length
    size_t req_bytes;
    const char *data = luaL_checklstring(L, 2, &req_bytes);

    // write up to that number of bytes
    const uint32_t written_bytes = port->write((const uint8_t*)data, req_bytes);

    // return the number of bytes that were actually written
    lua_pushinteger(L, written_bytes);

    return 1;
}

// port:readstring(count): lua_serial_readstring(), into a stack buffer in place of a luaL_Buffer.
// At most IO_CHUNK bytes per call: in-process it returns up to count, here up to what one reply holds
int lua_serial_readstring(lua_State *L)
{
    binding_argcheck(L, 2);

    AP_Scripting_SerialAccess *port = check_AP_Scripting_SerialAccess(L, 1);

    const uint16_t req_bytes = get_uint16_t(L, 2);
    const uint16_t chunk_bytes = req_bytes < IO_CHUNK ? req_bytes : IO_CHUNK;
    uint8_t data[IO_CHUNK];

    // read up to that number of bytes
    const ssize_t read_bytes = port->read(data, chunk_bytes);
    if (read_bytes < 0) {
        return 0; // error, return nil
    }

    lua_pushlstring(L, reinterpret_cast<const char *>(data), size_t(read_bytes) < chunk_bytes ? size_t(read_bytes) : chunk_bytes);

    return 1;
}

// port:begin([baud]): lua_serial_begin()
int lua_serial_begin(lua_State *L)
{
    const int args = lua_gettop(L);
    if (args > 2) {
        return luaL_argerror(L, args, "too many arguments");
    } else if (args < 1) {
        return luaL_argerror(L, args, "too few arguments");
    }
    AP_Scripting_SerialAccess *port = check_AP_Scripting_SerialAccess(L, 1);

    // nil or absent argument treated as automatic baud (lua_isnoneornil())
    if (lua_type(L, 2) > 0) {
        port->begin(get_uint32(L, 2, 1, UINT32_MAX));
    } else {
        port->begin();
    }

    return 0;
}

}  // namespace ScriptingM4Remote

#endif  // AP_SCRIPTING_M4_ENABLED
