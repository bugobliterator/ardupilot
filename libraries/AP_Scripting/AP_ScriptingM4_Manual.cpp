/*
  Manual bindings (lua_bindings.cpp, AP_Scripting_helpers.cpp) the M4 calls
  through the generated dispatch, ported to the remote adapter; the other
  areas are in AP_ScriptingM4_Manual*.cpp (gen-bindings -r:
  remote_ready_manual[]).
 */
#include "AP_Scripting_config.h"

#if AP_SCRIPTING_M4_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_Scripting_helpers.h"
#include "AP_ScriptingM4_Objects.h"
#include "AP_ScriptingM4_Remote.h"

extern const AP_HAL::HAL& hal;

namespace ScriptingM4Remote {

// millis(): the M7's clock, which mission and MAVLink timestamps are compared against
int lua_millis(lua_State *L)
{
    binding_argcheck(L, 0);
    *new_uint32_t(L) = AP_HAL::millis();
    return 1;
}

int lua_micros(lua_State *L)
{
    binding_argcheck(L, 0);
    *new_uint32_t(L) = AP_HAL::micros();
    return 1;
}

// generated (lua_remote_dispatch.cpp)
Parameter *new_Parameter(lua_State *L);

// Parameter([name]): lua_new_Parameter() of AP_Scripting_helpers.cpp. The object is made first,
// so an unknown name's error destroys it with the call (run())
int lua_new_Parameter(lua_State *L)
{
    const int args = lua_gettop(L);
    if (args > 1) {
        return luaL_argerror(L, args, "too many arguments");
    }
    const char *name = nullptr;
    if (args == 1) {
        name = luaL_checkstring(L, 1);
    }
    Parameter *p = new_Parameter(L);
    if (args == 0) {
        // no arguments, nothing to do
        return 1;
    }
    if (!p->init(name)) {
        return luaL_error(L, "No parameter: %s", name);
    }
    return 1;
}

// Parameter:init_by_info() allocates an AP_Param on the M7 per call that nothing frees, so a
// script looping on it would use the M7's heap up: not served to the M4
int lua_m4_Parameter_init_by_info(lua_State *L)
{
    return luaL_error(L, "init_by_info not supported on M4");
}

#if !defined(HAL_DISABLE_ADC_DRIVER)
// generated (lua_remote_dispatch.cpp)
AP_HAL::AnalogSource **new_AP_HAL__AnalogSource(lua_State *L);

// the analog sources M4 sessions were handed: the HAL never frees one (ChibiOS has 16), and every
// session's scripts ask again, so new ones would use the HAL's slots up after a few M4 restarts
static ScriptingM4::SessionRecycler<AP_HAL::AnalogSource, 16> analog_sources;

// analog:channel(): hal_analogin_channel() of the generated bindings, handing out a source an
// earlier session was handed before asking the HAL for a new one. A reused source keeps the pin
// its last script set until set_pin(); a new one has none, as in-process
int lua_m4_analog_channel(lua_State *L)
{
    binding_argcheck(L, 1);
    AP_HAL::AnalogSource *data = analog_sources.take([]() { return hal.analogin->channel(ANALOG_INPUT_NONE); });
    if (data == nullptr) {
        return 0;
    }
    *new_AP_HAL__AnalogSource(L) = data;
    return 1;
}
#endif  // !defined(HAL_DISABLE_ADC_DRIVER)

void analog_new_session()
{
#if !defined(HAL_DISABLE_ADC_DRIVER)
    analog_sources.new_session();
#endif
}

}  // namespace ScriptingM4Remote

#endif  // AP_SCRIPTING_M4_ENABLED
