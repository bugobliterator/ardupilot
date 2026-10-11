/*
  Manual bindings (lua_bindings.cpp) the M4 calls through the generated
  dispatch, ported to the remote adapter. The rest follow in the IO and
  Manual Bindings sub-phase (gen-bindings -r: remote_ready_manual[]).
 */
#include "AP_Scripting_config.h"

#if AP_SCRIPTING_M4_ENABLED

#include <AP_HAL/AP_HAL.h>
#include "AP_ScriptingM4_Remote.h"

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

}  // namespace ScriptingM4Remote

#endif  // AP_SCRIPTING_M4_ENABLED
