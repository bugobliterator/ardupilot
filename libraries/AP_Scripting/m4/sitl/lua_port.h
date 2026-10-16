#pragma once

#include "../m4_service.h"

// the Lua thread of the SITL M4 process (a pthread); false if it could not start.
// forward_posted: M4Service::forward_posted() of the service on link
bool lua_port_start(ScriptingM4::LuaLink &link, bool (*forward_posted)());
// M4Platform::wake_lua
void lua_port_wake();
