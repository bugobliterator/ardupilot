#pragma once

#include "../m4_service.h"

// the Lua thread of the SITL M4 process (a pthread); false if it could not start
bool lua_port_start(ScriptingM4::LuaLink &link);
// M4Platform::wake_lua
void lua_port_wake();
