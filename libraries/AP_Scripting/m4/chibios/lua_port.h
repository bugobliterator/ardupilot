#pragma once

#include "ch.h"
#include "../m4_service.h"

// start the Lua thread: privileged until it drops to unprivileged mode on its Lua RAM stack
void lua_port_start(ScriptingM4::LuaLink &link, binary_semaphore_t &service_wake);
// M4Platform::wake_lua
void lua_port_wake();
// the lowest 32 bytes of the Lua thread's stack: an MPU no-access guard (region 6)
uint32_t lua_stack_guard();
