/*
  The generated local userdata in the M4 builds: uint32_t and uint64_t, and
  the pure-data types Vector2f, Vector3f, Quaternion and Location from AP_Math
  and AP_Common/Location (m4/local, through m4/shim). Lua RAM code, run by the
  Lua thread; built with the M7's math flags (m4/chibios/Makefile).
 */
// first: the local types' own headers, before m4_binding_deps.h's fallback macros
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
// not_supported_error() of generated_cpp_deps.h serves the singletons, which are remote here
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#include <AP_Scripting/lua_remote_local.cpp>
#pragma GCC diagnostic pop
