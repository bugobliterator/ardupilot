#pragma once

/*
  The generated bindings on the M4: the sandbox lookup and the remote stub
  (m4_bindings.cpp), and the local userdata code, uint32_t and uint64_t
  (lua_remote_local.cpp, generated; m4_boxed_numerics.cpp).
 */

#include <stdint.h>
extern "C" {
#include <lua/src/lua.h>
#include <lua/src/lualib.h>
#include <lua/src/lauxlib.h>
}
#include <AP_ScriptingM4_Wire.h>

namespace ScriptingM4 {

// fetches the M7's m4:bindings image; false if the call fails or the size is not this contract's
bool fetch_binding_info();
// the sandbox table on top of the stack: print, and the generated bindings resolved on first use
void open_remote_functions(lua_State *L);
// the stub of every remote row (upvalues: method id, row flags, owning type id)
int remote_call(lua_State *L);
// sends the m4:release of the owned objects collected since the last call; what it could not
// send stays queued for the next call
void flush_releases();
// owned objects collected while the release queue was full: the M7 frees them at the session end
uint32_t releases_dropped();

// a pure-data type the M4 implements itself (gen-bindings -r, lua_remote_local.cpp)
static const uint16_t LOCAL_OBJECT_MAX = 32;   // the largest local type is 16 B (Location, Quaternion)
static const uint16_t LOCAL_WIRE_MAX = 32;     // its largest wire form is 17 B (Location)
struct LocalCodec {
    uint8_t type_id;
    uint16_t size;                                          // sizeof the C++ type: the userdata's length
    uint16_t wire_size;                                     // the blob length a remote call carries
    void *(*create)(lua_State *L);                          // new_<type>(): pushes it with its metatable
    bool (*decode)(WireReader &reader, void *storage);      // constructs it in storage from the wire form
    void (*encode)(WireWriter &writer, const void *object);
};
// lua_remote_local.cpp (generated)
const LocalCodec *local_codec(uint8_t type_id);             // nullptr: no local blob type
lua_CFunction remote_local_constructor(uint8_t type_id);    // the local types' constructors, else nullptr
// m4_bindings.cpp: a remote row of a local type (Location:change_alt_frame) on top; 0 if none
int remote_row_index(lua_State *L, uint8_t type_id);

}  // namespace ScriptingM4

// lua_remote_local.cpp (generated)
uint32_t *new_uint32_t(lua_State *L);
uint32_t *check_uint32_t(lua_State *L, int arg);
uint64_t *new_uint64_t(lua_State *L);
uint64_t *check_uint64_t(lua_State *L, int arg);
int binding_argcheck(lua_State *L, int expected_arg_count);
bool remote_local_type(const char *name, lua_CFunction &index, const luaL_Reg *&operators);

// m4_bindings.cpp: the generated allocators' metatables
void set_userdata_metatable(lua_State *L, const char *name);

// m4_boxed_numerics.cpp
uint32_t coerce_to_uint32_t(lua_State *L, int arg);
uint64_t coerce_to_uint64_t(lua_State *L, int arg);
int lua_new_uint32_t(lua_State *L);
int lua_new_uint64_t(lua_State *L);
int uint32_t___tostring(lua_State *L);
int uint32_t_toint(lua_State *L);
int uint32_t_tofloat(lua_State *L);
int uint64_t___tostring(lua_State *L);
int uint64_t_toint(lua_State *L);
int uint64_t_tofloat(lua_State *L);
int uint64_t_split(lua_State *L);

// m4_lua.cpp: a warning the M7 prints and keeps as the prearm failure, as in-process
namespace lua_scripts {
void set_and_print_new_error_message(uint8_t severity, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
}
