/*
  The generated bindings on the M4 (spec §6). The sandbox resolves a binding
  name on first use, as the in-process load_generated_sandbox() does:
  singletons, blob types (pure-data userdata held as the M7's wire bytes) and
  handle types (M7 objects) get a metatable whose __index finds the name in
  the generated rows and caches a remote_stub closure for it. uint32_t and
  uint64_t stay local (lua_remote_local.cpp, m4_boxed_numerics.cpp). What
  this firmware compiled in, and the enum values, come from the M7's
  m4:bindings image at every start. Runs on the Lua thread.
 */
#include "m4_bindings.h"

#include <string.h>
#include <AP_ScriptingM4_Bindings.h>
#include "m4_rpc_client.h"

namespace ScriptingM4 {

// a blob userdata: the M7's wire bytes of a pure-data type follow the header
struct BlobHeader {
    uint8_t type_id;
    uint8_t reserved;
    uint16_t length;
};

// a handle userdata: an M7 object of this M4 session
struct HandleUserdata {
    uint8_t type_id;
    uint8_t reserved;
    uint16_t index;
    uint32_t generation;
};

// the type loops count with a uint8_t id up to and including BINDING_TYPE_COUNT
static_assert(BINDING_TYPE_COUNT < UINT8_MAX, "type ids fit a uint8_t loop");

static uint8_t binding_info[BINDING_INFO_SIZE];

bool fetch_binding_info()
{
    (void)rpc.begin(SystemMethod::Bindings);
    if (rpc.invoke() != 0 || rpc.status() != ReplyStatus::Ok || rpc.count() != 1) {
        return false;
    }
    const RpcValue &image = rpc.value(0);
    if (image.tag != ValueTag::String || image.length != BINDING_INFO_SIZE) {
        return false;
    }
    memcpy(binding_info, image.string, BINDING_INFO_SIZE);
    return true;
}

static const BindingType &type_of(uint8_t type_id)
{
    return binding_types[type_id - 1];
}

static uint8_t find_type(const char *name)
{
    for (uint8_t id = 1; id <= BINDING_TYPE_COUNT; id++) {
        if (strcmp(type_of(id).name, name) == 0) {
            return id;
        }
    }
    return 0;
}

// the __m4type of a userdata's metatable, checked against the userdata's size; 0 if none
static uint8_t userdata_type(lua_State *L, int index)
{
    if (lua_type(L, index) != LUA_TUSERDATA || luaL_getmetafield(L, index, "__m4type") == LUA_TNIL) {
        return 0;
    }
    const lua_Integer id = lua_tointeger(L, -1);
    lua_pop(L, 1);
    if (id < 1 || id > BINDING_TYPE_COUNT) {
        return 0;
    }
    const size_t size = lua_rawlen(L, index);
    const void *data = lua_touserdata(L, index);
    switch (BindingKind(type_of(uint8_t(id)).kind)) {
    case BindingKind::Blob: {
        const BlobHeader *blob = static_cast<const BlobHeader *>(data);
        return (size >= sizeof(BlobHeader) && blob->type_id == id && size >= sizeof(BlobHeader) + blob->length) ?
               uint8_t(id) : 0;
    }
    case BindingKind::Handle:
        return size == sizeof(HandleUserdata) ? uint8_t(id) : 0;
    case BindingKind::Local:
        return size == (id == BINDING_TYPE_UINT32 ? sizeof(uint32_t) : sizeof(uint64_t)) ? uint8_t(id) : 0;
    case BindingKind::Singleton:
    case BindingKind::Global:
        break;
    }
    return 0;
}

static void encode_value(lua_State *L, int index, RpcWriter &args)
{
    switch (lua_type(L, index)) {
    case LUA_TNIL:
        args.nil();
        return;
    case LUA_TBOOLEAN:
        args.boolean(lua_toboolean(L, index) != 0);
        return;
    case LUA_TNUMBER:
        if (lua_isinteger(L, index)) {
            args.integer(int32_t(lua_tointeger(L, index)));
        } else {
            args.number(float(lua_tonumber(L, index)));
        }
        return;
    case LUA_TSTRING: {
        size_t length = 0;
        const char *text = lua_tolstring(L, index, &length);
        args.string(text, length);
        return;
    }
    case LUA_TUSERDATA: {
        const uint8_t type_id = userdata_type(L, index);
        const void *data = lua_touserdata(L, index);
        if (type_id == 0) {
            break;
        }
        switch (BindingKind(type_of(type_id).kind)) {
        case BindingKind::Blob: {
            const BlobHeader *blob = static_cast<const BlobHeader *>(data);
            args.blob(type_id, blob + 1, blob->length);
            return;
        }
        case BindingKind::Handle: {
            const HandleUserdata *handle = static_cast<const HandleUserdata *>(data);
            args.handle(type_id, handle->index, handle->generation);
            return;
        }
        case BindingKind::Local:
            args.blob(type_id, data, type_id == BINDING_TYPE_UINT32 ? sizeof(uint32_t) : sizeof(uint64_t));
            return;
        case BindingKind::Singleton:
        case BindingKind::Global:
            break;
        }
        break;
    }
    default:
        break;
    }
    args.opaque(uint8_t(lua_type(L, index)));   // the M7 names it in "... expected, got table"
}

static int remote_index(lua_State *L);
static int remote_stub(lua_State *L);

// upvalues: method id, row flags, owning type id
static void push_stub(lua_State *L, uint16_t method, uint8_t flags, uint8_t type_id)
{
    lua_pushinteger(L, method);
    lua_pushinteger(L, flags);
    lua_pushinteger(L, type_id);
    lua_pushcclosure(L, remote_stub, 3);
}

// the metatable of a binding type, made on its first use: __name, __m4type, __index and,
// for a blob type, its operators
static void push_metatable(lua_State *L, uint8_t type_id)
{
    const BindingType &type = type_of(type_id);
    if (!luaL_newmetatable(L, type.name)) {
        return;
    }
    lua_pushinteger(L, type_id);
    lua_setfield(L, -2, "__m4type");
    lua_pushinteger(L, type_id);
    lua_newtable(L);   // the stubs __index made so far
    lua_pushcclosure(L, remote_index, 2);
    lua_setfield(L, -2, "__index");
    for (uint16_t i = type.first_row; i < type.first_row + type.row_count; i++) {
        const BindingRow &row = binding_rows[i];
        if ((row.flags & ROW_OPERATOR) != 0 && info_method_available(binding_info, row.method)) {
            push_stub(L, row.method, row.flags, type_id);
            lua_setfield(L, -2, row.name);
        }
    }
}

static bool push_value(lua_State *L, const RpcValue &value)
{
    switch (value.tag) {
    case ValueTag::Nil:
    case ValueTag::Opaque:
        lua_pushnil(L);
        return true;
    case ValueTag::False:
    case ValueTag::True:
        lua_pushboolean(L, value.tag == ValueTag::True);
        return true;
    case ValueTag::Integer:
        lua_pushinteger(L, value.integer);
        return true;
    case ValueTag::Number:
        lua_pushnumber(L, value.number);
        return true;
    case ValueTag::String:
        lua_pushlstring(L, value.string, value.length);
        return true;
    case ValueTag::Blob: {
        if (value.type_id == 0 || value.type_id > BINDING_TYPE_COUNT) {
            return false;
        }
        const BindingKind kind = BindingKind(type_of(value.type_id).kind);
        if (kind == BindingKind::Local) {
            if (value.type_id == BINDING_TYPE_UINT32 && value.length == sizeof(uint32_t)) {
                memcpy(new_uint32_t(L), value.blob, sizeof(uint32_t));
                return true;
            }
            if (value.type_id == BINDING_TYPE_UINT64 && value.length == sizeof(uint64_t)) {
                memcpy(new_uint64_t(L), value.blob, sizeof(uint64_t));
                return true;
            }
            return false;
        }
        if (kind != BindingKind::Blob) {
            return false;
        }
        BlobHeader *blob = static_cast<BlobHeader *>(lua_newuserdata(L, sizeof(BlobHeader) + value.length));
        blob->type_id = value.type_id;
        blob->reserved = 0;
        blob->length = value.length;
        memcpy(blob + 1, value.blob, value.length);
        push_metatable(L, value.type_id);
        lua_setmetatable(L, -2);
        return true;
    }
    case ValueTag::Handle: {
        if (value.type_id == 0 || value.type_id > BINDING_TYPE_COUNT ||
            BindingKind(type_of(value.type_id).kind) != BindingKind::Handle) {
            return false;
        }
        HandleUserdata *handle = static_cast<HandleUserdata *>(lua_newuserdata(L, sizeof(HandleUserdata)));
        handle->type_id = value.type_id;
        handle->reserved = 0;
        handle->index = value.handle_index;
        handle->generation = value.handle_generation;
        push_metatable(L, value.type_id);
        lua_setmetatable(L, -2);
        return true;
    }
    }
    return false;
}

// the updated self of a blob method or field, into argument 1
static bool write_back(lua_State *L, uint8_t owner, const RpcValue &value)
{
    if (value.tag != ValueTag::Blob || value.type_id != owner || userdata_type(L, 1) != owner) {
        return false;
    }
    BlobHeader *blob = static_cast<BlobHeader *>(lua_touserdata(L, 1));
    if (blob->length != value.length) {
        return false;
    }
    memcpy(blob + 1, value.blob, value.length);
    return true;
}

// raises the error of a failed reply; luaL_argerror() renumbers for method calls, as in-process
static int raise_reply_error(lua_State *L)
{
    char message[128];
    message[0] = 0;
    if (rpc.count() >= 1 && rpc.value(0).tag == ValueTag::String) {
        const RpcValue &text = rpc.value(0);
        const size_t length = text.length < sizeof(message) - 1 ? text.length : sizeof(message) - 1;
        memcpy(message, text.string, length);
        message[length] = 0;
    }
    switch (rpc.status()) {
    case ReplyStatus::ArgError:
        return luaL_argerror(L, rpc.error_arg(), message);
    case ReplyStatus::Error:
        return luaL_error(L, "%s", message);
    case ReplyStatus::UnknownMethod:
        return luaL_error(L, "M4: method not served by the M7");
    case ReplyStatus::Ok:
    case ReplyStatus::Malformed:
        break;
    }
    return luaL_error(L, "M4: malformed call");
}

// the client stub of every remote binding (upvalues: method id, row flags, owning type id)
static int remote_stub(lua_State *L)
{
    const uint16_t method = uint16_t(lua_tointeger(L, lua_upvalueindex(1)));
    const uint8_t flags = uint8_t(lua_tointeger(L, lua_upvalueindex(2)));
    const uint8_t owner = uint8_t(lua_tointeger(L, lua_upvalueindex(3)));
    RpcWriter &args = rpc.begin(method);
    const int top = lua_gettop(L);
    for (int i = 1; i <= top; i++) {
        if (i == 1 && (flags & ROW_HAS_SELF) != 0) {
            args.nil();   // a singleton: the M7 finds it itself
            continue;
        }
        encode_value(L, i, args);
    }
    if (!args.ok()) {
        return luaL_error(L, "M4: call arguments too large");
    }
    const int ret = rpc.invoke();
    if (ret != 0) {
        return luaL_error(L, "M4: call failed (%d)", ret);
    }
    if (rpc.status() != ReplyStatus::Ok) {
        return raise_reply_error(L);
    }
    // pushing allocates, which may run a finalizer that calls a binding and reuses the
    // client's reply: work from a copy. No GC step runs inside a finalizer (GCTM), so this
    // nests at most twice
    uint8_t reply[MAX_FRAME];
    RpcValue values[MAX_VALUES];
    uint8_t count = 0;
    if (!rpc.copy_reply(reply, sizeof(reply), values, count)) {
        return luaL_error(L, "M4: malformed reply");
    }
    if ((flags & ROW_WRITES_SELF) != 0) {
        if (count == 0 || !write_back(L, owner, values[count - 1])) {
            return luaL_error(L, "M4: malformed reply");
        }
        count--;
    }
    luaL_checkstack(L, count, "M4 results");
    for (uint8_t i = 0; i < count; i++) {
        if (!push_value(L, values[i])) {
            return luaL_error(L, "M4: malformed reply");
        }
    }
    return count;
}

// __index of a binding type (upvalues: type id, cache): a method or field stub, or an enum value
static int remote_index(lua_State *L)
{
    const char *name = luaL_checkstring(L, 2);
    lua_pushvalue(L, 2);
    if (lua_rawget(L, lua_upvalueindex(2)) != LUA_TNIL) {
        return 1;
    }
    lua_pop(L, 1);
    const uint8_t type_id = uint8_t(lua_tointeger(L, lua_upvalueindex(1)));
    const BindingType &type = type_of(type_id);
    for (uint16_t i = type.first_row; i < type.first_row + type.row_count; i++) {
        const BindingRow &row = binding_rows[i];
        if ((row.flags & (ROW_OPERATOR | ROW_CREATION)) == 0 && strcmp(row.name, name) == 0 &&
            info_method_available(binding_info, row.method)) {
            push_stub(L, row.method, row.flags, type_id);
            lua_pushvalue(L, 2);
            lua_pushvalue(L, -2);
            lua_rawset(L, lua_upvalueindex(2));
            return 1;
        }
    }
    for (uint8_t i = 0; i < type.enum_count; i++) {
        if (strcmp(binding_enum_names[type.first_enum + i], name) == 0) {
            lua_pushinteger(L, info_enum_value(binding_info, uint16_t(type.first_enum + i)));
            return 1;
        }
    }
    return 0;
}

// keeps the value on top in the sandbox (argument 1) under the name, so the lookup runs once
static int cache_global(lua_State *L)
{
    lua_pushvalue(L, 2);
    lua_pushvalue(L, -2);
    lua_rawset(L, 1);
    return 1;
}

// the sandbox's __index: singletons, constructors and global functions of the generated bindings
static int global_index(lua_State *L)
{
    const char *name = luaL_checkstring(L, 2);
    for (uint8_t id = 1; id <= BINDING_TYPE_COUNT; id++) {
        const BindingType &type = type_of(id);
        if (!info_type_available(binding_info, id)) {
            continue;
        }
        if (BindingKind(type.kind) == BindingKind::Global) {
            for (uint16_t i = type.first_row; i < type.first_row + type.row_count; i++) {
                const BindingRow &row = binding_rows[i];
                if (strcmp(row.name, name) == 0 && info_method_available(binding_info, row.method)) {
                    push_stub(L, row.method, row.flags, id);
                    return cache_global(L);
                }
            }
            continue;
        }
        if (strcmp(type.name, name) != 0) {
            continue;
        }
        switch (BindingKind(type.kind)) {
        case BindingKind::Singleton:
            lua_newuserdata(L, 0);
            push_metatable(L, id);
            lua_setmetatable(L, -2);
            return cache_global(L);
        case BindingKind::Blob:
            for (uint16_t i = type.first_row; i < type.first_row + type.row_count; i++) {
                const BindingRow &row = binding_rows[i];
                if ((row.flags & ROW_CREATION) != 0 && info_method_available(binding_info, row.method)) {
                    push_stub(L, row.method, row.flags, id);
                    return cache_global(L);
                }
            }
            return 0;
        case BindingKind::Local:
            lua_pushcfunction(L, id == BINDING_TYPE_UINT32 ? lua_new_uint32_t : lua_new_uint64_t);
            return cache_global(L);
        case BindingKind::Handle:
        case BindingKind::Global:
            return 0;
        }
    }
    return 0;
}

void open_remote_functions(lua_State *L)
{
    // print stays a system method until the manual bindings are ported (IO and Manual Bindings)
    lua_pushinteger(L, lua_Integer(SystemMethod::Print));
    lua_pushinteger(L, 0);
    lua_pushinteger(L, 0);
    lua_pushcclosure(L, remote_stub, 3);
    lua_setfield(L, -2, "print");
    // the generated bindings resolve on first use, as the in-process load_generated_sandbox()
    lua_createtable(L, 0, 1);
    lua_pushcfunction(L, global_index);
    lua_setfield(L, -2, "__index");
    lua_setmetatable(L, -2);
}

}  // namespace ScriptingM4

// the metatable of a generated local userdata (uint32_t, uint64_t), as in-process
void set_userdata_metatable(lua_State *L, const char *name)
{
    if (luaL_newmetatable(L, name)) {
        lua_CFunction index = nullptr;
        const luaL_Reg *operators = nullptr;
        if (remote_local_type(name, index, operators)) {
            lua_pushcfunction(L, index);
            lua_setfield(L, -2, "__index");
            if (operators != nullptr) {
                luaL_setfuncs(L, operators, 0);
            }
        }
        const uint8_t type_id = ScriptingM4::find_type(name);
        if (type_id != 0) {
            lua_pushinteger(L, type_id);
            lua_setfield(L, -2, "__m4type");
        }
    }
    lua_setmetatable(L, -2);
}
