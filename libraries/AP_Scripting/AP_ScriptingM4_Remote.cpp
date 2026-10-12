#include "AP_ScriptingM4_Remote.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace ScriptingM4Remote {

using ScriptingM4::ReplyStatus;
using ScriptingM4::RpcArgs;
using ScriptingM4::RpcValue;
using ScriptingM4::RpcWriter;
using ScriptingM4::ValueTag;

// the call run() is in: set_and_print_new_error_message() and the type namer have no lua_State
static RemoteCall *current;

[[noreturn]] static void raise_error(lua_State *L, ReplyStatus status, int arg, const char *message)
{
    if (current != L) {
        // no run() of this call is active: its jump buffer is a dead frame
        __builtin_trap();
    }
    L->status = status;
    L->error_arg = uint8_t(arg);
    snprintf(L->message, sizeof(L->message), "%s", message);
    ap_longjmp(L->jump, 1);
}

static const BlobCodec *codec_of(const lua_State *L, uint8_t type_id)
{
    return (type_id >= 1 && type_id <= L->codec_count) ? &L->codecs[type_id - 1] : nullptr;
}

static const char *codec_name(uint8_t type_id)
{
    const BlobCodec *codec = current != nullptr ? codec_of(current, type_id) : nullptr;
    return codec != nullptr ? codec->name : "userdata";
}

static const RpcValue *arg_value(const lua_State *L, int arg)
{
    return (arg >= 1 && arg <= L->argc) ? &L->args[arg - 1] : nullptr;
}

static RpcArgs checks(const lua_State *L)
{
    return RpcArgs(L->args, L->argc, codec_name);
}

static void *arena_alloc(lua_State *L, size_t size)
{
    const size_t rounded = (size + 7) & ~size_t(7);
    if (rounded > sizeof(L->arena) - L->arena_used) {
        raise_error(L, ReplyStatus::Error, 0, "M4: call too large");
    }
    void *block = &L->arena[L->arena_used];
    L->arena_used += uint16_t(rounded);
    return block;
}

static RemoteCall::Result &push_result(lua_State *L)
{
    if (L->result_count >= ScriptingM4::MAX_VALUES) {
        raise_error(L, ReplyStatus::Error, 0, "M4: results too large");
    }
    RemoteCall::Result &result = L->results[L->result_count++];
    memset(&result, 0, sizeof(result));
    return result;
}

int lua_gettop(lua_State *L)
{
    return L->argc;
}

int luaL_argerror(lua_State *L, int arg, const char *message)
{
    raise_error(L, ReplyStatus::ArgError, arg, message);
}

int luaL_error(lua_State *L, const char *fmt, ...)
{
    char text[sizeof(L->message)];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);
    raise_error(L, ReplyStatus::Error, 0, text);
}

int binding_argcheck(lua_State *L, int expected_arg_count)
{
    if (expected_arg_count < 0) {
        return luaL_argerror(L, L->argc, "too many arguments");   // field_argerror()
    }
    RpcArgs args = checks(L);
    if (!args.count_is(uint8_t(expected_arg_count))) {
        return luaL_argerror(L, args.error_arg(), args.message());
    }
    return 0;
}

int field_argerror(lua_State *L)
{
    return binding_argcheck(L, -1);
}

bool userdata_zero_arg_check(lua_State *L)
{
    if (L->argc == 0) {
        return false;
    }
    // the in-process text names the script and line; the M7 has no Lua call stack
    lua_scripts::set_and_print_new_error_message(4, "Warning: userdata creation does not take arguments, will be fatal in future");
    return true;
}

lua_Integer luaL_checkinteger(lua_State *L, int arg)
{
    RpcArgs args = checks(L);
    int32_t value = 0;
    if (!args.integer(uint8_t(arg), value)) {
        luaL_argerror(L, args.error_arg(), args.message());
    }
    return value;
}

lua_Number luaL_checknumber(lua_State *L, int arg)
{
    RpcArgs args = checks(L);
    float value = 0;
    if (!args.number(uint8_t(arg), value)) {
        luaL_argerror(L, args.error_arg(), args.message());
    }
    return value;
}

const char *luaL_checklstring(lua_State *L, int arg, size_t *length)
{
    RpcArgs args = checks(L);
    const char *text = nullptr;
    uint16_t text_length = 0;
    if (!args.string(uint8_t(arg), text, text_length)) {
        luaL_argerror(L, args.error_arg(), args.message());
    }
    // the AP functions take C strings: a terminated copy, as Lua's strings are
    char *copy = static_cast<char *>(arena_alloc(L, size_t(text_length) + 1));
    memcpy(copy, text, text_length);
    copy[text_length] = 0;
    if (length != nullptr) {
        *length = text_length;
    }
    return copy;
}

int lua_toboolean(lua_State *L, int arg)
{
    const RpcValue *value = arg_value(L, arg);
    return value != nullptr && value->tag != ValueTag::Nil && value->tag != ValueTag::False;
}

lua_Integer get_integer(lua_State *L, int arg, lua_Integer min_value, lua_Integer max_value)
{
    const lua_Integer value = luaL_checkinteger(L, arg);
    if (value < min_value || value > max_value) {
        luaL_argerror(L, arg, "out of range");
    }
    return value;
}

int8_t get_int8_t(lua_State *L, int arg)
{
    return static_cast<int8_t>(get_integer(L, arg, INT8_MIN, INT8_MAX));
}

int16_t get_int16_t(lua_State *L, int arg)
{
    return static_cast<int16_t>(get_integer(L, arg, INT16_MIN, INT16_MAX));
}

uint8_t get_uint8_t(lua_State *L, int arg)
{
    return static_cast<uint8_t>(get_integer(L, arg, 0, UINT8_MAX));
}

uint16_t get_uint16_t(lua_State *L, int arg)
{
    return static_cast<uint16_t>(get_integer(L, arg, 0, UINT16_MAX));
}

float get_number(lua_State *L, int arg, float min_value, float max_value)
{
    const float value = luaL_checknumber(L, arg);
    if (!(value >= min_value && value <= max_value)) {
        luaL_argerror(L, arg, "out of range");
    }
    return value;
}

// lua_boxed_numerics.cpp: a boxed uint32_t, then an integer, then a number in range
uint32_t coerce_to_uint32_t(lua_State *L, int arg)
{
    const RpcValue *value = arg_value(L, arg);
    if (value != nullptr && value->tag == ValueTag::Blob && value->type_id == L->uint32_type && value->length == 4) {
        uint32_t boxed;
        memcpy(&boxed, value->blob, sizeof(boxed));
        return boxed;
    }
    RpcArgs args = checks(L);
    int32_t as_integer = 0;
    if (args.integer(uint8_t(arg), as_integer)) {
        return static_cast<uint32_t>(as_integer);
    }
    float as_number = 0;
    if (args.number(uint8_t(arg), as_number) && as_number >= 0 && as_number <= float(UINT32_MAX)) {
        return static_cast<uint32_t>(as_number);
    }
    luaL_argerror(L, arg, "Unable to coerce to uint32_t");
}

uint32_t get_uint32(lua_State *L, int arg, uint32_t min_value, uint32_t max_value)
{
    const uint32_t value = coerce_to_uint32_t(L, arg);
    if (value < min_value || value > max_value) {
        luaL_argerror(L, arg, "out of range");
    }
    return value;
}

void luaL_checkstack(lua_State *, int, const char *)
{
    // results are bounded by push_result()
}

void lua_pushboolean(lua_State *L, int value)
{
    RemoteCall::Result &result = push_result(L);
    result.kind = RemoteCall::Result::Value;
    result.value.tag = value ? ValueTag::True : ValueTag::False;
}

void lua_pushinteger(lua_State *L, lua_Integer value)
{
    RemoteCall::Result &result = push_result(L);
    result.kind = RemoteCall::Result::Value;
    result.value.tag = ValueTag::Integer;
    result.value.integer = value;
}

void lua_pushnumber(lua_State *L, lua_Number value)
{
    RemoteCall::Result &result = push_result(L);
    result.kind = RemoteCall::Result::Value;
    result.value.tag = ValueTag::Number;
    result.value.number = value;
}

void lua_pushstring(lua_State *L, const char *text)
{
    RemoteCall::Result &result = push_result(L);
    result.kind = RemoteCall::Result::Value;
    if (text == nullptr) {
        result.value.tag = ValueTag::Nil;
        return;
    }
    const size_t length = strlen(text);
    if (length > ScriptingM4::MAX_STRING) {
        raise_error(L, ReplyStatus::Error, 0, "M4: results too large");
    }
    char *copy = static_cast<char *>(arena_alloc(L, length));
    memcpy(copy, text, length);
    result.value.tag = ValueTag::String;
    result.value.string = copy;
    result.value.length = uint16_t(length);
}

uint32_t *new_uint32_t(lua_State *L)
{
    return static_cast<uint32_t *>(remote_new_blob(L, L->uint32_type, sizeof(uint32_t)));
}

uint64_t *new_uint64_t(lua_State *L)
{
    return static_cast<uint64_t *>(remote_new_blob(L, L->uint64_type, sizeof(uint64_t)));
}

int not_supported_error(lua_State *L, int arg, const char *name)
{
    char text[50];
    snprintf(text, sizeof(text), "%s not supported on this firmware", name);
    luaL_argerror(L, arg, text);
}

namespace lua_scripts {
void set_and_print_new_error_message(uint8_t severity, const char *fmt, ...)
{
    if (current == nullptr || current->warn == nullptr) {
        return;
    }
    char text[128];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);
    current->warn(current->warn_context, severity, text);
}
}

[[noreturn]] static void type_error(lua_State *L, int arg, const char *expected)
{
    char text[sizeof(L->message)];
    snprintf(text, sizeof(text), "%s expected, got %s", expected, checks(L).type_name(uint8_t(arg)));
    raise_error(L, ReplyStatus::ArgError, arg, text);
}

void *remote_check_blob(lua_State *L, int arg, uint8_t type_id)
{
    const BlobCodec *codec = codec_of(L, type_id);
    const RpcValue *value = arg_value(L, arg);
    if (codec == nullptr || value == nullptr || value->tag != ValueTag::Blob || value->type_id != type_id) {
        type_error(L, arg, codec != nullptr ? codec->name : "userdata");
    }
    char text[sizeof(L->message)];
    snprintf(text, sizeof(text), "M4: invalid %s", codec->name);
    if (codec->decode == nullptr) {
        raise_error(L, ReplyStatus::Error, 0, text);   // not on this firmware: the M4 cannot have made one
    }
    void *object = arena_alloc(L, codec->size);
    WireReader reader(value->blob, value->length);
    if (!codec->decode(reader, object)) {
        raise_error(L, ReplyStatus::Error, 0, text);
    }
    if (arg == 1) {
        L->self = object;
        L->self_type = type_id;
    }
    return object;
}

void *remote_new_blob(lua_State *L, uint8_t type_id, size_t size)
{
    RemoteCall::Result &result = push_result(L);
    result.kind = RemoteCall::Result::Object;
    result.type_id = type_id;
    result.object = arena_alloc(L, size);
    memset(result.object, 0, size);
    return result.object;
}

void **remote_check_handle(lua_State *L, int arg, uint8_t type_id)
{
    const BlobCodec *codec = codec_of(L, type_id);
    const char *name = codec != nullptr ? codec->name : "userdata";
    const RpcValue *value = arg_value(L, arg);
    if (value == nullptr || value->tag != ValueTag::Handle || value->type_id != type_id) {
        type_error(L, arg, name);
    }
    void *object = L->handles != nullptr ?
        L->handles->lookup(type_id, value->handle_index, value->handle_generation) : nullptr;
    if (object == nullptr) {
        char text[sizeof(L->message)];
        snprintf(text, sizeof(text), "M4: stale %s", name);
        raise_error(L, ReplyStatus::Error, 0, text);
    }
    void **slot = static_cast<void **>(arena_alloc(L, sizeof(void *)));
    *slot = object;
    return slot;
}

void **remote_new_handle(lua_State *L, uint8_t type_id)
{
    RemoteCall::Result &result = push_result(L);
    result.kind = RemoteCall::Result::Handle;
    result.type_id = type_id;
    void **slot = static_cast<void **>(arena_alloc(L, sizeof(void *)));
    *slot = nullptr;
    result.object = slot;
    return slot;
}

static ReplyStatus fail(RpcWriter &results, const char *text)
{
    results.restart();
    results.string(text, strlen(text));
    return ReplyStatus::Error;
}

static bool encode_object(const RemoteCall &call, uint8_t type_id, const void *object, RpcWriter &results)
{
    if (type_id == call.uint32_type || type_id == call.uint64_type) {
        results.blob(type_id, object, type_id == call.uint32_type ? sizeof(uint32_t) : sizeof(uint64_t));
        return results.ok();
    }
    const BlobCodec *codec = codec_of(&call, type_id);
    if (codec == nullptr || codec->encode == nullptr) {
        return false;
    }
    uint8_t wire_buf[ScriptingM4::MAX_BLOB];
    WireWriter wire(wire_buf, sizeof(wire_buf));
    codec->encode(wire, object);
    if (!wire.ok()) {
        return false;
    }
    results.blob(type_id, wire.data(), wire.length());
    return results.ok();
}

ReplyStatus run(RemoteCall &call, const RpcValue *args, uint8_t argc, lua_CFunction function, uint8_t flags,
                RpcWriter &results, uint8_t &error_arg)
{
    error_arg = 0;
    call.args = args;
    call.argc = argc;
    call.result_count = 0;
    call.self = nullptr;
    call.self_type = 0;
    call.arena_used = 0;
    call.status = ReplyStatus::Ok;
    call.error_arg = 0;
    call.message[0] = 0;
    current = &call;
    if (ap_setjmp(call.jump) != 0) {
        current = nullptr;
        results.restart();
        results.string(call.message, strlen(call.message));
        error_arg = call.error_arg;
        return call.status;
    }
    const int returned = function(&call);
    current = nullptr;
    if (returned < 0 || returned > call.result_count) {
        return fail(results, "M4: malformed binding");
    }
    // a binding returns the last values it pushed, as a lua_CFunction does
    for (uint8_t i = uint8_t(call.result_count - returned); i < call.result_count; i++) {
        const RemoteCall::Result &result = call.results[i];
        switch (result.kind) {
        case RemoteCall::Result::Value:
            switch (result.value.tag) {
            case ValueTag::True:
            case ValueTag::False:
                results.boolean(result.value.tag == ValueTag::True);
                break;
            case ValueTag::Integer:
                results.integer(result.value.integer);
                break;
            case ValueTag::Number:
                results.number(result.value.number);
                break;
            case ValueTag::String:
                results.string(result.value.string, result.value.length);
                break;
            case ValueTag::Nil:
            case ValueTag::Blob:
            case ValueTag::Handle:
            case ValueTag::Opaque:
                results.nil();
                break;
            }
            break;
        case RemoteCall::Result::Object:
            if (!encode_object(call, result.type_id, result.object, results)) {
                return fail(results, "M4: results too large");
            }
            break;
        case RemoteCall::Result::Handle: {
            void *object = *static_cast<void *const *>(result.object);
            if (object == nullptr) {
                results.nil();
                break;
            }
            const int16_t index = call.handles != nullptr ? call.handles->insert(result.type_id, object) : -1;
            if (index < 0) {
                return fail(results, "M4: too many objects");
            }
            results.handle(result.type_id, uint16_t(index), call.handles->generation());
            break;
        }
        }
    }
    if ((flags & METHOD_WRITES_SELF) != 0 && call.self != nullptr &&
        !encode_object(call, call.self_type, call.self, results)) {
        return fail(results, "M4: results too large");
    }
    if (!results.ok()) {
        return fail(results, "M4: results too large");
    }
    return ReplyStatus::Ok;
}

}  // namespace ScriptingM4Remote
