/*
  Lua runtime of the M4 scripting core, portable: the ChibiOS image runs it
  on its unprivileged Lua thread, the SITL process on a pthread. It follows
  lua_scripts.cpp (sandbox, scheduling, instruction limit, error texts);
  scripts, output, errors and statistics go through the M7 (RpcClient).
 */
#include "m4_lua.h"

#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// the Lua headers alone: lua.hpp pulls in AP_Filesystem, which the M4 image does not have
extern "C" {
#include <lua/src/lua.h>
#include <lua/src/lualib.h>
#include <lua/src/lauxlib.h>
}
#include <AP_ScriptingM4_Rpc.h>
#include "m4_port.h"
#include "m4_rpc_client.h"

namespace ScriptingM4 {

// SCR_DEBUG_OPTS bits the runtime acts on (AP_Scripting::DebugOption)
static const uint32_t OPTION_NO_SCRIPTS_TO_RUN = 1U << 0;
static const uint32_t OPTION_RUNTIME_MSG = 1U << 1;
static const uint32_t OPTION_LOG_RUNTIME = 1U << 3;
// MAV_SEVERITY values the runtime reports with
static const int32_t SEVERITY_CRITICAL = 2;
static const int32_t SEVERITY_DEBUG = 7;
// longest idle sleep: Lua-thread fault requests are taken this promptly
static const uint32_t MAX_SLEEP_MS = 100;
// m4:list_script tries per index before the list counts as failed, spaced out and
// within LIST_WINDOW_MS, so a slow first build_list (CRCs over the SD card) is waited for
static const uint8_t LIST_ATTEMPTS = 3;
static const uint32_t LIST_RETRY_MS = 3000;
static const uint32_t LIST_WINDOW_MS = 10000;

struct Script {
    int env_ref;               // the script's environment table
    int run_ref;               // the function to run next
    uint32_t next_run_ms;      // M4 clock
    uint8_t index;             // in the M7's script table
    char name[MAX_SCRIPT_PATH];
    Script *next;
};

static struct {
    Script pool[MAX_SCRIPTS];
    bool used[MAX_SCRIPTS];
    Script *scripts;           // sorted by next run time, soonest first
    int32_t vm_steps;
    uint32_t options;
    uint32_t heap_limit;
    uint32_t heap_used;        // charged as block_cost() per live block
    int current_env_ref;
    bool overtime;
} rt;

// one script load at a time
static struct {
    uint8_t index;
    uint32_t offset;
    bool failed;
    bool done;
    char buffer[MAX_STRING];
} reader;

// what a block costs newlib's malloc, approximately: 8-byte rounding and a chunk
// header; both platforms charge it, so a script that fits in SITL fits on the M4
static uint32_t block_cost(size_t size)
{
    return size == 0 ? 0 : uint32_t(((size + 7) & ~size_t(7)) + 8);
}

static void *lua_alloc(void *, void *ptr, size_t osize, size_t nsize)
{
    if (ptr == nullptr) {
        osize = 0;   // osize then names the type of the new object
    }
    const uint32_t old_cost = block_cost(osize);
    if (nsize == 0) {
        free(ptr);
        rt.heap_used -= old_cost;
        return nullptr;
    }
    const uint32_t new_cost = block_cost(nsize);
    if (new_cost > old_cost && rt.heap_used + (new_cost - old_cost) > rt.heap_limit) {
        return nullptr;
    }
    void *grown = realloc(ptr, nsize);
    if (grown == nullptr) {
        return nullptr;
    }
    rt.heap_used = rt.heap_used - old_cost + new_cost;
    return grown;
}

static int32_t memory_in_use(lua_State *L)
{
    return lua_gc(L, LUA_GCCOUNT, 0) * 1024 + lua_gc(L, LUA_GCCOUNTB, 0);
}

static const char *error_object_message(lua_State *L)
{
    const char *message = lua_tostring(L, -1);
    return message != nullptr ? message : "<error>";
}

static void send_text(int32_t severity, const char *text)
{
    RpcWriter &args = rpc.begin(SystemMethod::GcsSendText);
    args.nil();   // self
    args.integer(severity);
    args.string(text, strlen(text));
    (void)rpc.invoke();
}

// lua_scripts::set_and_print_new_error_message(): the M7 prints it and keeps it for prearm
static void report_error(int32_t severity, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void report_error(int32_t severity, const char *fmt, ...)
{
    static char text[200];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);
    RpcWriter &args = rpc.begin(SystemMethod::ScriptError, true);
    args.integer(severity);
    args.string(text, strlen(text));
    (void)rpc.invoke();
}

// lua_scripts::update_stats(): SCR rows and RUNTIME_MSG texts on the M7
static void send_run_stats(uint8_t index, uint32_t run_time_us, int32_t total_mem, int32_t run_mem)
{
    if ((rt.options & (OPTION_RUNTIME_MSG | OPTION_LOG_RUNTIME)) == 0) {
        return;
    }
    RpcWriter &args = rpc.begin(SystemMethod::RunStats, true);
    args.integer(index);
    args.integer(int32_t(run_time_us));
    args.integer(total_mem);
    args.integer(run_mem);
    (void)rpc.invoke();
}

static void take_fault_request()
{
    const FaultKind kind = port_fault_request();
    if (kind != FaultKind::None) {
        port_lua_fault(kind);
    }
}

static void sleep_ms(uint32_t ms)
{
    while (ms > 0) {
        const uint32_t slice = ms < MAX_SLEEP_MS ? ms : MAX_SLEEP_MS;
        port_sleep_ms(slice);
        ms -= slice;
        take_fault_request();
    }
}

static Script *allocate_script()
{
    for (uint8_t i = 0; i < MAX_SCRIPTS; i++) {
        if (!rt.used[i]) {
            rt.used[i] = true;
            memset(&rt.pool[i], 0, sizeof(rt.pool[i]));
            return &rt.pool[i];
        }
    }
    return nullptr;
}

static void free_script(Script *script)
{
    rt.used[script - rt.pool] = false;
}

static bool runs_before(const Script *a, const Script *b)
{
    return int32_t(a->next_run_ms - b->next_run_ms) < 0;
}

// lua_scripts::reschedule_script(): equal times keep their order
static void reschedule(Script *script)
{
    script->next = nullptr;
    if (rt.scripts == nullptr || runs_before(script, rt.scripts)) {
        script->next = rt.scripts;
        rt.scripts = script;
        return;
    }
    Script *previous = rt.scripts;
    while (previous->next != nullptr && !runs_before(script, previous->next)) {
        previous = previous->next;
    }
    script->next = previous->next;
    previous->next = script;
}

static void remove_script(lua_State *L, Script *script)
{
    if (rt.scripts == script) {
        rt.scripts = script->next;
    } else {
        for (Script *current = rt.scripts; current != nullptr && current->next != nullptr; current = current->next) {
            if (current->next == script) {
                current->next = script->next;
                break;
            }
        }
    }
    RpcWriter &args = rpc.begin(SystemMethod::ScriptStopped, true);
    args.integer(script->index);
    (void)rpc.invoke();
    luaL_unref(L, LUA_REGISTRYINDEX, script->env_ref);
    luaL_unref(L, LUA_REGISTRYINDEX, script->run_ref);
    free_script(script);
}

static void hook(lua_State *L, lua_Debug *)
{
    rt.overtime = true;
    // bail out aggressively until the script has unwound
    lua_sethook(L, hook, LUA_MASKCOUNT, 1);
    luaL_error(L, "Exceeded CPU time");
}

static void reset_loop_overtime(lua_State *L)
{
    rt.overtime = false;
    lua_sethook(L, hook, LUA_MASKCOUNT, rt.vm_steps > 1000 ? rt.vm_steps : 1000);
}

static void create_sandbox(lua_State *L)
{
    lua_newtable(L);
    luaopen_base_sandbox(L);
    lua_pushstring(L, "math");
    luaopen_math(L);
    lua_settable(L, -3);
    lua_pushstring(L, "table");
    luaopen_table(L);
    lua_settable(L, -3);
    lua_pushstring(L, "string");
    luaopen_string(L);
    lua_settable(L, -3);
    // io and package (require) need the M7 file proxy: IO and Manual Bindings sub-phase
    open_remote_functions(L);
}

// lua_Reader over m4:read_script; never raises
static const char *read_chunk(lua_State *, void *, size_t *size)
{
    *size = 0;
    if (reader.done) {
        return nullptr;
    }
    RpcWriter &args = rpc.begin(SystemMethod::ReadScript);
    args.integer(reader.index);
    args.integer(int32_t(reader.offset));
    if (rpc.invoke() != 0 || rpc.status() != ReplyStatus::Ok || rpc.count() != 1 ||
        rpc.value(0).tag != ValueTag::String) {
        reader.done = true;
        reader.failed = true;
        return nullptr;
    }
    const RpcValue &chunk = rpc.value(0);
    if (chunk.length == 0) {
        reader.done = true;
        return nullptr;
    }
    memcpy(reader.buffer, chunk.string, chunk.length);
    reader.offset += chunk.length;
    *size = chunk.length;
    return reader.buffer;
}

// lua_scripts::load_script()
static bool load_script(lua_State *L, Script *script)
{
    memset(&reader, 0, sizeof(reader));
    reader.index = script->index;
    char chunkname[MAX_SCRIPT_PATH + 1];
    snprintf(chunkname, sizeof(chunkname), "@%s", script->name);
    const int error = lua_load(L, read_chunk, nullptr, chunkname, nullptr);
    if (reader.failed) {
        lua_pop(L, lua_gettop(L));
        report_error(SEVERITY_CRITICAL, "Unable to load the file: %s", script->name);
        return false;
    }
    switch (error) {
    case LUA_OK:
        break;
    case LUA_ERRSYNTAX:
        report_error(SEVERITY_CRITICAL, "Error: %s", error_object_message(L));
        lua_pop(L, lua_gettop(L));
        return false;
    case LUA_ERRMEM:
        report_error(SEVERITY_CRITICAL, "Insufficent memory loading %s", script->name);
        lua_pop(L, lua_gettop(L));
        return false;
    default:
        report_error(SEVERITY_CRITICAL, "Unknown error (%d) loading %s", error, script->name);
        lua_pop(L, lua_gettop(L));
        return false;
    }

    const int32_t load_mem = memory_in_use(L);
    const uint32_t load_start = port_micros();
    create_sandbox(L);
    lua_pushvalue(L, -1);
    lua_setupvalue(L, -3, 1);
    const uint32_t load_end = port_micros();
    send_run_stats(script->index, load_end - load_start, memory_in_use(L), load_mem);

    script->env_ref = luaL_ref(L, LUA_REGISTRYINDEX);
    script->run_ref = luaL_ref(L, LUA_REGISTRYINDEX);
    script->next_run_ms = port_millis() - 1;   // due now

    RpcWriter &args = rpc.begin(SystemMethod::ScriptLoaded, true);
    args.integer(script->index);
    (void)rpc.invoke();
    return true;
}

enum class ListEntry : uint8_t {
    Script,   // the path is rpc.value(0)
    End,
    Failed,
};

// m4:list_script(index); a failed call (e.g. timed out while the M7 builds the list) is retried
static ListEntry list_script(int32_t index)
{
    const uint32_t start_ms = port_millis();
    for (uint8_t attempt = 0; attempt < LIST_ATTEMPTS; attempt++) {
        if (attempt > 0) {
            if (port_millis() - start_ms + LIST_RETRY_MS > LIST_WINDOW_MS) {
                break;
            }
            sleep_ms(LIST_RETRY_MS);
        }
        RpcWriter &args = rpc.begin(SystemMethod::ListScript);
        args.integer(index);
        if (rpc.invoke() == 0 && rpc.status() == ReplyStatus::Ok && rpc.count() == 1) {
            if (rpc.value(0).tag == ValueTag::String) {
                return ListEntry::Script;
            }
            if (rpc.value(0).tag == ValueTag::Nil) {
                return ListEntry::End;
            }
        }
    }
    return ListEntry::Failed;
}

// false if the list could not be read: then no script runs (fail closed), and the error
// the M7 keeps fails prearm
static bool load_all_scripts(lua_State *L)
{
    int32_t loaded = 0;
    for (int32_t index = 0; index < int32_t(MAX_SCRIPTS); index++) {
        const ListEntry entry = list_script(index);
        if (entry == ListEntry::End) {
            break;
        }
        if (entry == ListEntry::Failed) {
            report_error(SEVERITY_CRITICAL, "M4 script list failed");
            while (rt.scripts != nullptr) {
                remove_script(L, rt.scripts);
            }
            return false;
        }
        Script *script = allocate_script();
        if (script == nullptr) {
            break;
        }
        const RpcValue &path = rpc.value(0);
        const uint16_t length = path.length < MAX_SCRIPT_PATH ? path.length : MAX_SCRIPT_PATH - 1;
        memcpy(script->name, path.string, length);
        script->name[length] = 0;
        script->index = uint8_t(index);
        script->env_ref = LUA_NOREF;
        script->run_ref = LUA_NOREF;
        if (!load_script(L, script)) {
            free_script(script);
            continue;
        }
        reschedule(script);
        loaded++;
    }
    RpcWriter &args = rpc.begin(SystemMethod::Started, true);
    args.integer(loaded);
    args.integer(int32_t(rt.heap_limit));
    (void)rpc.invoke();
    return true;
}

// lua_scripts::run_next_script()
static void run_next_script(lua_State *L)
{
    Script *script = rt.scripts;
    rt.scripts = script->next;
    const uint32_t start_ms = port_millis();
    reset_loop_overtime(L);
    const int stack_top = lua_gettop(L);
    lua_rawgeti(L, LUA_REGISTRYINDEX, script->run_ref);
    rt.current_env_ref = script->env_ref;
    if (lua_pcall(L, 0, LUA_MULTRET, 0)) {
        if (rt.overtime) {
            report_error(SEVERITY_CRITICAL, "%s exceeded time limit", script->name);
        } else {
            report_error(SEVERITY_CRITICAL, "%s", error_object_message(L));
        }
        remove_script(L, script);
        lua_pop(L, 1);
        return;
    }
    const int returned = lua_gettop(L) - stack_top;
    switch (returned) {
    case 0:
        remove_script(L, script);   // no reschedule
        break;
    case 2:
        if (lua_type(L, -1) != LUA_TNUMBER) {
            report_error(SEVERITY_CRITICAL, "%s did not return a delay (0x%d)", script->name, lua_type(L, -1));
            lua_pop(L, 2);
            remove_script(L, script);
            return;
        }
        if (lua_type(L, -2) != LUA_TFUNCTION) {
            report_error(SEVERITY_CRITICAL, "%s did not return a function (0x%d)", script->name, lua_type(L, -2));
            lua_pop(L, 2);
            remove_script(L, script);
            return;
        }
        script->next_run_ms = start_ms + uint32_t(luaL_checknumber(L, -1));
        lua_pop(L, 1);
        luaL_unref(L, LUA_REGISTRYINDEX, script->run_ref);
        script->run_ref = luaL_ref(L, LUA_REGISTRYINDEX);
        reschedule(script);
        break;
    default:
        report_error(SEVERITY_CRITICAL, "%s returned bad result count (%d)", script->name, returned);
        remove_script(L, script);
        lua_pop(L, returned);
        break;
    }
}

// lua_scripts::run_engine(), in protected mode
static int run_engine(lua_State *L)
{
    // one string metatable for all scripts, out of their reach
    lua_pushliteral(L, "");
    lua_createtable(L, 0, 1);
    luaopen_string(L);
    lua_setfield(L, -2, "__index");
    lua_setmetatable(L, -2);
    lua_pop(L, 1);

    char text[64];
    if (rt.options & OPTION_RUNTIME_MSG) {
        snprintf(text, sizeof(text), "Lua: State memory usage: %i", int(memory_in_use(L)));
        send_text(SEVERITY_DEBUG, text);
    }
    if (!load_all_scripts(L)) {
        return 0;
    }

    while (true) {
        take_fault_request();
        if (rt.scripts == nullptr) {
            if (rt.options & OPTION_NO_SCRIPTS_TO_RUN) {
                send_text(SEVERITY_DEBUG, "Lua: No scripts to run");
            }
            sleep_ms(1000);
            continue;
        }
        const int32_t wait_ms = int32_t(rt.scripts->next_run_ms - port_millis());
        if (wait_ms > 0) {
            sleep_ms(uint32_t(wait_ms));
            continue;
        }
        if (rt.options & OPTION_RUNTIME_MSG) {
            snprintf(text, sizeof(text), "Lua: Running %s", rt.scripts->name);
            send_text(SEVERITY_DEBUG, text);
        }
        const uint8_t index = rt.scripts->index;
        const int32_t start_mem = memory_in_use(L);
        const uint32_t start_us = port_micros();
        // rt.scripts may change completely inside: nothing of it is read afterwards
        run_next_script(L);
        const uint32_t end_us = port_micros();
        const int32_t end_mem = memory_in_use(L);
        send_run_stats(index, end_us - start_us, end_mem, end_mem - start_mem);
        lua_gc(L, LUA_GCCOLLECT, 0);
    }
    return 0;
}

// SCR_HEAP_SIZE, SCR_VM_I_COUNT, SCR_DEBUG_OPTS from the M7
static bool configure()
{
    (void)rpc.begin(SystemMethod::Config);
    if (rpc.invoke() != 0 || rpc.status() != ReplyStatus::Ok || rpc.count() != 3) {
        return false;
    }
    for (uint8_t i = 0; i < 3; i++) {
        if (rpc.value(i).tag != ValueTag::Integer) {
            return false;
        }
    }
    const int32_t heap_size = rpc.value(0).integer;
    rt.vm_steps = rpc.value(1).integer;
    rt.options = uint32_t(rpc.value(2).integer);
    const uint32_t capacity = port_heap_capacity();
    rt.heap_limit = heap_size <= 0 ? 0 : (uint32_t(heap_size) < capacity ? uint32_t(heap_size) : capacity);
    return true;
}

void lua_main()
{
    memset(&rt, 0, sizeof(rt));
    while (!configure()) {
        sleep_ms(1000);   // the call timed out: the M7 is busy or restarting us
    }
    lua_State *L = lua_newstate(lua_alloc, nullptr);
    if (L == nullptr) {
        send_text(SEVERITY_CRITICAL, "Lua: Couldn't allocate a lua state");
    } else {
        lua_pushcfunction(L, run_engine);
        if (lua_pcall(L, 0, 0, 0) != LUA_OK) {
            report_error(SEVERITY_CRITICAL, "Engine Error: %s", error_object_message(L));
        }
    }
    // the engine stopped: scripts stay off until the next M4 start; faults are still taken
    while (true) {
        sleep_ms(1000);
    }
}

}  // namespace ScriptingM4

extern "C" int lua_get_current_env_ref(void)
{
    return ScriptingM4::rt.current_env_ref;
}

// require: IO and Manual Bindings sub-phase
extern "C" const char *lua_get_modules_path(void)
{
    return "";
}

// Lua raised an error outside protected mode: fault, so the M7 records it and restarts the M4
extern "C" void lua_abort(void)
{
    ScriptingM4::port_lua_fault(ScriptingM4::FaultKind::HardFault);
}
