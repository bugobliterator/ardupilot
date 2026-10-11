#include "AP_ScriptingM4_Server.h"

#if AP_SCRIPTING_M4_ENABLED

#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_Scripting.h"
#include "AP_ScriptingM4_Bindings.h"
#include "AP_ScriptingM4_Placement.h"
#include "lua_common_defs.h"

extern const AP_HAL::HAL& hal;

using namespace ScriptingM4;

// bytes per m4:read_script reply
static const uint16_t READ_CHUNK = 512;
// the in-process engine re-prints its last error this often, this many times
static const uint32_t ERROR_REPRINT_MS = 10000;
static const uint8_t ERROR_REPRINTS = 10;

static ReplyStatus arg_error(const RpcArgs &args, RpcWriter &results, uint8_t &error_arg)
{
    results.string(args.message(), strlen(args.message()));
    error_arg = args.error_arg();
    return ReplyStatus::ArgError;
}

// a value's text, terminated and cut to fit
static void copy_text(char *buf, size_t size, const char *text, uint16_t length)
{
    const size_t n = length < size - 1 ? length : size - 1;
    memcpy(buf, text, n);
    buf[n] = 0;
}

static bool option_set(AP_Scripting::DebugOption option)
{
    return (AP_Scripting::get_singleton()->debug_options() & uint8_t(option)) != 0;
}

AP_ScriptingM4_Server::AP_ScriptingM4_Server() :
    _fd(-1),
    _fd_index(-1)
{
    _call.handles = &_handles;
    _call.warn = remote_warning;
    _call.warn_context = this;
}

void AP_ScriptingM4_Server::reset(uint32_t session)
{
    close_file();
    {
        WITH_SEMAPHORE(_table_sem);
        _table.clear();
    }
    _listed = false;
    _next_read_index = 0;
    // handles of the previous session are stale from here on, even if the M4 repeats
    // its session number: the table is emptied unconditionally
    _handles.reset(session);
    _quota.reset(AP_HAL::millis());
    WITH_SEMAPHORE(_error_sem);
    _last_error[0] = 0;
    _error_prints = ERROR_REPRINTS;
    _started = false;
}

ReplyStatus AP_ScriptingM4_Server::handle(uint16_t method, const RpcValue *values, uint8_t argc,
                                          RpcWriter &results, uint8_t &error_arg)
{
    if (method >= FIRST_BINDING_METHOD) {
        return binding_call(method, values, argc, results, error_arg);
    }
    RpcArgs args(values, argc);
    switch (SystemMethod(method)) {
    case SystemMethod::Config:
        return config(args, results, error_arg);
    case SystemMethod::ListScript:
        return list_script(args, results, error_arg);
    case SystemMethod::ReadScript:
        return read_script(args, results, error_arg);
    case SystemMethod::ScriptLoaded:
        return script_loaded(args);
    case SystemMethod::ScriptStopped:
        return script_stopped(args);
    case SystemMethod::ScriptError:
        return script_error(args);
    case SystemMethod::RunStats:
        return run_stats(args);
    case SystemMethod::Started:
        return started(args);
    case SystemMethod::Noop:
        return ReplyStatus::Ok;
    case SystemMethod::RuntimeText:
        return gcs_send_text(args, results, error_arg);
    case SystemMethod::Print:
        return print(args, results, error_arg);
    case SystemMethod::Bindings:
        return bindings(args, results, error_arg);
    case SystemMethod::Other:
    case SystemMethod::SystemOther:
        return ReplyStatus::Malformed;   // a stats row name, never a call
    }
    return ReplyStatus::UnknownMethod;
}

// m4:config() -> SCR_HEAP_SIZE, SCR_VM_I_COUNT, SCR_DEBUG_OPTS
ReplyStatus AP_ScriptingM4_Server::config(RpcArgs &args, RpcWriter &results, uint8_t &error_arg)
{
    if (!args.count_is(0)) {
        return arg_error(args, results, error_arg);
    }
    const AP_Scripting *scripting = AP_Scripting::get_singleton();
    results.integer(scripting->heap_size());
    results.integer(scripting->vm_steps());
    results.integer(scripting->debug_options());
    return ReplyStatus::Ok;
}

// m4:list_script(index) -> path, or nil past the end; the list is built at the session's first call
ReplyStatus AP_ScriptingM4_Server::list_script(RpcArgs &args, RpcWriter &results, uint8_t &error_arg)
{
    int32_t index = 0;
    if (!args.count_is(1) || !args.integer(1, index)) {
        return arg_error(args, results, error_arg);
    }
    if (!_listed) {
        build_list();
    }
    const char *path = _table.path(index);
    if (path == nullptr) {
        results.nil();
    } else {
        results.string(path, strlen(path));
    }
    return ReplyStatus::Ok;
}

// the in-process loader's directories and filters (lua_scripts::load_all_scripts_in_dir)
void AP_ScriptingM4_Server::build_list()
{
    _listed = true;
    const uint16_t dir_disable = AP_Scripting::get_singleton()->get_disabled_dir();
    bool listed = false;
    if ((dir_disable & uint16_t(AP_Scripting::SCR_DIR::SCRIPTS)) == 0) {
        list_dir(SCRIPTING_DIRECTORY);
        listed = true;
    }
#ifdef HAL_HAVE_AP_ROMFS_EMBEDDED_LUA
    if ((dir_disable & uint16_t(AP_Scripting::SCR_DIR::ROMFS)) == 0) {
        list_dir("@ROMFS/scripts");
        listed = true;
    }
#endif
    if (!listed) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Lua: All directory's disabled see SCR_DIR_DISABLE");
    }
}

void AP_ScriptingM4_Server::list_dir(const char *dirname)
{
    auto *d = AP::FS().opendir(dirname);
    if (d == nullptr) {
        // 0 without a real filesystem: no warning then (as in-process)
        if (AP::FS().disk_space(dirname) != 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Lua: open directory (%s) failed", dirname);
        }
        return;
    }
    for (struct dirent *de = AP::FS().readdir(d); de; de = AP::FS().readdir(d)) {
        const size_t length = strlen(de->d_name);
        if (length < 5 || de->d_name[0] == '.' || strncmp(&de->d_name[length - 4], ".lua", 4) != 0) {
            continue;
        }
        char path[MAX_SCRIPT_PATH];
        if (hal.util->snprintf(path, sizeof(path), "%s/%s", dirname, de->d_name) >= int(sizeof(path))) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Lua: M4 skips a script: path over %u chars",
                          unsigned(MAX_SCRIPT_PATH - 1));
            continue;
        }
        uint32_t crc = 0;
        IGNORE_RETURN(AP::FS().crc32(path, crc));   // 0 when unreadable: no effect on the checksums
        bool added;
        {
            WITH_SEMAPHORE(_table_sem);
            added = _table.add(path, crc);
        }
        if (!added) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Lua: M4 runs at most %u scripts", unsigned(MAX_SCRIPTS));
            break;
        }
    }
    AP::FS().closedir(d);
}

// m4:read_script(index, offset) -> up to READ_CHUNK bytes ("" at the end), or nil if unreadable;
// an error once the script has been closed (end, unreadable, or a later script opened)
ReplyStatus AP_ScriptingM4_Server::read_script(RpcArgs &args, RpcWriter &results, uint8_t &error_arg)
{
    int32_t index = 0;
    int32_t offset = 0;
    if (!args.count_is(2) || !args.integer(1, index) || !args.integer_in(2, 0, INT32_MAX, offset)) {
        return arg_error(args, results, error_arg);
    }
    const char *path = _table.path(index);
    if (path == nullptr) {
        results.nil();
        return ReplyStatus::Ok;
    }
    // scripts are read once each, in list order: an M4 that goes back cannot make
    // the M7 reopen (and on ROMFS decompress) a file again
    if (index < _next_read_index) {
        static const char out_of_order[] = "M4: script read out of order";
        results.string(out_of_order, sizeof(out_of_order) - 1);
        return ReplyStatus::Error;
    }
    if (_fd < 0 || _fd_index != index) {
        close_file();
        _next_read_index = index;
        _fd = AP::FS().open(path, O_RDONLY);
        if (_fd < 0) {
            _next_read_index = index + 1;
            results.nil();
            return ReplyStatus::Ok;
        }
        _fd_index = index;
    }
    uint8_t chunk[READ_CHUNK];
    int32_t n = -1;
    if (AP::FS().lseek(_fd, offset, SEEK_SET) == offset) {
        n = AP::FS().read(_fd, chunk, sizeof(chunk));
    }
    if (n <= 0) {
        close_file();
    }
    if (n == 0) {
        // end of file: the M4 has been served the whole script
        WITH_SEMAPHORE(_table_sem);
        _table.mark_served(index);
    }
    if (n < 0) {
        results.nil();
        return ReplyStatus::Ok;
    }
    results.string(reinterpret_cast<const char *>(chunk), size_t(n));
    return ReplyStatus::Ok;
}

void AP_ScriptingM4_Server::close_file()
{
    if (_fd >= 0) {
        AP::FS().close(_fd);
        _next_read_index = _fd_index + 1;
    }
    _fd = -1;
    _fd_index = -1;
}

// m4:script_loaded(index) (notify)
ReplyStatus AP_ScriptingM4_Server::script_loaded(RpcArgs &args)
{
    int32_t index = 0;
    if (!args.count_is(1) || !args.integer(1, index)) {
        return ReplyStatus::Malformed;
    }
    {
        WITH_SEMAPHORE(_table_sem);
        if (!_table.mark_loaded(index)) {
            return ReplyStatus::Malformed;
        }
    }
#if HAL_LOGGER_FILE_CONTENTS_ENABLED
    if (!option_set(AP_Scripting::DebugOption::SUPPRESS_SCRIPT_LOG)) {
        AP::logger().log_file_content(_table.path(index));
    }
#endif
    return ReplyStatus::Ok;
}

// m4:script_stopped(index) (notify)
ReplyStatus AP_ScriptingM4_Server::script_stopped(RpcArgs &args)
{
    int32_t index = 0;
    if (!args.count_is(1) || !args.integer(1, index)) {
        return ReplyStatus::Malformed;
    }
    WITH_SEMAPHORE(_table_sem);
    return _table.mark_stopped(index) ? ReplyStatus::Ok : ReplyStatus::Malformed;
}

// m4:script_error(severity, text) (notify): lua_scripts::set_and_print_new_error_message()
ReplyStatus AP_ScriptingM4_Server::script_error(RpcArgs &args)
{
    int32_t severity = 0;
    const char *text = nullptr;
    uint16_t length = 0;
    if (!args.count_is(2) || !args.integer_in(1, MAV_SEVERITY_EMERGENCY, MAV_SEVERITY_DEBUG, severity) ||
        !args.string(2, text, length)) {
        return ReplyStatus::Malformed;
    }
    char message[sizeof(_last_error)];
    copy_text(message, sizeof(message), text, length);
    set_error(uint8_t(severity), message);
    return ReplyStatus::Ok;
}

// m4:run_stats(index, run_time_us, total_mem, run_mem) (notify): lua_scripts::update_stats()
ReplyStatus AP_ScriptingM4_Server::run_stats(RpcArgs &args)
{
    int32_t index = 0;
    int32_t run_time = 0;
    int32_t total_mem = 0;
    int32_t run_mem = 0;
    if (!args.count_is(4) || !args.integer(1, index) || !args.integer(2, run_time) ||
        !args.integer(3, total_mem) || !args.integer(4, run_mem)) {
        return ReplyStatus::Malformed;
    }
    const char *name = _table.path(index);
    if (name == nullptr) {
        return ReplyStatus::Malformed;
    }
    if (option_set(AP_Scripting::DebugOption::RUNTIME_MSG)) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Lua: Time: %u Mem: %d + %d",
                      unsigned(run_time), int(total_mem), int(run_mem));
    }
#if HAL_LOGGING_ENABLED
    if (option_set(AP_Scripting::DebugOption::LOG_RUNTIME)) {
        struct log_Scripting pkt {
            LOG_PACKET_HEADER_INIT(LOG_SCRIPTING_MSG),
            time_us      : AP_HAL::micros64(),
            name         : {},
            run_time     : uint32_t(run_time),
            total_mem    : total_mem,
            run_mem      : run_mem
        };
        const char *name_short = strrchr(name, '/');
        if ((strlen(name) > sizeof(pkt.name)) && (name_short != nullptr)) {
            strncpy_noterm(pkt.name, name_short + 1, sizeof(pkt.name));
        } else {
            strncpy_noterm(pkt.name, name, sizeof(pkt.name));
        }
        AP::logger().WriteBlock(&pkt, sizeof(pkt));
    }
#endif
    return ReplyStatus::Ok;
}

// m4:started(scripts, heap bytes) (notify)
ReplyStatus AP_ScriptingM4_Server::started(RpcArgs &args)
{
    int32_t scripts = 0;
    int32_t heap = 0;
    // the heap lives in M4 RAM: the bound also keeps the text within 50 characters
    if (!args.count_is(2) || !args.integer_in(1, 0, MAX_SCRIPTS, scripts) ||
        !args.integer_in(2, 0, int32_t(AP_SCRIPTING_M4_RAM_SIZE), heap)) {
        return ReplyStatus::Malformed;
    }
    {
        WITH_SEMAPHORE(_error_sem);
        _started = true;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scripting: started on M4 (%u scripts, heap %uKB)",
                  unsigned(scripts), unsigned(heap / 1024));
    return ReplyStatus::Ok;
}

// m4:send_text(self, severity, text): the runtime's own texts, with the generated GCS_send_text() checks
ReplyStatus AP_ScriptingM4_Server::gcs_send_text(RpcArgs &args, RpcWriter &results, uint8_t &error_arg)
{
    int32_t severity = 0;
    const char *text = nullptr;
    uint16_t length = 0;
    if (!args.count_is(3) || !args.integer_in(2, MAV_SEVERITY_EMERGENCY, MAV_SEVERITY_DEBUG, severity) ||
        !args.string(3, text, length)) {
        return arg_error(args, results, error_arg);
    }
    // an M4 flooding statustexts must not push out the vehicle's own: the runtime's
    // texts keep the Text quota of gcs:send_text
    if (!_quota.allow(QuotaClass::Text, AP_HAL::millis())) {
        return ReplyStatus::Ok;   // dropped, as from a full statustext queue
    }
    char message[256];
    copy_text(message, sizeof(message), text, length);
    GCS_SEND_TEXT(MAV_SEVERITY(severity), "%s", message);
    return ReplyStatus::Ok;
}

// print(text): lua_print()
ReplyStatus AP_ScriptingM4_Server::print(RpcArgs &args, RpcWriter &results, uint8_t &error_arg)
{
    const char *text = nullptr;
    uint16_t length = 0;
    if (!args.count_is(1) || !args.string(1, text, length)) {
        return arg_error(args, results, error_arg);
    }
    if (!_quota.allow(QuotaClass::Text, AP_HAL::millis())) {
        return ReplyStatus::Ok;   // dropped, as gcs:send_text over its quota
    }
    char message[256];
    copy_text(message, sizeof(message), text, length);
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "%s", message);
    return ReplyStatus::Ok;
}

// m4:bindings() -> what this firmware compiled in and the enum values (AP_ScriptingM4_Bindings.h)
ReplyStatus AP_ScriptingM4_Server::bindings(RpcArgs &args, RpcWriter &results, uint8_t &error_arg)
{
    if (!args.count_is(0)) {
        return arg_error(args, results, error_arg);
    }
#if AP_SCRIPTING_M4_FAULT_INJECTION_ENABLED
    if (_refuse_bindings) {
        // an error reply, not Malformed: a refused image is no protocol error
        static const char refused[] = "M4: bindings refused";
        results.string(refused, sizeof(refused) - 1);
        return ReplyStatus::Error;
    }
#endif
    uint8_t info[BINDING_INFO_SIZE];
    ScriptingM4Remote::binding_info(info);
    results.string(reinterpret_cast<const char *>(info), sizeof(info));
    return ReplyStatus::Ok;
}

// a generated binding (gen-bindings -r): its quota, then the in-process code through the adapter
ReplyStatus AP_ScriptingM4_Server::binding_call(uint16_t method, const RpcValue *values, uint8_t argc,
                                                RpcWriter &results, uint8_t &error_arg)
{
    const uint8_t flags = ScriptingM4Remote::method_flags(method);
    if (!_quota.allow(QuotaClass(flags >> 4), AP_HAL::millis())) {
        if ((flags & ScriptingM4Remote::METHOD_QUOTA_FALSE) != 0) {
            results.boolean(false);   // over its quota the call fails, as it can in-process
        }
        return ReplyStatus::Ok;       // or it is dropped, like a statustext that finds its queue full
    }
    return ScriptingM4Remote::dispatch(method, values, argc, _call, results, error_arg);
}

// lua_scripts::set_and_print_new_error_message(): printed, and kept as the prearm failure
void AP_ScriptingM4_Server::set_error(uint8_t severity, const char *text)
{
    {
        WITH_SEMAPHORE(_error_sem);
        hal.util->snprintf(_last_error, sizeof(_last_error), "%s", text);
        _error_prints = 0;
        _last_error_print_ms = AP_HAL::millis();
    }
    GCS_SEND_TEXT(MAV_SEVERITY(severity), "Lua: %s", text);
}

// a generated binding's warning (deprecated method, userdata creation with arguments)
void AP_ScriptingM4_Server::remote_warning(void *context, uint8_t severity, const char *text)
{
    static_cast<AP_ScriptingM4_Server *>(context)->set_error(MIN(severity, uint8_t(MAV_SEVERITY_DEBUG)), text);
}

void AP_ScriptingM4_Server::update(uint32_t now_ms)
{
    WITH_SEMAPHORE(_error_sem);
    if (_last_error[0] == 0 || _error_prints >= ERROR_REPRINTS || now_ms - _last_error_print_ms <= ERROR_REPRINT_MS) {
        return;
    }
    // the buffer is kept after the last print: it stays the prearm failure
    _last_error_print_ms = now_ms;
    _error_prints++;
    GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "Lua: %s", _last_error);
}

uint32_t AP_ScriptingM4_Server::loaded_checksum() const
{
    WITH_SEMAPHORE(_table_sem);
    return _table.loaded_checksum();
}

uint32_t AP_ScriptingM4_Server::running_checksum() const
{
    WITH_SEMAPHORE(_table_sem);
    return _table.running_checksum();
}

bool AP_ScriptingM4_Server::arming_checks(size_t buflen, char *buffer) const
{
    WITH_SEMAPHORE(_error_sem);
    if (_last_error[0] != 0) {
        hal.util->snprintf(buffer, buflen, "Scripting: %s", _last_error);
        return false;
    }
    // fail closed until the M4 has listed and loaded its scripts: a lost error
    // report must not let the vehicle arm without them
    if (!_started) {
        hal.util->snprintf(buffer, buflen, "Scripting: M4 scripts not started");
        return false;
    }
    return true;
}

#endif  // AP_SCRIPTING_M4_ENABLED
