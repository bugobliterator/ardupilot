#pragma once

#include "AP_Scripting_config.h"

#if AP_SCRIPTING_M4_ENABLED

#include <stddef.h>
#include <stdint.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_ScriptingM4_FileServer.h"
#include "AP_ScriptingM4_Handles.h"
#include "AP_ScriptingM4_Objects.h"
#include "AP_ScriptingM4_Quota.h"
#include "AP_ScriptingM4_Remote.h"
#include "AP_ScriptingM4_Rpc.h"
#include "AP_ScriptingM4_ScriptTable.h"

/*
  M7 side of the M4's system methods: configuration, the script list and
  script reads (SD card and ROMFS), script lifecycle reports (loaded,
  stopped, errors, run statistics), print, the m4:bindings image, and the
  generated binding calls (ids from FIRST_BINDING_METHOD) with their quotas,
  handles and owned objects. Arguments come from the untrusted M4 and are
  checked as luaL_check* does in-process, with the same texts.
  Runs on the scripting host thread; arming_checks() and the checksums are
  also read from other threads. handle() and reset() are host thread only:
  the generated dispatch keeps per-call state that no other thread may touch.
 */
class AP_ScriptingM4_Server {
public:
    AP_ScriptingM4_Server();
    CLASS_NO_COPY(AP_ScriptingM4_Server);

    // a new M4 session (its peer-header session number): the script list, the open file,
    // the checksums, the last error and the previous session's handles go. Host thread
    // only, never during a call; called at every M4 start and restart
    void reset(uint32_t session);
    // the M4's files: closed when scripting goes off. Host thread, or init() before it runs
    void close_files() { _files.reset(); }
    // calls the quotas refused since the last call (M4ST QHit)
    uint32_t take_quota_hits() { return _quota.take_hits(); }
    // one Call or Notify; for a Call the results, or the error text, go to results
    ScriptingM4::ReplyStatus handle(uint16_t method, const ScriptingM4::RpcValue *args, uint8_t argc,
                                    ScriptingM4::RpcWriter &results, uint8_t &error_arg);
    // re-print the last script error, as the in-process engine does
    void update(uint32_t now_ms);
    bool arming_checks(size_t buflen, char *buffer) const;
    uint32_t loaded_checksum() const;
    uint32_t running_checksum() const;
#if AP_SCRIPTING_M4_FAULT_INJECTION_ENABLED
    // test only: m4:bindings answers an error, so the M4 runs no script
    void set_refuse_bindings(bool refuse) { _refuse_bindings = refuse; }
#endif

private:
    using Status = ScriptingM4::ReplyStatus;

    Status config(ScriptingM4::RpcArgs &args, ScriptingM4::RpcWriter &results, uint8_t &error_arg);
    Status list_script(ScriptingM4::RpcArgs &args, ScriptingM4::RpcWriter &results, uint8_t &error_arg);
    Status read_script(ScriptingM4::RpcArgs &args, ScriptingM4::RpcWriter &results, uint8_t &error_arg);
    Status script_loaded(ScriptingM4::RpcArgs &args);
    Status script_stopped(ScriptingM4::RpcArgs &args);
    Status script_error(ScriptingM4::RpcArgs &args);
    Status run_stats(ScriptingM4::RpcArgs &args);
    Status started(ScriptingM4::RpcArgs &args);
    Status gcs_send_text(ScriptingM4::RpcArgs &args, ScriptingM4::RpcWriter &results, uint8_t &error_arg);
    Status print(ScriptingM4::RpcArgs &args, ScriptingM4::RpcWriter &results, uint8_t &error_arg);
    Status bindings(ScriptingM4::RpcArgs &args, ScriptingM4::RpcWriter &results, uint8_t &error_arg);
    Status binding_call(uint16_t method, const ScriptingM4::RpcValue *args, uint8_t argc,
                        ScriptingM4::RpcWriter &results, uint8_t &error_arg);
    void set_error(uint8_t severity, const char *text);
    static void remote_warning(void *context, uint8_t severity, const char *text);
    void build_list();
    void list_dir(const char *dirname);
    void close_file();

    // written by the host thread only, so its own reads need no lock; the
    // checksums are read from other threads under _table_sem
    ScriptingM4::ScriptTable _table;
    mutable HAL_Semaphore _table_sem;
    bool _listed;
    int _fd;
    int32_t _fd_index;
    int32_t _next_read_index;   // lowest script index read_script may still open
    mutable HAL_Semaphore _error_sem;
    char _last_error[128];
    uint8_t _error_prints;
    uint32_t _last_error_print_ms;
    bool _started;   // m4:started of this session received; under _error_sem
    ScriptingM4::HandleTable _handles;
    ScriptingM4::ObjectPool _objects;   // the M4 scripts' owned objects (Parameter, ports, menus)
    ScriptingM4::Quota _quota;
    AP_ScriptingM4_FileServer _files;        // the M4's open files (m4:fs_*)
    ScriptingM4Remote::RemoteCall _call;   // the generated dispatch's call state (about 2.8 KB)
#if AP_SCRIPTING_M4_FAULT_INJECTION_ENABLED
    bool _refuse_bindings;
#endif
};

#endif  // AP_SCRIPTING_M4_ENABLED
