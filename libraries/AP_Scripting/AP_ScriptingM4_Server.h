#pragma once

#include "AP_Scripting_config.h"

#if AP_SCRIPTING_M4_ENABLED

#include <stddef.h>
#include <stdint.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_ScriptingM4_Rpc.h"
#include "AP_ScriptingM4_ScriptTable.h"

/*
  M7 side of the M4's system methods: configuration, the script list and
  script reads (SD card and ROMFS), script lifecycle reports (loaded,
  stopped, errors, run statistics), and gcs:send_text and print until the
  generated bindings take them over. Arguments come from the untrusted M4
  and are checked as luaL_check* does in-process, with the same texts.
  Runs on the scripting host thread; arming_checks() and the checksums are
  also read from other threads.
 */
class AP_ScriptingM4_Server {
public:
    AP_ScriptingM4_Server();
    CLASS_NO_COPY(AP_ScriptingM4_Server);

    // a new M4 session: the script list, the open file, the checksums and the last error go
    void reset();
    // one Call or Notify; for a Call the results, or the error text, go to results
    ScriptingM4::ReplyStatus handle(uint16_t method, const ScriptingM4::RpcValue *args, uint8_t argc,
                                    ScriptingM4::RpcWriter &results, uint8_t &error_arg);
    // re-print the last script error, as the in-process engine does
    void update(uint32_t now_ms);
    bool arming_checks(size_t buflen, char *buffer) const;
    uint32_t loaded_checksum() const;
    uint32_t running_checksum() const;

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
    mutable HAL_Semaphore _error_sem;
    char _last_error[128];
    uint8_t _error_prints;
    uint32_t _last_error_print_ms;
};

#endif  // AP_SCRIPTING_M4_ENABLED
