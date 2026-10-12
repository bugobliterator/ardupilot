#pragma once

#include "AP_Scripting_config.h"

#if AP_SCRIPTING_M4_ENABLED

#include <stdint.h>
#include <AP_Common/AP_Common.h>
#include "AP_ScriptingM4_Files.h"
#include "AP_ScriptingM4_Quota.h"
#include "AP_ScriptingM4_Rpc.h"

/*
  The M4's files (Lua io, require, dirlist, remove): the m4:fs_* system
  methods on AP_Filesystem, with the reach of in-process scripting less a ".."
  above the root and writes on the "@" filesystems. Paths and ids come from the
  untrusted M4 and are checked here (check_script_path(), FileTable). Failures
  are replies with a negative errno, so the M4's io library returns its usual
  nil, text, errno. Files belong to the M4 session: reset() closes them.
  Host thread only.
 */
class AP_ScriptingM4_FileServer {
public:
    AP_ScriptingM4_FileServer() {}
    CLASS_NO_COPY(AP_ScriptingM4_FileServer);

    // one m4:fs_* call; writes are charged to quota (QuotaClass::FileIo)
    ScriptingM4::ReplyStatus handle(ScriptingM4::SystemMethod method, ScriptingM4::RpcArgs &args,
                                    ScriptingM4::RpcWriter &results, uint8_t &error_arg, ScriptingM4::Quota &quota);
    // closes every file the M4 left open; their ids go stale
    void reset();

private:
    using Status = ScriptingM4::ReplyStatus;

    Status open(ScriptingM4::RpcArgs &args, ScriptingM4::RpcWriter &results, uint8_t &error_arg, ScriptingM4::Quota &quota);
    Status read(ScriptingM4::RpcArgs &args, ScriptingM4::RpcWriter &results, uint8_t &error_arg);
    Status write(ScriptingM4::RpcArgs &args, ScriptingM4::RpcWriter &results, uint8_t &error_arg, ScriptingM4::Quota &quota);
    Status seek(ScriptingM4::RpcArgs &args, ScriptingM4::RpcWriter &results, uint8_t &error_arg);
    Status flush(ScriptingM4::RpcArgs &args, ScriptingM4::RpcWriter &results, uint8_t &error_arg);
    Status close(ScriptingM4::RpcArgs &args, ScriptingM4::RpcWriter &results, uint8_t &error_arg);
    Status remove(ScriptingM4::RpcArgs &args, ScriptingM4::RpcWriter &results, uint8_t &error_arg, ScriptingM4::Quota &quota);
    Status dir(ScriptingM4::RpcArgs &args, ScriptingM4::RpcWriter &results, uint8_t &error_arg);

    ScriptingM4::FileTable _table;
};

#endif  // AP_SCRIPTING_M4_ENABLED
