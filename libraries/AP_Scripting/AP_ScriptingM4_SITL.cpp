#include "AP_ScriptingM4.h"

#if AP_SCRIPTING_M4_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <errno.h>
#include <limits.h>
#include <signal.h>
#include <spawn.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <CubePilotFW/Channel.h>
#include <CubePilotFW/PeerHeader.h>
#include <CubePilotFW/pal/Posix.h>
#include <CubePilotFW/pal/PosixChannel.h>
#include "AP_ScriptingM4_Layout.h"

extern char **environ;
extern const AP_HAL::HAL& hal;

namespace PeerHeader = CubePilotFW::PeerHeader;

// SupervisorBlock.reserved[0]: pid of the M4 process this M7 spawned
static const uint8_t M4_PID_SLOT = 0;

// SCRIPTING_M4_BIN, else scripting_m4 next to this executable
static bool m4_binary_path(char *path, size_t len)
{
    const char *configured = getenv("SCRIPTING_M4_BIN");
    if (configured != nullptr) {
        return hal.util->snprintf(path, len, "%s", configured) < int(len);
    }
    char self[PATH_MAX] {};
#if defined(__APPLE__)
    uint32_t size = sizeof(self);
    if (_NSGetExecutablePath(self, &size) != 0) {
        return false;
    }
#else
    if (readlink("/proc/self/exe", self, sizeof(self) - 1) <= 0) {
        return false;
    }
#endif
    char *slash = strrchr(self, '/');
    if (slash == nullptr) {
        return false;
    }
    *slash = 0;
    return hal.util->snprintf(path, len, "%s/scripting_m4", self) < int(len);
}

// a SITL reboot re-executes this process with the same pid: the M4 process of
// the previous run is still our child and still holds the M4 core lock
static void kill_stale_m4(ScriptingM4::SupervisorBlock *block)
{
    const pid_t stale = pid_t(block->reserved[M4_PID_SLOT].load());
    if (stale <= 0) {
        return;
    }
    int status = 0;
    // 0: our child and alive; anything else (exited, or not our child) is left alone
    if (waitpid(stale, &status, WNOHANG) == 0) {
        kill(stale, SIGKILL);
        waitpid(stale, &status, 0);
    }
}

bool AP_ScriptingM4::platform_start(char *reason, size_t reason_len)
{
    char name[32];
    hal.util->snprintf(name, sizeof(name), "/cfm4lua_%d", int(getpid()));
    // read by PosixChannel::start here and inherited by the M4 process
    setenv("CUBEFW_SHM_NAME", name, 1);
    _shared = cubefw_posix_attach(name);
    if (_shared == nullptr) {
        hal.util->snprintf(reason, reason_len, "shm errno %d", errno);
        return false;
    }
    kill_stale_m4(ScriptingM4::supervisor_block(_shared));
    memset(_shared, 0, ScriptingM4::WINDOW_SIZE);
    ScriptingM4::SupervisorBlock *block = ScriptingM4::supervisor_block(_shared);
    block->run_permit.store(1);
    block->restart_grant.store(0);
    block->magic.store(ScriptingM4::SUPERVISOR_MAGIC);
    const int ret = CubePilotFW::PosixChannel::start(*_channel, CubePilotFW::Channel::Core::M7,
                                                     ScriptingM4::DOORBELL_GROUP,
                                                     ScriptingM4::CHANNEL_WINDOW_SIZE);
    if (ret != 0) {
        _shared = nullptr;   // start() detached the segment
        hal.util->snprintf(reason, reason_len, "channel start %d", ret);
        return false;
    }
    PeerHeader::publish(_shared, PeerHeader::Core::M7, ScriptingM4::PROTOCOL_VERSION, ScriptingM4::CONTRACT_HASH, 0);
    return spawn_m4(reason, reason_len);
}

bool AP_ScriptingM4::spawn_m4(char *reason, size_t reason_len)
{
    char path[PATH_MAX];
    if (!m4_binary_path(path, sizeof(path))) {
        hal.util->snprintf(reason, reason_len, "no M4 binary");
        return false;
    }
    char *const argv[] = { path, nullptr };
    pid_t pid = 0;
    const int err = posix_spawn(&pid, path, nullptr, nullptr, argv, environ);
    if (err != 0) {
        hal.util->snprintf(reason, reason_len, "spawn errno %d", err);
        return false;
    }
    _m4_pid = pid;
    ScriptingM4::supervisor_block(_shared)->reserved[M4_PID_SLOT].store(uint32_t(pid));
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Scripting: M4 process pid %d", int(pid));
    return true;
}

void AP_ScriptingM4::platform_request_restart()
{
    if (_m4_pid > 0) {
        kill(_m4_pid, SIGUSR1);   // the M4 process re-executes itself
        return;
    }
    char reason[40] {};
    if (!spawn_m4(reason, sizeof(reason))) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Scripting: M4 %s", reason);
    }
}

bool AP_ScriptingM4::platform_m4_exited()
{
    if (_m4_pid <= 0) {
        return false;
    }
    int status = 0;
    if (waitpid(_m4_pid, &status, WNOHANG) != _m4_pid) {
        return false;
    }
    _m4_pid = 0;
    if (_shared != nullptr) {
        ScriptingM4::supervisor_block(_shared)->reserved[M4_PID_SLOT].store(0);
    }
    return true;
}

void AP_ScriptingM4::platform_stop()
{
    if (_m4_pid > 0) {
        kill(_m4_pid, SIGKILL);   // reaped by platform_m4_exited() from the Off loop
    }
}

uint32_t AP_ScriptingM4::platform_wait_us() const
{
    return 1000;   // POSIX HSEM raises no interrupt: poll
}

#endif  // AP_SCRIPTING_M4_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL
