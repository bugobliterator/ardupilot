#include "AP_ModuleM4_Runtime.h"

#if AP_MODULE_ENABLED && AP_MODULE_M4_ENABLED

#include <AP_Module/Runtime.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_ModuleM4.h"

namespace AP_Module
{

Runtime *Runtime::_singleton;

const AP_Param::GroupInfo Runtime::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Native modules enable
    // @Description: Enable experimental native modules. On this board they run unprivileged on the Cortex-M4 under its MPU.
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, Runtime, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: HEAP_MAX
    // @DisplayName: Native module heap capacity limit
    // @Description: Maximum backing heap arena capacity per module. Initial size comes from the module file. Armed expansion is capped here and retains 20 KiB of system memory unless disabled by DEBUG_OPTS bit 6. Excludes code, stack and allocator metadata. On the M4 the heap does not expand: a module gets its initial size, at most 64 KiB.
    // @Units: B
    // @Range: 256 16777216
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("HEAP_MAX", 2, Runtime, _heap_size, 32768),

    // @Param: CODE_MAXSIZE
    // @DisplayName: Native module image limit
    // @Description: Per-module code and data image limit in bytes, rounded up to a multiple of 4096. Only the loaded image is allocated, not this entire limit. Alignment and allocator overhead are additional.
    // @Units: B
    // @Range: 4096 16777216
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("CODE_MAXSIZE", 3, Runtime, _code_size, 131072),

    // @Param: STACK_MAX
    // @DisplayName: Native module stack limit
    // @Description: Maximum logical worker stack per module, including the declared module requirement plus host callback margin. Only the requested stack is allocated, rounded to platform minimum and alignment. OS and guard overhead are additional.
    // @Units: B
    // @Range: 256 16777216
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("STACK_MAX", 4, Runtime, _stack_size, 32768),

    // @Param: MAX_MOD
    // @DisplayName: Maximum Native modules
    // @Description: Maximum unique module filenames loaded across ROMFS and the writable module directory. A ROMFS module overrides a writable module with the same filename. Extra unique files cause a load failure rather than being silently ignored.
    // @Range: 1 8
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("MAX_MOD", 5, Runtime, _max_modules, 2),

    // @Param: RESTART
    // @DisplayName: Native module restart request counter
    // @Description: Change this value to request cooperative restart of all Native modules, including while armed. Non-cooperating workers prevent reload and retain their memory.
    // @Range: 0 32767
    // @User: Advanced
    AP_GROUPINFO("RESTART", 6, Runtime, _restart, 0),

    // @Param: LD_CSUM
    // @DisplayName: Required loaded Native module checksum
    // @Description: Required masked XOR of whole-file CRC32 values of loaded modules. Minus one disables this pre-arm requirement.
    // @Range: -1 8388607
    // @User: Advanced
    AP_GROUPINFO("LD_CSUM", 7, Runtime, _loaded_required, -1),

    // @Param: RUN_CSUM
    // @DisplayName: Required running Native module checksum
    // @Description: Required masked XOR of whole-file CRC32 values of running modules. Minus one disables this pre-arm requirement.
    // @Range: -1 8388607
    // @User: Advanced
    AP_GROUPINFO("RUN_CSUM", 8, Runtime, _running_required, -1),

    // @Param: DEBUG_OPTS
    // @DisplayName: Native module debug options
    // @Description: Disable module pre-arm checks or save current loaded and running checksums. The save bit clears after storing both values. On the M4 the heap does not expand and the call latency probe is MOD_M4_OPTS bit 2.
    // @Bitmask: 4:Disable pre-arm checks,5:Save checksums
    // @User: Advanced
    AP_GROUPINFO("DEBUG_OPTS", 9, Runtime, _debug_options, 0),
    AP_GROUPEND
};

Runtime::Runtime()
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

Runtime::Config Runtime::config() const
{
    return Config { _enable.get(), _heap_size.get(), _code_size.get(), _stack_size.get(), _max_modules.get(),
                    _restart.get(), _loaded_required.get(), _running_required.get(), _debug_options.get() };
}

void Runtime::init()
{
    AP_ModuleM4 *m4 = AP_ModuleM4::get_singleton();
    if (m4 != nullptr) {
        m4->init(config());
    }
}

void Runtime::update()
{
    AP_ModuleM4 *m4 = AP_ModuleM4::get_singleton();
    if (m4 == nullptr) {
        return;
    }
    m4->update(config());
    // MOD_DEBUG_OPTS bit 5, as the PR: the M7's checksums saved as the required ones, then cleared
    if (_enable != 0 && (uint8_t(_debug_options.get()) & (1U << 5)) != 0) {
        uint32_t loaded, running;
        m4->checksums(loaded, running);
        _loaded_required.set_and_save(loaded);
        _running_required.set_and_save(running);
        _debug_options.set_and_save(uint8_t(_debug_options.get()) & ~(1U << 5));
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Module: saved checksums");
    }
}

bool Runtime::arming_checks(size_t length, char *message) const
{
    const AP_ModuleM4 *m4 = AP_ModuleM4::get_singleton();
    return m4 == nullptr || m4->arming_checks(length, message, config());
}

#if AP_MISSION_ENABLED
void Runtime::handle_mission_command(uint16_t p1, float content_p1, float content_p2, float content_p3)
{
    if (_enable == 0) {
        return;
    }
    WITH_SEMAPHORE(_mission_sem);
    if (_mission_count >= MISSION_QUEUE_SIZE) {
        // Match Lua's ObjectBuffer: a full queue drops the newest command.
        return;
    }
    const uint8_t tail = (_mission_head + _mission_count) % MISSION_QUEUE_SIZE;
    _mission_commands[tail] = {p1, content_p1, content_p2, content_p3, AP_HAL::millis()};
    _mission_count++;
}

bool Runtime::pop_mission_command(MissionCommand &command)
{
    WITH_SEMAPHORE(_mission_sem);
    if (_mission_count == 0) {
        return false;
    }
    command = _mission_commands[_mission_head];
    _mission_head = (_mission_head + 1) % MISSION_QUEUE_SIZE;
    _mission_count--;
    return true;
}
#endif // AP_MISSION_ENABLED

}

#endif  // AP_MODULE_ENABLED && AP_MODULE_M4_ENABLED
