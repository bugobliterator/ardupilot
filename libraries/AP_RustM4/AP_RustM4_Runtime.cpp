#include "AP_RustM4_Runtime.h"

#if AP_SCRIPTING_RUST_ENABLED && AP_RUST_M4_ENABLED

#include <AP_Rust/Runtime.h>
#include "AP_RustM4.h"

namespace AP_Rust
{

Runtime *Runtime::_singleton;

const AP_Param::GroupInfo Runtime::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Native Rust modules enable
    // @Description: Enable experimental native Rust modules. On this board they run unprivileged on the Cortex-M4 under its MPU.
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, Runtime, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: HEAP_SIZE
    // @DisplayName: Rust module initial heap size
    // @Description: Initial allocation arena size per module. May expand while armed unless DEBUG_OPTS bit 6 is set, retaining 20 KiB of system memory. Excludes code, stack and allocator metadata.
    // @Units: B
    // @Range: 256 16777216
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("HEAP_SIZE", 2, Runtime, _heap_size, 32768),

    // @Param: CODE_SIZE
    // @DisplayName: Rust module image limit
    // @Description: Per-module code and data image limit in bytes, rounded up to a multiple of 4096. Only the loaded image is allocated, not this entire limit. Alignment and allocator overhead are additional.
    // @Units: B
    // @Range: 4096 16777216
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("CODE_SIZE", 3, Runtime, _code_size, 131072),

    // @Param: STACK_SIZE
    // @DisplayName: Rust module stack limit
    // @Description: Requested stack per module in bytes, including host callback margin. Rounded up to the platform minimum and alignment. OS and guard overhead are additional.
    // @Units: B
    // @Range: 8192 16777216
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("STACK_SIZE", 4, Runtime, _stack_size, 32768),

    // @Param: MAX_MOD
    // @DisplayName: Maximum Rust modules
    // @Description: Maximum unique module filenames loaded across ROMFS and the writable module directory. A ROMFS module overrides a writable module with the same filename. Extra unique files cause a load failure rather than being silently ignored.
    // @Range: 1 8
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("MAX_MOD", 5, Runtime, _max_modules, 2),

    // @Param: RESTART
    // @DisplayName: Rust restart request counter
    // @Description: Change this value to request cooperative restart of all Rust modules, including while armed. Non-cooperating workers prevent reload and retain their memory.
    // @Range: 0 32767
    // @User: Advanced
    AP_GROUPINFO("RESTART", 6, Runtime, _restart, 0),

    // @Param: LD_CSUM
    // @DisplayName: Required loaded Rust module checksum
    // @Description: Required masked XOR of whole-file CRC32 values of loaded modules. Minus one disables this pre-arm requirement.
    // @Range: -1 8388607
    // @User: Advanced
    AP_GROUPINFO("LD_CSUM", 7, Runtime, _loaded_required, -1),

    // @Param: RUN_CSUM
    // @DisplayName: Required running Rust module checksum
    // @Description: Required masked XOR of whole-file CRC32 values of running modules. Minus one disables this pre-arm requirement.
    // @Range: -1 8388607
    // @User: Advanced
    AP_GROUPINFO("RUN_CSUM", 8, Runtime, _running_required, -1),

    // @Param: DEBUG_OPTS
    // @DisplayName: Rust module debug options
    // @Description: Disable Rust pre-arm checks, save current loaded and running checksums, or disable heap expansion. The save bit clears after storing both values. Changes to the heap expansion option take effect on module restart.
    // @Bitmask: 4:Disable pre-arm checks,5:Save checksums,6:Disable heap expansion
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
    AP_RustM4 *m4 = AP_RustM4::get_singleton();
    if (m4 != nullptr) {
        m4->init(config());
    }
}

void Runtime::update()
{
    AP_RustM4 *m4 = AP_RustM4::get_singleton();
    if (m4 != nullptr) {
        m4->update(config());
    }
}

bool Runtime::arming_checks(size_t length, char *message) const
{
    const AP_RustM4 *m4 = AP_RustM4::get_singleton();
    return m4 == nullptr || m4->arming_checks(length, message);
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

#endif  // AP_SCRIPTING_RUST_ENABLED && AP_RUST_M4_ENABLED
