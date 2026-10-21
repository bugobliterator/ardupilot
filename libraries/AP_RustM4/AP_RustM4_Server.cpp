#include "AP_RustM4_Server.h"

#if AP_SCRIPTING_RUST_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Rust/BindingAnalogPWM.h>
#include <AP_Rust/BindingArming.h>
#include <AP_Rust/BindingCAN.h>
#include <AP_Rust/BindingCRSF.h>
#include <AP_Rust/BindingCustomMode.h>
#include <AP_Rust/BindingDroneCAN.h>
#include <AP_Rust/BindingI2C.h>
#include <AP_Rust/BindingM4Policy_generated.h>
#include <AP_Rust/BindingMAVLink.h>
#include <AP_Rust/BindingNetwork.h>
#include <AP_Rust/BindingONVIF.h>
#include <AP_Rust/BindingParam.h>
#include <AP_Rust/BindingSupport.h>
#include <AP_Rust/BindingSerialLED.h>
#include <AP_Rust/Bindings.h>
#include <AP_Rust/HostApi.h>
#include "AP_RustM4_ServeWait.h"

using namespace RustM4;

static bool binding_on_fast_lane(uint32_t id)
{
    return id < AP_Rust::M4_BINDING_COUNT && AP_Rust::M4_BINDING_POLICY[id].lane == AP_Rust::M4Lane::Fast;
}

bool AP_RustM4_Server::frame_on_fast_lane(const uint8_t *frame, uint16_t len) const
{
    return RustM4::frame_on_fast_lane(frame, len, binding_on_fast_lane);
}

uint16_t AP_RustM4_Server::serve_binding(const uint8_t *frame, uint16_t len, uint8_t *reply, uint16_t cap,
                                         uint32_t now_ms)
{
#if AP_RUST_M4_TEST_HOOKS
    _last_dispatch_ran = false;
#endif
    BindingCallHeader call {};
    if (!parse_binding_call(frame, len, call)) {
        MsgHeader header {};
        memcpy(&header, frame, len < sizeof(header) ? len : sizeof(header));
        return pack_binding_reply(reply, cap, header.seq, ReplyStatus::Malformed, 0, nullptr, 0, 0, 0);
    }
    if (call.module >= _module_count) {
        return pack_binding_reply(reply, cap, call.header.seq, ReplyStatus::Malformed, 0, nullptr, 0, 0, 0);
    }
    if (call.id >= AP_Rust::M4_BINDING_COUNT || AP_Rust::M4_BINDING_POLICY[call.id].lane == AP_Rust::M4Lane::Unknown ||
        AP_Rust::M4_BINDING_POLICY[call.id].lane == AP_Rust::M4Lane::Local) {
        return pack_binding_reply(reply, cap, call.header.seq, ReplyStatus::UnknownMethod, 0, nullptr, 0, 0, 0);
    }
    _quotas.new_session(now_ms);   // full buckets at the first call of the boot
    const uint8_t quota = AP_Rust::M4_BINDING_POLICY[call.id].quota;
    if (quota != 0 && !_quotas.allow(QuotaClass(quota), now_ms)) {
        // as M4 Lua: a refused call is answered OK and not run; a bool result reads false
        memset(_out, 0, call.out_size);
        return pack_binding_reply(reply, cap, call.header.seq, ReplyStatus::Ok, STATUS_OK, _out, call.out_size, 0, 0);
    }
    memset(_in, 0, sizeof(_in));
    memcpy(_in, frame + sizeof(call), call.in_size);
    memset(_out, 0, sizeof(_out));
    const uint32_t start_us = AP_HAL::micros();
    const int32_t status = dispatch(call.module, call.id, call.in_size, call.out_size);
    const uint32_t exec_us = AP_HAL::micros() - start_us;
    return pack_binding_reply(reply, cap, call.header.seq, ReplyStatus::Ok, status, _out, call.out_size, 0, exec_us);
}

// sizes within the frame limits, input already in _in. The buffers are passed even for an empty
// input or output: most generated adapters ignore the sizes and would dereference a null pointer
int32_t AP_RustM4_Server::dispatch(uint16_t module, uint32_t id, uint16_t in_size, uint16_t out_size)
{
#if AP_RUST_M4_TEST_HOOKS
    _last_input_end = 0;
    for (uint16_t i = SERVE_BUFFER_SIZE; i > 0; i--) {
        if (_in[i - 1] != 0) {
            _last_input_end = i;
            break;
        }
    }
    _last_dispatch_ran = true;
#endif
    switch (AP_Rust::BindingId(id)) {
    case AP_Rust::BindingId::PARAMETER_INIT:
    case AP_Rust::BindingId::PARAMETER_GET:
    case AP_Rust::BindingId::PARAMETER_SET:
    case AP_Rust::BindingId::PARAMETER_SET_AND_SAVE:
    case AP_Rust::BindingId::PARAMETER_SET_DEFAULT:
    case AP_Rust::BindingId::PARAMETER_CONFIGURED:
        return serve_parameter(module, id);
    default:
        // PARAMETER_LAYOUT, which reports sizeof and alignof(Parameter), included
        return AP_Rust::binding_dispatch(&_slots[module], AP_Rust::BindingId(id), _in, in_size, _out, out_size);
    }
}

AP_RustM4_Server::AP_RustM4_Server()
{
    clear_handles();
}

void AP_RustM4_Server::clear_handles()
{
    memset(_handle_module, HANDLE_FREE, sizeof(_handle_module));
    for (auto &parameter : _handle) {
        parameter = Parameter();
    }
    _handle_count = 0;
}

uint16_t AP_RustM4_Server::handle_home(uint16_t module, uint64_t token)
{
    const uint64_t mixed = (token ^ (uint64_t(module) << 56)) * 0x9E3779B97F4A7C15ULL;
    return uint16_t(mixed >> 56) & (PARAMETER_HANDLE_SLOTS - 1U);
}

// the slot of (module, token), or with insert a free slot claimed for it; -1 if absent or full
int16_t AP_RustM4_Server::find_handle(uint16_t module, uint64_t token, bool insert)
{
    uint16_t slot = handle_home(module, token);
    for (uint16_t probes = 0; probes < PARAMETER_HANDLE_SLOTS; probes++) {
        if (_handle_module[slot] == HANDLE_FREE) {
            if (!insert || _handle_count >= PARAMETER_HANDLES_MAX) {
                return -1;
            }
            _handle_module[slot] = uint8_t(module);
            _handle_token[slot] = token;
            _handle_count++;
            return int16_t(slot);
        }
        if (_handle_module[slot] == module && _handle_token[slot] == token) {
            return int16_t(slot);
        }
        slot = (slot + 1U) & (PARAMETER_HANDLE_SLOTS - 1U);
    }
    return -1;
}

// linear-probing deletion: later entries of the run move back so every lookup still finds them
void AP_RustM4_Server::erase_handle(uint16_t slot)
{
    const uint16_t mask = PARAMETER_HANDLE_SLOTS - 1U;
    uint16_t next = slot;
    while (true) {
        next = (next + 1U) & mask;
        if (_handle_module[next] == HANDLE_FREE) {
            break;
        }
        const uint16_t home = handle_home(_handle_module[next], _handle_token[next]);
        // next stays unless its home lies cyclically in (slot, next]
        const bool stays = slot <= next ? (slot < home && home <= next) : (slot < home || home <= next);
        if (!stays) {
            _handle_module[slot] = _handle_module[next];
            _handle_token[slot] = _handle_token[next];
            _handle[slot] = _handle[next];
            slot = next;
        }
    }
    _handle_module[slot] = HANDLE_FREE;
    _handle[slot] = Parameter();
    _handle_count--;
}

void AP_RustM4_Server::release_handles(uint16_t module)
{
    uint16_t slot = 0;
    while (slot < PARAMETER_HANDLE_SLOTS && _handle_count != 0) {
        if (_handle_module[slot] == module) {
            erase_handle(slot);   // may move another entry into slot: look at it again
        } else {
            slot++;
        }
    }
}

// the PR's generated Parameter adapters' wire and checks, on an M7-owned Parameter
int32_t AP_RustM4_Server::serve_parameter(uint16_t module, uint32_t id)
{
    const AP_Rust::BindingId binding = AP_Rust::BindingId(id);
    memset(_out, 0, binding == AP_Rust::BindingId::PARAMETER_GET ? 5 : 1);
    const uint64_t token = uint64_t(AP_Rust::BindingWire::read(_in, 4)) |
                           (uint64_t(AP_Rust::BindingWire::read(_in + 4, 4)) << 32);
    if (token == 0 || token > UINTPTR_MAX || token % alignof(Parameter) != 0) {
        return AP_RUST_STATUS_INVALID;
    }
    if (binding == AP_Rust::BindingId::PARAMETER_INIT) {
        const char *name = AP_Rust::BindingWire::read_string(_in + 8);
        if (name == nullptr) {
            return AP_RUST_STATUS_INVALID;
        }
        const int16_t slot = find_handle(module, token, true);
        if (slot < 0) {
            return AP_RUST_STATUS_NO_MEMORY;
        }
        _handle[slot] = Parameter();
#if AP_RUST_M4_TEST_HOOKS
        if (!_bind_parameters) {
            return AP_RUST_STATUS_OK;
        }
#endif
        _out[0] = _handle[slot].init(name);
        return AP_RUST_STATUS_OK;
    }
    const int16_t slot = find_handle(module, token, false);
    if (slot < 0) {
        return AP_RUST_STATUS_INVALID;
    }
    Parameter &parameter = _handle[slot];
    if (binding == AP_Rust::BindingId::PARAMETER_GET) {
        float value;
        if (parameter.get(value)) {
            _out[0] = 1;
            AP_Rust::BindingWire::write_float(_out + 1, value);
        }
        return AP_RUST_STATUS_OK;
    }
    if (binding == AP_Rust::BindingId::PARAMETER_CONFIGURED) {
        _out[0] = parameter.configured();
        return AP_RUST_STATUS_OK;
    }
    const float value = AP_Rust::BindingWire::read_float(_in + 8);
    if (!AP_Rust::BindingParam::valid_value(parameter.type(), value)) {
        return AP_RUST_STATUS_INVALID;
    }
    switch (binding) {
    case AP_Rust::BindingId::PARAMETER_SET:
        _out[0] = parameter.set(value);
        break;
    case AP_Rust::BindingId::PARAMETER_SET_AND_SAVE:
        _out[0] = parameter.set_and_save(value);
        break;
    default:
        _out[0] = parameter.set_default(value);
        break;
    }
    return AP_RUST_STATUS_OK;
}

// the PR's session cleanups (Runtime.cpp:549-559), in its order
static void (*const CLEANUPS[])(void *) = {
    AP_Rust::binding_dronecan_cleanup, AP_Rust::binding_i2c_cleanup, AP_Rust::binding_can_cleanup,
    AP_Rust::binding_crsf_cleanup, AP_Rust::binding_custom_mode_cleanup, AP_Rust::binding_analog_pwm_cleanup,
    AP_Rust::binding_mavlink_cleanup, AP_Rust::binding_network_cleanup, AP_Rust::binding_onvif_cleanup,
    AP_Rust::binding_serialled_cleanup, AP_Rust::binding_arming_cleanup,
};

uint8_t AP_RustM4_Server::cleanup_count()
{
    return uint8_t(ARRAY_SIZE(CLEANUPS));
}

void AP_RustM4_Server::end_module(uint16_t index)
{
    if (index >= _module_count) {
        return;
    }
    ModuleSlot &slot = _slots[index];
    release_handles(index);
#if AP_RUST_M4_TEST_HOOKS
    _ended_modules++;
    if (!_cleanups_run) {
        return;
    }
#endif
    for (auto cleanup : CLEANUPS) {
        cleanup(&slot);
    }
}

void AP_RustM4_Server::reset(uint32_t session, uint8_t module_count)
{
    for (uint16_t index = 0; index < _module_count; index++) {
        end_module(index);
    }
    // a slot index is reused by the next session's module of that index
    clear_handles();
    _module_count = module_count < ARRAY_SIZE(_slots) ? module_count : uint8_t(ARRAY_SIZE(_slots));
    for (uint16_t index = 0; index < _module_count; index++) {
        _slots[index] = ModuleSlot { index, session };
    }
}

#if AP_RUST_M4_TEST_HOOKS
bool AP_RustM4_Server::dispatch_guarded(uint32_t id, uint16_t in_size, uint16_t out_size)
{
    memset(_in, 0, MAX_FRAME);
    memset(_out, 0, MAX_FRAME);
    memset(_in + MAX_FRAME, 0xA5, SERVE_BUFFER_SIZE - MAX_FRAME);
    memset(_out + MAX_FRAME, 0xA5, SERVE_BUFFER_SIZE - MAX_FRAME);
    IGNORE_RETURN(dispatch(0, id, in_size, out_size));
    for (uint16_t i = MAX_FRAME; i < SERVE_BUFFER_SIZE; i++) {
        if (_in[i] != 0xA5 || _out[i] != 0xA5) {
            return false;
        }
    }
    return true;
}

void AP_RustM4_Server::fill_input_for_test(uint8_t byte)
{
    memset(_in, byte, sizeof(_in));
}

bool AP_RustM4_Server::needs_vehicle(uint32_t id)
{
    switch (AP_Rust::BindingId(id)) {
    // Location::get_alt_cm(): SITL builds panic on the all-zero Location of zeroed input
    case AP_Rust::BindingId::LOCATION_CHANGE_ALT_FRAME:
    case AP_Rust::BindingId::LOCATION_GET_ALT_M:
    case AP_Rust::BindingId::LOCATION_GET_VECTOR_FROM_ORIGIN_NEU_CM:
    case AP_Rust::BindingId::LOCATION_GET_VECTOR_FROM_ORIGIN_NEU_M:
    // AP_DroneCAN::get_dronecan(): AP::can() panics without the CAN manager
    case AP_Rust::BindingId::DRONE_CAN_GET_FLEX_DEBUG:
        return true;
    default:
        return false;
    }
}
#endif  // AP_RUST_M4_TEST_HOOKS

#endif  // AP_SCRIPTING_RUST_ENABLED
