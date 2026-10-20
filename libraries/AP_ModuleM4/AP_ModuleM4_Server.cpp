#include "AP_ModuleM4_Server.h"

#if AP_MODULE_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Module/BindingAnalogPWM.h>
#include <AP_Module/BindingArming.h>
#include <AP_Module/BindingCAN.h>
#include <AP_Module/BindingCRSF.h>
#include <AP_Module/BindingCustomMode.h>
#include <AP_Module/BindingDroneCAN.h>
#include <AP_Module/BindingI2C.h>
#include <AP_Module/BindingM4Policy_generated.h>
#include <AP_Module/BindingMAVLink.h>
#include <AP_Module/BindingNetwork.h>
#include <AP_Module/BindingONVIF.h>
#include <AP_Module/BindingSerialLED.h>
#include <AP_Module/Bindings.h>
#include "AP_ModuleM4_ServeWait.h"

using namespace ModuleM4;

static bool binding_on_fast_lane(uint32_t id)
{
    return id < AP_Module::M4_BINDING_COUNT && AP_Module::M4_BINDING_POLICY[id].lane == AP_Module::M4Lane::Fast;
}

bool AP_ModuleM4_Server::frame_on_fast_lane(const uint8_t *frame, uint16_t len) const
{
    return ModuleM4::frame_on_fast_lane(frame, len, binding_on_fast_lane);
}

uint16_t AP_ModuleM4_Server::serve_binding(const uint8_t *frame, uint16_t len, uint8_t *reply, uint16_t cap,
                                         uint32_t now_ms)
{
#if AP_MODULE_M4_TEST_HOOKS
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
    if (call.id >= AP_Module::M4_BINDING_COUNT || AP_Module::M4_BINDING_POLICY[call.id].lane == AP_Module::M4Lane::Unknown ||
        AP_Module::M4_BINDING_POLICY[call.id].lane == AP_Module::M4Lane::Local) {
        return pack_binding_reply(reply, cap, call.header.seq, ReplyStatus::UnknownMethod, 0, nullptr, 0, 0, 0);
    }
    _quotas.new_session(now_ms);   // full buckets at the first call of the boot
    const uint8_t quota = AP_Module::M4_BINDING_POLICY[call.id].quota;
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
int32_t AP_ModuleM4_Server::dispatch(uint16_t module, uint32_t id, uint16_t in_size, uint16_t out_size)
{
#if AP_MODULE_M4_TEST_HOOKS
    _last_input_end = 0;
    for (uint16_t i = SERVE_BUFFER_SIZE; i > 0; i--) {
        if (_in[i - 1] != 0) {
            _last_input_end = i;
            break;
        }
    }
    _last_dispatch_ran = true;
#endif
    return AP_Module::binding_dispatch(&_slots[module], AP_Module::BindingId(id), _in, in_size, _out, out_size);
}

// the PR's session cleanups (Runtime.cpp:549-559), in its order
static void (*const CLEANUPS[])(void *) = {
    AP_Module::binding_dronecan_cleanup, AP_Module::binding_i2c_cleanup, AP_Module::binding_can_cleanup,
    AP_Module::binding_crsf_cleanup, AP_Module::binding_custom_mode_cleanup, AP_Module::binding_analog_pwm_cleanup,
    AP_Module::binding_mavlink_cleanup, AP_Module::binding_network_cleanup, AP_Module::binding_onvif_cleanup,
    AP_Module::binding_serialled_cleanup, AP_Module::binding_arming_cleanup,
};

uint8_t AP_ModuleM4_Server::cleanup_count()
{
    return uint8_t(ARRAY_SIZE(CLEANUPS));
}

void AP_ModuleM4_Server::end_module(uint16_t index)
{
    if (index >= _module_count) {
        return;
    }
    ModuleSlot &slot = _slots[index];
#if AP_MODULE_M4_TEST_HOOKS
    _ended_modules++;
    if (!_cleanups_run) {
        return;
    }
#endif
    for (auto cleanup : CLEANUPS) {
        cleanup(&slot);
    }
}

void AP_ModuleM4_Server::reset(uint32_t session, uint8_t module_count)
{
    for (uint16_t index = 0; index < _module_count; index++) {
        end_module(index);
    }
    _module_count = module_count < ARRAY_SIZE(_slots) ? module_count : uint8_t(ARRAY_SIZE(_slots));
    for (uint16_t index = 0; index < _module_count; index++) {
        _slots[index] = ModuleSlot { index, session };
    }
}

#if AP_MODULE_M4_TEST_HOOKS
bool AP_ModuleM4_Server::dispatch_guarded(uint32_t id, uint16_t in_size, uint16_t out_size)
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

void AP_ModuleM4_Server::fill_input_for_test(uint8_t byte)
{
    memset(_in, byte, sizeof(_in));
}

bool AP_ModuleM4_Server::needs_vehicle(uint32_t id)
{
    switch (AP_Module::BindingId(id)) {
    // Location::get_alt_cm(): SITL builds panic on the all-zero Location of zeroed input
    case AP_Module::BindingId::LOCATION_CHANGE_ALT_FRAME:
    case AP_Module::BindingId::LOCATION_GET_ALT_M:
    case AP_Module::BindingId::LOCATION_GET_VECTOR_FROM_ORIGIN_NEU_CM:
    case AP_Module::BindingId::LOCATION_GET_VECTOR_FROM_ORIGIN_NEU_M:
    // AP_DroneCAN::get_dronecan(): AP::can() panics without the CAN manager
    case AP_Module::BindingId::DRONE_CAN_GET_FLEX_DEBUG:
        return true;
    default:
        return false;
    }
}
#endif  // AP_MODULE_M4_TEST_HOOKS

#endif  // AP_MODULE_ENABLED
