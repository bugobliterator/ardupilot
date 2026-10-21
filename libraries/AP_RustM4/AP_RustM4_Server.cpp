#include "AP_RustM4_Server.h"

#if AP_SCRIPTING_RUST_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
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
#include <AP_Rust/CallLatencyLog.h>
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

static bool binding_posted(uint32_t id)
{
    return id < AP_Rust::M4_BINDING_COUNT && (AP_Rust::M4_BINDING_POLICY[id].flags & AP_Rust::M4_FLAG_POSTED) != 0;
}

bool AP_RustM4_Server::frame_is_posted(const uint8_t *frame, uint16_t len) const
{
    BindingCallHeader call;
    return parse_binding_call(frame, len, call) && (call.flags & CALL_FLAG_NOTIFY) != 0 && binding_posted(call.id);
}

// word stores where p is 4-byte aligned, then the tail. volatile keeps GCC from folding the
// loops back into a memset call
static void zero_bytes(uint8_t *p, uint32_t n)
{
    volatile uint8_t *byte = p;
    while (n > 0 && (uintptr_t(byte) & 3U) != 0) {
        *byte++ = 0;
        n--;
    }
    volatile uint32_t *word = static_cast<volatile uint32_t *>(static_cast<volatile void *>(byte));
    for (; n >= 4; n -= 4) {
        *word++ = 0;
    }
    byte = static_cast<volatile uint8_t *>(static_cast<volatile void *>(word));
    while (n-- > 0) {
        *byte++ = 0;
    }
}

uint16_t AP_RustM4_Server::serve_binding(const uint8_t *frame, uint16_t len, uint8_t *reply, uint16_t cap,
                                         uint32_t now_ms, ServeProbe *probe)
{
    _last_posted_outcome = PostedOutcome::NotPosted;
    _last_call_wrote_parameters = false;
    _last_call_module = -1;
    using AP_Rust::CallLatencyLog;
    if (probe != nullptr) {
        // until the adapter runs, every later stage is the dispatch stamp: a frame answered before it
        // has no lock or adapter time
        const uint32_t start_cycles = CallLatencyLog::cycles();
        probe->dispatch = probe->lock_wait = probe->adapter = probe->adapter_done = CallLatencyLog::clock_us();
        probe->overhead_cycles += CallLatencyLog::cycles() - start_cycles;
    }
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
    _last_call_module = int16_t(call.module);
    if (call.id >= AP_Rust::M4_BINDING_COUNT || AP_Rust::M4_BINDING_POLICY[call.id].lane == AP_Rust::M4Lane::Unknown ||
        AP_Rust::M4_BINDING_POLICY[call.id].lane == AP_Rust::M4Lane::Local) {
        return pack_binding_reply(reply, cap, call.header.seq, ReplyStatus::UnknownMethod, 0, nullptr, 0, 0, 0);
    }
    const bool notify = (call.flags & CALL_FLAG_NOTIFY) != 0;
    if (notify && !binding_posted(call.id)) {
        _last_posted_outcome = PostedOutcome::Refused;
        return 0;
    }
    if (probe != nullptr) {
        probe->chk_parsed = CallLatencyLog::clock_us();
        probe->taken |= AP_Rust::CALL_STAMP_CHK_PARSED;
    }
    _quotas.new_session(now_ms);   // full buckets at the first call of the boot
    const uint8_t quota = AP_Rust::M4_BINDING_POLICY[call.id].quota;
    if (quota != 0 && !_quotas.allow(QuotaClass(quota), now_ms)) {
        if (notify) {
            _last_posted_outcome = PostedOutcome::Dropped;
            return 0;
        }
        // as M4 Lua: a refused call is answered OK and not run; a bool result reads false
        if (probe != nullptr) {
            probe->flags |= AP_Rust::CALL_QUOTA;
        }
        memset(_out, 0, call.out_size);
        return pack_binding_reply(reply, cap, call.header.seq, ReplyStatus::Ok, STATUS_OK, _out, call.out_size, 0, 0);
    }
    if (probe != nullptr) {
        probe->chk_quota = CallLatencyLog::clock_us();
        probe->taken |= AP_Rust::CALL_STAMP_CHK_QUOTA;
    }
    // newlib-nano's memset stores one byte per iteration: clear only what earlier calls touched
    if (_in_dirty > call.in_size) {
        zero_bytes(_in + call.in_size, _in_dirty - call.in_size);
    }
    memcpy(_in, frame + sizeof(call), call.in_size);
    _in_dirty = call.in_size;
    zero_bytes(_out, _out_dirty);
    // adapters do not all honour out_size; none writes past MAX_FRAME (NoAdapterWritesPastTheFrame)
    _out_dirty = MAX_FRAME;
    if (probe != nullptr) {
        probe->chk_copied = CallLatencyLog::clock_us();
        probe->taken |= AP_Rust::CALL_STAMP_CHK_COPIED;
    }
    const uint32_t start_us = AP_HAL::micros();
    int32_t status;
    if (probe == nullptr) {
        status = dispatch(call.module, call.id, call.in_size, call.out_size);
    } else {
        // the adapter's own take of its (recursive) lock then waits for nothing
        uint32_t cycles = CallLatencyLog::cycles();
        AP_HAL::Semaphore *lock = CallLatencyLog::adapter_lock(call.id, probe->flags);
        probe->lock_wait = CallLatencyLog::clock_us();
        probe->overhead_cycles += CallLatencyLog::cycles() - cycles;
        if (lock != nullptr) {
            lock->take_blocking();
        }
        probe->adapter = CallLatencyLog::clock_us();
        status = dispatch(call.module, call.id, call.in_size, call.out_size);
        probe->adapter_done = CallLatencyLog::clock_us();
        cycles = CallLatencyLog::cycles();
        if (lock != nullptr) {
            lock->give();
        }
        probe->overhead_cycles += CallLatencyLog::cycles() - cycles;
    }
    const uint32_t exec_us = AP_HAL::micros() - start_us;
    _last_call_wrote_parameters = (AP_Rust::M4_BINDING_POLICY[call.id].flags &
                                   (AP_Rust::M4_FLAG_MIRROR_HANDLE_WRITER | AP_Rust::M4_FLAG_MIRROR_NAME_WRITER)) != 0;
    if (notify) {
        _last_posted_outcome = status == AP_RUST_STATUS_OK ? PostedOutcome::Served : PostedOutcome::Failed;
        return 0;
    }
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
    _last_output_zero = true;
    for (const uint8_t byte : _out) {
        if (byte != 0) {
            _last_output_zero = false;
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

static uint32_t slot_mask(uint16_t slot)
{
    return 1U << (slot & 31U);
}

void AP_RustM4_Server::set_dirty(uint16_t slot)
{
    _dirty_slots[slot / 32U].fetch_or(slot_mask(slot), std::memory_order_release);
}

void AP_RustM4_Server::set_sent(uint16_t slot, bool sent, bool bound)
{
    const uint32_t mask = slot_mask(slot);
    _sent_slots[slot / 32U] = sent ? (_sent_slots[slot / 32U] | mask) : (_sent_slots[slot / 32U] & ~mask);
    _sent_bound_slots[slot / 32U] = bound ? (_sent_bound_slots[slot / 32U] | mask) :
                                            (_sent_bound_slots[slot / 32U] & ~mask);
}

void AP_RustM4_Server::mark_changed(const AP_Param *param)
{
    if (param == nullptr) {
        return;
    }
    for (uint16_t slot = 0; slot < PARAMETER_HANDLE_SLOTS; slot++) {
        if (_handle_param[slot].load(std::memory_order_relaxed) == param) {
            set_dirty(slot);
        }
    }
}

void AP_RustM4_Server::mark_all_live()
{
    for (uint16_t slot = 0; slot < PARAMETER_HANDLE_SLOTS; slot++) {
        if (_handle_module[slot] != HANDLE_FREE) {
            set_dirty(slot);
        }
    }
}

bool AP_RustM4_Server::mirror_dirty() const
{
    for (const auto &word : _dirty_slots) {
        if (word.load(std::memory_order_relaxed) != 0) {
            return true;
        }
    }
    return false;
}

uint16_t AP_RustM4_Server::pack_mirror_changes(uint8_t *frame, uint16_t cap)
{
    if (cap < sizeof(ParameterChangedHeader) + sizeof(ParameterValue)) {
        return 0;
    }
    const uint16_t room = uint16_t(MIN(uint32_t(PARAMETER_CHANGED_MAX),
                                       (uint32_t(cap) - sizeof(ParameterChangedHeader)) / sizeof(ParameterValue)));
    uint16_t count = 0;
    for (uint16_t word = 0; word < SLOT_WORDS && count < room; word++) {
        uint32_t bits = _dirty_slots[word].exchange(0, std::memory_order_acquire);
        while (bits != 0) {
            const uint16_t bit = uint16_t(__builtin_ctz(bits));
            bits &= bits - 1U;
            const uint16_t slot = uint16_t(word * 32U + bit);
            if (count == room) {
                set_dirty(slot);   // for the next frame
                continue;
            }
            if (_handle_module[slot] == HANDLE_FREE) {
                continue;
            }
            float value = 0;
            const bool bound = _handle[slot].get(value);
            if (!bound) {
                value = 0;
            }
            const uint32_t mask = slot_mask(slot);
            if ((_sent_slots[word] & mask) != 0 && ((_sent_bound_slots[word] & mask) != 0) == bound &&
                memcmp(&value, &_sent_value[slot], sizeof(value)) == 0) {
                continue;
            }
            const ParameterValue entry { _handle_token[slot], value, _handle_module[slot],
                                         uint8_t(bound ? PARAMETER_VALUE_BOUND : 0), 0 };
            memcpy(frame + sizeof(ParameterChangedHeader) + uint32_t(count) * sizeof(entry), &entry, sizeof(entry));
            count++;
            _sent_value[slot] = value;
            set_sent(slot, true, bound);
        }
    }
    if (count == 0) {
        return 0;
    }
    ParameterChangedHeader h {};
    const uint16_t length = uint16_t(sizeof(h) + uint32_t(count) * sizeof(ParameterValue));
    fill_header(h.header, MsgType::ParameterChanged, length, _session);
    h.count = count;
    memcpy(frame, &h, sizeof(h));
    return length;
}

void AP_RustM4_Server::mirror_send_failed(const uint8_t *frame, uint16_t len)
{
    ParameterChangedHeader h;
    if (!parse_parameter_changed(frame, len, h)) {
        return;
    }
    for (uint16_t i = 0; i < h.count; i++) {
        const ParameterValue entry = parameter_value_at(frame, i);
        const int16_t slot = find_handle(entry.module, entry.token, false);
        if (slot >= 0) {
            set_sent(uint16_t(slot), false, false);
            set_dirty(uint16_t(slot));
        }
        if (entry.module < ARRAY_SIZE(_resync_owed)) {
            _resync_owed[entry.module] = true;
        }
    }
}

bool AP_RustM4_Server::take_resync(uint16_t module)
{
    if (module >= ARRAY_SIZE(_resync_owed) || !_resync_owed[module]) {
        return false;
    }
    _resync_owed[module] = false;
    return true;
}

void AP_RustM4_Server::owe_resync(uint16_t module)
{
    if (module < ARRAY_SIZE(_resync_owed)) {
        _resync_owed[module] = true;
    }
}

void AP_RustM4_Server::snapshot_module(uint16_t module)
{
    for (uint16_t slot = 0; slot < PARAMETER_HANDLE_SLOTS; slot++) {
        if (_handle_module[slot] == module) {
            set_sent(slot, false, false);
            set_dirty(slot);
        }
    }
}

uint32_t AP_RustM4_Server::input_memset_cycles()
{
    static const uint8_t ROUNDS = 16;
    const uint32_t start = AP_Rust::CallLatencyLog::cycles();
    for (uint8_t i = 0; i < ROUNDS; i++) {
        memset(_in, 0, sizeof(_in));
        __asm__ volatile("" : : "r"(_in) : "memory");
    }
    return (AP_Rust::CallLatencyLog::cycles() - start) / ROUNDS;
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
    for (auto &param : _handle_param) {
        param.store(nullptr, std::memory_order_relaxed);
    }
    for (auto &word : _dirty_slots) {
        word.store(0, std::memory_order_relaxed);
    }
    memset(_sent_slots, 0, sizeof(_sent_slots));
    memset(_sent_bound_slots, 0, sizeof(_sent_bound_slots));
    memset(_resync_owed, 0, sizeof(_resync_owed));
    _handle_count = 0;
    _in_dirty = SERVE_BUFFER_SIZE;
    _out_dirty = SERVE_BUFFER_SIZE;
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
            _handle_param[slot].store(_handle_param[next].load(std::memory_order_relaxed), std::memory_order_relaxed);
            _sent_value[slot] = _sent_value[next];
            set_sent(slot, (_sent_slots[next / 32U] & slot_mask(next)) != 0,
                     (_sent_bound_slots[next / 32U] & slot_mask(next)) != 0);
            set_dirty(slot);   // a pending change of the moved entry is not left on its old slot
            slot = next;
        }
    }
    _handle_module[slot] = HANDLE_FREE;
    _handle[slot] = Parameter();
    _handle_param[slot].store(nullptr, std::memory_order_relaxed);
    set_sent(slot, false, false);
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
        _handle_param[slot].store(nullptr, std::memory_order_relaxed);
        set_sent(uint16_t(slot), false, false);   // the writer's own entry always goes out
#if AP_RUST_M4_TEST_HOOKS
        if (!_bind_parameters) {
            return AP_RUST_STATUS_OK;
        }
#endif
        _out[0] = _handle[slot].init(name);
        enum ap_var_type type;
        _handle_param[slot].store(_out[0] ? AP_Param::find(name, &type) : nullptr, std::memory_order_relaxed);
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
    set_sent(uint16_t(slot), false, false);   // a rejected or unchanged set still sends its entry
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
    _session = session;
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
    _in_dirty = SERVE_BUFFER_SIZE;
    _out_dirty = SERVE_BUFFER_SIZE;
    for (uint16_t i = MAX_FRAME; i < SERVE_BUFFER_SIZE; i++) {
        if (_in[i] != 0xA5 || _out[i] != 0xA5) {
            return false;
        }
    }
    return true;
}

const AP_Param *AP_RustM4_Server::bound_param_for_test(uint16_t module, uint64_t token)
{
    const int16_t slot = find_handle(module, token, false);
    return slot < 0 ? nullptr : _handle_param[slot].load(std::memory_order_relaxed);
}

void AP_RustM4_Server::fill_input_for_test(uint8_t byte)
{
    memset(_in, byte, sizeof(_in));
    _in_dirty = SERVE_BUFFER_SIZE;
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
