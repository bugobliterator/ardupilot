#pragma once

/*
  The privileged bodies of the module's host callbacks, independent of how
  the call arrived (an SVC on the M4 image, a direct call in the SITL M4
  process). Every module pointer is range-checked before it is used; a
  violation is INVALID and sends nothing to the M7. Order as the PR's
  Module: a stop request is answered before anything else.
 */

#include <atomic>
#include <stdint.h>
#include <string.h>
#include "AP_RustM4_CallStatus.h"
#include "AP_RustM4_GranuleHeap.h"
#include "AP_RustM4_ModuleRanges.h"
#include "AP_RustM4_Protocol.h"

namespace RustM4 {

struct BindingCallArgs {
    void *context;
    uint32_t id;
    const uint8_t *input;
    uint32_t input_size;
    uint8_t *output;
    uint32_t output_size;
};

// binding_mirror_role() values (generator m4_binding.py ROLE_VALUES)
enum class MirrorRole : uint8_t { None = 0, Get = 1, HandleWriter = 2, NameWriter = 3 };

struct HostPlatform {
    // a whole call frame; the reply copied, its length returned, or -errno with no reply
    int32_t (*call)(void *platform, const uint8_t *request, uint16_t len, uint8_t *reply, uint16_t cap);
    void (*notify)(void *platform, const uint8_t *frame, uint16_t len);
    // true for an id the M4 answers itself, with the exact sizes its adapter reads and writes
    bool (*local_wire)(uint32_t id, uint32_t &input_size, uint32_t &output_size);
    int32_t (*local)(uint32_t id, const uint8_t *input, uint32_t input_size, uint8_t *output, uint32_t output_size);
    void *platform;
    // the Parameter mirror and posted ids; null: every call goes to the M7
    uint8_t (*mirror_role)(uint32_t id);
    // the module's Parameter.get answered from the mirror into out5; false on a miss (nothing written)
    bool (*mirror_get)(void *platform, uint16_t module, uint64_t token, uint8_t *out5);
    void (*mirror_pending)(void *platform, uint16_t module, uint64_t token);
    void (*mirror_settle)(void *platform, uint16_t module, uint64_t token);
    // a forwarded Parameter.get answered OK: out5 refreshes the module's STALE entry (never inserts)
    void (*mirror_refresh)(void *platform, uint16_t module, uint64_t token, const uint8_t *out5);
    bool (*is_posted)(uint32_t id);
};

struct ModuleContext {
    void *token;                     // module_token(index): the context the module was started with
    uint16_t index;
    ModuleRanges ranges;
    GranuleHeap heap;
    std::atomic<bool> stop;
    std::atomic<uint8_t> state;      // ModuleStateKind
    uint8_t *request;                // the ModuleLink's buffers, MAX_FRAME each
    uint8_t *reply;
    char panic_text[192];
};

// never null (the SDK's enter() refuses a null context) and never a privileged address
inline void *module_token(uint16_t index)
{
    return reinterpret_cast<void *>(uintptr_t(index) + 1U);
}

inline int32_t host_binding_call(ModuleContext &m, const BindingCallArgs &a, const HostPlatform &p)
{
    if (m.stop.load()) {
        return STATUS_STOPPED;
    }
    if (a.context != m.token) {
        return STATUS_INVALID;
    }
    uint32_t local_in = 0, local_out = 0;
    const bool local = p.local_wire(a.id, local_in, local_out);
    // a local adapter reads and writes its fixed sizes whatever the call says: exact sizes only,
    // so that the range checks below cover every byte it touches
    if (local && (a.input_size != local_in || a.output_size != local_out)) {
        return STATUS_INVALID;
    }
    const uintptr_t input = uintptr_t(a.input);
    const uintptr_t output = uintptr_t(a.output);
    if (!m.ranges.input_ok(input, a.input_size) || !m.ranges.output_ok(output, a.output_size) ||
        buffers_overlap(input, a.input_size, output, a.output_size)) {
        return STATUS_INVALID;
    }
    if (local) {
        return p.local(a.id, a.input, a.input_size, a.output, a.output_size);
    }
    if (a.input_size > MAX_CALL_INPUT || a.output_size > MAX_REPLY_OUTPUT) {
        return STATUS_INVALID;
    }
    const MirrorRole role = p.mirror_role != nullptr ? MirrorRole(p.mirror_role(a.id)) : MirrorRole::None;
    uint64_t token = 0;
    if ((role == MirrorRole::Get || role == MirrorRole::HandleWriter) && a.input_size >= sizeof(token)) {
        memcpy(&token, a.input, sizeof(token));   // the PR wire: the receiver token first
    }
    // exactly the PR wire (8 B token in, [bound, f32] out); any other size goes to the M7 as before
    if (role == MirrorRole::Get && a.input_size == 8 && a.output_size == 5 && p.mirror_get != nullptr &&
        p.mirror_get(p.platform, m.index, token, a.output)) {
        return STATUS_OK;
    }
    const bool settle = role == MirrorRole::HandleWriter && a.input_size >= sizeof(token) &&
                        p.mirror_pending != nullptr && p.mirror_settle != nullptr;
    const uint16_t length = pack_binding_call(m.request, MAX_FRAME, 0, m.index, 0, a.id, a.input,
                                              uint16_t(a.input_size), uint16_t(a.output_size));
    if (length == 0) {
        return STATUS_INVALID;
    }
    if (settle) {
        p.mirror_pending(p.platform, m.index, token);
    }
    const int32_t result = p.call(p.platform, m.request, length, m.reply, MAX_FRAME);
    int32_t status;
    BindingReplyHeader reply {};
    const uint8_t *body = nullptr;
    if (result > 0 && !parse_binding_reply(m.reply, uint16_t(result), reply, body)) {
        status = STATUS_HOST_ERROR;
    } else {
        const CallOutcome outcome = call_outcome(result, reply, uint16_t(a.output_size));
        if (outcome.copy_output && a.output_size != 0) {
            memcpy(a.output, body, a.output_size);
        }
        status = outcome.status;
        if (role == MirrorRole::Get && a.input_size == 8 && a.output_size == 5 && outcome.copy_output &&
            status == STATUS_OK && p.mirror_refresh != nullptr) {
            p.mirror_refresh(p.platform, m.index, token, a.output);
        }
    }
    if (settle) {
        p.mirror_settle(p.platform, m.index, token);
    }
    return status;
}

// allocation after a stop is allowed: Rust destructors may still allocate (PR Module::allocate)
inline uint8_t *host_allocate(ModuleContext &m, uint32_t size, uint32_t alignment)
{
    return static_cast<uint8_t *>(m.heap.allocate(size, alignment));
}

inline bool host_deallocate(ModuleContext &m, uint8_t *allocation)
{
    return m.heap.free(allocation) == GranuleHeap::FreeResult::Freed;
}

inline int32_t host_text(ModuleContext &m, uint8_t severity, const uint8_t *message, uint32_t length,
                         const HostPlatform &p)
{
    if (m.stop.load()) {
        return STATUS_STOPPED;
    }
    if (severity > 7 || length > 512 || message == nullptr || !m.ranges.input_ok(uintptr_t(message), length)) {
        return STATUS_INVALID;
    }
    TextNotify text {};
    fill_header(text.call.header, MsgType::SystemCall, sizeof(text), 0);
    text.call.method = uint16_t(SystemMethod::Text);
    text.call.flags = CALL_FLAG_NOTIFY;
    text.module = m.index;
    text.severity = severity;
    text.length = uint16_t(length);
    memcpy(text.text, message, length);
    p.notify(p.platform, reinterpret_cast<const uint8_t *>(&text), sizeof(text));
    return STATUS_OK;
}

// the panic of m.panic_text: state Panicked, reported to the M7 once
inline void host_panic_report(ModuleContext &m, const HostPlatform &p)
{
    m.panic_text[sizeof(m.panic_text) - 1] = 0;
    m.state.store(uint8_t(ModuleStateKind::Panicked));
    ModuleStateNotify notify {};
    static_assert(sizeof(m.panic_text) == sizeof(notify.text), "the panic text is sent whole");
    fill_header(notify.call.header, MsgType::SystemCall, sizeof(notify), 0);
    notify.call.method = uint16_t(SystemMethod::ModuleState);
    notify.call.flags = CALL_FLAG_NOTIFY;
    notify.module = m.index;
    notify.state = uint8_t(ModuleStateKind::Panicked);
    memcpy(notify.text, m.panic_text, sizeof(notify.text));
    p.notify(p.platform, reinterpret_cast<const uint8_t *>(&notify), sizeof(notify));
}

inline void host_panic_record(ModuleContext &m, const uint8_t *message, uint32_t length, const HostPlatform &p)
{
    uint32_t count = length < sizeof(m.panic_text) - 1 ? length : uint32_t(sizeof(m.panic_text) - 1);
    if (message == nullptr || !m.ranges.input_ok(uintptr_t(message), count)) {
        count = 0;
    }
    if (count != 0) {
        memcpy(m.panic_text, message, count);
    }
    m.panic_text[count] = 0;
    host_panic_report(m, p);
}

}  // namespace RustM4
