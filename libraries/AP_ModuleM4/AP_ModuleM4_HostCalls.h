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
#include "AP_ModuleM4_CallProbe.h"
#include "AP_ModuleM4_CallStatus.h"
#include "AP_ModuleM4_GranuleHeap.h"
#include "AP_ModuleM4_ModuleRanges.h"
#include "AP_ModuleM4_Protocol.h"

namespace ModuleM4 {

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

// the bits of a call policy byte (AP_Module/Bindings.h M4_CALL_*): posted, the MirrorRole, and for an id
// the M4 answers itself 1 + the index of its exact [input, output] sizes in HostPlatform::local_wires
static const uint8_t CALL_POLICY_POSTED = 1U;
static const uint8_t CALL_POLICY_ROLE_SHIFT = 1U;
static const uint8_t CALL_POLICY_ROLE_MASK = 3U;
static const uint8_t CALL_POLICY_WIRE_SHIFT = 3U;

struct HostPlatform {
    // a whole call frame; the reply copied, its length returned, or -errno with no reply
    int32_t (*call)(void *platform, const uint8_t *request, uint16_t len, uint8_t *reply, uint16_t cap);
    void (*notify)(void *platform, const uint8_t *frame, uint16_t len);
    // one policy byte per id from call_policy_first; any other id is a plain call to the M7
    const uint8_t *call_policy;
    uint32_t call_policy_first;
    uint32_t call_policy_count;
    const uint8_t (*local_wires)[2];
    int32_t (*local)(uint32_t id, const uint8_t *input, uint32_t input_size, uint8_t *output, uint32_t output_size);
    void *platform;
    // the Parameter mirror; null: every call goes to the M7
    // the module's Parameter.get answered from the mirror into out5; false on a miss (nothing written)
    bool (*mirror_get)(void *platform, uint16_t module, uint64_t token, uint8_t *out5);
    void (*mirror_pending)(void *platform, uint16_t module, uint64_t token);
    void (*mirror_settle)(void *platform, uint16_t module, uint64_t token);
    // a forwarded Parameter.get answered OK: out5 refreshes the module's STALE entry (never inserts)
    void (*mirror_refresh)(void *platform, uint16_t module, uint64_t token, const uint8_t *out5);
    M4CallProbe *probe;   // the call latency probe; may be null
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
    uint32_t heap_needed {};         // the largest HeapNeeded reported for this load
};

// never null (the SDK's enter() refuses a null context) and never a privileged address
inline void *module_token(uint16_t index)
{
    return reinterpret_cast<void *>(uintptr_t(index) + 1U);
}

inline int32_t host_binding_call(ModuleContext &m, const BindingCallArgs &a, const HostPlatform &p)
{
    M4CallProbe *probe = probing(p.probe) ? p.probe : nullptr;
    uint32_t entry_cycles = 0, svc_cycles = 0;
    if (probe != nullptr) {
        entry_cycles = probe_cycles_now(*probe);
        svc_cycles = probe->svc_cycles;
        probe->svc_cycles = 0;
    }
    if (probe != nullptr && !probe->memset_measured) {
        // once per M4 start, outside any call's stages; the link buffers are free here
        measure_memset(*probe, m.request, m.reply, MAX_FRAME);
    }
    const uint32_t entry_us = probe != nullptr ? probe->clock_us() : 0;
    const uint32_t checks_cycles = probe != nullptr ? probe_cycles_now(*probe) : 0;
    if (m.stop.load()) {
        return STATUS_STOPPED;
    }
    if (a.context != m.token) {
        return STATUS_INVALID;
    }
    const uint32_t slot = a.id - p.call_policy_first;
    const uint8_t policy = slot < p.call_policy_count ? p.call_policy[slot] : 0U;
    const uint8_t wire = policy >> CALL_POLICY_WIRE_SHIFT;
    const bool local = wire != 0;
    // a local adapter reads and writes its fixed sizes whatever the call says: exact sizes only,
    // so that the range checks below cover every byte it touches
    if (local && (a.input_size != p.local_wires[wire - 1][0] || a.output_size != p.local_wires[wire - 1][1])) {
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
    // a posted id (generated policy: output 0, no vehicle command): the module waits only for the
    // ring copy, then gets OK; the M7 serves it without a reply
    if (a.output_size == 0 && (policy & CALL_POLICY_POSTED) != 0) {
        const uint16_t length = pack_binding_call(m.request, MAX_FRAME, 0, m.index, CALL_FLAG_NOTIFY, a.id, a.input,
                                                  uint16_t(a.input_size), 0);
        if (length == 0) {
            return STATUS_INVALID;
        }
        p.notify(p.platform, m.request, length);
        return STATUS_OK;
    }
    const MirrorRole role = MirrorRole((policy >> CALL_POLICY_ROLE_SHIFT) & CALL_POLICY_ROLE_MASK);
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
    // probed when the trailer fits the frame; word-aligned for pack_binding_call()'s word copy. Read only
    // after begin_probed_call() set it: not zeroed on every call
    alignas(4) CallProbeTrailer trailer;
    uint32_t begin_cycles = 0, begun_cycles = 0;
    if (probe != nullptr && a.input_size + sizeof(trailer) <= MAX_CALL_INPUT) {
        begin_cycles = probe_cycles_now(*probe);
        trailer = begin_probed_call(*probe, entry_us);
        begun_cycles = probe_cycles_now(*probe);
    } else {
        probe = nullptr;
    }
    const uint16_t length = pack_binding_call(m.request, MAX_FRAME, 0, m.index, 0, a.id, a.input,
                                              uint16_t(a.input_size), uint16_t(a.output_size),
                                              probe != nullptr ? &trailer : nullptr);
    if (length == 0) {
        if (probe != nullptr) {
            probe->in_call = false;
        }
        return STATUS_INVALID;
    }
    if (probe != nullptr) {
        const uint32_t packed_cycles = probe_cycles_now(*probe);
        M4CallStamps &current = probe->current;
        current.svc_to_entry_cycles = svc_cycles != 0 ? cycles_between(svc_cycles, entry_cycles) : 0;
        current.checks_cycles = cycles_between(checks_cycles, begin_cycles);
        current.begin_probe_cycles = cycles_between(begin_cycles, begun_cycles);
        current.pack_cycles = cycles_between(begun_cycles, packed_cycles);
        probe_stamp(*probe, current.packed_us, PROBE_STAMP_PACKED);
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
        if (probe != nullptr) {
            probe->in_call = false;
            probe = nullptr;
        }
    } else {
        const CallOutcome outcome = call_outcome(result, reply, uint16_t(a.output_size));
        if (outcome.copy_output) {
            copy_frame_bytes(a.output, body, a.output_size);
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
    if (probe != nullptr) {
        end_probed_call(*probe, result > 0, probe->clock_us());
    }
    return status;
}

/*
  allocation after a stop is allowed: Rust destructors may still allocate (PR Module::allocate).
  The M4 heap does not expand as the PR's does: a failed valid request is reported to the M7 as the
  initial heap the module needs, the heap plus the request, when that exceeds the last report and
  the module is not stopping
 */
inline uint8_t *host_allocate(ModuleContext &m, uint32_t size, uint32_t alignment, const HostPlatform &p)
{
    uint8_t *allocation = static_cast<uint8_t *>(m.heap.allocate(size, alignment));
    if (allocation != nullptr || size == 0 || alignment == 0 || alignment > GranuleHeap::MAX_ALIGNMENT ||
        (alignment & (alignment - 1U)) != 0) {
        return allocation;
    }
    const uint64_t granules = (uint64_t(size) + GranuleHeap::GRANULE - 1U) / GranuleHeap::GRANULE;
    const uint64_t padding = alignment > GranuleHeap::GRANULE ? alignment - GranuleHeap::GRANULE : 0U;
    const uint64_t needed = m.heap.capacity() + granules * GranuleHeap::GRANULE + padding;
    const uint32_t needed32 = needed > UINT32_MAX ? UINT32_MAX : uint32_t(needed);
    if (needed32 <= m.heap_needed || m.stop.load()) {
        return nullptr;
    }
    m.heap_needed = needed32;
    ModuleStateNotify notify {};
    fill_header(notify.call.header, MsgType::SystemCall, sizeof(notify), 0);
    notify.call.method = uint16_t(SystemMethod::ModuleState);
    notify.call.flags = CALL_FLAG_NOTIFY;
    notify.module = m.index;
    notify.state = uint8_t(ModuleStateKind::HeapNeeded);
    notify.heap_size = needed32;
    p.notify(p.platform, reinterpret_cast<const uint8_t *>(&notify), sizeof(notify));
    return nullptr;
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

}  // namespace ModuleM4
