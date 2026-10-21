#pragma once

/*
  Messages between the M7 supervisor and the M4 module runtime, carried as
  CubeFramework SpscRing payloads (the ring adds length, magic and CRC).
  C++11, no ArduPilot or ChibiOS dependencies: the M4 image and the SITL M4
  process include it too.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "AP_RustM4_Placement.h"

namespace RustM4 {

enum class MsgType : uint8_t {
    // 1 and 2 were Ping/Pong (protocol 1)
    Stats = 3,       // M4 -> M7: round-trip statistics of one method, once per second
    FaultInject = 4, // M7 -> M4: test-only fault request
    // 5, 6, 7: Lua Call, Reply, Notify; reserved
    Ready = 8,       // M7 -> M4: the M7 serves calls of this session
    BindingCall = 9,   // M4 -> M7: a module's binding call
    BindingReply = 10, // M7 -> M4: its reply
    SystemCall = 11,   // M4 -> M7: a runtime call or notify (SystemMethod)
    SystemReply = 12,  // M7 -> M4: its reply
    Control = 13,      // M7 -> M4: a request to one module
};

enum class FaultKind : uint8_t {
    None = 0,
    HardFault = 1,        // service: undefined instruction, escalated to HardFault
    MemManage = 2,        // service: stray write to RCC_GCR, stopped by the M4 MPU
    Stall = 3,            // service loop spins with interrupts enabled
    Lockup = 4,           // spins with interrupts disabled: cannot be restarted
    RingCorrupt = 5,      // invalid producer index in the M4 -> M7 ring
    Flood = 6,            // service fills the M4 -> M7 ring for FLOOD_MS
    // 7 to 10: not FaultKinds, see below
    HandlerFault = 11,    // service: pends IRQ 147, whose handler faults inside that exception
};
static const uint8_t FAULT_KIND_MAX = 11;

// RUST_M4_INJECT values outside FaultKind. 10 and 12 are handled on the M7 (refused bindings:
// unused on Rust; a HardFault right after the restarted M4 publishes its header).
static const uint8_t INJECT_REFUSE_BINDINGS = 10;
static const uint8_t INJECT_FAULT_AFTER_PUBLISH = 12;
// sent to the M4 by an explicit path (not a FaultKind): start the built-in thread-path selftest
static const uint8_t INJECT_MODULE_SELFTEST = 17;
// performed by the m4_faults.rs test module, which reads RUST_M4_INJECT itself; never sent
static const uint8_t MODULE_FAULT_AIRCR = 7;
static const uint8_t MODULE_FAULT_STACK_OVERFLOW = 8;
static const uint8_t MODULE_FAULT_RCC_WRITE = 9;
static const uint8_t MODULE_FAULT_JUMP_TO_HEAP = 13;
static const uint8_t MODULE_FAULT_WRITE_CODE = 14;
static const uint8_t MODULE_FAULT_BAD_POINTER = 15;
static const uint8_t MODULE_FAULT_PANIC = 16;
inline bool module_fault_kind(uint8_t kind)
{
    return kind == 7 || kind == 8 || kind == 9 || (kind >= 13 && kind <= 16);
}
static_assert(INJECT_REFUSE_BINDINGS > uint8_t(FaultKind::Flood) &&
              INJECT_REFUSE_BINDINGS < uint8_t(FaultKind::HandlerFault), "not a FaultKind");
static_assert(INJECT_FAULT_AFTER_PUBLISH > FAULT_KIND_MAX && INJECT_MODULE_SELFTEST > FAULT_KIND_MAX,
              "not FaultKinds");

struct __attribute__((packed)) MsgHeader {
    uint8_t type;
    uint8_t reserved;
    uint16_t length;   // whole message, header included
    uint32_t seq;
};

// largest message, header included (CubeFramework's SpscRing::maxPayload() for the 16 KB rings is about 8 KB)
static const uint16_t MAX_FRAME = 2048;

enum class ReplyStatus : uint8_t {
    Ok = 0,
    // 1, 2: Lua ArgError, Error; unused
    UnknownMethod = 3,
    Malformed = 4,
};

static const uint16_t CALL_FLAG_NOTIFY = 1;   // no reply
// a binding call with a CallProbeTrailer after its input (the call latency probe)
static const uint16_t CALL_FLAG_PROBE = 2;
// RUST_M4_OPTS bit, sent in ConfigReply::m4_options: the M4 probes this session's binding calls
static const uint8_t M4_OPTION_CALL_PROBE = 1U << 2;

/*
  The M4's stamps of a probed binding call (TIM5's count, the counter the M7
  stamps with too). The stamps the M4 takes after the send (sent, reply seen,
  return) come in the trailer of the module's next probed call. Untrusted on
  the M7: logged only.
 */
struct __attribute__((packed)) CallProbeTrailer {
    uint32_t svc_entry_us;     // the module's call entered the host
    uint32_t pre_send_us;      // set by the service as it sends the frame
    uint32_t stamp_cycles;     // M4 core cycles of one counter read
    uint32_t prev_seq;         // the previous probed call of this module; 0: none to complete
    uint32_t prev_sent_us;
    uint32_t prev_reply_seen_us;
    uint32_t prev_return_us;
    uint16_t prev_flags;       // PROBE_M4_* of that call
    uint16_t prev_stamps;      // PROBE_STAMP_*: which of the sub-stage stamps below that call took
    // that call's sub-stage stamps
    uint32_t prev_packed_us;            // frame packed, before the platform call
    uint32_t prev_posted_us;            // link posted, before forward_posted()
    uint32_t prev_forward_returned_us;  // forward_posted() returned with the call sent
    uint32_t prev_reply_isr_us;         // the doorbell interrupt before the service saw the reply
    uint32_t prev_reply_wake_us;        // the service thread resumed after it
    uint32_t prev_signalled_us;         // complete() signalled the module thread
    uint32_t prev_resumed_us;           // the module thread resumed after its wait
    // memset cost on the M4, hundredths of a cycle per byte: 64 B on the syscall stack and 4 KB of
    // the link buffers (privileged RAM), 1000 times each at the session's first probed call; 0 before
    uint16_t memset_small_centicycles;
    uint16_t memset_large_centicycles;
};
// CallProbeTrailer::prev_stamps, the values of AP_Rust::CALL_STAMP_M4_* (AP_Rust/CallLatency.h)
static const uint16_t PROBE_STAMP_PACKED = 1U << 4;
static const uint16_t PROBE_STAMP_POSTED = 1U << 5;
static const uint16_t PROBE_STAMP_FORWARD_RETURNED = 1U << 6;
static const uint16_t PROBE_STAMP_REPLY_WAKE = 1U << 7;   // prev_reply_isr_us and prev_reply_wake_us
static const uint16_t PROBE_STAMP_SIGNALLED = 1U << 8;
static const uint16_t PROBE_STAMP_RESUMED = 1U << 9;
// CallProbeTrailer::prev_flags, the values of AP_Rust::CALL_M4_* (AP_Rust/CallLatency.h)
static const uint16_t PROBE_M4_SLEPT = 1U << 2;          // the module thread slept for its reply
static const uint16_t PROBE_M4_POLL_WAKE = 1U << 3;      // the service's wait before the reply timed out
static const uint16_t PROBE_M4_SENT_BY_STEP = 1U << 4;   // the service's step sent it, not the module thread

struct __attribute__((packed)) BindingCallHeader {
    MsgHeader header;   // BindingCall; seq set by the service
    uint16_t module;    // module index
    uint16_t flags;
    uint32_t id;        // BindingId
    uint16_t in_size;   // input bytes that follow (PR wire)
    uint16_t out_size;  // output bytes the caller expects
};

struct __attribute__((packed)) BindingReplyHeader {
    MsgHeader header;   // BindingReply, seq of the call
    uint8_t transport;  // ReplyStatus
    uint8_t reserved[3];
    int32_t status;     // the adapter's AP_RUST_STATUS_*
    uint16_t out_size;  // output bytes that follow: 0 unless transport Ok and status OK
    uint16_t reserved2;
    uint32_t m7_wake_us;
    uint32_t m7_exec_us;
};

enum class SystemMethod : uint16_t {
    Config = 1, ListModules = 2, Resolve = 3, ModuleState = 4, Text = 5, StopAck = 6, RunStats = 7,
    Noop = 9, FsOpen = 15, FsRead = 16, FsSeek = 18, FsClose = 20,
};

struct __attribute__((packed)) SystemCallHeader {
    MsgHeader header;   // SystemCall
    uint16_t method;    // SystemMethod
    uint16_t flags;     // CALL_FLAG_NOTIFY
};

struct __attribute__((packed)) SystemReplyHeader {
    MsgHeader header;   // SystemReply, seq of the call
    uint16_t method;
    uint8_t transport;  // ReplyStatus
    uint8_t reserved;
    int32_t result;     // method result, < 0: -errno
};

static const uint16_t CONTROL_STOP = 1;
struct __attribute__((packed)) ControlMsg {
    MsgHeader header;   // Control, M7 -> M4
    uint16_t module;
    uint16_t action;
};

static const uint16_t MAX_CALL_INPUT = MAX_FRAME - sizeof(BindingCallHeader);
static const uint16_t MAX_REPLY_OUTPUT = MAX_FRAME - sizeof(BindingReplyHeader);

// system method contents, without the value codec
struct __attribute__((packed)) ConfigRequest { SystemCallHeader call; uint32_t crc_short; uint32_t crc_pattern; };
struct __attribute__((packed)) ConfigReply {
    SystemReplyHeader reply;
    int32_t heap_size, code_size, stack_size;   // RUST_HEAP_SIZE, RUST_CODE_SIZE (clamped), RUST_STACK_SIZE
    uint8_t max_modules, debug_options, m4_options, reserved;
    // the M7's AP_HAL::millis() and AP_HAL::micros() read together as the reply is packed: the M4
    // calibrates its copy of the M7 clock (AP_RustM4_M7Clock.h) from them
    uint32_t m7_millis, m7_micros;
};
// path: what FsOpen takes for the module (module_path_allowed())
struct __attribute__((packed)) ModuleEntry { char name[64]; char path[96]; uint32_t size; };
struct __attribute__((packed)) ListModulesReply { SystemReplyHeader reply; uint8_t count; uint8_t reserved[3]; ModuleEntry modules[8]; };
static const uint16_t RESOLVE_MAX = 250;
struct __attribute__((packed)) ResolveRequest { SystemCallHeader call; uint16_t count; uint16_t reserved; uint64_t hashes[RESOLVE_MAX]; };
struct __attribute__((packed)) ResolveReply { SystemReplyHeader reply; uint16_t count; uint16_t reserved; uint32_t ids[RESOLVE_MAX]; };
enum class ModuleStateKind : uint8_t { Loaded = 1, Started = 2, Returned = 3, Panicked = 4, Failed = 5 };
struct __attribute__((packed)) ModuleStateNotify {
    SystemCallHeader call;   // CALL_FLAG_NOTIFY
    uint16_t module; uint8_t state; uint8_t reserved;
    uint32_t rx_size, rw_size, heap_size, stack_size, crc;
    // Loaded: where the code and the stack were placed, as the module sees them (fault texts)
    uint32_t rx_base, stack_base;
    int32_t result;          // start()'s result for Returned
    char text[192];          // panic message or failure reason, terminated
};
struct __attribute__((packed)) TextNotify { SystemCallHeader call; uint16_t module; uint8_t severity; uint8_t reserved; uint16_t length; char text[512]; };
struct __attribute__((packed)) FsOpenRequest { SystemCallHeader call; char path[128]; };   // read only
// an FsRead reply carries the bytes after a SystemReplyHeader whose result is the count, at most 2000
struct __attribute__((packed)) FsReadRequest { SystemCallHeader call; int32_t fd; uint16_t count; uint16_t reserved; };
static const uint16_t FS_READ_MAX = 2000;
// files are read in order: a seek is accepted only to the current offset
struct __attribute__((packed)) FsSeekRequest { SystemCallHeader call; int32_t fd; int32_t offset; };
struct __attribute__((packed)) FsCloseRequest { SystemCallHeader call; int32_t fd; };
// crc: zlib's CRC-32 of exactly the bytes served, served: their count
struct __attribute__((packed)) FsCloseReply { SystemReplyHeader reply; uint32_t crc; uint32_t served; };
static_assert(sizeof(SystemReplyHeader) + FS_READ_MAX <= MAX_FRAME, "a read reply fits a frame");
static_assert(sizeof(ResolveRequest) <= MAX_FRAME && sizeof(ResolveReply) <= MAX_FRAME, "resolve fits a frame");
static_assert(sizeof(ListModulesReply) <= MAX_FRAME && sizeof(TextNotify) <= MAX_FRAME, "system frames fit");

struct __attribute__((packed)) StatsMsg {
    MsgHeader header;
    uint16_t method;       // the method these round trips belong to
    uint16_t reserved;
    uint32_t count;        // round trips completed in the window
    uint32_t p50_us;       // log2 bucket bounds, as BindingHistogram
    uint32_t p99_us;
    uint32_t max_us;
    uint32_t total_us;
    uint32_t m7_max_us;    // largest m7_wake_us + m7_exec_us seen
    uint32_t lost;         // calls without a reply within CALL_TIMEOUT_MS
    char name[64];         // "type:method" (binding_method_label()), terminated; logged, never trusted
};

struct __attribute__((packed)) FaultInjectMsg {
    MsgHeader header;
    uint32_t kind;         // FaultKind
};

struct __attribute__((packed)) ReadyMsg {
    MsgHeader header;
};

// true if buf holds exactly one message of a known type; the header is copied out
inline bool parse_header(const uint8_t *buf, uint16_t len, MsgHeader &header)
{
    if (len < sizeof(MsgHeader) || len > MAX_FRAME) {
        return false;
    }
    memcpy(&header, buf, sizeof(header));
    if (header.length != len) {
        return false;
    }
    switch (MsgType(header.type)) {
    case MsgType::Stats:
        return len == sizeof(StatsMsg);
    case MsgType::FaultInject:
        return len == sizeof(FaultInjectMsg);
    case MsgType::Ready:
        return len == sizeof(ReadyMsg);
    case MsgType::BindingCall:
        return len >= sizeof(BindingCallHeader);
    case MsgType::BindingReply:
        return len >= sizeof(BindingReplyHeader);
    case MsgType::SystemCall:
        return len >= sizeof(SystemCallHeader);
    case MsgType::SystemReply:
        return len >= sizeof(SystemReplyHeader);
    case MsgType::Control:
        return len == sizeof(ControlMsg);
    }
    return false;
}

inline void fill_header(MsgHeader &header, MsgType type, uint16_t length, uint32_t seq)
{
    header.type = uint8_t(type);
    header.reserved = 0;
    header.length = length;
    header.seq = seq;
}

// with a trailer, CALL_FLAG_PROBE is set and the trailer follows the input
inline uint16_t pack_binding_call(uint8_t *frame, uint16_t cap, uint32_t seq, uint16_t module, uint16_t flags,
                                  uint32_t id, const uint8_t *input, uint16_t in_size, uint16_t out_size,
                                  const CallProbeTrailer *trailer = nullptr)
{
    const uint32_t trailer_size = trailer != nullptr ? uint32_t(sizeof(CallProbeTrailer)) : 0U;
    const uint32_t length = uint32_t(sizeof(BindingCallHeader)) + in_size + trailer_size;
    if (trailer != nullptr) {
        flags |= CALL_FLAG_PROBE;
    } else {
        flags = uint16_t(flags & ~CALL_FLAG_PROBE);
    }
    if (length > MAX_FRAME || length > cap || out_size > MAX_REPLY_OUTPUT || (in_size != 0 && input == nullptr)) {
        return 0;
    }
    BindingCallHeader h;
    fill_header(h.header, MsgType::BindingCall, uint16_t(length), seq);
    h.module = module;
    h.flags = flags;
    h.id = id;
    h.in_size = in_size;
    h.out_size = out_size;
    memcpy(frame, &h, sizeof(h));
    if (in_size != 0) {
        memcpy(frame + sizeof(h), input, in_size);
    }
    if (trailer != nullptr) {
        memcpy(frame + sizeof(h) + in_size, trailer, sizeof(*trailer));
    }
    return uint16_t(length);
}

inline bool parse_binding_call(const uint8_t *frame, uint16_t len, BindingCallHeader &h)
{
    if (len < sizeof(h) || len > MAX_FRAME) {
        return false;
    }
    memcpy(&h, frame, sizeof(h));
    const uint32_t trailer_size = (h.flags & CALL_FLAG_PROBE) != 0 ? uint32_t(sizeof(CallProbeTrailer)) : 0U;
    return h.header.type == uint8_t(MsgType::BindingCall) && h.header.length == len &&
           uint32_t(sizeof(h)) + h.in_size + trailer_size == len && h.out_size <= MAX_REPLY_OUTPUT;
}

// the trailer of a probed call that parse_binding_call() accepted; false if it has none
inline bool binding_call_trailer(const uint8_t *frame, uint16_t len, const BindingCallHeader &h,
                                 CallProbeTrailer &trailer)
{
    if ((h.flags & CALL_FLAG_PROBE) == 0 || len < sizeof(h) + sizeof(trailer)) {
        return false;
    }
    memcpy(&trailer, frame + len - sizeof(trailer), sizeof(trailer));
    return true;
}

inline uint16_t pack_binding_reply(uint8_t *frame, uint16_t cap, uint32_t seq, ReplyStatus transport, int32_t status,
                                   const uint8_t *output, uint16_t out_size, uint32_t wake_us, uint32_t exec_us)
{
    if (transport != ReplyStatus::Ok || status != 0) {
        out_size = 0;
    }
    const uint32_t length = uint32_t(sizeof(BindingReplyHeader)) + out_size;
    if (length > MAX_FRAME || length > cap || (out_size != 0 && output == nullptr)) {
        return 0;
    }
    BindingReplyHeader h {};
    fill_header(h.header, MsgType::BindingReply, uint16_t(length), seq);
    h.transport = uint8_t(transport);
    h.status = status;
    h.out_size = out_size;
    h.m7_wake_us = wake_us;
    h.m7_exec_us = exec_us;
    memcpy(frame, &h, sizeof(h));
    if (out_size != 0) {
        memcpy(frame + sizeof(h), output, out_size);
    }
    return uint16_t(length);
}

inline bool parse_binding_reply(const uint8_t *frame, uint16_t len, BindingReplyHeader &h, const uint8_t *&output)
{
    if (len < sizeof(h) || len > MAX_FRAME) {
        return false;
    }
    memcpy(&h, frame, sizeof(h));
    if (h.header.type != uint8_t(MsgType::BindingReply) || h.header.length != len ||
        uint32_t(sizeof(h)) + h.out_size != len ||
        (h.out_size != 0 && (h.transport != uint8_t(ReplyStatus::Ok) || h.status != 0))) {
        return false;
    }
    output = frame + sizeof(h);
    return true;
}

// short name of an M4 fault record; the SITL M4 records a signal number instead
inline const char *fault_name(uint32_t cfsr, uint32_t hfsr)
{
    if (hfsr & (1U << 30)) {   // HFSR.FORCED: a configurable fault escalated
        return "HardFault";
    }
    if (cfsr & 0xFFU) {
        return "MemManage";
    }
    if (cfsr & 0xFF00U) {
        return "BusFault";
    }
    if (cfsr & 0xFFFF0000U) {
        return "UsageFault";
    }
    return "HardFault";
}

// a fault taken while stacking its exception frame (CFSR MSTKERR or STKERR): no frame was
// written, so the M4 records no PC, and the SP the stacking used in place of LR
inline bool fault_on_stacking(uint32_t cfsr)
{
    return (cfsr & ((1U << 4) | (1U << 12))) != 0;
}

// how far below the module stack a stacking SP is still taken for a module stack overflow: the
// frame that ran into the guard or past it, and the exception frame below it
static const uint32_t MODULE_STACK_OVERFLOW_REACH = 1024;

// the reason of an M4 image fault record: "MemManage PC=0x081D047C", or for a fault on stacking
// "MemManage on stacking SP=0x30007F60 CFSR=0x00000092", then " (module stack overflow)" when that
// SP is in the guard of the module stack at module_stack_base (0: none) or just below it
inline void fault_reason_text(char *text, size_t text_len, uint32_t cfsr, uint32_t hfsr, uint32_t pc, uint32_t lr,
                              uint32_t module_stack_base)
{
    const char *name = fault_name(cfsr, hfsr);
    if (!fault_on_stacking(cfsr)) {
        snprintf(text, text_len, "%s PC=0x%08X", name, unsigned(pc));
        return;
    }
    const uint32_t sp = lr;
    const bool overflow = module_stack_base != 0 && sp >= module_stack_base - MODULE_STACK_OVERFLOW_REACH &&
                          sp < module_stack_base + AP_RUST_M4_STACK_GUARD_SIZE;
    snprintf(text, text_len, "%s on stacking SP=0x%08X CFSR=0x%08X%s", name, unsigned(sp), unsigned(cfsr),
             overflow ? " (module stack overflow)" : "");
}

}  // namespace RustM4
