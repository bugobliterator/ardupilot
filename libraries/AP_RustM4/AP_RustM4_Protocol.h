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

// largest message, header included
static const uint16_t MAX_FRAME = 1024;

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
