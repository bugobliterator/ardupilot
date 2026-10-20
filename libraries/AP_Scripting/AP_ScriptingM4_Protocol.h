#pragma once

/*
  Messages between the M7 scripting host and the M4 Lua runtime, carried as
  CubeFramework SpscRing payloads (the ring adds length, magic and CRC).
  Lua calls are Call and Notify frames from the M4; a Call is answered by a
  Reply. Their values use the codec in AP_ScriptingM4_Rpc.h.
  C++11, no ArduPilot or ChibiOS dependencies: the M4 image and the SITL M4
  process include it too.
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "AP_ScriptingM4_Placement.h"

namespace ScriptingM4 {

enum class MsgType : uint8_t {
    // 1 and 2 were Ping/Pong (protocol 1)
    Stats = 3,       // M4 -> M7: round-trip statistics of one method, once per second
    FaultInject = 4, // M7 -> M4: test-only fault request
    Call = 5,        // M4 -> M7: Lua call, a Reply follows
    Reply = 6,       // M7 -> M4: results of a Call
    Notify = 7,      // M4 -> M7: Lua call without a reply
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
    LuaAircr = 7,         // Lua thread: SYSRESETREQ write to AIRCR (unprivileged: BusFault)
    LuaStackOverflow = 8, // Lua thread: recursion into its stack guard
    LuaRccWrite = 9,      // Lua thread: stray write to RCC_GCR
    // 10 is SCR_M4_INJECT's refused bindings image, handled on the M7
    HandlerFault = 11,    // service: pends IRQ 147, whose handler faults inside that exception
};
static const uint8_t FAULT_KIND_MAX = 11;

// the faults the Lua thread performs; the service performs the others
inline bool lua_thread_fault(FaultKind kind)
{
    return kind == FaultKind::LuaAircr || kind == FaultKind::LuaStackOverflow || kind == FaultKind::LuaRccWrite;
}

struct __attribute__((packed)) MsgHeader {
    uint8_t type;
    uint8_t reserved;
    uint16_t length;   // whole message, header included
    uint32_t seq;
};

// largest message, header included
static const uint16_t MAX_FRAME = 1024;

struct __attribute__((packed)) CallHeader {
    MsgHeader header;   // Call or Notify
    uint16_t method;    // SystemMethod, or a generated binding id
    uint8_t argc;       // encoded values that follow
    uint8_t reserved;
};

enum class ReplyStatus : uint8_t {
    Ok = 0,
    ArgError = 1,       // bad argument error_arg; the message is the first value
    Error = 2,          // Lua error; the message is the first value
    UnknownMethod = 3,
    Malformed = 4,      // the M7 could not decode the call
};

struct __attribute__((packed)) ReplyHeader {
    MsgHeader header;      // Reply, seq of the call
    uint8_t status;        // ReplyStatus
    uint8_t count;         // encoded values that follow
    uint8_t error_arg;     // Lua stack index of the bad argument (ArgError)
    uint8_t reserved;
    uint32_t m7_wake_us;   // doorbell interrupt to host thread running
    uint32_t m7_exec_us;   // host time spent on this call
};

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
    case MsgType::Call:
    case MsgType::Notify:
        return len >= sizeof(CallHeader);
    case MsgType::Reply:
        return len >= sizeof(ReplyHeader);
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

// how far below the Lua stack a stacking SP is still taken for a Lua stack overflow: the frame
// that ran into the guard or past it into privileged RAM, and the exception frame below it.
// The privileged stacks and data sit at the bottom of privileged RAM, well below this
static const uint32_t LUA_STACK_OVERFLOW_REACH = 1024;

// the reason of an M4 image fault record: "MemManage PC=0x081D047C", or for a fault on stacking
// "MemManage on stacking SP=0x30007F60 CFSR=0x00000093", then " (Lua stack overflow)" when that
// SP is in the Lua stack guard or just below it
inline void fault_reason_text(char *text, size_t text_len, uint32_t cfsr, uint32_t hfsr, uint32_t pc, uint32_t lr)
{
    const char *name = fault_name(cfsr, hfsr);
    if (!fault_on_stacking(cfsr)) {
        snprintf(text, text_len, "%s PC=0x%08X", name, unsigned(pc));
        return;
    }
    const uint32_t lua_stack_base = AP_SCRIPTING_M4_RAM_BASE + AP_SCRIPTING_M4_PRIV_RAM_SIZE;
    const uint32_t sp = lr;
    const bool lua_overflow = sp >= lua_stack_base - LUA_STACK_OVERFLOW_REACH &&
                              sp < lua_stack_base + AP_SCRIPTING_M4_LUA_STACK_GUARD_SIZE;
    snprintf(text, text_len, "%s on stacking SP=0x%08X CFSR=0x%08X%s", name, unsigned(sp), unsigned(cfsr),
             lua_overflow ? " (Lua stack overflow)" : "");
}

}  // namespace ScriptingM4
