#pragma once

/*
  Messages between the M7 scripting host and the M4 for the M4 Boot &
  Supervisor phase, carried as CubeFramework SpscRing payloads (the ring adds
  length, magic and CRC). Remote Bindings replaces Ping/Pong with the RPC
  frames and changes CONTRACT_HASH in AP_ScriptingM4_Layout.h.
  C++11, no ArduPilot or ChibiOS dependencies: the M4 image and the SITL M4
  process include it too.
 */

#include <stdint.h>
#include <string.h>

namespace ScriptingM4 {

enum class MsgType : uint8_t {
    Ping = 1,        // M4 -> M7: timed request, stands in for an RPC
    Pong = 2,        // M7 -> M4: reply with the M7-side timings
    Stats = 3,       // M4 -> M7: round-trip statistics, once per second
    FaultInject = 4, // M7 -> M4: test-only fault request
    Call = 5,        // M4 -> M7: Lua call, a Reply follows
    Reply = 6,       // M7 -> M4: results of a Call
    Notify = 7,      // M4 -> M7: Lua call without a reply
    Ready = 8,       // M7 -> M4: the M7 serves calls of this session
};

enum class FaultKind : uint8_t {
    None = 0,
    HardFault = 1,   // undefined instruction, escalated to HardFault
    MemManage = 2,   // stray write to RCC_GCR, stopped by the M4 MPU
    Stall = 3,       // service loop spins with interrupts enabled
    Lockup = 4,      // spins with interrupts disabled: cannot be restarted
    RingCorrupt = 5, // invalid producer index in the M4 -> M7 ring
};
static const uint8_t FAULT_KIND_MAX = 5;

struct __attribute__((packed)) MsgHeader {
    uint8_t type;
    uint8_t reserved;
    uint16_t length;   // whole message, header included
    uint32_t seq;
};

struct __attribute__((packed)) PingMsg {
    MsgHeader header;
};

struct __attribute__((packed)) PongMsg {
    MsgHeader header;
    uint32_t m7_wake_us;   // doorbell interrupt to host thread running
    uint32_t m7_exec_us;   // host time spent on this request
};

struct __attribute__((packed)) StatsMsg {
    MsgHeader header;
    uint32_t count;        // round trips completed in the window
    uint32_t p50_us;       // log2 bucket bounds, as BindingHistogram
    uint32_t p99_us;
    uint32_t max_us;
    uint32_t total_us;
    uint32_t m7_max_us;    // largest m7_wake_us + m7_exec_us seen
    uint32_t lost;         // pings without a reply within PING_TIMEOUT_MS
};

struct __attribute__((packed)) FaultInjectMsg {
    MsgHeader header;
    uint32_t kind;         // FaultKind
};

// largest RPC frame (Call, Notify, Reply), header included
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

struct __attribute__((packed)) ReadyMsg {
    MsgHeader header;
};

static const uint16_t MAX_MESSAGE = 64;
static_assert(sizeof(StatsMsg) <= MAX_MESSAGE, "receive buffers hold any message");

// size a message of this type has, 0 for unknown types
inline uint16_t message_length(uint8_t type)
{
    switch (MsgType(type)) {
    case MsgType::Ping:
        return sizeof(PingMsg);
    case MsgType::Pong:
        return sizeof(PongMsg);
    case MsgType::Stats:
        return sizeof(StatsMsg);
    case MsgType::FaultInject:
        return sizeof(FaultInjectMsg);
    case MsgType::Call:
    case MsgType::Reply:
    case MsgType::Notify:
    case MsgType::Ready:
        return 0;   // variable length: handled by the Call Path parse_header()
    }
    return 0;
}

// true if buf holds exactly one message of a known type; the header is copied out
inline bool parse_header(const uint8_t *buf, uint16_t len, MsgHeader &header)
{
    if (len < sizeof(MsgHeader)) {
        return false;
    }
    memcpy(&header, buf, sizeof(header));
    const uint16_t expected = message_length(header.type);
    return expected != 0 && header.length == expected && len == expected;
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

}  // namespace ScriptingM4
