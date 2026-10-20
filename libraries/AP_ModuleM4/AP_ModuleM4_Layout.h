#pragma once

/*
  Shared-window layout of the M4 module runtime (C++11; the M4 image and
  CubeFramework users build it as C++17). Window = [CubeFramework Channel:
  peer header, control, two rings][SupervisorBlock]. Used by the M7
  supervisor, the M4 image and the SITL M4 process.
 */

#include <atomic>
#include <stddef.h>
#include <stdint.h>
#include "AP_ModuleM4_Placement.h"

namespace ModuleM4 {

// FNV-1a, C++11 constexpr (single return statements)
static const uint32_t FNV_OFFSET = 2166136261U;
static const uint32_t FNV_PRIME = 16777619U;
constexpr uint32_t fnv1a_byte(uint32_t hash, uint8_t byte)
{
    return (hash ^ byte) * FNV_PRIME;
}
constexpr uint32_t fnv1a_word(uint32_t word, uint32_t hash)
{
    return fnv1a_byte(fnv1a_byte(fnv1a_byte(fnv1a_byte(hash, uint8_t(word)), uint8_t(word >> 8)),
                                 uint8_t(word >> 16)), uint8_t(word >> 24));
}

// peer header contract: a mismatched M4 image is refused
// "RS" 1: never equal to a Lua image's RPC_PROTOCOL_VERSION
static const uint32_t PROTOCOL_VERSION = 0x52530001U;
// the binding fingerprint and the local catalogue join it in Generator M4 Policy
static const uint32_t CONTRACT_HASH = fnv1a_word(PROTOCOL_VERSION, FNV_OFFSET);

static const uint32_t WINDOW_SIZE = AP_MODULE_M4_WINDOW_SIZE;
static const uint32_t SUPERVISOR_BLOCK_SIZE = 32;
static const uint32_t CHANNEL_WINDOW_SIZE = WINDOW_SIZE - SUPERVISOR_BLOCK_SIZE;
static const uint8_t DOORBELL_GROUP = AP_MODULE_M4_DOORBELL_GROUP;
static const uint8_t RESTART_GROUP = AP_MODULE_M4_RESTART_GROUP;
static const uint32_t SUPERVISOR_MAGIC = 0x5350344D;   // "M4PS"

// supervisor timing (heartbeat limit: spec §8)
static const uint32_t HEARTBEAT_TIMEOUT_MS = 200;
static const uint32_t START_TIMEOUT_MS = 2000;
static const uint32_t RESTART_TIMEOUT_MS = 500;
// a binding call without a reply this long returns HOST_ERROR
static const uint32_t CALL_TIMEOUT_MS = 1000;
// FaultKind::Flood fills the M4 -> M7 ring this long
static const uint32_t FLOOD_MS = 2000;

// written only by the M7; the M4 reads it at start, every loop and while parked
struct SupervisorBlock {
    std::atomic<uint32_t> magic;          // SUPERVISOR_MAGIC once the M7 has set the window up
    std::atomic<uint32_t> run_permit;     // 1: the M4 may run; 0: park
    std::atomic<uint32_t> restart_grant;  // restarts granted so far, published by the M4 as its restart count
    std::atomic<uint32_t> reserved[4];
    // test only (MOD_M4_INJECT 12): the session started with this grant faults right after its publish()
    std::atomic<uint32_t> fault_after_publish_grant;
};
static_assert(sizeof(SupervisorBlock) == SUPERVISOR_BLOCK_SIZE, "supervisor block layout is frozen");
#if __cplusplus >= 201703L
static_assert(std::atomic<uint32_t>::is_always_lock_free, "shared words must be lock-free");
#else
static_assert(ATOMIC_INT_LOCK_FREE == 2 && ATOMIC_LONG_LOCK_FREE == 2, "shared words must be lock-free");
#endif
static_assert(CHANNEL_WINDOW_SIZE % 32 == 0, "the Channel window and the block stay 32-byte aligned");

inline SupervisorBlock *supervisor_block(uint8_t *window)
{
    return reinterpret_cast<SupervisorBlock *>(reinterpret_cast<uintptr_t>(window) + CHANNEL_WINDOW_SIZE);
}

}  // namespace ModuleM4
