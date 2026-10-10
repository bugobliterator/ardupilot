#pragma once

/*
  Shared-window layout of the M4 scripting split (C++17: CubeFramework
  headers). Window = [CubeFramework Channel: peer header, control, two
  rings][SupervisorBlock]. Used by the M7 supervisor, the M4 image and the
  SITL M4 process.
 */

#include <atomic>
#include <stddef.h>
#include <stdint.h>
#include "AP_ScriptingM4_Placement.h"
#include "AP_ScriptingM4_Rpc.h"

namespace ScriptingM4 {

// peer header contract: a mismatched M4 image is refused (spec §5)
static const uint32_t PROTOCOL_VERSION = RPC_PROTOCOL_VERSION;
static const uint32_t CONTRACT_HASH = SYSTEM_CONTRACT_HASH;

static const uint32_t WINDOW_SIZE = AP_SCRIPTING_M4_WINDOW_SIZE;
static const uint32_t SUPERVISOR_BLOCK_SIZE = 32;
static const uint32_t CHANNEL_WINDOW_SIZE = WINDOW_SIZE - SUPERVISOR_BLOCK_SIZE;
static const uint8_t DOORBELL_GROUP = AP_SCRIPTING_M4_DOORBELL_GROUP;
static const uint8_t RESTART_GROUP = AP_SCRIPTING_M4_RESTART_GROUP;
static const uint32_t SUPERVISOR_MAGIC = 0x5350344D;   // "M4PS"

// supervisor timing (heartbeat limit: spec §8)
static const uint32_t HEARTBEAT_TIMEOUT_MS = 200;
static const uint32_t START_TIMEOUT_MS = 2000;
static const uint32_t RESTART_TIMEOUT_MS = 500;
// a Lua call without a reply this long fails in the script (-ETIMEDOUT)
static const uint32_t CALL_TIMEOUT_MS = 1000;
// FaultKind::Flood fills the M4 -> M7 ring this long
static const uint32_t FLOOD_MS = 2000;

// written only by the M7; the M4 reads it at start, every loop and while parked
struct SupervisorBlock {
    std::atomic<uint32_t> magic;          // SUPERVISOR_MAGIC once the M7 has set the window up
    std::atomic<uint32_t> run_permit;     // 1: the M4 may run; 0: park
    std::atomic<uint32_t> restart_grant;  // restarts granted so far, published by the M4 as its restart count
    std::atomic<uint32_t> reserved[5];
};
static_assert(sizeof(SupervisorBlock) == SUPERVISOR_BLOCK_SIZE, "supervisor block layout is frozen");
static_assert(std::atomic<uint32_t>::is_always_lock_free, "shared words must be lock-free");
static_assert(CHANNEL_WINDOW_SIZE % 32 == 0, "the Channel window and the block stay 32-byte aligned");

inline SupervisorBlock *supervisor_block(uint8_t *window)
{
    return reinterpret_cast<SupervisorBlock *>(reinterpret_cast<uintptr_t>(window) + CHANNEL_WINDOW_SIZE);
}

}  // namespace ScriptingM4
