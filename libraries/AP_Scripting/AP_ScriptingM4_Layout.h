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
#include <string.h>
#include "AP_ScriptingM4_Placement.h"
#include "AP_ScriptingM4_Rpc.h"
#include <AP_Scripting/lua_remote_contract.h>

namespace ScriptingM4 {

// peer header contract: a mismatched M4 image is refused (spec §5); the generated
// binding table is part of it
static const uint32_t PROTOCOL_VERSION = RPC_PROTOCOL_VERSION;
static const uint32_t CONTRACT_HASH = fnv1a_word(BINDING_HASH, SYSTEM_CONTRACT_HASH);

static const uint32_t WINDOW_SIZE = AP_SCRIPTING_M4_WINDOW_SIZE;
static const uint32_t SUPERVISOR_BLOCK_SIZE = 32;
static const uint32_t CHANNEL_WINDOW_SIZE = AP_SCRIPTING_M4_CHANNEL_REGION_SIZE - SUPERVISOR_BLOCK_SIZE;
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
// the network rings start here, from the window base
static const uint32_t RING_REGION_OFFSET = AP_SCRIPTING_M4_CHANNEL_REGION_SIZE;
static_assert(AP_SCRIPTING_M4_WINDOW_BASE + RING_REGION_OFFSET == AP_NETWORKING_M4_RING_BASE,
              "the rings follow the supervisor block");
static_assert(RING_REGION_OFFSET + AP_NETWORKING_M4_RING_SIZE == WINDOW_SIZE, "the rings end the window");
// SupervisorBlock.reserved[1]: 1 the M4 starts the Lua thread, 0 it runs the network alone
static const uint8_t LUA_PERMIT_SLOT = 1;
#endif
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
    std::atomic<uint32_t> reserved[4];
    // test only (SCR_M4_INJECT 12): the session started with this grant faults right after its publish()
    std::atomic<uint32_t> fault_after_publish_grant;
};
static_assert(sizeof(SupervisorBlock) == SUPERVISOR_BLOCK_SIZE, "supervisor block layout is frozen");
static_assert(std::atomic<uint32_t>::is_always_lock_free, "shared words must be lock-free");
static_assert(CHANNEL_WINDOW_SIZE % 32 == 0, "the Channel window and the block stay 32-byte aligned");

inline SupervisorBlock *supervisor_block(uint8_t *window)
{
    return reinterpret_cast<SupervisorBlock *>(reinterpret_cast<uintptr_t>(window) + CHANNEL_WINDOW_SIZE);
}

// at scripting start: the Channel and the supervisor block (the whole window on plain builds).
// On networking builds the rings belong to AP_Networking's proxy (AP_Networking::m4_region_reset())
inline void clear_channel_region(uint8_t *window)
{
    memset(window, 0, AP_SCRIPTING_M4_CHANNEL_REGION_SIZE);
}

}  // namespace ScriptingM4
