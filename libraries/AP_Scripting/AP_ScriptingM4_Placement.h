#pragma once

/*
  Placement of Lua scripting on the STM32H757 Cortex-M4. This is the only
  place that chooses the shared window, the doorbells and the M4 image
  addresses: the M7 MPU set-up (hwdef/common/stm32h7_mcuconf.h), the M7
  supervisor, the M4 image and the SITL M4 process all read it. The M4 image
  linker script (m4/chibios/scripting_m4.ld, asserted against this file at
  link time) and FLASH_RESERVE_END_KB in hwdef/CubeOrangePlus-M4Lua repeat
  the flash numbers and must match.
 */

// shared window: SRAM3 (D2), non-cacheable and shareable on the M7
#define AP_SCRIPTING_M4_WINDOW_BASE      0x30040000U
#define AP_SCRIPTING_M4_WINDOW_SIZE      (32U * 1024U)
// MPU RASR SIZE field of the window: 2^(14+1) bytes = 32 KB
#define AP_SCRIPTING_M4_WINDOW_MPU_SIZE  14U

// CubeFramework HSEM groups (group g uses semaphores 2g and 2g+1)
#define AP_SCRIPTING_M4_DOORBELL_GROUP   1U   // Channel doorbell, both directions
#define AP_SCRIPTING_M4_RESTART_GROUP    2U   // M7 -> M4 restart request

// M4 image: the last 256 KB of flash bank 2 (sectors 14-15)
#define AP_SCRIPTING_M4_FLASH_BASE       0x081C0000U
#define AP_SCRIPTING_M4_FLASH_SIZE       (256U * 1024U)
// M4 RAM: D2 SRAM1 + SRAM2
#define AP_SCRIPTING_M4_RAM_BASE         0x30000000U
#define AP_SCRIPTING_M4_RAM_SIZE         (256U * 1024U)
// M4 RAM split (m4/chibios/scripting_m4.ld): privileged RAM first (kernel, service,
// stacks; MPU region privileged-only), then Lua RAM (Lua thread, newlib, Lua heap)
#define AP_SCRIPTING_M4_PRIV_RAM_SIZE    (32U * 1024U)
#define AP_SCRIPTING_M4_LUA_STACK_SIZE   (16U * 1024U)
// MPU no-access guard at the bottom of the Lua stack (the smallest MPU region)
#define AP_SCRIPTING_M4_LUA_STACK_GUARD_SIZE 32U
// the Lua heap the M4 image guarantees (linker assert); the SITL M4 caps its heap here too
#define AP_SCRIPTING_M4_LUA_HEAP_MIN     (192U * 1024U)
// of that heap, what Lua's count (SCR_HEAP_SIZE) never reaches: newlib's own allocations
// (strtod and _dtoa_r Bigints) and fragmentation; both platforms cap Lua below it
#define AP_SCRIPTING_M4_NEWLIB_RESERVE   (8U * 1024U)

// M4 core clock: CubeOrangePlus sys_ck 400 MHz, D1HPRE /2 (stm32h7_mcuconf.h, 24 MHz HSE)
#define AP_SCRIPTING_M4_CORE_CLOCK_HZ    200000000U
