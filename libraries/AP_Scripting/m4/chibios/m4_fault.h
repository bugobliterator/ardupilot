#pragma once

/*
  M4 fault handling and software warm restart (spec §8). There is no
  per-core reset (ES0445 2.3.1), so the image restarts itself: an exception
  return into its reset handler re-runs crt0 (RAM, VTOR, stacks) and
  ChibiOS. Restarts happen only when the M7 grants one.
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// handler mode only: returns to Reset_Handler in thread mode; parks if another exception is active
__attribute__((noreturn)) void m4_warm_restart(void);
// interrupts off, low-power wait, forever
__attribute__((noreturn)) void m4_park_forever(void);
// latch SupervisorBlock.restart_grant at start-up; after a fault the M4
// restarts once the M7 has moved it off this value. Returns the latched value.
uint32_t m4_latch_restart_grant(void);
// true once the M7 has moved restart_grant off the latched value
bool m4_restart_granted(void);

#ifdef __cplusplus
}
#endif
