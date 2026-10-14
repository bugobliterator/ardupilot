#pragma once
/*
  M4 Lua build: ap_setjmp without AP_HAL_Boards.h. On the hard-float M4 the
  AP implementation (AP_HAL_ChibiOS/ap_setjmp.c) saves d8-d15, which the
  gcc 10 newlib setjmp does not; on a host (an ARM Linux one included) it is
  the C library's setjmp.
 */
#include <setjmp.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__arm__) && defined(__ARM_FP) && !defined(__unix__) && !defined(__APPLE__)
#define AP_SETJMP_CORTEX_ENABLE 1
#include <inttypes.h>
struct _ap_jmp_buf {
    int64_t data[13];
};
typedef struct _ap_jmp_buf ap_jmp_buf[1];
int __attribute__((naked, noinline)) ap_setjmp(ap_jmp_buf env);
void __attribute__((noreturn, naked, noinline)) ap_longjmp(ap_jmp_buf env, int val);
#elif defined(__unix__) || defined(__APPLE__)
// the SITL M4 process: _setjmp/_longjmp leave the signal mask alone. The Lua thread blocks
// SIGUSR1 (the restart signal), and with setjmp/longjmp a caught Lua error left SIGUSR1 blocked
// in the service thread too (macOS), which then ignored every restart
#define AP_SETJMP_CORTEX_ENABLE 0
#define ap_setjmp(env) (_setjmp(env))
#define ap_longjmp(env, val) (_longjmp(env, val))
typedef jmp_buf ap_jmp_buf;
#else
#define AP_SETJMP_CORTEX_ENABLE 0
#define ap_setjmp(env) (setjmp(env))
#define ap_longjmp(env, val) (longjmp(env, val))
typedef jmp_buf ap_jmp_buf;
#endif

#ifdef __cplusplus
}
#endif
