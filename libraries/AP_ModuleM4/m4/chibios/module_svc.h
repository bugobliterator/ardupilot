#pragma once

/*
  The SVC numbers of the module thread's host callbacks and the inline
  callers the unprivileged trampolines use. r0-r3 carry the arguments, r0
  (and r1 for a 64-bit result) the result; port_syscall writes them back.
 */

#include <stdint.h>

enum : uint32_t {
    SYS_ALLOCATE = 0,         // (context, size, alignment) -> pointer or 0
    SYS_DEALLOCATE = 1,       // (context, pointer); a bad pointer panics the module
    SYS_TIME_US = 2,          // (context) -> the M7's micros64() in r0 (low) and r1 (high)
    SYS_SLEEP_MS = 3,         // (context, ms) -> status
    SYS_STOP_REQUESTED = 4,   // (context) -> 0 or 1
    SYS_TEXT = 5,             // (context, severity, message, length) -> status
    SYS_PANIC = 6,            // (context, message, length); never returns
    SYS_BINDING_CALL = 7,     // (BindingCallArgs on the module stack) -> status
    SYS_MODULE_RETURNED = 8,  // (start()'s result); ends the thread
    SYS_FAULT_REQUEST = 9,    // () -> FaultKind; the built-in selftest only
    SYS_COUNT = 10,
};

// r0-r3 are clobbered, so the compiler keeps no argument in them before the moves
template <uint32_t N>
static inline uint32_t svc(uint32_t a0 = 0, uint32_t a1 = 0, uint32_t a2 = 0, uint32_t a3 = 0)
{
    uint32_t result;
    __asm volatile(
        "mov    r0, %[a0]\n"
        "mov    r1, %[a1]\n"
        "mov    r2, %[a2]\n"
        "mov    r3, %[a3]\n"
        "svc    %[number]\n"
        "mov    %[result], r0\n"
        : [result] "=r"(result)
        : [number] "i"(N), [a0] "r"(a0), [a1] "r"(a1), [a2] "r"(a2), [a3] "r"(a3)
        : "r0", "r1", "r2", "r3", "memory");
    return result;
}

struct SvcPair {
    uint32_t r0, r1;
};

template <uint32_t N>
static inline SvcPair svc_pair(uint32_t a0)
{
    uint32_t r0, r1;
    __asm volatile("mov r0, %[a0]\n" "svc %[number]\n" "mov %[r0], r0\n" "mov %[r1], r1\n"
                   : [r0] "=r"(r0), [r1] "=r"(r1) : [number] "i"(N), [a0] "r"(a0) : "r0", "r1", "r2", "r3", "memory");
    return SvcPair { r0, r1 };
}
