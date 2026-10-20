/*
  The module's host callbacks as the module calls them: unprivileged code in
  the image (MPU region 0), touching only the module stack. Each one is an
  SVC answered by module_port.cpp.
 */
#include <stdint.h>
#include <AP_Rust/HostApi.h>
#include <AP_RustM4_HostCalls.h>
#include "module_svc.h"

using namespace RustM4;

extern "C" {

static uint8_t *tramp_allocate(void *context, uint32_t size, uint32_t alignment)
{
    return reinterpret_cast<uint8_t *>(uintptr_t(svc<SYS_ALLOCATE>(uint32_t(uintptr_t(context)), size, alignment)));
}

static void tramp_deallocate(void *context, uint8_t *allocation)
{
    (void)svc<SYS_DEALLOCATE>(uint32_t(uintptr_t(context)), uint32_t(uintptr_t(allocation)));
}

static uint64_t tramp_time_us(void *context)
{
    const SvcPair r = svc_pair<SYS_TIME_US>(uint32_t(uintptr_t(context)));
    return uint64_t(r.r0) | (uint64_t(r.r1) << 32);
}

static int32_t tramp_sleep_ms(void *context, uint32_t milliseconds)
{
    return int32_t(svc<SYS_SLEEP_MS>(uint32_t(uintptr_t(context)), milliseconds));
}

static uint8_t tramp_stop_requested(void *context)
{
    return uint8_t(svc<SYS_STOP_REQUESTED>(uint32_t(uintptr_t(context))));
}

static int32_t tramp_text(void *context, uint8_t severity, const uint8_t *message, uint32_t length)
{
    return int32_t(svc<SYS_TEXT>(uint32_t(uintptr_t(context)), severity, uint32_t(uintptr_t(message)), length));
}

static void tramp_panic(void *context, const uint8_t *message, uint32_t length)
{
    (void)svc<SYS_PANIC>(uint32_t(uintptr_t(context)), uint32_t(uintptr_t(message)), length);
    for (;;) {
        __builtin_trap();   // SYS_PANIC never returns
    }
}

// six arguments: packed on the module stack, r0 points at them; the handler checks the address
static int32_t tramp_binding_call(void *context, uint32_t id, const uint8_t *input, uint32_t input_size,
                                  uint8_t *output, uint32_t output_size)
{
    const BindingCallArgs args { context, id, input, input_size, output, output_size };
    return int32_t(svc<SYS_BINDING_CALL>(uint32_t(uintptr_t(&args))));
}

extern const AP_RustHostApi module_host_api;
const AP_RustHostApi module_host_api {
    AP_RUST_ABI_VERSION, sizeof(AP_RustHostApi), AP_RUST_API_FINGERPRINT,
    tramp_allocate, tramp_deallocate, tramp_time_us, tramp_sleep_ms, tramp_stop_requested, tramp_text, tramp_panic,
    tramp_binding_call
};

// the module thread's first unprivileged code: SP points at its LaunchRecord (module_port.cpp)
__attribute__((naked, noreturn)) void module_entry_stub(void)
{
    __asm volatile(
        "mov    r4, sp\n"
        "ldr    r9, [r4, #12]\n"   // the module's RWPI static base
        "ldr    r12, [r4, #0]\n"
        "ldr    r0, [r4, #4]\n"
        "ldr    r1, [r4, #8]\n"
        "blx    r12\n"             // start(api, context)
        "svc    %[returned]\n"     // r0: start()'s result; the thread ends there
        "b      .\n"
        :: [returned] "i"(SYS_MODULE_RETURNED));
}

}  // extern "C"
