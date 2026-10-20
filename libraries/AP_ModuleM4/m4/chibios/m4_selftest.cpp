/*
  MOD_M4_INJECT 17: a module built into the image that checks the module
  thread path before a loader exists. It runs unprivileged as a module
  does, through the module_host_api trampolines and the SVC table, with its
  code and constants as its RX range: both are placed in the sections
  module_m4.ld links into .m4_selftest (the Makefile fails the image if
  anything of this object is left in .text or .rodata). No statics: those
  would be privileged RAM. No libm: newlib's errno is privileged.
 */
#include <stdint.h>
#include <AP_Module/BindingIds.h>
#include <AP_Module/HostApi.h>
#include "module_svc.h"

// in privileged RAM (ram0): what a module must never reach
static const uintptr_t PRIVILEGED_ADDRESS = 0x30000100U;

#define SELFTEST_CODE __attribute__((section(".m4_selftest.text"), noinline))
#define SELFTEST_CONST __attribute__((section(".m4_selftest.rodata")))

union FloatBytes {
    float value;
    uint8_t bytes[4];
};

extern "C" SELFTEST_CODE int32_t m4_selftest_start(const AP_ModuleHostApi *api, void *context)
{
    const uint32_t sinf_id = uint32_t(AP_Module::BindingId::LIBM_SINF);
    uint32_t failed = 0;

    uint8_t *block = api->allocate(context, 64, 8);
    if (block == nullptr) {
        failed |= 1U << 0;
    } else {
        block[0] = 1;
        block[63] = 2;
        api->deallocate(context, block);
    }

    // libm.sinf, answered on the M4 with its exact sizes, buffers on this stack
    FloatBytes input;
    FloatBytes output;
    input.value = 0.5f;
    output.value = 0.0f;
    if (api->binding_call(context, sinf_id, input.bytes, 4, output.bytes, 4) != AP_MODULE_STATUS_OK ||
        output.value < 0.4794f || output.value > 0.4795f) {
        failed |= 1U << 1;
    }

    // containment: privileged buffers, a size other than the local wire's, and a foreign context
    // are INVALID, never a fault
    const uint8_t *privileged_input = reinterpret_cast<const uint8_t *>(PRIVILEGED_ADDRESS);
    uint8_t *privileged_output = reinterpret_cast<uint8_t *>(PRIVILEGED_ADDRESS);
    if (api->binding_call(context, sinf_id, privileged_input, 4, output.bytes, 4) != AP_MODULE_STATUS_INVALID ||
        api->binding_call(context, sinf_id, input.bytes, 4, privileged_output, 4) != AP_MODULE_STATUS_INVALID ||
        api->binding_call(context, sinf_id, input.bytes, 4, privileged_output, 0) != AP_MODULE_STATUS_INVALID ||
        api->binding_call(context, sinf_id, input.bytes, 4, output.bytes, 2) != AP_MODULE_STATUS_INVALID ||
        api->binding_call(reinterpret_cast<void *>(PRIVILEGED_ADDRESS), sinf_id, input.bytes, 4, output.bytes, 4) !=
        AP_MODULE_STATUS_INVALID ||
        api->text(context, 6, privileged_input, 4) != AP_MODULE_STATUS_INVALID) {
        failed |= 1U << 2;
    }

    const uint64_t before = api->time_us(context);
    if (api->sleep_ms(context, 2) != AP_MODULE_STATUS_OK || api->stop_requested(context) != 0 ||
        api->time_us(context) < before + 1000U) {
        failed |= 1U << 3;
    }
    (void)svc<SYS_FAULT_REQUEST>();

    static const char ok[] SELFTEST_CONST = "M4 thread ok";
    static const char bad[] SELFTEST_CONST = "M4 thread check failed";
    if (failed == 0) {
        api->text(context, 6, reinterpret_cast<const uint8_t *>(ok), sizeof(ok) - 1);
    } else {
        api->text(context, 3, reinterpret_cast<const uint8_t *>(bad), sizeof(bad) - 1);
    }
    return int32_t(failed);
}
