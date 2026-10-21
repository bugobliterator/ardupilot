// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"
#if AP_MODULE_ENABLED
#include <AP_Module/BindingIds.h>
#include <stdint.h>

namespace AP_Module
{
// Load-time hash lookup only. UINT32_MAX means no compatible public contract.
uint32_t binding_resolve(void *context, uint64_t hash);
// Firmware-only dispatcher. Non-overlapping buffers are borrowed for this call,
// never retained. These trusted native pointers are not sandbox-validated.
// Module validates its stop token before forwarding to this entry point.
int32_t binding_dispatch(void *context, BindingId id, const uint8_t *input, uint32_t input_size,
                         uint8_t *output, uint32_t output_size);
int32_t binding_math_dispatch(BindingId id, const uint8_t *input, uint32_t input_size,
                              uint8_t *output, uint32_t output_size);
// M4 builds: the bindings the Cortex-M4 answers itself (BindingLocal_generated.cpp)
bool binding_is_local(uint32_t id);
bool binding_local_wire(uint32_t id, uint32_t &input_size, uint32_t &output_size);
int32_t binding_local_dispatch(uint32_t id, const uint8_t *input, uint32_t input_size, uint8_t *output,
                               uint32_t output_size);
// M4 builds: ids the M4 sends without waiting for a reply, and the Parameter mirror role of an id
// (0 none, 1 Parameter.get, 2 a Parameter handle writer, 3 a by-name parameter writer)
bool binding_is_posted(uint32_t id);
uint8_t binding_mirror_role(uint32_t id);
// M4 builds: the bits of an M4_CALL_POLICY byte (BindingM4Policy_generated.h), which the four functions
// above read: posted, the mirror role, and for a local id 1 + the index of its sizes in M4_LOCAL_WIRES
static const uint8_t M4_CALL_POSTED = 1U;
static const uint8_t M4_CALL_ROLE_SHIFT = 1U;
static const uint8_t M4_CALL_ROLE_MASK = 3U;
static const uint8_t M4_CALL_WIRE_SHIFT = 3U;
struct LocalImport {
    uint64_t hash;
    uint32_t id;
};
extern const LocalImport M4_LOCAL_IMPORTS[];
extern const uint32_t M4_LOCAL_IMPORT_COUNT;
}
#endif
