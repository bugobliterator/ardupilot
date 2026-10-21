#include <AP_gtest.h>
#include <AP_Rust/AP_Rust_config.h>

#if AP_SCRIPTING_RUST_ENABLED
#include <math.h>
#include <string.h>
#include <AP_Rust/Bindings.h>
#include <AP_Rust/BindingM4Policy_generated.h>
#include <AP_Rust/HostApi.h>
#include <AP_HAL/AP_HAL.h>

// binding_dispatch links the generated adapters, which reach the HAL
const AP_HAL::HAL &hal = AP_HAL::get_HAL();

using namespace AP_Rust;

static void fill_input(uint8_t *input, uint32_t size)
{
    for (uint32_t i = 0; i < size; i += 4) {
        const float value = 0.25f + 0.125f * float(i / 4U);   // finite, inside every libm domain used
        memcpy(input + i, &value, sizeof(value));
    }
}

// every local id is accepted only with its exact sizes, and nothing is written otherwise: the
// adapters write fixed sizes, and a zero-length output passes ModuleRanges at any address
TEST(RustM4LocalMath, LocalIdsRefuseAnyOtherSize)
{
    uint32_t local = 0;
    for (uint32_t id = 0; id < M4_BINDING_COUNT; id++) {
        uint32_t in_size = 0, out_size = 0;
        const bool wire = binding_local_wire(id, in_size, out_size);
        EXPECT_EQ(wire, binding_is_local(id)) << id;
        EXPECT_EQ(binding_is_local(id), M4_BINDING_POLICY[id].lane == M4Lane::Local) << id;
        if (!wire) {
            uint8_t out[4] {};
            EXPECT_EQ(binding_local_dispatch(id, nullptr, 0, out, 0), AP_RUST_STATUS_INVALID) << id;
            continue;
        }
        local++;
        uint8_t input[128];
        fill_input(input, sizeof(input));
        const uint32_t ins[] = { 0U, in_size - 1U, in_size + 4U };
        const uint32_t outs[] = { 0U, out_size == 0 ? 1U : out_size - 1U, out_size + 4U };
        for (uint32_t i : ins) {
            for (uint32_t o : outs) {
                if (i == in_size && o == out_size) {
                    continue;
                }
                uint8_t output[128];
                memset(output, 0x5A, sizeof(output));
                EXPECT_EQ(binding_local_dispatch(id, input, i, output, o), AP_RUST_STATUS_INVALID) << id;
                for (uint8_t byte : output) {
                    ASSERT_EQ(byte, 0x5A) << "local id " << id << " wrote with sizes " << i << "/" << o;
                }
            }
        }
        uint8_t output[128] {};
        EXPECT_EQ(binding_local_dispatch(id, input, in_size, output, out_size), AP_RUST_STATUS_OK) << id;
    }
    EXPECT_EQ(local, 37U);
}

// the packed call policy says what the M4 policy table says, for every id and past the last one
TEST(RustM4LocalMath, CallPolicyMatchesTheBindingPolicy)
{
    uint32_t posted = 0, mirrored = 0;
    for (uint32_t id = 0; id < M4_BINDING_COUNT + 4U; id++) {
        const uint8_t flags = id < M4_BINDING_COUNT ? M4_BINDING_POLICY[id].flags : 0U;
        EXPECT_EQ(binding_is_posted(id), (flags & M4_FLAG_POSTED) != 0) << id;
        const uint8_t role = (flags & M4_FLAG_MIRROR_GET) != 0 ? 1U : (flags & M4_FLAG_MIRROR_HANDLE_WRITER) != 0 ? 2U :
                             (flags & M4_FLAG_MIRROR_NAME_WRITER) != 0 ? 3U : 0U;
        EXPECT_EQ(binding_mirror_role(id), role) << id;
        EXPECT_EQ(binding_is_local(id), id < M4_BINDING_COUNT && M4_BINDING_POLICY[id].lane == M4Lane::Local) << id;
        posted += binding_is_posted(id) ? 1U : 0U;
        mirrored += role != 0 ? 1U : 0U;
    }
    EXPECT_GT(posted, 0U);
    EXPECT_GT(mirrored, 0U);
    EXPECT_FALSE(binding_is_local(0xFFFFFFFFU) || binding_is_posted(0xFFFFFFFFU) || binding_mirror_role(0xFFFFFFFFU) != 0);
}

// the M4's libm adapters are generated apart from the M7's: same bytes for the same input. The math
// adapters are the same generated code on both cores, so an x86 comparison of them proves nothing;
// they are compared M4-versus-in-process on Renode and the bench (SITL AirShow, Renode AirShow).
TEST(RustM4LocalMath, LibmEqualsTheM7Dispatch)
{
    uint32_t checked = 0;
    for (uint32_t id = 0; id < M4_BINDING_COUNT; id++) {
        uint32_t in_size = 0, out_size = 0;
        if (!binding_local_wire(id, in_size, out_size) || out_size != 4U || in_size > 8U ||
            binding_math_dispatch(BindingId(id), nullptr, 0, nullptr, 0) != AP_RUST_STATUS_INVALID) {
            continue;   // not a libm id: binding_math_dispatch answers only math ids
        }
        uint8_t input[8];
        fill_input(input, sizeof(input));
        uint8_t local_out[4] {}, m7_out[4] {};
        EXPECT_EQ(binding_local_dispatch(id, input, in_size, local_out, 4), AP_RUST_STATUS_OK) << id;
        EXPECT_EQ(binding_dispatch(nullptr, BindingId(id), input, in_size, m7_out, 4), AP_RUST_STATUS_OK) << id;
        EXPECT_EQ(memcmp(local_out, m7_out, sizeof(local_out)), 0) << id;
        checked++;
    }
    EXPECT_EQ(checked, 13U);
}
#endif

AP_GTEST_MAIN()
