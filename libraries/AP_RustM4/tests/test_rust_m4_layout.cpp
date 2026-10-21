#include <AP_gtest.h>
#include <AP_RustM4/AP_RustM4_Layout.h>

using namespace RustM4;

TEST(RustM4Layout, ProtocolNeverMatchesLua)
{
    EXPECT_EQ(PROTOCOL_VERSION, 0x52530002U);
}

TEST(RustM4Layout, ContractHashCoversTheCatalogue)
{
    EXPECT_NE(CONTRACT_HASH, fnv1a_word(PROTOCOL_VERSION, FNV_OFFSET));
}

TEST(RustM4Layout, ModuleAreaEndsAtTheWindow)
{
    EXPECT_EQ(AP_RUST_M4_MODULE_AREA_BASE, 0x30008000U);
    EXPECT_EQ(AP_RUST_M4_MODULE_AREA_SIZE, 229376U);
    EXPECT_EQ(AP_RUST_M4_MODULE_AREA_BASE + AP_RUST_M4_MODULE_AREA_SIZE, AP_RUST_M4_WINDOW_BASE);
}

TEST(RustM4Layout, SupervisorBlockIsAtTheWindowEnd)
{
    static uint8_t window[AP_RUST_M4_WINDOW_SIZE];
    EXPECT_EQ(reinterpret_cast<uint8_t *>(supervisor_block(window)) + sizeof(SupervisorBlock), window + sizeof(window));
}

AP_GTEST_MAIN()
