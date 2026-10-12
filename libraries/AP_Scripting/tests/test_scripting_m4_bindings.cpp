#include <AP_gtest.h>
#include <string.h>
#include <AP_Scripting/AP_ScriptingM4_Bindings.h>
#if __cplusplus >= 201703L
// C++17 (CubeFramework): builds configured with --enable-scripting-m4
#include <AP_Scripting/AP_ScriptingM4_Layout.h>
#define HAVE_SCRIPTING_M4_LAYOUT 1
#endif
// the M4's tables as gen-bindings -r wrote them for this tree
#include <AP_Scripting/lua_remote_tables.cpp>

using namespace ScriptingM4;

static uint8_t type_named(const char *name)
{
    for (uint8_t id = 1; id <= BINDING_TYPE_COUNT; id++) {
        if (strcmp(binding_types[id - 1].name, name) == 0) {
            return id;
        }
    }
    return 0;
}

static const BindingRow *row_named(uint8_t type_id, const char *name)
{
    const BindingType &type = binding_types[type_id - 1];
    for (uint16_t i = type.first_row; i < type.first_row + type.row_count; i++) {
        if (strcmp(binding_rows[i].name, name) == 0) {
            return &binding_rows[i];
        }
    }
    return nullptr;
}

TEST(ScriptingM4Bindings, TypesHaveTheirKinds)
{
    struct { const char *name; BindingKind kind; } expected[] = {
        { "ahrs", BindingKind::Singleton }, { "gcs", BindingKind::Singleton }, { "rc", BindingKind::Singleton },
        { "Location", BindingKind::LocalBlob }, { "Vector3f", BindingKind::LocalBlob },
        { "Vector2f", BindingKind::LocalBlob }, { "Quaternion", BindingKind::LocalBlob },
        { "CANFrame", BindingKind::Blob }, { "RangeFinder_State", BindingKind::Blob },
        { "ServoTelemetryData", BindingKind::Blob },
        { "RC_Channel", BindingKind::Handle }, { "AP_EFI_Backend", BindingKind::Handle },
        { "Parameter", BindingKind::Owned },
        { "uint32_t", BindingKind::Local }, { "uint64_t", BindingKind::Local },
        { "global", BindingKind::Global },
    };
    for (const auto &e : expected) {
        const uint8_t id = type_named(e.name);
        ASSERT_NE(id, 0) << e.name;
        EXPECT_EQ(binding_types[id - 1].kind, uint8_t(e.kind)) << e.name;
    }
    EXPECT_EQ(type_named("uint32_t"), BINDING_TYPE_UINT32);
    EXPECT_EQ(type_named("uint64_t"), BINDING_TYPE_UINT64);
    // userdata carrying M7 pointers or state, and the manual-only ap_objects, wait for
    // IO and Manual Bindings (none of these is renamed: the C++ name is the Lua name)
    for (const char *name : { "AP_Scripting_SerialAccess", "CRSFMenu", "CRSFParameter",
                              "DroneCAN_Handle", "SocketAPM", "ScriptingCANBuffer", "AP_HAL::I2CDevice",
                              "AP_HAL::PWMSource" }) {
        EXPECT_EQ(type_named(name), 0) << name;
    }
}

TEST(ScriptingM4Bindings, RowsAndIdsAgree)
{
    char label[64];
    for (uint8_t id = 1; id <= BINDING_TYPE_COUNT; id++) {
        const BindingType &type = binding_types[id - 1];
        ASSERT_LE(type.first_row + type.row_count, BINDING_ROW_COUNT) << type.name;
        for (uint16_t i = type.first_row; i < type.first_row + type.row_count; i++) {
            const BindingRow &row = binding_rows[i];
            EXPECT_EQ(row.type, id) << row.name;
            ASSERT_GE(row.method, FIRST_BINDING_METHOD) << row.name;
            ASSERT_LT(row.method - FIRST_BINDING_METHOD, BINDING_METHOD_COUNT) << row.name;
            const bool plain = (row.flags & (ROW_OPERATOR | ROW_CREATION)) == 0;
            if (type.kind == uint8_t(BindingKind::Singleton)) {
                EXPECT_TRUE(row.flags & ROW_HAS_SELF) << type.name << ":" << row.name;
            }
            if ((type.kind == uint8_t(BindingKind::Blob) || type.kind == uint8_t(BindingKind::LocalBlob)) && plain) {
                EXPECT_TRUE(row.flags & ROW_WRITES_SELF) << type.name << ":" << row.name;
            }
        }
    }
    for (uint16_t i = 0; i < BINDING_METHOD_COUNT; i++) {
        const uint16_t row = binding_method_rows[i];
        if (row == NO_ROW) {
            continue;
        }
        ASSERT_LT(row, BINDING_ROW_COUNT);
        EXPECT_EQ(binding_rows[row].method, FIRST_BINDING_METHOD + i);
        // an SRPC name fits StatsMsg::name
        binding_method_label(FIRST_BINDING_METHOD + i, label, sizeof(label));
        const BindingType &type = binding_types[binding_rows[row].type - 1];
        EXPECT_EQ(strlen(label), strlen(type.name) + 1 + strlen(binding_rows[row].name)) << label;
    }
}

TEST(ScriptingM4Bindings, KnownBindings)
{
    const uint8_t ahrs = type_named("ahrs");
    const BindingRow *get_location = row_named(ahrs, "get_location");
    ASSERT_NE(get_location, nullptr);
    char label[64];
    binding_method_label(get_location->method, label, sizeof(label));
    EXPECT_STREQ(label, "ahrs:get_location");
    binding_method_label(uint16_t(SystemMethod::Config), label, sizeof(label));
    EXPECT_STREQ(label, "m4:config");
    // a short buffer is cut and terminated
    char cut[8];
    memset(cut, 'x', sizeof(cut));
    binding_method_label(get_location->method, cut, sizeof(cut));
    EXPECT_STREQ(cut, "ahrs:ge");
    memset(cut, 'x', sizeof(cut));
    binding_method_label(get_location->method, cut, 1);
    EXPECT_STREQ(cut, "");
    // the longest label in the description
    const BindingRow *differential = row_named(type_named("AP_Airspeed_Backend"), "handle_script_differential_pressure");
    ASSERT_NE(differential, nullptr);
    binding_method_label(differential->method, label, sizeof(label));
    EXPECT_STREQ(label, "AP_Airspeed_Backend:handle_script_differential_pressure");

    const BindingRow *creation = row_named(type_named("CANFrame"), "new");
    ASSERT_NE(creation, nullptr);
    EXPECT_TRUE(creation->flags & ROW_CREATION);
    EXPECT_NE(row_named(type_named("rc"), "get_channel"), nullptr);
    EXPECT_NE(row_named(type_named("param"), "set"), nullptr);   // set_by_name, renamed
    EXPECT_NE(row_named(type_named("efi"), "get_backend"), nullptr);
    const uint8_t global = type_named("global");
    EXPECT_NE(row_named(global, "millis"), nullptr);
    EXPECT_NE(row_named(global, "micros"), nullptr);
    EXPECT_EQ(row_named(global, "dirlist"), nullptr);           // manual: IO and Manual Bindings
    EXPECT_EQ(row_named(global, "PWMSource"), nullptr);         // manual-only ap_object

    // an owned type's constructor is its ported creation function, called as the global Parameter
    const BindingRow *parameter_new = row_named(type_named("Parameter"), "new");
    ASSERT_NE(parameter_new, nullptr);
    EXPECT_EQ(parameter_new->flags, ROW_CREATION);

    const BindingType &gps = binding_types[type_named("gps") - 1];
    bool found = false;
    for (uint8_t i = 0; i < gps.enum_count; i++) {
        found |= strcmp(binding_enum_names[gps.first_enum + i], "GPS_OK_FIX_3D") == 0;
    }
    EXPECT_TRUE(found);
}

TEST(ScriptingM4Bindings, LocalTypesKeepOnlyStateRows)
{
    // a local type's fields, operators, constructor and listed methods run on the M4: no row
    for (const char *name : { "Vector2f", "Vector3f", "Quaternion" }) {
        EXPECT_EQ(binding_types[type_named(name) - 1].row_count, 0) << name;
    }
    const uint8_t location = type_named("Location");
    for (const char *name : { "new", "lat", "loiter_xtrack", "get_distance", "offset", "get_distance_NE", "copy",
                              "set_alt_m", "get_alt_frame" }) {
        EXPECT_EQ(row_named(location, name), nullptr) << name;
    }
    // the methods that read home, origin or terrain stay rows and write the result back
    for (const char *name : { "change_alt_frame", "get_alt_m", "get_vector_from_origin_NEU",
                              "get_vector_from_origin_NEU_m", "get_vector_from_origin_NEU_cm" }) {
        const BindingRow *row = row_named(location, name);
        ASSERT_NE(row, nullptr) << name;
        EXPECT_TRUE(row->flags & ROW_WRITES_SELF) << name;
    }
    EXPECT_EQ(binding_types[location - 1].row_count, 5);
}

TEST(ScriptingM4Bindings, ContractCoversTheTable)
{
#ifdef HAVE_SCRIPTING_M4_LAYOUT
    EXPECT_EQ(CONTRACT_HASH, fnv1a_word(BINDING_HASH, SYSTEM_CONTRACT_HASH));
#endif
    EXPECT_NE(BINDING_HASH, FNV_OFFSET);
}

TEST(ScriptingM4Bindings, InfoImage)
{
    uint8_t info[BINDING_INFO_SIZE] = {};
    info_set_method(info, FIRST_BINDING_METHOD + 3);
    EXPECT_TRUE(info_method_available(info, FIRST_BINDING_METHOD + 3));
    EXPECT_FALSE(info_method_available(info, FIRST_BINDING_METHOD + 2));
    EXPECT_FALSE(info_method_available(info, uint16_t(SystemMethod::Config)));
    EXPECT_FALSE(info_method_available(info, FIRST_BINDING_METHOD + BINDING_METHOD_COUNT));
    info_set_type(info, BINDING_TYPE_COUNT);
    EXPECT_TRUE(info_type_available(info, BINDING_TYPE_COUNT));
    EXPECT_FALSE(info_type_available(info, 0));
    info_set_enum(info, BINDING_ENUM_COUNT - 1, -7);
    EXPECT_EQ(info_enum_value(info, BINDING_ENUM_COUNT - 1), -7);
    EXPECT_EQ(info_enum_value(info, BINDING_ENUM_COUNT), 0);   // past the end
}

AP_GTEST_MAIN()
