#include <AP_gtest.h>
#include <string>
#include <AP_Scripting/AP_ScriptingM4_Rpc.h>

using namespace ScriptingM4;

TEST(ScriptingM4Rpc, ValuesRoundTrip)
{
    uint8_t buf[MAX_FRAME];
    RpcWriter writer;
    writer.begin(buf, sizeof(buf), 0);
    writer.nil();
    writer.boolean(true);
    writer.boolean(false);
    writer.integer(-42);
    writer.number(1.5f);
    writer.string("hello", 5);
    writer.opaque(5);   // LUA_TTABLE
    ASSERT_TRUE(writer.ok());
    EXPECT_EQ(writer.count(), 7U);
    RpcValue values[MAX_VALUES];
    ASSERT_TRUE(decode_values(buf, writer.length(), writer.count(), values));
    EXPECT_EQ(values[0].tag, ValueTag::Nil);
    EXPECT_EQ(values[1].tag, ValueTag::True);
    EXPECT_EQ(values[2].tag, ValueTag::False);
    EXPECT_EQ(values[3].integer, -42);
    EXPECT_FLOAT_EQ(values[4].number, 1.5f);
    ASSERT_EQ(values[5].length, 5U);
    EXPECT_EQ(std::string(values[5].string, values[5].length), "hello");
    EXPECT_STREQ(value_type_name(&values[6]), "table");
    EXPECT_STREQ(value_type_name(nullptr), "no value");
}

TEST(ScriptingM4Rpc, DecodeRejectsBadInput)
{
    uint8_t buf[32];
    RpcWriter writer;
    writer.begin(buf, sizeof(buf), 0);
    writer.integer(7);
    writer.string("abc", 3);
    ASSERT_TRUE(writer.ok());
    const uint16_t len = writer.length();
    RpcValue values[MAX_VALUES];
    EXPECT_FALSE(decode_values(buf, len - 1, 2, values));             // truncated string
    EXPECT_FALSE(decode_values(buf, len, 3, values));                 // more values than encoded
    EXPECT_FALSE(decode_values(buf, len, 1, values));                 // trailing bytes
    EXPECT_FALSE(decode_values(buf, len, MAX_VALUES + 1, values));    // over the value limit
    buf[0] = 9;                                                       // past Opaque: an unknown tag
    EXPECT_FALSE(decode_values(buf, len, 2, values));
    const uint8_t long_string[] = { uint8_t(ValueTag::String), 0xFF, 0xFF };
    EXPECT_FALSE(decode_values(long_string, sizeof(long_string), 1, values));   // length past the frame
    const uint8_t short_integer[] = { uint8_t(ValueTag::Integer), 1, 2, 3 };
    EXPECT_FALSE(decode_values(short_integer, sizeof(short_integer), 1, values));
    const uint8_t short_number[] = { uint8_t(ValueTag::Number), 1, 2, 3 };
    EXPECT_FALSE(decode_values(short_number, sizeof(short_number), 1, values));
    const uint8_t short_opaque[] = { uint8_t(ValueTag::Opaque) };
    EXPECT_FALSE(decode_values(short_opaque, sizeof(short_opaque), 1, values));
    const uint8_t unknown_tag[] = { 255 };
    EXPECT_FALSE(decode_values(unknown_tag, sizeof(unknown_tag), 1, values));
    EXPECT_FALSE(decode_values(buf, len, 0, values));                 // bytes with no value expected
    EXPECT_TRUE(decode_values(buf, 0, 0, values));                    // empty
    static uint8_t oversized[3 + MAX_STRING + 1];
    oversized[0] = uint8_t(ValueTag::String);
    const uint16_t oversized_length = MAX_STRING + 1;
    memcpy(&oversized[1], &oversized_length, sizeof(oversized_length));
    EXPECT_FALSE(decode_values(oversized, sizeof(oversized), 1, values));   // MAX_STRING + 1 inside the frame
}

TEST(ScriptingM4Rpc, WriterRefusesOverflow)
{
    uint8_t small[8];
    RpcWriter writer;
    writer.begin(small, sizeof(small), 4);
    writer.integer(1);                       // 5 bytes after a 4-byte header: does not fit
    EXPECT_FALSE(writer.ok());
    uint8_t big[MAX_FRAME];
    writer.begin(big, sizeof(big), 0);
    for (uint8_t i = 0; i <= MAX_VALUES; i++) {
        writer.nil();
    }
    EXPECT_FALSE(writer.ok());               // more than MAX_VALUES
    static char text[MAX_STRING + 1];
    writer.begin(big, sizeof(big), 0);
    writer.string(text, sizeof(text));
    EXPECT_FALSE(writer.ok());               // longer than MAX_STRING
}

TEST(ScriptingM4Rpc, ArgsGiveLauxlibTexts)
{
    uint8_t buf[64];
    RpcWriter writer;
    writer.begin(buf, sizeof(buf), 0);
    writer.nil();             // 1: self of a singleton method
    writer.string("x", 1);    // 2
    writer.number(2.5f);      // 3
    writer.opaque(5);         // 4: a table
    RpcValue values[MAX_VALUES];
    ASSERT_TRUE(decode_values(buf, writer.length(), writer.count(), values));
    RpcArgs args(values, writer.count());
    int32_t integer = 0;
    EXPECT_FALSE(args.integer(2, integer));
    EXPECT_STREQ(args.message(), "number expected, got string");
    EXPECT_EQ(args.error_arg(), 2U);
    EXPECT_FALSE(args.integer(3, integer));
    EXPECT_STREQ(args.message(), "number has no integer representation");
    EXPECT_FALSE(args.integer(5, integer));
    EXPECT_STREQ(args.message(), "number expected, got no value");
    const char *text = nullptr;
    uint16_t len = 0;
    EXPECT_FALSE(args.string(4, text, len));
    EXPECT_STREQ(args.message(), "string expected, got table");
    EXPECT_FALSE(args.count_is(3));
    EXPECT_STREQ(args.message(), "too many arguments");
    EXPECT_EQ(args.error_arg(), 4U);
    EXPECT_FALSE(args.count_is(5));
    EXPECT_STREQ(args.message(), "too few arguments");
    EXPECT_TRUE(args.count_is(4));
}

TEST(ScriptingM4Rpc, ArgsConvertLikeLua)
{
    uint8_t buf[64];
    RpcWriter writer;
    writer.begin(buf, sizeof(buf), 0);
    writer.number(3.0f);      // 1
    writer.string("10", 2);   // 2
    writer.integer(9);        // 3
    writer.number(4.0f);      // 4
    writer.number(2.5f);      // 5
    RpcValue values[MAX_VALUES];
    ASSERT_TRUE(decode_values(buf, writer.length(), writer.count(), values));
    RpcArgs args(values, writer.count());
    int32_t value = 0;
    ASSERT_TRUE(args.integer(1, value));
    EXPECT_EQ(value, 3);
    ASSERT_TRUE(args.integer(2, value));
    EXPECT_EQ(value, 10);
    EXPECT_FALSE(args.integer_in(3, 0, 7, value));
    EXPECT_STREQ(args.message(), "out of range");
    EXPECT_EQ(args.error_arg(), 3U);
    const char *text = nullptr;
    uint16_t len = 0;
    ASSERT_TRUE(args.string(3, text, len));
    EXPECT_EQ(std::string(text, len), "9");
    ASSERT_TRUE(args.string(4, text, len));
    EXPECT_EQ(std::string(text, len), "4.0");
    ASSERT_TRUE(args.string(5, text, len));
    EXPECT_EQ(std::string(text, len), "2.5");
}

TEST(ScriptingM4Rpc, ArgsRejectStringsBeyondInt32)
{
    // Lua reads these as floats without an integer representation, whatever the host long size
    const char *const texts[] = { "3000000000", "2147483648", "-3000000000", "99999999999999999999" };
    for (const char *text : texts) {
        uint8_t buf[64];
        RpcWriter writer;
        writer.begin(buf, sizeof(buf), 0);
        writer.string(text, strlen(text));
        RpcValue values[MAX_VALUES];
        ASSERT_TRUE(decode_values(buf, writer.length(), writer.count(), values));
        RpcArgs args(values, writer.count());
        int32_t value = 0;
        EXPECT_FALSE(args.integer(1, value)) << text;
        EXPECT_STREQ(args.message(), "number has no integer representation") << text;
    }
    uint8_t buf[64];
    RpcWriter writer;
    writer.begin(buf, sizeof(buf), 0);
    writer.string("2147483647", 10);
    writer.string("-2147483648", 11);
    RpcValue values[MAX_VALUES];
    ASSERT_TRUE(decode_values(buf, writer.length(), writer.count(), values));
    RpcArgs args(values, writer.count());
    int32_t value = 0;
    ASSERT_TRUE(args.integer(1, value));
    EXPECT_EQ(value, INT32_MAX);
    ASSERT_TRUE(args.integer(2, value));
    EXPECT_EQ(value, INT32_MIN);
}

TEST(ScriptingM4Rpc, ContractHashCoversTheTable)
{
    struct Info {
        uint16_t id;
        const char *name;
        MethodKind kind;
    };
    static const Info table[] = {
#define TEST_METHOD_INFO(id, symbol, name, kind) { id, name, MethodKind::kind },
        SCRIPTING_M4_SYSTEM_METHODS(TEST_METHOD_INFO)
#undef TEST_METHOD_INFO
    };
    // the constexpr chain folds from the last method to the first
    uint32_t hash = fnv1a_word(MAX_BLOB, fnv1a_word(RPC_PROTOCOL_VERSION, fnv1a_word(MAX_FRAME, fnv1a_word(sizeof(CallHeader),
                    fnv1a_word(sizeof(ReplyHeader), FNV_OFFSET)))));
    for (int i = int(sizeof(table) / sizeof(table[0])) - 1; i >= 0; i--) {
        hash = fnv1a_string(table[i].name, fnv1a_word((uint32_t(table[i].id) << 1) | uint32_t(table[i].kind), hash));
    }
    EXPECT_EQ(hash, SYSTEM_CONTRACT_HASH);
    EXPECT_NE(SYSTEM_CONTRACT_HASH, 0x4D344231U);   // the contract of the ping/pong protocol
    EXPECT_STREQ(method_name(uint16_t(SystemMethod::GcsSendText)), "gcs:send_text");
    EXPECT_STREQ(method_name(999), "m4:unknown");
    EXPECT_EQ(method_kind(uint16_t(SystemMethod::ScriptError)), MethodKind::Notify);
    EXPECT_LT(uint16_t(SystemMethod::Print), FIRST_BINDING_METHOD);
}

TEST(ScriptingM4Rpc, BlobAndHandleRoundTrip)
{
    uint8_t buf[MAX_FRAME];
    RpcWriter writer;
    writer.begin(buf, sizeof(buf), 0);
    const uint8_t bytes[] = { 1, 2, 3, 4, 5 };
    writer.blob(7, bytes, sizeof(bytes));
    writer.handle(9, 3, 0x01020304);
    ASSERT_TRUE(writer.ok());
    RpcValue values[MAX_VALUES];
    ASSERT_TRUE(decode_values(buf, writer.length(), writer.count(), values));
    EXPECT_EQ(values[0].tag, ValueTag::Blob);
    EXPECT_EQ(values[0].type_id, 7);
    EXPECT_EQ(values[0].length, sizeof(bytes));
    EXPECT_EQ(memcmp(values[0].blob, bytes, sizeof(bytes)), 0);
    EXPECT_EQ(values[1].tag, ValueTag::Handle);
    EXPECT_EQ(values[1].type_id, 9);
    EXPECT_EQ(values[1].handle_index, 3);
    EXPECT_EQ(values[1].handle_generation, 0x01020304U);
}

TEST(ScriptingM4Rpc, BlobAndHandleRejected)
{
    uint8_t buf[MAX_FRAME];
    RpcWriter writer;
    writer.begin(buf, sizeof(buf), 0);
    uint8_t big[MAX_BLOB + 1] = {};
    writer.blob(1, big, sizeof(big));
    EXPECT_FALSE(writer.ok());                 // larger than a blob may be
    writer.restart();
    EXPECT_TRUE(writer.ok());
    EXPECT_EQ(writer.length(), 0);
    EXPECT_EQ(writer.count(), 0);
    writer.blob(0, big, 4);
    EXPECT_FALSE(writer.ok());                 // type 0 is no type
    RpcValue values[1];
    const uint8_t zero_type[] = { uint8_t(ValueTag::Blob), 0, 1, 0, 0xAA };
    EXPECT_FALSE(decode_values(zero_type, sizeof(zero_type), 1, values));
    const uint8_t past_end[] = { uint8_t(ValueTag::Blob), 3, 9, 0, 0xAA };
    EXPECT_FALSE(decode_values(past_end, sizeof(past_end), 1, values));
    const uint8_t too_long[] = { uint8_t(ValueTag::Blob), 3, 0x01, 0x02 };   // 513 bytes declared
    EXPECT_FALSE(decode_values(too_long, sizeof(too_long), 1, values));
    const uint8_t short_handle[] = { uint8_t(ValueTag::Handle), 3, 0, 0, 1, 0 };
    EXPECT_FALSE(decode_values(short_handle, sizeof(short_handle), 1, values));
    const uint8_t zero_handle[] = { uint8_t(ValueTag::Handle), 0, 0, 0, 1, 0, 0, 0 };
    EXPECT_FALSE(decode_values(zero_handle, sizeof(zero_handle), 1, values));
    static uint8_t over[4 + MAX_BLOB + 1] = { uint8_t(ValueTag::Blob), 3, uint8_t((MAX_BLOB + 1) & 0xFF), uint8_t((MAX_BLOB + 1) >> 8) };
    EXPECT_FALSE(decode_values(over, sizeof(over), 1, values));          // 513 bytes present
    static uint8_t at_limit[4 + MAX_BLOB] = { uint8_t(ValueTag::Blob), 3, uint8_t(MAX_BLOB & 0xFF), uint8_t(MAX_BLOB >> 8) };
    EXPECT_TRUE(decode_values(at_limit, sizeof(at_limit), 1, values));   // factor_table is exactly 512
    const uint8_t short_blob[] = { uint8_t(ValueTag::Blob), 3, 1 };
    EXPECT_FALSE(decode_values(short_blob, sizeof(short_blob), 1, values));
    const uint8_t handle_6[] = { uint8_t(ValueTag::Handle), 3, 0, 0, 1, 0, 0 };   // one byte short
    EXPECT_FALSE(decode_values(handle_6, sizeof(handle_6), 1, values));
    writer.restart();
    writer.handle(0, 1, 1);
    EXPECT_FALSE(writer.ok());                                           // type 0 is no type
}

static const char *test_namer(uint8_t type_id)
{
    return type_id == 4 ? "Location" : "userdata";
}

TEST(ScriptingM4Rpc, NumberAndUserdataNames)
{
    uint8_t buf[MAX_FRAME];
    RpcWriter writer;
    writer.begin(buf, sizeof(buf), 0);
    writer.integer(5);
    writer.string("2.5", 3);
    const uint8_t bytes[] = { 0 };
    writer.blob(4, bytes, sizeof(bytes));
    RpcValue values[MAX_VALUES];
    ASSERT_TRUE(decode_values(buf, writer.length(), writer.count(), values));
    RpcArgs args(values, writer.count(), test_namer);
    float number = 0;
    EXPECT_TRUE(args.number(1, number));
    EXPECT_FLOAT_EQ(number, 5.0f);
    EXPECT_TRUE(args.number(2, number));
    EXPECT_FLOAT_EQ(number, 2.5f);
    EXPECT_FALSE(args.number(3, number));
    EXPECT_EQ(args.error_arg(), 3);
    // luaL_typeerror() names a userdata by its metatable's __name
    EXPECT_STREQ(args.message(), "number expected, got Location");
    EXPECT_STREQ(args.type_name(4), "no value");
    RpcArgs unnamed(values, writer.count());
    EXPECT_STREQ(unnamed.type_name(3), "userdata");
}

AP_GTEST_MAIN()
