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
    buf[0] = 6;                                                       // Blob: not in protocol 2
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
    uint32_t hash = fnv1a_word(RPC_PROTOCOL_VERSION, fnv1a_word(MAX_FRAME, fnv1a_word(sizeof(CallHeader),
                    fnv1a_word(sizeof(ReplyHeader), FNV_OFFSET))));
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

AP_GTEST_MAIN()
