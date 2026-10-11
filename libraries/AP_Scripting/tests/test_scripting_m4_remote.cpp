#include <AP_gtest.h>
#include <new>
#include <string>
#include <string.h>
#include <type_traits>
#include <AP_Scripting/AP_ScriptingM4_Remote.h>

using namespace ScriptingM4;

namespace ScriptingM4Remote {

// a blob type with two fields (a: int32 written with the range -100..100, b: float),
// a handle type and uint32_t, as the generated dispatch has them
struct Pair {
    int32_t a;
    float b;
    Pair() : a(0), b(1.5f) {}
};
static_assert(std::is_trivially_destructible<Pair>::value, "remote blobs are never destroyed");
static const uint8_t PAIR = 1;
static const uint8_t OBJECT = 2;
static const uint8_t UINT32 = 3;

static bool read_Pair(WireReader &r, Pair *ud)
{
    {
        int32_t v;
        if (!r.read(v)) {
            return false;
        }
        if (int64_t(v) < int64_t(-100) || int64_t(v) > int64_t(100)) {
            return false;
        }
        ud->a = v;
    }
    {
        float v;
        if (!r.read(v)) {
            return false;
        }
        ud->b = v;
    }
    return true;
}
static void write_Pair(WireWriter &w, const Pair *ud)
{
    w.write(int32_t(ud->a));
    w.write(float(ud->b));
}
static bool decode_Pair(WireReader &r, void *storage)
{
    Pair *ud = new (storage) Pair();
    return read_Pair(r, ud) && r.done();
}
static void encode_Pair(WireWriter &w, const void *object)
{
    write_Pair(w, static_cast<const Pair *>(object));
}
static const BlobCodec test_codecs[] = {
    { "Pair", sizeof(Pair), decode_Pair, encode_Pair },
    { "Object", 0, nullptr, nullptr },
    { "uint32_t", 0, nullptr, nullptr },
};

static Pair * check_Pair(lua_State *L, int arg) {
    return static_cast<Pair *>(remote_check_blob(L, arg, PAIR));
}
static Pair * new_Pair(lua_State *L) {
    void *ud = remote_new_blob(L, PAIR, sizeof(Pair));
    new (ud) Pair();
    return static_cast<Pair *>(ud);
}
static int ** check_Object(lua_State *L, int arg) {
    return reinterpret_cast<int **>(remote_check_handle(L, arg, OBJECT));
}
static int ** new_Object(lua_State *L) {
    return reinterpret_cast<int **>(remote_new_handle(L, OBJECT));
}

static int object_value = 42;

// singleton method: set(severity 0..7, text) -> boolean
static int thing_set(lua_State *L) {
    binding_argcheck(L, 3);
    const lua_Integer raw_data_2 = get_integer(L, 2, 0, 7);
    const int32_t data_2 = raw_data_2;
    const char * data_3 = luaL_checkstring(L, 3);
    lua_pushboolean(L, data_2 == 3 && strcmp(data_3, "ok") == 0);
    return 1;
}
// blob field a, read and write
static int Pair_a(lua_State *L) {
    Pair *ud = check_Pair(L, 1);
    switch(lua_gettop(L)) {
        case 1:
            lua_pushinteger(L, ud->a);
            return 1;
        case 2: {
            const lua_Integer raw_data_2 = get_integer(L, 2, -100, 100);
            const int32_t data_2 = raw_data_2;
            ud->a = data_2;
            return 0;
         }
        default:
            return field_argerror(L);
    }
}
// singleton methods returning a handle, a blob and a uint32_t, and taking them back
static int thing_object(lua_State *L) {
    binding_argcheck(L, 1);
    int *data = &object_value;
    if (data == NULL) {
        return 0;
    }
    *new_Object(L) = data;
    return 1;
}
static int thing_read(lua_State *L) {
    binding_argcheck(L, 2);
    int * data_2 = *check_Object(L, 2);
    lua_pushinteger(L, *data_2);
    return 1;
}
static int thing_pair(lua_State *L) {
    binding_argcheck(L, 1);
    const Pair data = Pair();
    *new_Pair(L) = data;
    return 1;
}
static int thing_clock(lua_State *L) {
    binding_argcheck(L, 1);
    *new_uint32_t(L) = 4000000000U;
    return 1;
}
static int thing_mod(lua_State *L) {
    binding_argcheck(L, 2);
    const uint32_t raw_data_2 = coerce_to_uint32_t(L, 2);
    lua_pushinteger(L, int32_t(raw_data_2 % 1000));
    return 1;
}
static int thing_deprecated(lua_State *L) {
    binding_argcheck(L, 1);
    lua_scripts::set_and_print_new_error_message(4, "thing:deprecated %s", "use thing:new");
    return 0;
}
static int thing_many(lua_State *L) {
    binding_argcheck(L, 1);
    for (int i = 0; i < 20; i++) {
        lua_pushinteger(L, i);
    }
    return 20;
}

}  // namespace ScriptingM4Remote

using namespace ScriptingM4Remote;

struct Call {
    RemoteCall call {};
    HandleTable handles;
    uint8_t args_frame[MAX_FRAME];
    RpcWriter args;
    RpcValue arg_values[MAX_VALUES];
    uint8_t reply_frame[MAX_FRAME];
    RpcWriter results;
    RpcValue values[MAX_VALUES];
    uint8_t count = 0;
    uint8_t error_arg = 0;
    uint8_t warned_severity = 0;
    char warned[128] {};

    static void warn(void *context, uint8_t severity, const char *text)
    {
        Call *self = static_cast<Call *>(context);
        self->warned_severity = severity;
        strncpy(self->warned, text, sizeof(self->warned) - 1);
    }

    Call()
    {
        handles.reset(7);
        call.handles = &handles;
        call.warn = warn;
        call.warn_context = this;
        call.codecs = test_codecs;
        call.codec_count = 3;
        call.uint32_type = UINT32;
        call.uint64_type = 0;   // no uint64_t in these tests
        args.begin(args_frame, sizeof(args_frame), 0);
    }

    ReplyStatus run(lua_CFunction function, uint8_t flags = 0)
    {
        EXPECT_TRUE(args.ok());
        EXPECT_TRUE(decode_values(args_frame, args.length(), args.count(), arg_values));
        results.begin(reply_frame, sizeof(reply_frame), 0);
        const ReplyStatus status = ScriptingM4Remote::run(call, arg_values, args.count(), function, flags,
                                                          results, error_arg);
        EXPECT_TRUE(results.ok());
        count = results.count();
        EXPECT_TRUE(decode_values(reply_frame, results.length(), count, values));
        args.begin(args_frame, sizeof(args_frame), 0);
        return status;
    }

    std::string text(uint8_t i) const
    {
        return std::string(values[i].string, values[i].length);
    }

    void pair(int32_t a, float b)
    {
        uint8_t wire[8];
        memcpy(&wire[0], &a, 4);
        memcpy(&wire[4], &b, 4);
        args.blob(PAIR, wire, sizeof(wire));
    }
};

TEST(ScriptingM4Remote, ArgumentTexts)
{
    Call c;
    c.args.nil();
    c.args.string("x", 1);
    c.args.string("y", 1);
    EXPECT_EQ(c.run(thing_set), ReplyStatus::ArgError);
    EXPECT_EQ(c.error_arg, 2);
    EXPECT_EQ(c.text(0), "number expected, got string");

    c.args.nil();
    c.args.integer(9);
    c.args.string("y", 1);
    EXPECT_EQ(c.run(thing_set), ReplyStatus::ArgError);
    EXPECT_EQ(c.text(0), "out of range");

    c.args.nil();
    c.args.integer(6);
    EXPECT_EQ(c.run(thing_set), ReplyStatus::ArgError);
    EXPECT_EQ(c.error_arg, 2);
    EXPECT_EQ(c.text(0), "too few arguments");

    c.args.nil();
    c.args.integer(3);
    c.args.string("ok", 2);
    c.args.nil();
    EXPECT_EQ(c.run(thing_set), ReplyStatus::ArgError);
    EXPECT_EQ(c.error_arg, 4);
    EXPECT_EQ(c.text(0), "too many arguments");

    c.args.nil();
    c.args.integer(3);
    c.args.string("ok", 2);
    EXPECT_EQ(c.run(thing_set), ReplyStatus::Ok);
    ASSERT_EQ(c.count, 1);
    EXPECT_EQ(c.values[0].tag, ValueTag::True);
}

TEST(ScriptingM4Remote, StringsAreTerminated)
{
    Call c;
    RpcValue values[3] {};
    values[0].tag = ValueTag::Nil;
    values[1].tag = ValueTag::Integer;
    values[1].integer = 3;
    static const char frame_bytes[] = "okX";   // a frame string is not terminated
    values[2].tag = ValueTag::String;
    values[2].string = frame_bytes;
    values[2].length = 2;
    uint8_t reply[MAX_FRAME];
    RpcWriter results;
    results.begin(reply, sizeof(reply), 0);
    uint8_t error_arg = 0;
    EXPECT_EQ(run(c.call, values, 3, thing_set, 0, results, error_arg), ReplyStatus::Ok);
    RpcValue out[1];
    ASSERT_TRUE(decode_values(reply, results.length(), results.count(), out));
    EXPECT_EQ(out[0].tag, ValueTag::True);
}

TEST(ScriptingM4Remote, BlobsRoundTrip)
{
    Call c;
    c.args.nil();
    EXPECT_EQ(c.run(thing_pair), ReplyStatus::Ok);
    ASSERT_EQ(c.count, 1);
    ASSERT_EQ(c.values[0].tag, ValueTag::Blob);
    EXPECT_EQ(c.values[0].type_id, PAIR);
    ASSERT_EQ(c.values[0].length, 8);
    uint8_t made[8];
    memcpy(made, c.values[0].blob, sizeof(made));
    // what the M7 made comes back unchanged through a getter, which writes the self back
    c.args.blob(PAIR, made, sizeof(made));
    EXPECT_EQ(c.run(Pair_a, METHOD_WRITES_SELF), ReplyStatus::Ok);
    ASSERT_EQ(c.count, 2);
    EXPECT_EQ(c.values[0].integer, 0);
    ASSERT_EQ(c.values[1].tag, ValueTag::Blob);
    EXPECT_EQ(memcmp(c.values[1].blob, made, sizeof(made)), 0);
    // a setter returns nothing but the updated self
    c.pair(5, 2.5f);
    c.args.integer(7);
    EXPECT_EQ(c.run(Pair_a, METHOD_WRITES_SELF), ReplyStatus::Ok);
    ASSERT_EQ(c.count, 1);
    int32_t a;
    float b;
    memcpy(&a, &c.values[0].blob[0], 4);
    memcpy(&b, &c.values[0].blob[4], 4);
    EXPECT_EQ(a, 7);
    EXPECT_FLOAT_EQ(b, 2.5f);
}

TEST(ScriptingM4Remote, BadBlobs)
{
    Call c;
    c.pair(5, 2.5f);
    c.args.integer(200);
    EXPECT_EQ(c.run(Pair_a, METHOD_WRITES_SELF), ReplyStatus::ArgError);
    EXPECT_EQ(c.text(0), "out of range");
    // a field outside its declared range never decodes
    c.pair(1000, 0);
    EXPECT_EQ(c.run(Pair_a, METHOD_WRITES_SELF), ReplyStatus::Error);
    EXPECT_EQ(c.text(0), "M4: invalid Pair");
    const uint8_t short_wire[4] = {};
    c.args.blob(PAIR, short_wire, sizeof(short_wire));
    EXPECT_EQ(c.run(Pair_a, METHOD_WRITES_SELF), ReplyStatus::Error);
    // another type where a Pair is expected: the in-process luaL_checkudata() text
    c.args.handle(OBJECT, 0, 7);
    EXPECT_EQ(c.run(Pair_a, METHOD_WRITES_SELF), ReplyStatus::ArgError);
    EXPECT_EQ(c.error_arg, 1);
    EXPECT_EQ(c.text(0), "Pair expected, got Object");
    c.args.integer(1);
    EXPECT_EQ(c.run(Pair_a, METHOD_WRITES_SELF), ReplyStatus::ArgError);
    EXPECT_EQ(c.text(0), "Pair expected, got number");
}

TEST(ScriptingM4Remote, Handles)
{
    Call c;
    c.args.nil();
    EXPECT_EQ(c.run(thing_object), ReplyStatus::Ok);
    ASSERT_EQ(c.values[0].tag, ValueTag::Handle);
    EXPECT_EQ(c.values[0].type_id, OBJECT);
    EXPECT_EQ(c.values[0].handle_index, 0);
    EXPECT_EQ(c.values[0].handle_generation, 7U);
    c.args.nil();
    EXPECT_EQ(c.run(thing_object), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].handle_index, 0);   // deduplicated
    c.args.nil();
    c.args.handle(OBJECT, 0, 7);
    EXPECT_EQ(c.run(thing_read), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].integer, 42);
}

TEST(ScriptingM4Remote, StaleHandle)
{
    Call c;
    c.args.nil();
    ASSERT_EQ(c.run(thing_object), ReplyStatus::Ok);
    c.handles.reset(8);   // the M4 restarted
    c.args.nil();
    c.args.handle(OBJECT, 0, 7);
    EXPECT_EQ(c.run(thing_read), ReplyStatus::Error);
    EXPECT_EQ(c.text(0), "M4: stale Object");
    c.args.nil();
    c.args.handle(OBJECT, 5, 8);   // never handed out
    EXPECT_EQ(c.run(thing_read), ReplyStatus::Error);
}

TEST(ScriptingM4Remote, FullHandleTable)
{
    Call c;
    static int others[HandleTable::CAPACITY];
    for (int &other : others) {
        ASSERT_GE(c.handles.insert(OBJECT, &other), 0);
    }
    c.args.nil();
    EXPECT_EQ(c.run(thing_object), ReplyStatus::Error);
    EXPECT_EQ(c.text(0), "M4: too many objects");
}

TEST(ScriptingM4Remote, Uint32)
{
    Call c;
    c.args.nil();
    EXPECT_EQ(c.run(thing_clock), ReplyStatus::Ok);
    ASSERT_EQ(c.values[0].tag, ValueTag::Blob);
    EXPECT_EQ(c.values[0].type_id, UINT32);
    uint32_t clock;
    ASSERT_EQ(c.values[0].length, 4);
    memcpy(&clock, c.values[0].blob, 4);
    EXPECT_EQ(clock, 4000000000U);
    // coerce_to_uint32_t(): a boxed uint32_t, an integer, a number in range
    c.args.nil();
    c.args.blob(UINT32, &clock, 4);
    EXPECT_EQ(c.run(thing_mod), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].integer, 0);
    c.args.nil();
    c.args.integer(1234);
    EXPECT_EQ(c.run(thing_mod), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].integer, 234);
    c.args.nil();
    c.args.number(1500.5f);
    EXPECT_EQ(c.run(thing_mod), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].integer, 500);
    c.args.nil();
    c.args.string("x", 1);
    EXPECT_EQ(c.run(thing_mod), ReplyStatus::ArgError);
    EXPECT_EQ(c.text(0), "Unable to coerce to uint32_t");
}

TEST(ScriptingM4Remote, WarningsAndLimits)
{
    Call c;
    c.args.nil();
    EXPECT_EQ(c.run(thing_deprecated), ReplyStatus::Ok);
    EXPECT_EQ(c.warned_severity, 4);
    EXPECT_STREQ(c.warned, "thing:deprecated use thing:new");
    c.args.nil();
    EXPECT_EQ(c.run(thing_many), ReplyStatus::Error);
    EXPECT_EQ(c.text(0), "M4: results too large");
}

AP_GTEST_MAIN()
