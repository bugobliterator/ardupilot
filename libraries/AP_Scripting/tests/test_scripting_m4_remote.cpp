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
    { "Counter", 0, nullptr, nullptr },
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

static int thing_echo_many(lua_State *L) {
    binding_argcheck(L, 2);
    for (int i = 0; i < 4; i++) {
        luaL_checkstring(L, 2);
    }
    return 0;
}
static int thing_fail(lua_State *L) {
    binding_argcheck(L, 1);
    return luaL_error(L, "internal error: %s is null", "Object");
}
static int thing_absent(lua_State *L) {
    return not_supported_error(L, 1, "thing");
}
static int thing_usec(lua_State *L) {
    binding_argcheck(L, 1);
    *new_uint64_t(L) = 5000000000ULL;
    return 1;
}
static int thing_last(lua_State *L) {
    binding_argcheck(L, 1);
    lua_pushinteger(L, 1);
    lua_pushinteger(L, 2);
    lua_pushinteger(L, 3);
    return 1;
}
static int thing_malformed(lua_State *L) {
    binding_argcheck(L, 1);
    lua_pushinteger(L, 1);
    return 2;
}
static int thing_no_object(lua_State *L) {
    binding_argcheck(L, 1);
    *new_Object(L) = nullptr;
    return 1;
}

// binary results, and the type of an argument, as lua_bindings.cpp's manual functions use them
static int thing_bytes(lua_State *L) {
    binding_argcheck(L, 1);
    static const char bytes[] = { 'a', 0, 'b' };
    lua_pushlstring(L, bytes, sizeof(bytes));
    return 1;
}
static int thing_types(lua_State *L) {
    for (int arg = 1; arg <= 4; arg++) {
        lua_pushinteger(L, lua_type(L, arg));
    }
    return 4;
}
// the most a readstring() reply carries, with the values that follow it
static int thing_chunk(lua_State *L) {
    binding_argcheck(L, 1);
    static char chunk[IO_CHUNK];
    memset(chunk, 'c', sizeof(chunk));
    lua_pushlstring(L, chunk, sizeof(chunk));
    *new_uint32_t(L) = 7;
    lua_pushinteger(L, 5);
    return 3;
}
static int thing_too_long(lua_State *L) {
    binding_argcheck(L, 1);
    // past what a uint16_t length holds, and past the arena
    static char text[70000];
    lua_pushlstring(L, text, sizeof(text));
    return 1;
}

// an owned type: the object lives in the pool, the M4 holds an owned handle
struct Counter {
    int32_t value;
};
static const uint8_t COUNTER = 4;
static int counters_destroyed = 0;
static void destroy_Counter(void *object)
{
    counters_destroyed++;
    static_cast<Counter *>(object)->~Counter();
}
static Counter * check_Counter(lua_State *L, int arg) {
    return static_cast<Counter *>(remote_check_owned(L, arg, COUNTER));
}
static Counter * new_Counter(lua_State *L) {
    void *ud = remote_new_owned(L, COUNTER, destroy_Counter);
    new (ud) Counter();
    return static_cast<Counter *>(ud);
}
// Counter(start): made before its argument is checked, as the in-process creators make their userdata first
static int counter_new(lua_State *L) {
    binding_argcheck(L, 1);
    Counter *counter = new_Counter(L);
    counter->value = luaL_checkinteger(L, 1);
    return 1;
}
static int counter_add(lua_State *L) {
    binding_argcheck(L, 2);
    Counter *ud = check_Counter(L, 1);
    ud->value += luaL_checkinteger(L, 2);
    return 0;
}
static int counter_value(lua_State *L) {
    binding_argcheck(L, 1);
    lua_pushinteger(L, check_Counter(L, 1)->value);
    return 1;
}
static int counter_new_dropped(lua_State *L) {
    binding_argcheck(L, 0);
    new_Counter(L);
    return 0;
}
// two objects, only the last returned
static int counter_two(lua_State *L) {
    binding_argcheck(L, 0);
    new_Counter(L)->value = 1;
    new_Counter(L)->value = 2;
    return 1;
}
// two objects, both returned
static int counter_pair(lua_State *L) {
    binding_argcheck(L, 0);
    new_Counter(L)->value = 1;
    new_Counter(L)->value = 2;
    return 2;
}
static int thing_string_of(lua_State *L) {
    binding_argcheck(L, 1);
    static char text[MAX_STRING + 1];
    memset(text, 's', sizeof(text));
    lua_pushlstring(L, text, size_t(luaL_checkinteger(L, 1)));
    return 1;
}

}  // namespace ScriptingM4Remote

using namespace ScriptingM4Remote;

struct Call {
    RemoteCall call {};
    HandleTable handles;
    ObjectPool objects;
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
        call.objects = &objects;
        call.warn = warn;
        call.warn_context = this;
        call.codecs = test_codecs;
        call.codec_count = 4;
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
    EXPECT_EQ(c.error_arg, 0);   // not the previous call's
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
    const uint32_t generation = c.values[0].handle_generation;
    EXPECT_EQ(generation, c.handles.generation(0));
    EXPECT_EQ(generation >> 8, 7U);
    c.args.nil();
    EXPECT_EQ(c.run(thing_object), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].handle_index, 0);   // deduplicated
    EXPECT_EQ(c.values[0].handle_generation, generation);
    c.args.nil();
    c.args.handle(OBJECT, 0, generation);
    EXPECT_EQ(c.run(thing_read), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].integer, 42);
}

TEST(ScriptingM4Remote, StaleHandle)
{
    Call c;
    c.args.nil();
    ASSERT_EQ(c.run(thing_object), ReplyStatus::Ok);
    const uint32_t generation = c.values[0].handle_generation;
    c.handles.reset(8);   // the M4 restarted
    c.args.nil();
    c.args.handle(OBJECT, 0, generation);
    EXPECT_EQ(c.run(thing_read), ReplyStatus::Error);
    EXPECT_EQ(c.text(0), "M4: stale Object");
    c.args.nil();
    c.args.handle(OBJECT, 5, c.handles.generation(5));   // never handed out
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

TEST(ScriptingM4Remote, WrongHandleType)
{
    Call c;
    c.args.nil();
    c.args.handle(PAIR, 0, 7);
    EXPECT_EQ(c.run(thing_read), ReplyStatus::ArgError);
    EXPECT_EQ(c.error_arg, 2);
    EXPECT_EQ(c.text(0), "Object expected, got Pair");
}

TEST(ScriptingM4Remote, BlobWithTrailingBytes)
{
    Call c;
    const uint8_t long_wire[12] = {};
    c.args.blob(PAIR, long_wire, sizeof(long_wire));
    EXPECT_EQ(c.run(Pair_a, METHOD_WRITES_SELF), ReplyStatus::Error);
    EXPECT_EQ(c.text(0), "M4: invalid Pair");
}

TEST(ScriptingM4Remote, ArenaExhausted)
{
    Call c;
    const std::string text(600, 'x');
    c.args.nil();
    c.args.string(text.c_str(), text.size());
    EXPECT_EQ(c.run(thing_echo_many), ReplyStatus::Error);
    EXPECT_EQ(c.text(0), "M4: call too large");
}

TEST(ScriptingM4Remote, ErrorFunctions)
{
    Call c;
    c.args.nil();
    EXPECT_EQ(c.run(thing_fail), ReplyStatus::Error);
    EXPECT_EQ(c.error_arg, 0);
    EXPECT_EQ(c.text(0), "internal error: Object is null");
    c.args.nil();
    EXPECT_EQ(c.run(thing_absent), ReplyStatus::ArgError);
    EXPECT_EQ(c.error_arg, 1);
    EXPECT_EQ(c.text(0), "thing not supported on this firmware");
}

TEST(ScriptingM4Remote, Uint64)
{
    Call c;
    c.call.uint64_type = 4;
    c.args.nil();
    EXPECT_EQ(c.run(thing_usec), ReplyStatus::Ok);
    ASSERT_EQ(c.count, 1);
    ASSERT_EQ(c.values[0].tag, ValueTag::Blob);
    EXPECT_EQ(c.values[0].type_id, 4);
    ASSERT_EQ(c.values[0].length, 8);
    uint64_t usec;
    memcpy(&usec, c.values[0].blob, sizeof(usec));
    EXPECT_EQ(usec, 5000000000ULL);
}

TEST(ScriptingM4Remote, ReturnShapes)
{
    Call c;
    // the last values pushed, as a lua_CFunction returns
    c.args.nil();
    EXPECT_EQ(c.run(thing_last), ReplyStatus::Ok);
    ASSERT_EQ(c.count, 1);
    EXPECT_EQ(c.values[0].integer, 3);
    c.args.nil();
    EXPECT_EQ(c.run(thing_malformed), ReplyStatus::Error);
    EXPECT_EQ(c.text(0), "M4: malformed binding");
    // a null object is nil and takes no handle
    c.args.nil();
    EXPECT_EQ(c.run(thing_no_object), ReplyStatus::Ok);
    ASSERT_EQ(c.count, 1);
    EXPECT_EQ(c.values[0].tag, ValueTag::Nil);
    EXPECT_EQ(c.handles.count(), 0);
}

TEST(ScriptingM4Remote, RaiseOutsideRunTraps)
{
    Call c;
    // no run() is active: the jump buffer is not a live frame
    EXPECT_DEATH(thing_set(&c.call), "");
}

TEST(ScriptingM4Remote, WireWriterCapacity)
{
    // the writer fills its caller's buffer: a value past the end is refused whole
    uint8_t buf[6];
    ScriptingM4::WireWriter w(buf, sizeof(buf));
    w.write(uint32_t(0x01020304));
    EXPECT_TRUE(w.ok());
    EXPECT_EQ(w.length(), 4);
    w.write(uint32_t(5));
    EXPECT_FALSE(w.ok());
    EXPECT_EQ(w.length(), 4);
    // once failed, it stays failed: a codec cannot leave a blob with a hole in it
    w.write(uint8_t(6));
    EXPECT_FALSE(w.ok());
    EXPECT_EQ(w.length(), 4);
    EXPECT_EQ(w.data(), buf);
    ScriptingM4::WireReader r(buf, 4);
    uint32_t value = 0;
    EXPECT_TRUE(r.read(value));
    EXPECT_EQ(value, 0x01020304U);
    EXPECT_TRUE(r.done());
    uint8_t extra = 0;
    EXPECT_FALSE(r.read(extra));
}

TEST(ScriptingM4Remote, OwnedObjectsKeepTheirStateOnTheM7)
{
    Call c;
    counters_destroyed = 0;
    c.args.integer(5);
    ASSERT_EQ(c.run(counter_new), ReplyStatus::Ok);
    ASSERT_EQ(c.count, 1);
    ASSERT_EQ(c.values[0].tag, ValueTag::Handle);
    EXPECT_EQ(c.values[0].type_id, COUNTER);
    const uint16_t index = c.values[0].handle_index;
    const uint32_t generation = c.values[0].handle_generation;
    EXPECT_EQ(c.objects.used(), 1);
    c.args.handle(COUNTER, index, generation);
    c.args.integer(2);
    EXPECT_EQ(c.run(counter_add), ReplyStatus::Ok);
    EXPECT_EQ(c.count, 0);                       // no write-back: the object is the M7's
    c.args.handle(COUNTER, index, generation);
    ASSERT_EQ(c.run(counter_value), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].integer, 7);
    // two constructions are two objects under two handles, never deduplicated
    c.args.integer(1);
    ASSERT_EQ(c.run(counter_new), ReplyStatus::Ok);
    EXPECT_NE(c.values[0].handle_index, index);
    EXPECT_EQ(c.objects.used(), 2);
    EXPECT_EQ(counters_destroyed, 0);
}

TEST(ScriptingM4Remote, OwnedObjectsOfAFailedCallAreDestroyed)
{
    Call c;
    counters_destroyed = 0;
    // made, then an argument error: without the clean-up its slot stays taken until the session ends
    c.args.string("x", 1);
    EXPECT_EQ(c.run(counter_new), ReplyStatus::ArgError);
    EXPECT_EQ(c.objects.used(), 0);
    EXPECT_EQ(counters_destroyed, 1);
    EXPECT_EQ(c.handles.count(), 0);
    // made and not returned
    EXPECT_EQ(c.run(counter_new_dropped), ReplyStatus::Ok);
    EXPECT_EQ(c.count, 0);
    EXPECT_EQ(c.objects.used(), 0);
    EXPECT_EQ(counters_destroyed, 2);
    // two made, the last one returned
    ASSERT_EQ(c.run(counter_two), ReplyStatus::Ok);
    ASSERT_EQ(c.count, 1);
    EXPECT_EQ(c.objects.used(), 1);
    EXPECT_EQ(counters_destroyed, 3);
    c.args.handle(COUNTER, c.values[0].handle_index, c.values[0].handle_generation);
    ASSERT_EQ(c.run(counter_value), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].integer, 2);
}

TEST(ScriptingM4Remote, FailedSecondOwnedResultRetiresTheFirstEntry)
{
    Call c;
    counters_destroyed = 0;
    static int others[HandleTable::CAPACITY - 1];
    for (int &other : others) {
        ASSERT_GE(c.handles.insert(OBJECT, &other), 0);
    }
    // the first object gets the last entry, the second finds the table full: without the forget()
    // in abandon_owned the first entry names a freed slot
    EXPECT_EQ(c.run(counter_pair), ReplyStatus::Error);
    EXPECT_EQ(c.text(0), TOO_MANY_OBJECTS);
    EXPECT_EQ(c.objects.used(), 0);
    EXPECT_EQ(counters_destroyed, 2);
    EXPECT_EQ(c.handles.count(), HandleTable::CAPACITY - 1);
}

TEST(ScriptingM4Remote, OwnedObjectLimits)
{
    Call c;
    counters_destroyed = 0;
    for (uint8_t i = 0; i < ObjectPool::SLOTS; i++) {
        c.args.integer(i);
        ASSERT_EQ(c.run(counter_new), ReplyStatus::Ok) << int(i);
    }
    // the pool is full: refused with the text the M4 retries on; without the bound the 65th is made
    c.args.integer(0);
    EXPECT_EQ(c.run(counter_new), ReplyStatus::Error);
    EXPECT_EQ(c.text(0), TOO_MANY_OBJECTS);
    EXPECT_EQ(c.objects.used(), uint8_t(ObjectPool::SLOTS));
    EXPECT_EQ(counters_destroyed, 0);
}

TEST(ScriptingM4Remote, FullHandleTableDestroysTheNewObject)
{
    Call c;
    counters_destroyed = 0;
    static int others[HandleTable::CAPACITY];
    for (int &other : others) {
        ASSERT_GE(c.handles.insert(OBJECT, &other), 0);
    }
    c.args.integer(3);
    EXPECT_EQ(c.run(counter_new), ReplyStatus::Error);
    EXPECT_EQ(c.text(0), TOO_MANY_OBJECTS);
    // without the clean-up the object would hold its slot with no handle naming it
    EXPECT_EQ(c.objects.used(), 0);
    EXPECT_EQ(counters_destroyed, 1);
}

TEST(ScriptingM4Remote, ReleasedOwnedHandleIsStale)
{
    Call c;
    c.args.integer(1);
    ASSERT_EQ(c.run(counter_new), ReplyStatus::Ok);
    const RpcValue handle = c.values[0];
    void *object = c.handles.release_owned(COUNTER, handle.handle_index, handle.handle_generation);
    ASSERT_NE(object, nullptr);
    EXPECT_TRUE(c.objects.destroy(object));
    c.args.handle(COUNTER, handle.handle_index, handle.handle_generation);
    EXPECT_EQ(c.run(counter_value), ReplyStatus::Error);
    EXPECT_EQ(c.text(0), "M4: stale Counter");
    // another type's handle where a Counter is expected keeps the in-process text
    c.args.handle(OBJECT, 0, c.handles.generation(0));
    EXPECT_EQ(c.run(counter_value), ReplyStatus::ArgError);
    EXPECT_EQ(c.text(0), "Counter expected, got Object");
}

TEST(ScriptingM4Remote, BinaryStringResults)
{
    Call c;
    c.args.nil();
    ASSERT_EQ(c.run(thing_bytes), ReplyStatus::Ok);
    ASSERT_EQ(c.values[0].tag, ValueTag::String);
    // lua_pushstring() would stop at the NUL: 1 byte
    EXPECT_EQ(c.values[0].length, 3);
    EXPECT_EQ(memcmp(c.values[0].string, "a\0b", 3), 0);
}

TEST(ScriptingM4Remote, ArgumentTypes)
{
    Call c;
    c.args.nil();
    c.args.integer(3);
    c.args.string("s", 1);
    ASSERT_EQ(c.run(thing_types), ReplyStatus::Ok);
    // lua.h: LUA_TNIL 0, LUA_TNUMBER 3, LUA_TSTRING 4, LUA_TNONE -1
    EXPECT_EQ(c.values[0].integer, 0);
    EXPECT_EQ(c.values[1].integer, 3);
    EXPECT_EQ(c.values[2].integer, 4);
    EXPECT_EQ(c.values[3].integer, -1);

    // LUA_TBOOLEAN 1, LUA_TNUMBER for a float, LUA_TTABLE 5 (an opaque value), LUA_TUSERDATA 7
    c.args.boolean(true);
    c.args.number(1.5f);
    c.args.opaque(5);
    c.args.handle(OBJECT, 0, c.handles.generation(0));
    ASSERT_EQ(c.run(thing_types), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].integer, 1);
    EXPECT_EQ(c.values[1].integer, 3);
    EXPECT_EQ(c.values[2].integer, 5);
    EXPECT_EQ(c.values[3].integer, 7);
}

TEST(ScriptingM4Remote, ReadChunkFitsOneReply)
{
    Call c;
    c.args.nil();
    // without IO_CHUNK sized for the values that follow the string the reply overflows
    ASSERT_EQ(c.run(thing_chunk), ReplyStatus::Ok);
    ASSERT_EQ(c.count, 3);
    EXPECT_EQ(c.values[0].length, IO_CHUNK);
    EXPECT_EQ(c.values[2].integer, 5);
}

TEST(ScriptingM4Remote, StringResultLimit)
{
    Call c;
    c.args.nil();
    // without the bound in lua_pushlstring() the arena's "M4: call too large" is the text
    EXPECT_EQ(c.run(thing_too_long), ReplyStatus::Error);
    EXPECT_EQ(c.text(0), "M4: results too large");
}

TEST(ScriptingM4Remote, StringResultBoundary)
{
    Call c;
    c.args.integer(MAX_STRING);
    ASSERT_EQ(c.run(thing_string_of), ReplyStatus::Ok);
    ASSERT_EQ(c.count, 1);
    EXPECT_EQ(c.values[0].length, MAX_STRING);
    // one more byte: refused, not truncated
    c.args.integer(MAX_STRING + 1);
    EXPECT_EQ(c.run(thing_string_of), ReplyStatus::Error);
    EXPECT_EQ(c.text(0), "M4: results too large");
}

AP_GTEST_MAIN()
