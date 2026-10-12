#include <AP_gtest.h>
#include <new>
#include <string>
#include <string.h>
#include <type_traits>
#include <stdlib.h>
#include <AP_HAL/utility/RingBuffer.h>
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
// lua_tolstring() as AP_HAL__I2CDevice_transfer() uses it
static int thing_tolstring(lua_State *L) {
    size_t length = 99;
    const char *text = lua_tolstring(L, 1, &length);
    lua_pushboolean(L, text != nullptr);
    lua_pushinteger(L, int32_t(length));
    return 2;
}
// the bus clock argument as lua_get_i2c_device() reads it
static int thing_i2c_clock(lua_State *L) {
    lua_pushinteger(L, lua_Integer(i2c_bus_clock_argument(L, 1)));
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
// a kept handle result naming a pool object, as a manual port could write it
static int counter_as_kept_handle(lua_State *L) {
    binding_argcheck(L, 0);
    Counter *counter = new_Counter(L);
    *remote_new_handle(L, OBJECT) = counter;
    return 2;
}
static int thing_string_of(lua_State *L) {
    binding_argcheck(L, 1);
    static char text[MAX_STRING + 1];
    memset(text, 's', sizeof(text));
    lua_pushlstring(L, text, size_t(luaL_checkinteger(L, 1)));
    return 1;
}

// check_uint64_t() as DroneCAN_Handle::new_handle() uses it
static int thing_signature(lua_State *L) {
    binding_argcheck(L, 1);
    const uint64_t value = *check_uint64_t(L, 1);
    lua_pushinteger(L, int32_t(value >> 32));
    return 1;
}
// ScriptingCANBuffer:write_frame()'s timeout as the M7 port reads it
static int thing_can_timeout(lua_State *L) {
    binding_argcheck(L, 1);
    lua_pushinteger(L, int32_t(can_write_timeout_argument(L, 1)));
    return 1;
}
// DroneCAN_Handle:check_message()'s results with a payload of the given length
static int thing_dronecan_message(lua_State *L) {
    binding_argcheck(L, 1);
    static char payload[MAX_STRING];
    memset(payload, 'p', sizeof(payload));
    lua_pushlstring(L, payload, size_t(luaL_checkinteger(L, 1)));
    lua_pushinteger(L, 125);
    *new_uint64_t(L) = 5000000000ULL;
    lua_pushboolean(L, true);
    return 4;
}

// a DroneCAN payload as DroneCAN_Handle::Subscriber::Payload has it
struct TestPayload {
    uint8_t *data;
    uint16_t length;
    uint8_t node_id;
    uint64_t timestamp;
};
static ObjectBuffer<TestPayload> *test_payloads;
// check_message()'s results through the helper the port calls
static int thing_check_message(lua_State *L) {
    TestPayload payload;
    return dronecan_push_payload(L, *test_payloads, payload) ? 3 : 0;
}
// a scripting CAN buffer's write_frame(), recording the timeout it was given
struct TestFrameBuffer {
    uint32_t timeout_us = 0;
    bool write_frame(Pair &frame, uint32_t timeout)
    {
        (void)frame;
        timeout_us = timeout;
        return true;
    }
};
static TestFrameBuffer test_frame_buffer;
static int thing_write_frame(lua_State *L) {
    Pair frame;
    return CAN_write_frame(L, test_frame_buffer, frame, 1);
}
// a SocketAPM as the socket ports call it: records what it was asked, returns result with error
// as errno (error 0 leaves errno as it was)
struct TestSocket {
    ssize_t result = 0;
    int error = 0;
    size_t size = 0;
    uint32_t address = 0;
    uint16_t port = 0;
    uint32_t timeout_ms = 0;
    int blocking = -1;   // the last set_blocking() argument, -1 for none
    bool open = true;
    ssize_t send(const void *, size_t length)
    {
        size = length;
        if (error != 0) {
            errno = error;
        }
        return result;
    }
    ssize_t sendto(const void *buf, size_t length, uint32_t to_address, uint16_t to_port)
    {
        address = to_address;
        port = to_port;
        return send(buf, length);
    }
    bool pollin(uint32_t timeout) { timeout_ms = timeout; return true; }
    bool pollout(uint32_t timeout) { timeout_ms = timeout + 1; return true; }
    bool set_blocking(bool value) { blocking = value ? 1 : 0; return open; }
};
static TestSocket test_socket;
// sock:send(data, len), sock:sendto(data, len, addr, port), sock:pollin/pollout(timeout) and
// sock:set_blocking(blocking) through the helpers the M7 ports call (argument 1: the self)
static int thing_socket_send(lua_State *L) {
    return socket_send(L, test_socket);
}
static int thing_socket_sendto(lua_State *L) {
    return socket_sendto(L, test_socket);
}
static int thing_socket_pollin(lua_State *L) {
    return socket_poll(L, test_socket, false);
}
static int thing_socket_pollout(lua_State *L) {
    return socket_poll(L, test_socket, true);
}
static int thing_socket_set_blocking(lua_State *L) {
    return socket_set_blocking(L, test_socket);
}

// broadcast()'s and request()'s quota charge for their payload
static int thing_can_charge(lua_State *L) {
    size_t length;
    luaL_checklstring(L, 1, &length);
    lua_pushboolean(L, can_transfer_allowed(L, length));
    return 1;
}
static int thing_target_node(lua_State *L) {
    lua_pushinteger(L, dronecan_target_node(L, 1));
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
    uint16_t reply_start = 0;   // sizeof(ReplyHeader): the reply frame the server sends
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
        results.begin(reply_frame, sizeof(reply_frame), reply_start);
        const ReplyStatus status = ScriptingM4Remote::run(call, arg_values, args.count(), function, flags,
                                                          results, error_arg);
        EXPECT_TRUE(results.ok());
        count = results.count();
        EXPECT_TRUE(decode_values(reply_frame + reply_start, uint16_t(results.length() - reply_start), count, values));
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

TEST(ScriptingM4Remote, PoolObjectNeverGetsAKeptEntry)
{
    Call c;
    counters_destroyed = 0;
    // without the pool check the Handle result inserts a kept entry that outlives the owned one
    EXPECT_EQ(c.run(counter_as_kept_handle), ReplyStatus::Error);
    EXPECT_EQ(c.text(0), "M4: malformed binding");
    EXPECT_EQ(c.objects.used(), 0);
    EXPECT_EQ(counters_destroyed, 1);
    EXPECT_EQ(c.handles.count(), 0);
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


// without the nullptr/0 for a non-string, lua_tolstring() would raise an argument error (as
// luaL_checklstring does) or hand back garbage with a stale length: the third run fails
TEST(ScriptingM4Remote, ToLString)
{
    Call c;
    c.args.string("a\0b", 3);
    ASSERT_EQ(c.run(thing_tolstring), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].tag, ValueTag::True);
    EXPECT_EQ(c.values[1].integer, 3);
    c.args.integer(42);                      // a number converts, as in-process
    ASSERT_EQ(c.run(thing_tolstring), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].tag, ValueTag::True);
    EXPECT_EQ(c.values[1].integer, 2);
    c.args.nil();                            // anything else: nullptr and length 0, no error
    ASSERT_EQ(c.run(thing_tolstring), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].tag, ValueTag::False);
    EXPECT_EQ(c.values[1].integer, 0);
}

// without the clamp a clock of 0 divides by zero in ChibiOS' timeout and 1000 Hz makes a 255-byte
// transfer time out after 4 s; without the byte cap the 21 attempts exceed the host thread's 900 ms
TEST(ScriptingM4Remote, I2CLimits)
{
    EXPECT_EQ(i2c_clamp_bus_clock(0), 100000U);
    EXPECT_EQ(i2c_clamp_bus_clock(1000), 100000U);
    EXPECT_EQ(i2c_clamp_bus_clock(400000), 400000U);
    EXPECT_EQ(i2c_clamp_bus_clock(UINT32_MAX), 1000000U);
    EXPECT_TRUE(i2c_transfer_fits(1, 255));
    EXPECT_TRUE(i2c_transfer_fits(256, 0));
    EXPECT_FALSE(i2c_transfer_fits(2, 255));
    EXPECT_FALSE(i2c_transfer_fits(0, 257));
    EXPECT_FALSE(i2c_transfer_fits(1000, 1));
}

// the argument the call site reads: without the clamp in i2c_bus_clock_argument() a clock of 0
// reaches ChibiOS' timeout formula as a division by zero and 1000 Hz times a transfer out after 4 s
TEST(ScriptingM4Remote, I2CBusClockArgument)
{
    Call c;
    const int32_t requested[] = { 0, 1000, 400000, 5000000 };
    const int32_t expected[] = { 100000, 100000, 400000, 1000000 };
    for (size_t i = 0; i < 4; i++) {
        c.args.integer(requested[i]);
        ASSERT_EQ(c.run(thing_i2c_clock), ReplyStatus::Ok);
        EXPECT_EQ(c.values[0].integer, expected[i]);
    }
}

TEST(ScriptingM4Remote, Uint64Arguments)
{
    Call c;
    c.call.uint64_type = 13;   // an id no other value of these tests uses
    const uint64_t signature = 0x0F0868D0C1A7C6F1ULL;
    c.args.blob(13, &signature, sizeof(signature));
    ASSERT_EQ(c.run(thing_signature), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].integer, 0x0F0868D0);
    // another value where a uint64_t is expected: luaL_checkudata()'s text
    c.args.integer(5);
    EXPECT_EQ(c.run(thing_signature), ReplyStatus::ArgError);
    EXPECT_EQ(c.text(0), "uint64_t expected, got number");
    // the right type with the wrong length never reaches the copy
    const uint32_t short_value = 1;
    c.args.blob(13, &short_value, sizeof(short_value));
    EXPECT_EQ(c.run(thing_signature), ReplyStatus::ArgError);
}

// without the clamp a timeout of 4000 s reaches CANSensor::write_frame()'s select() and holds the
// host thread until the bus takes the frame, past the M4's 1 s call timeout
TEST(ScriptingM4Remote, CanWriteTimeoutArgument)
{
    Call c;
    const int32_t requested[] = { 0, 10000, 900000, 900001, INT32_MAX };
    const int32_t expected[] = { 0, 10000, 900000, 900000, 900000 };
    for (size_t i = 0; i < 5; i++) {
        c.args.integer(requested[i]);
        ASSERT_EQ(c.run(thing_can_timeout), ReplyStatus::Ok);
        EXPECT_EQ(c.values[0].integer, expected[i]) << i;
    }
    const uint32_t large = 4000000000U;   // a uint32_t userdata, as coerce_to_uint32_t() takes it
    c.args.blob(UINT32, &large, sizeof(large));
    ASSERT_EQ(c.run(thing_can_timeout), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].integer, 900000);
}

// the longest payload check_message() returns crosses with the node id, timestamp and canfd
TEST(ScriptingM4Remote, DroneCANPayloadFitsOneReply)
{
    Call c;
    c.call.uint64_type = 4;
    c.reply_start = sizeof(ReplyHeader);
    c.args.integer(DRONECAN_PAYLOAD_MAX);
    ASSERT_EQ(c.run(thing_dronecan_message), ReplyStatus::Ok);
    ASSERT_EQ(c.count, 4);
    EXPECT_EQ(c.values[0].length, DRONECAN_PAYLOAD_MAX);
    EXPECT_EQ(c.values[1].integer, 125);
    // one byte more no longer fits the reply: dronecan_peek_payload() drops such a payload
    c.args.integer(DRONECAN_PAYLOAD_MAX + 1);
    EXPECT_EQ(c.run(thing_dronecan_message), ReplyStatus::Error);
    EXPECT_EQ(c.text(0), "M4: results too large");
}

static TestPayload test_payload(uint16_t length)
{
    TestPayload payload {};
    payload.data = static_cast<uint8_t *>(malloc(length));
    memset(payload.data, 'd', length);
    payload.length = length;
    payload.node_id = 42;
    payload.timestamp = 5000000000ULL;
    return payload;
}

// without the drop the oversized payload stays at the head of the queue: every later
// check_message() fails with "M4: results too large" and the subscriber never delivers again
TEST(ScriptingM4Remote, DroneCANOversizedPayloadIsDropped)
{
    ObjectBuffer<TestPayload> payloads(8);
    TestPayload empty {};
    EXPECT_FALSE(dronecan_peek_payload(payloads, empty));
    ASSERT_TRUE(payloads.push(test_payload(DRONECAN_PAYLOAD_MAX + 1)));
    ASSERT_TRUE(payloads.push(test_payload(7)));
    TestPayload payload {};
    EXPECT_FALSE(dronecan_peek_payload(payloads, payload));   // nil for the oversized one
    EXPECT_EQ(payloads.available(), 1U);                      // popped
    ASSERT_TRUE(dronecan_peek_payload(payloads, payload));
    EXPECT_EQ(payload.length, 7);
    EXPECT_EQ(payloads.available(), 1U);                      // peeked: the caller pops once it is copied
    ASSERT_TRUE(payloads.pop(payload));
    free(payload.data);
    ASSERT_TRUE(payloads.push(test_payload(DRONECAN_PAYLOAD_MAX)));
    ASSERT_TRUE(dronecan_peek_payload(payloads, payload));    // the longest that fits
    ASSERT_TRUE(payloads.pop(payload));
    free(payload.data);
}

// a scripting CAN buffer as ScriptingCANBuffer has it: the members CANBufferAccess resets
struct TestCANBuffer {
    struct Semaphore {
        bool available = true;
        int held = 0;
        bool take(uint32_t timeout_ms)
        {
            (void)timeout_ms;
            held += available ? 1 : 0;
            return available;
        }
        void give() { held--; }
    } sem;
    ObjectBuffer<uint32_t> buffer {2};
    struct {
        uint32_t mask;
        uint32_t value;
    } filter[8];
    uint8_t num_filters;
};

// an earlier session's buffer handed out again: without the reset the new script reads the old
// session's frames and its buffer drops every frame the old filters do not pass
TEST(ScriptingM4Remote, ReusedCANBufferStartsEmpty)
{
    TestCANBuffer can_buffer {};
    ASSERT_TRUE(can_buffer.buffer.push(1));
    ASSERT_TRUE(can_buffer.buffer.push(2));
    can_buffer.num_filters = 3;
    EXPECT_TRUE(CANBufferAccess::reuse(can_buffer, 5));
    EXPECT_EQ(can_buffer.buffer.available(), 0U);
    EXPECT_EQ(can_buffer.buffer.get_size(), 5U);             // the depth the new script asked for
    EXPECT_EQ(can_buffer.num_filters, 0);
    EXPECT_EQ(can_buffer.sem.held, 0);
    // a buffer the CAN thread holds past the bounded wait is not handed out
    ASSERT_TRUE(can_buffer.buffer.push(3));
    can_buffer.num_filters = 1;
    can_buffer.sem.available = false;
    EXPECT_FALSE(CANBufferAccess::reuse(can_buffer, 5));
    EXPECT_EQ(can_buffer.num_filters, 1);
    EXPECT_EQ(can_buffer.buffer.available(), 1U);
}

// check_message()'s port through dronecan_push_payload(): without the drop an oversized payload
// stays queued and every later call fails "M4: results too large"; the next payload then crosses
TEST(ScriptingM4Remote, CheckMessageDropsAnOversizedPayload)
{
    ObjectBuffer<TestPayload> payloads(8);
    test_payloads = &payloads;
    ASSERT_TRUE(payloads.push(test_payload(DRONECAN_PAYLOAD_MAX + 1)));
    ASSERT_TRUE(payloads.push(test_payload(7)));
    Call c;
    c.call.uint64_type = 4;
    c.reply_start = sizeof(ReplyHeader);
    c.args.nil();
    ASSERT_EQ(c.run(thing_check_message), ReplyStatus::Ok);
    EXPECT_EQ(c.count, 0);                                    // nil
    c.args.nil();
    ASSERT_EQ(c.run(thing_check_message), ReplyStatus::Ok);
    ASSERT_EQ(c.count, 3);
    EXPECT_EQ(c.text(0), "ddddddd");
    EXPECT_EQ(c.values[1].integer, 42);
    EXPECT_EQ(payloads.available(), 0U);                      // popped once copied
    c.args.nil();
    ASSERT_EQ(c.run(thing_check_message), ReplyStatus::Ok);
    EXPECT_EQ(c.count, 0);
    test_payloads = nullptr;
}

// write_frame()'s port through CAN_write_frame(): without the clamp the buffer gets 4000 s
TEST(ScriptingM4Remote, WriteFrameGetsTheClampedTimeout)
{
    Call c;
    const uint32_t large = 4000000000U;
    c.args.blob(UINT32, &large, sizeof(large));
    ASSERT_EQ(c.run(thing_write_frame), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].tag, ValueTag::True);
    EXPECT_EQ(test_frame_buffer.timeout_us, CAN_WRITE_TIMEOUT_MAX_US);
}

// CAN:get_device*()'s buffer through take_CAN_buffer(): without the reset the next session's
// script gets the old session's frames and filters; without the cap a 9th buffer is made
TEST(ScriptingM4Remote, TakeCANBufferResetsEarlierSessionsBuffers)
{
    static TestCANBuffer buffers[CAN_BUFFERS_PER_SESSION + 1];
    int made = 0;
    auto make = [&made]() { return &buffers[made++]; };
    SessionRecycler<TestCANBuffer, CAN_BUFFERS_PER_SESSION> recycler;
    TestCANBuffer *first = take_CAN_buffer(recycler, make, 3);
    ASSERT_EQ(first, &buffers[0]);
    for (uint8_t i = 1; i < CAN_BUFFERS_PER_SESSION; i++) {
        ASSERT_NE(take_CAN_buffer(recycler, make, 3), nullptr);
    }
    EXPECT_EQ(take_CAN_buffer(recycler, make, 3), nullptr);
    EXPECT_EQ(made, CAN_BUFFERS_PER_SESSION);
    ASSERT_TRUE(first->buffer.push(1));
    first->num_filters = 2;
    recycler.new_session();
    EXPECT_EQ(take_CAN_buffer(recycler, make, 4), first);
    EXPECT_EQ(first->buffer.available(), 0U);
    EXPECT_EQ(first->num_filters, 0);
    EXPECT_EQ(first->buffer.get_size(), 4U);
    EXPECT_EQ(made, CAN_BUFFERS_PER_SESSION);
}

// broadcast()/request() charge their frames: without the weighting a 1000-byte broadcast costs
// one unit and 800 of them pass in a second, about 115,000 frames
TEST(ScriptingM4Remote, DroneCANTransfersChargeTheirFrames)
{
    EXPECT_EQ(dronecan_transfer_frames(0), 1);
    EXPECT_EQ(dronecan_transfer_frames(7), 1);
    EXPECT_EQ(dronecan_transfer_frames(8), 2);
    EXPECT_EQ(dronecan_transfer_frames(12), 2);
    EXPECT_EQ(dronecan_transfer_frames(13), 3);
    EXPECT_EQ(dronecan_transfer_frames(1000), 144);
    Quota quota;
    quota.reset(0);
    Call c;
    c.call.quota = &quota;
    c.call.now_ms = 0;
    static char payload[1000];
    memset(payload, 'x', sizeof(payload));
    for (int i = 0; i < 5; i++) {
        c.args.string(payload, sizeof(payload));
        ASSERT_EQ(c.run(thing_can_charge), ReplyStatus::Ok);
        EXPECT_EQ(c.values[0].tag, ValueTag::True) << i;      // 720 frames
    }
    c.args.string(payload, sizeof(payload));
    ASSERT_EQ(c.run(thing_can_charge), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].tag, ValueTag::False);              // 80 left
    c.args.string(payload, 7);
    ASSERT_EQ(c.run(thing_can_charge), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].tag, ValueTag::True);
    EXPECT_EQ(quota.take_hits(), 1U);
}

struct TestTransfer {
    uint64_t data_type_signature;
    uint16_t data_type_id;
    uint8_t *inout_transfer_id;
    uint8_t priority;
    const void *payload;
    uint16_t payload_len;
    uint32_t timeout_ms;
};
struct TestHandle {
    uint64_t signature;
    uint16_t data_type;
    uint8_t transfer_id;
};

// without the priority an M4 transfer goes at CANARD_TRANSFER_PRIORITY_HIGHEST (0), above ESC commands
TEST(ScriptingM4Remote, DroneCANTransfersAreMediumPriority)
{
    TestTransfer transfer {};
    TestHandle handle { 0x0F0868D0C1A7C6F1ULL, 341, 3 };
    const char data[] = "abc";
    dronecan_fill_transfer(transfer, handle, data, 3);
    EXPECT_EQ(transfer.priority, 16);    // CANARD_TRANSFER_PRIORITY_MEDIUM
    EXPECT_EQ(transfer.data_type_signature, handle.signature);
    EXPECT_EQ(transfer.data_type_id, 341);
    EXPECT_EQ(transfer.inout_transfer_id, &handle.transfer_id);
    EXPECT_EQ(transfer.payload, data);
    EXPECT_EQ(transfer.payload_len, 3);
    EXPECT_EQ(transfer.timeout_ms, 10U);
}

struct TestSubscriber {
    static int destroyed;
    ~TestSubscriber() { destroyed++; }
};
int TestSubscriber::destroyed = 0;

// without the cap a script's 64 handles each hold a subscriber and up to 8 payloads of M7 heap
TEST(ScriptingM4Remote, DroneCANSubscribersAreCapped)
{
    DroneCANSubscribers subscribers;
    TestSubscriber *slots[DRONECAN_SUBSCRIBERS_PER_SESSION + 1] {};
    int made = 0;
    auto make = [&made]() { made++; return new TestSubscriber; };
    for (uint8_t i = 0; i < DRONECAN_SUBSCRIBERS_PER_SESSION; i++) {
        EXPECT_TRUE(subscribers.replace(slots[i], make)) << int(i);
    }
    EXPECT_FALSE(subscribers.replace(slots[DRONECAN_SUBSCRIBERS_PER_SESSION], make));
    EXPECT_EQ(slots[DRONECAN_SUBSCRIBERS_PER_SESSION], nullptr);
    EXPECT_EQ(made, DRONECAN_SUBSCRIBERS_PER_SESSION);
    // a handle that subscribes again replaces its own subscriber, within the cap
    TestSubscriber::destroyed = 0;
    EXPECT_TRUE(subscribers.replace(slots[0], make));
    EXPECT_EQ(TestSubscriber::destroyed, 1);
    // a released handle frees its place
    subscribers.drop(slots[1]);
    EXPECT_EQ(slots[1], nullptr);
    EXPECT_EQ(subscribers.live(), DRONECAN_SUBSCRIBERS_PER_SESSION - 1);
    EXPECT_TRUE(subscribers.replace(slots[DRONECAN_SUBSCRIBERS_PER_SESSION], make));
    for (TestSubscriber *&slot : slots) {
        subscribers.drop(slot);
    }
    EXPECT_EQ(subscribers.live(), 0);
}

// request()'s node: without the check 300.0 becomes another node on x86 and NaN is undefined
TEST(ScriptingM4Remote, DroneCANTargetNode)
{
    Call c;
    c.args.integer(127);
    ASSERT_EQ(c.run(thing_target_node), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].integer, 127);
    c.args.integer(128);
    EXPECT_EQ(c.run(thing_target_node), ReplyStatus::ArgError);
    c.args.integer(-1);
    EXPECT_EQ(c.run(thing_target_node), ReplyStatus::ArgError);
    c.args.number(5.5f);
    EXPECT_EQ(c.run(thing_target_node), ReplyStatus::ArgError);
}

// sock:send(data, len) through socket_send(): without the bound SocketAPM reads 100 bytes from
// the 6-byte copy of "hello", past it into M7 memory
TEST(ScriptingM4Remote, SocketSendIsBoundedByTheString)
{
    Call c;
    test_socket = TestSocket();
    test_socket.result = 5;
    c.args.nil();
    c.args.string("hello", 5);
    c.args.integer(100);
    ASSERT_EQ(c.run(thing_socket_send), ReplyStatus::Ok);
    EXPECT_EQ(test_socket.size, 5U);
    ASSERT_EQ(c.count, 2);
    EXPECT_EQ(c.values[0].integer, 5);
    EXPECT_EQ(c.values[1].tag, ValueTag::False);
    // a shorter length is kept
    c.args.nil();
    c.args.string("hello", 5);
    c.args.integer(3);
    ASSERT_EQ(c.run(thing_socket_send), ReplyStatus::Ok);
    EXPECT_EQ(test_socket.size, 3U);
}

// sock:sendto(data, len, addr, port) through socket_sendto(): bounded as send is
TEST(ScriptingM4Remote, SocketSendToIsBoundedByTheString)
{
    Call c;
    test_socket = TestSocket();
    test_socket.result = 5;
    const uint32_t address = 0x7f000001;
    c.args.nil();
    c.args.string("hello", 5);
    c.args.integer(100);
    c.args.blob(UINT32, &address, sizeof(address));
    c.args.integer(15903);
    ASSERT_EQ(c.run(thing_socket_sendto), ReplyStatus::Ok);
    EXPECT_EQ(test_socket.size, 5U);
    EXPECT_EQ(test_socket.address, address);
    EXPECT_EQ(test_socket.port, 15903);
    ASSERT_EQ(c.count, 2);
    EXPECT_EQ(c.values[0].integer, 5);
    EXPECT_EQ(c.values[1].tag, ValueTag::False);
}

// the second result tells the M4's blocking emulation to try again: only for a call that would
// have blocked. Without the errno reset a failed send after an earlier EAGAIN is retried until
// the M4 gives up, where in-process it fails at once
TEST(ScriptingM4Remote, SocketSendReportsWouldBlock)
{
    Call c;
    const int errors[] = { EAGAIN, EWOULDBLOCK, EPIPE };
    const ValueTag expected[] = { ValueTag::True, ValueTag::True, ValueTag::False };
    for (size_t i = 0; i < 3; i++) {
        test_socket = TestSocket();
        test_socket.result = -1;
        test_socket.error = errors[i];
        c.args.nil();
        c.args.string("hello", 5);
        c.args.integer(5);
        ASSERT_EQ(c.run(thing_socket_send), ReplyStatus::Ok);
        EXPECT_EQ(c.values[0].integer, -1) << i;
        EXPECT_EQ(c.values[1].tag, expected[i]) << i;
    }
    // a closed socket fails without setting errno: an earlier EAGAIN is not this call's
    test_socket = TestSocket();
    test_socket.result = -1;
    errno = EAGAIN;
    c.args.nil();
    c.args.string("hello", 5);
    c.args.integer(5);
    ASSERT_EQ(c.run(thing_socket_send), ReplyStatus::Ok);
    EXPECT_EQ(c.values[1].tag, ValueTag::False);
}

// pollin/pollout through socket_poll(): without the clamp the host thread waits in select() for
// the script's 4000 s
TEST(ScriptingM4Remote, SocketPollWaitsAtMost900ms)
{
    Call c;
    const uint32_t large = 4000000000U;
    test_socket = TestSocket();
    c.args.nil();
    c.args.blob(UINT32, &large, sizeof(large));
    ASSERT_EQ(c.run(thing_socket_pollin), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].tag, ValueTag::True);
    EXPECT_EQ(test_socket.timeout_ms, SOCKET_POLL_TIMEOUT_MAX_MS);
    c.args.nil();
    c.args.integer(901);
    ASSERT_EQ(c.run(thing_socket_pollout), ReplyStatus::Ok);
    EXPECT_EQ(test_socket.timeout_ms, SOCKET_POLL_TIMEOUT_MAX_MS + 1);   // TestSocket's pollout mark
    c.args.nil();
    c.args.integer(250);
    ASSERT_EQ(c.run(thing_socket_pollin), ReplyStatus::Ok);
    EXPECT_EQ(test_socket.timeout_ms, 250U);
}

// set_blocking(true) through socket_set_blocking(), and new sockets through
// socket_nonblocking(): the M7's socket stays non-blocking, so no call waits on the network.
// Without them set_blocking(true) makes the M7's connect and send wait for the peer
TEST(ScriptingM4Remote, SocketsStayNonBlocking)
{
    Call c;
    test_socket = TestSocket();
    c.args.nil();
    c.args.boolean(true);
    ASSERT_EQ(c.run(thing_socket_set_blocking), ReplyStatus::Ok);
    EXPECT_EQ(test_socket.blocking, 0);
    EXPECT_EQ(c.values[0].tag, ValueTag::True);
    // a closed socket's set_blocking() fails, as in-process
    test_socket.open = false;
    c.args.nil();
    c.args.boolean(false);
    ASSERT_EQ(c.run(thing_socket_set_blocking), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].tag, ValueTag::False);

    TestSocket made;
    EXPECT_EQ(socket_nonblocking(&made), &made);
    EXPECT_EQ(made.blocking, 0);
    EXPECT_EQ(socket_nonblocking(static_cast<TestSocket *>(nullptr)), nullptr);
}

// sock:sendfile() through socket_sendfile(): each hand-over leaves a descriptor and a socket
// with AP_Networking until the file is sent. Without the cap the third is handed over
TEST(ScriptingM4Remote, SendfileHandOversAreCapped)
{
    SocketSendfiles sendfiles;
    int handed = 0;
    bool accept = true;
    auto hand_over = [&](int fd) { handed++; EXPECT_EQ(fd, 5); return accept; };
    EXPECT_FALSE(socket_sendfile(sendfiles, -1, hand_over));   // no open file: never handed over
    EXPECT_EQ(handed, 0);
    accept = false;
    EXPECT_FALSE(socket_sendfile(sendfiles, 5, hand_over));    // refused: spends nothing
    accept = true;
    for (uint8_t i = 0; i < SOCKET_SENDFILES_PER_SESSION; i++) {
        EXPECT_TRUE(socket_sendfile(sendfiles, 5, hand_over)) << int(i);
    }
    handed = 0;
    EXPECT_FALSE(socket_sendfile(sendfiles, 5, hand_over));
    EXPECT_EQ(handed, 0);
    sendfiles.new_session();
    EXPECT_TRUE(socket_sendfile(sendfiles, 5, hand_over));
}

AP_GTEST_MAIN()
