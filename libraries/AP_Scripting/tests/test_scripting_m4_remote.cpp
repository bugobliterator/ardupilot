#include <AP_gtest.h>
#include <new>
#include <string>
#include <string.h>
#include <type_traits>
#include <stdlib.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Scripting/AP_ScriptingM4_Remote.h>
#include <AP_Scripting/AP_Scripting_config.h>
#if AP_SCRIPTING_M4_ENABLED
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#endif

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
    for (int i = 0; i <= ScriptingM4::MAX_VALUES; i++) {
        lua_pushinteger(L, i);
    }
    return ScriptingM4::MAX_VALUES + 1;
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
// broadcast()'s path when canard refuses the transfer after the charge
static int thing_can_refused(lua_State *L) {
    size_t length;
    luaL_checklstring(L, 1, &length);
    if (can_transfer_allowed(L, length)) {
        can_transfer_refund(L, length);
    }
    lua_pushboolean(L, false);
    return 1;
}
static int thing_target_node(lua_State *L) {
    lua_pushinteger(L, dronecan_target_node(L, 1));
    return 1;
}

static int thing_crsf_required(lua_State *L) {
    int telem = 5;
    int *given = crsf_required(lua_toboolean(L, 1) ? &telem : nullptr, L);
    lua_pushinteger(L, *given);
    return 1;
}

// the argument count and the last argument, as a port of a wide call sees them
static int thing_count_args(lua_State *L) {
    const int top = lua_gettop(L);
    lua_pushinteger(L, top);
    lua_pushinteger(L, luaL_checkinteger(L, top));
    return 2;
}

// a port's own quota charge (send_chan, logger:write): true while the class allows the call
static int thing_mavtx(lua_State *L) {
    lua_pushboolean(L, remote_quota_allow(L, ScriptingM4::QuotaClass::MavTx));
    return 1;
}

// mavlink_command_int_t's fields that gcs:run_command_int fills, in the generated header's types
struct TestCommandInt {
    float param1 = 0, param2 = 0, param3 = 0, param4 = 0;
    int32_t x = 0, y = 0;
    float z = 0;
    uint16_t command = 0;
    uint8_t frame = 0;
};
static TestCommandInt test_command;
static int thing_command_int(lua_State *L) {
    test_command = TestCommandInt();
    lua_pushboolean(L, command_int_packet(L, test_command));
    return 1;
}

static int thing_uint64(lua_State *L) {
    *new_uint64_t(L) = coerce_to_uint64_t(L, 1);
    return 1;
}

// a semaphore that is taken only when it is free (a GPS driver's update holds the real one)
struct TestGpsSemaphore {
    bool free = true;
    uint32_t waited_ms = 0;
    uint8_t gives = 0;
    bool take(uint32_t wait_ms) {
        waited_ms = wait_ms;
        return free;
    }
    void give() { gives++; }
};

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

// a port gets all 21 values of a call: with MAX_VALUES 16 decode_values() refuses the frame and
// the server answers Malformed
TEST(ScriptingM4Remote, TwentyOneArgumentsReachAPort)
{
    Call c;
    for (int32_t i = 1; i <= 21; i++) {
        c.args.integer(i);
    }
    ASSERT_EQ(c.run(thing_count_args), ReplyStatus::Ok);
    ASSERT_EQ(c.count, 2);
    EXPECT_EQ(c.values[0].integer, 21);
    EXPECT_EQ(c.values[1].integer, 21);
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

// an I2C device's semaphore and the clock as i2c_bus_call() uses them: each run() takes busy_us
struct I2CBus {
    bool free = true;
    int held = 0;
    int runs = 0;
    uint64_t now_us = 5000000;
    uint32_t busy_us = 0;
    bool take(uint32_t timeout_ms) { EXPECT_EQ(timeout_ms, I2C_SEMAPHORE_WAIT_MS); if (free) { held++; } return free; }
    void give() { held--; }
    bool call(I2CBusBudget &budget)
    {
        return i2c_bus_call(budget, *this, [this]() { return now_us; }, [this]() {
            EXPECT_EQ(held, 1);
            runs++;
            now_us += busy_us;
            return true;
        });
    }
};

// calls are charged the time they held the bus; past 100 ms in a second they are refused until the
// window rolls. Without the budget a script's back-to-back transfers keep a shared bus busy
TEST(ScriptingM4Remote, I2CBusTimeIsBudgeted)
{
    I2CBusBudget budget;
    I2CBus bus;
    bus.busy_us = 20000;
    for (int i = 0; i < 5; i++) {
        EXPECT_TRUE(bus.call(budget)) << i;   // 100 ms used: not more than the budget
    }
    EXPECT_EQ(budget.used_us(), 100000U);
    bus.busy_us = 1;
    EXPECT_TRUE(bus.call(budget));
    EXPECT_FALSE(bus.call(budget));          // more than 100 ms: refused without the bus
    EXPECT_EQ(bus.runs, 6);
    EXPECT_EQ(bus.held, 0);
    bus.now_us = 5999999;
    EXPECT_FALSE(bus.call(budget));          // the window that started at 5 s still runs
    bus.now_us = 6000000;
    EXPECT_TRUE(bus.call(budget));           // the next one starts with the budget
    EXPECT_EQ(bus.runs, 7);
    // a busy semaphore: refused, nothing run or charged
    const uint64_t used = budget.used_us();
    bus.free = false;
    EXPECT_FALSE(bus.call(budget));
    EXPECT_EQ(budget.used_us(), used);
    EXPECT_EQ(bus.runs, 7);
}

// one long call is paid for in the windows after it: a script of 798 ms transfers gets about one
// in eight seconds, not one a second. A new session starts unused
TEST(ScriptingM4Remote, I2CBusTimePastTheBudgetCarriesOver)
{
    I2CBusBudget budget;
    I2CBus bus;
    bus.busy_us = 798000;
    EXPECT_TRUE(bus.call(budget));
    bus.now_us = 6000000;
    EXPECT_FALSE(bus.call(budget));          // 698 ms still owed
    bus.now_us = 11000000;
    EXPECT_FALSE(bus.call(budget));          // 198 ms
    bus.now_us = 12000000;
    EXPECT_TRUE(bus.call(budget));           // 98 ms
    EXPECT_EQ(bus.runs, 2);
    budget.new_session();
    EXPECT_EQ(budget.used_us(), 0U);
    EXPECT_TRUE(bus.call(budget));
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

// a transfer canard refuses after its charge: the frames go back, and no hit is counted
TEST(ScriptingM4Remote, DroneCANRefusedTransferRefundsItsFrames)
{
    Quota quota;
    quota.reset(0);
    Call c;
    c.call.quota = &quota;
    c.call.now_ms = 0;
    static char payload[960];
    memset(payload, 'x', sizeof(payload));
    c.args.string(payload, sizeof(payload));
    ASSERT_EQ(c.run(thing_can_refused), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].tag, ValueTag::False);
    EXPECT_EQ(quota.take_hits(), 0U);
    // the bucket is full again: the 800 frames of the burst are all there
    static char big[1000];
    memset(big, 'x', sizeof(big));
    for (int i = 0; i < 5; i++) {
        c.args.string(big, sizeof(big));
        ASSERT_EQ(c.run(thing_can_charge), ReplyStatus::Ok);
        EXPECT_EQ(c.values[0].tag, ValueTag::True) << i;      // 720 frames
    }
    c.args.string(payload, 80 * 7 - 2);                        // 80 frames
    ASSERT_EQ(c.run(thing_can_charge), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].tag, ValueTag::True);
    c.args.string(payload, 7);
    ASSERT_EQ(c.run(thing_can_charge), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].tag, ValueTag::False);               // empty
    EXPECT_EQ(quota.take_hits(), 1U);
    // a refund never fills the bucket past its burst
    quota.reset(0);
    quota.refund(QuotaClass::Can, 800);
    for (int i = 0; i < 5; i++) {
        c.args.string(big, sizeof(big));
        ASSERT_EQ(c.run(thing_can_charge), ReplyStatus::Ok);
    }
    c.args.string(payload, 80 * 7 - 2 + 1);                    // 81 frames: over the burst
    ASSERT_EQ(c.run(thing_can_charge), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].tag, ValueTag::False);
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

// Socket() makes a socket only while networking runs and its stack's core exists. With NET_ENABLE
// 0 (the default) lwIP is never initialised and its first call locks a null mutex: without the
// check an M4 script's Socket(0) faults the M7
TEST(ScriptingM4Remote, SocketsNeedARunningNetworkStack)
{
    EXPECT_TRUE(socket_stack_ready(true, true));
    EXPECT_FALSE(socket_stack_ready(false, true));    // NET_ENABLE 0, or no backend
    EXPECT_FALSE(socket_stack_ready(true, false));    // healthy before tcpip_init() ran
    EXPECT_FALSE(socket_stack_ready(false, false));
}

// AP_Networking's semaphore as socket_sendfile() takes it
struct TestSemaphore {
    bool free = true;
    uint32_t timeout_ms = 0;
    int held = 0;
    bool take(uint32_t timeout) { timeout_ms = timeout; if (free) { held++; } return free; }
    bool give() { held--; return true; }
};

// sock:sendfile() through socket_sendfile(): each hand-over leaves a descriptor and a socket
// with AP_Networking until the file is sent. Without the cap the third is handed over
TEST(ScriptingM4Remote, SendfileHandOversAreCapped)
{
    SocketSendfiles sendfiles;
    TestSemaphore sem;
    int handed = 0;
    bool accept = true;
    auto hand_over = [&](int fd) { handed++; EXPECT_EQ(fd, 5); EXPECT_EQ(sem.held, 1); return accept; };
    EXPECT_FALSE(socket_sendfile(sendfiles, -1, sem, hand_over));   // no open file: never handed over
    EXPECT_EQ(handed, 0);
    accept = false;
    EXPECT_FALSE(socket_sendfile(sendfiles, 5, sem, hand_over));    // refused: spends nothing
    accept = true;
    for (uint8_t i = 0; i < SOCKET_SENDFILES_PER_SESSION; i++) {
        EXPECT_TRUE(socket_sendfile(sendfiles, 5, sem, hand_over)) << int(i);
    }
    handed = 0;
    EXPECT_FALSE(socket_sendfile(sendfiles, 5, sem, hand_over));
    EXPECT_EQ(handed, 0);
    sendfiles.new_session();
    EXPECT_TRUE(socket_sendfile(sendfiles, 5, sem, hand_over));
    EXPECT_EQ(sem.held, 0);   // given back after every hand-over
}

// the networking semaphore is taken for at most 30 ms before AP_Networking::sendfile(), whose own
// take then re-enters it. Without the bounded take the host thread waits as long as the sendfile
// thread's pass of file reads holds it
TEST(ScriptingM4Remote, SendfileWaitsForTheSemaphoreAtMost30ms)
{
    SocketSendfiles sendfiles;
    TestSemaphore sem;
    sem.free = false;
    int handed = 0;
    auto hand_over = [&](int) { handed++; return true; };
    EXPECT_FALSE(socket_sendfile(sendfiles, 5, sem, hand_over));
    EXPECT_EQ(handed, 0);
    EXPECT_EQ(sem.timeout_ms, SOCKET_SENDFILE_WAIT_MS);
    EXPECT_EQ(SOCKET_SENDFILE_WAIT_MS, 30U);
    sem.free = true;                                              // a busy semaphore spent no hand-over
    for (uint8_t i = 0; i < SOCKET_SENDFILES_PER_SESSION; i++) {
        EXPECT_TRUE(socket_sendfile(sendfiles, 5, sem, hand_over)) << int(i);
    }
}

// a session's socket as socket_end() closes it
struct EndingSocket {
    int fd;
    int *aborted_fd;
    bool *deleted;
    ~EndingSocket() { *deleted = true; }
    int get_read_fd() const { return fd; }
};

// at the session end a stream socket is set to close with a reset before it is deleted. Without
// the reset the delete closes gracefully, which can hold the host thread waiting for a FIN segment
TEST(ScriptingM4Remote, SocketsEndWithAReset)
{
    int reset_fd = -2;
    bool deleted = false;
    EndingSocket *slot = new EndingSocket{7, nullptr, &deleted};
    bool deleted_at_reset = true;
    socket_end(slot, [](int) { return SocketKind::Stream; },
               [&](int fd) { reset_fd = fd; deleted_at_reset = deleted; });
    EXPECT_EQ(reset_fd, 7);
    EXPECT_FALSE(deleted_at_reset);   // the reset is set before the close
    EXPECT_TRUE(deleted);
    EXPECT_EQ(slot, nullptr);
    int calls = 0;
    socket_end(slot, [&](int) { calls++; return SocketKind::Stream; }, [&](int) { calls++; });   // an empty slot
    EXPECT_EQ(calls, 0);
}

// only a stream socket that is not listening is reset: lwIP's close of a listening socket with a
// linger of 0 reads unsent/unacked past its smaller listen PCB and aborts it; a datagram socket, a
// socket of unknown kind and one sendfile took (descriptor -1) are closed as they are. Without the
// skip a listening socket left open at the session end gets the reset
TEST(ScriptingM4Remote, SocketEndResetsOnlyStreamSockets)
{
    EXPECT_TRUE(socket_end_resets(SocketKind::Stream));
    EXPECT_FALSE(socket_end_resets(SocketKind::Listening));
    EXPECT_FALSE(socket_end_resets(SocketKind::Other));
    const SocketKind kinds[] = { SocketKind::Listening, SocketKind::Other };
    for (const SocketKind kind : kinds) {
        bool deleted = false;
        int resets = 0;
        EndingSocket *slot = new EndingSocket{9, nullptr, &deleted};
        socket_end(slot, [kind](int) { return kind; }, [&](int) { resets++; });
        EXPECT_EQ(resets, 0) << int(kind);
        EXPECT_TRUE(deleted) << int(kind);
    }
    bool deleted = false;
    int asked = 0;
    EndingSocket *handed_over = new EndingSocket{-1, nullptr, &deleted};
    socket_end(handed_over, [&](int) { asked++; return SocketKind::Stream; }, [&](int) { asked++; });
    EXPECT_EQ(asked, 0);
    EXPECT_TRUE(deleted);
}

// a menu name reaches AP_CRSF_Telem as at most 15 characters and a NUL: it stores 16 bytes of
// the string it is given in a 17-byte block and later reads it as a C string. Without the cut a
// name of 16 or more characters is stored unterminated
TEST(ScriptingM4Remote, CRSFMenuNamesAreCutTo15Characters)
{
    char cut[CRSF_NAME_LENGTH + 1];
    memset(cut, 0x55, sizeof(cut));
    crsf_cut_name(cut, "Roll Tuning Menu And Some More");
    EXPECT_STREQ(cut, "Roll Tuning Men");
    EXPECT_EQ(cut[CRSF_NAME_LENGTH], 0);
    memset(cut, 0x55, sizeof(cut));
    crsf_cut_name(cut, "123456789012345");   // 15: kept whole
    EXPECT_STREQ(cut, "123456789012345");
    memset(cut, 0x55, sizeof(cut));
    crsf_cut_name(cut, "1234567890123456");  // 16: the 16th goes
    EXPECT_STREQ(cut, "123456789012345");
    memset(cut, 0x55, sizeof(cut));
    crsf_cut_name(cut, "Tune");
    EXPECT_STREQ(cut, "Tune");
    EXPECT_EQ(CRSF_NAME_LENGTH, 15);
}


// the menu the port makes receives the cut name; a refusal makes nothing
TEST(ScriptingM4Remote, CRSFMenusAreMadeWithCutNames)
{
    CRSFMenuBudget budget;
    std::string received;
    auto make = [&received](const char *name) { received = name; return true; };
    EXPECT_TRUE(crsf_add_menu(budget, true, "A name of more than fifteen characters", make));
    EXPECT_EQ(received, "A name of more ");
    // AP_CRSF_Telem has no menu to give (heap exhausted): it is no menu of the session
    CRSFMenuBudget empty;
    EXPECT_FALSE(crsf_add_menu(empty, true, "x", [](const char *) { return false; }));
    EXPECT_FALSE(empty.any());
}

// AP_CRSF_Telem numbers menus up to MAX_SCRIPTED_MENUS and does not check it. Without the cap
// the 11th menu is made
TEST(ScriptingM4Remote, CRSFMenusAreCappedPerSession)
{
    CRSFMenuBudget budget;
    int made = 0;
    auto make = [&made](const char *) { made++; return true; };
    for (uint8_t i = 0; i < CRSF_MENUS_PER_SESSION; i++) {
        EXPECT_TRUE(crsf_add_menu(budget, true, "m", make)) << int(i);
    }
    EXPECT_EQ(made, CRSF_MENUS_PER_SESSION);
    EXPECT_TRUE(budget.any());
    EXPECT_FALSE(crsf_add_menu(budget, true, "m", make));
    EXPECT_EQ(made, CRSF_MENUS_PER_SESSION);   // nothing was made for the refused one
    // a new session starts from none
    budget.new_session();
    EXPECT_FALSE(budget.any());
    EXPECT_TRUE(crsf_add_menu(budget, true, "m", make));
}

// a sub-menu is a parameter of its parent: a parent holding CRSF_PARAMETERS_PER_MENU takes none,
// and the refusal spends none of the session's menus. Without the parent check the 21st entry
// of the parent is written past its 20 ids
TEST(ScriptingM4Remote, CRSFSubMenusNeedRoomInTheirParent)
{
    CRSFMenuBudget budget;
    int made = 0;
    auto make = [&made](const char *) { made++; return true; };
    EXPECT_TRUE(crsf_add_menu(budget, crsf_menu_has_room(CRSF_PARAMETERS_PER_MENU - 1), "s", make));
    EXPECT_FALSE(crsf_add_menu(budget, crsf_menu_has_room(CRSF_PARAMETERS_PER_MENU), "s", make));
    EXPECT_EQ(made, 1);
    for (uint8_t i = 1; i < CRSF_MENUS_PER_SESSION; i++) {
        EXPECT_TRUE(crsf_add_menu(budget, true, "s", make)) << int(i);   // the refusal spent none
    }
}

// 256 add_parameter() calls wrap the uint8_t num_params of AP_CRSF_Telem's menu and write
// params[-1]. Without the cap the 21st is added
TEST(ScriptingM4Remote, CRSFParametersAreCappedPerMenu)
{
    uint8_t num_params = 0;
    int made = 0;
    auto make = [&]() { made++; num_params++; return true; };
    for (uint8_t i = 0; i < CRSF_PARAMETERS_PER_MENU; i++) {
        EXPECT_TRUE(crsf_add_parameter(num_params, make)) << int(i);
    }
    for (int i = 0; i < 300; i++) {
        EXPECT_FALSE(crsf_add_parameter(num_params, make)) << i;
    }
    EXPECT_EQ(made, CRSF_PARAMETERS_PER_MENU);   // no object made for a refusal
    EXPECT_EQ(num_params, CRSF_PARAMETERS_PER_MENU);
    EXPECT_TRUE(crsf_menu_has_room(CRSF_PARAMETERS_PER_MENU - 1));
    EXPECT_FALSE(crsf_menu_has_room(CRSF_PARAMETERS_PER_MENU));
    EXPECT_FALSE(crsf_menu_has_room(255));
    // a parameter AP_CRSF_Telem could not allocate is not made
    uint8_t empty = 0;
    EXPECT_FALSE(crsf_add_parameter(empty, []() { return false; }));
}

// AP::crsf_telem() is null while armed if CRSF telemetry was never made. Without the check the
// call dereferences it: a fault of the M7
TEST(ScriptingM4Remote, CRSFTelemetryMustExist)
{
    Call c;
    c.args.boolean(true);
    ASSERT_EQ(c.run(thing_crsf_required), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].integer, 5);
    c.args.boolean(false);
    ASSERT_EQ(c.run(thing_crsf_required), ReplyStatus::Error);
    EXPECT_EQ(c.text(0), "internal error: crsf is null");
}

// remote_quota_allow() charges the class it names at the call's time: without it a port's quota
// is never taken and the 201st send in one run is allowed
TEST(ScriptingM4Remote, PortsChargeTheirQuota)
{
    Call c;
    Quota quota;
    quota.reset(0);
    c.call.quota = &quota;
    c.call.now_ms = 0;
    for (int i = 0; i < 200; i++) {
        c.args.nil();
        ASSERT_EQ(c.run(thing_mavtx), ReplyStatus::Ok);
        ASSERT_EQ(c.values[0].tag, ValueTag::True) << i;
    }
    c.args.nil();
    ASSERT_EQ(c.run(thing_mavtx), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].tag, ValueTag::False);
    EXPECT_EQ(quota.take_hits(), 1U);
}

// a session blocks at most 32 commands; a refused one spends none of them. Without the cap a
// script can put 65536 entries, each an M7 allocation, on the list every received command walks
TEST(ScriptingM4Remote, CommandBlocksAreCappedPerSession)
{
    CommandBlocks blocks;
    for (uint16_t id = 1; id <= MAVLINK_BLOCKED_COMMANDS_PER_SESSION; id++) {
        ASSERT_TRUE(blocks.allow(id)) << id;
        blocks.take();
    }
    EXPECT_FALSE(blocks.allow(1000));
    EXPECT_EQ(blocks.blocked(), MAVLINK_BLOCKED_COMMANDS_PER_SESSION);
    blocks.new_session();
    EXPECT_TRUE(blocks.allow(1000));
}

// MAV_CMD_SCRIPTING stays the GCS's: a script that blocked it would keep the GCS from stopping it
TEST(ScriptingM4Remote, BlockingTheScriptingCommandIsRefused)
{
    CommandBlocks blocks;
    EXPECT_EQ(MAVLINK_SCRIPTING_COMMAND, 42701);
    EXPECT_FALSE(blocks.allow(MAVLINK_SCRIPTING_COMMAND));
    EXPECT_TRUE(blocks.allow(31000));
}

// the mission items a session left unread go with it. A null queue (no mission item ever
// received) holds none. Without the drain the next session reads them
TEST(ScriptingM4Remote, MissionQueueIsDrained)
{
    ObjectBuffer<int> queue(5);
    for (int i = 0; i < 6; i++) {
        (void)queue.push(i);   // the sixth does not fit, as the M7's queue drops it
    }
    EXPECT_EQ(drain_queue(&queue), 5U);
    EXPECT_EQ(queue.available(), 0U);
    EXPECT_TRUE(queue.push(7));
    int item = 0;
    EXPECT_TRUE(queue.pop(item));
    EXPECT_EQ(item, 7);
    EXPECT_EQ(drain_queue(static_cast<ObjectBuffer<int> *>(nullptr)), 0U);
}

// a queue whose producer refills it as fast as it is popped: never empty, up to 1000 pops
struct RefillingQueue {
    uint32_t pops;
    uint32_t available() const { return 5; }
    bool pop() { return ++pops <= 1000; }
};

// the drain pops what the queue held when it started, so a producer that keeps pushing cannot
// hold the host thread. A drain that pops until empty takes all 1000 here (forever on the M7)
TEST(ScriptingM4Remote, MissionQueueDrainIgnoresNewItems)
{
    RefillingQueue queue {};
    EXPECT_EQ(drain_queue(&queue), 5U);
    EXPECT_EQ(queue.pops, 5U);
}

// mavlink_message_t's layout around the fields mavlink_clear_stale() uses (wide sysid)
struct __attribute__((packed)) TestMavlinkMessage {
    uint16_t checksum;
    uint8_t magic;
    uint8_t len;
    uint8_t incompat_flags;
    uint8_t compat_flags;
    uint8_t seq;
    uint32_t sysid;
    uint8_t compid;
    uint8_t msgid[3];
    uint64_t payload64[33];
    uint8_t ck[2];
    uint8_t signature[13];
    uint32_t target_sysid;
};

// receive_chan sends this message's bytes only: without the clear, the payload past len and the
// signature hold what earlier messages on the channel left (a SETUP_SIGNING key, for one)
TEST(ScriptingM4Remote, ReceivedMessageCarriesNoStaleBytes)
{
    TestMavlinkMessage msg;
    memset(&msg, 0xAA, sizeof(msg));
    msg.len = 9;
    msg.incompat_flags = 0;
    msg.target_sysid = 7;
    mavlink_clear_stale(msg);
    const uint8_t *payload = reinterpret_cast<const uint8_t *>(&msg) + offsetof(TestMavlinkMessage, payload64);
    for (size_t i = 0; i < sizeof(msg.payload64); i++) {
        ASSERT_EQ(payload[i], i < 9 ? 0xAA : 0) << i;
    }
    for (size_t i = 0; i < sizeof(msg.signature); i++) {
        ASSERT_EQ(msg.signature[i], 0) << i;
    }
    EXPECT_EQ(msg.ck[0], 0xAA);            // this message's own fields stay
    EXPECT_EQ(msg.target_sysid, 7U);
    EXPECT_EQ(msg.sysid, 0xAAAAAAAAU);

    // a signed message keeps its own signature; a full payload has no tail
    memset(&msg, 0xBB, sizeof(msg));
    msg.len = 255;
    msg.incompat_flags = MAVLINK_SIGNED_FLAG;
    mavlink_clear_stale(msg);
    EXPECT_EQ(payload[254], 0xBB);
    EXPECT_EQ(payload[255], 0);
    EXPECT_EQ(msg.signature[12], 0xBB);
}

// send_chan leaves a full signed frame of txspace to the vehicle: without the reserve a script
// sending as fast as the link drains takes every byte the vehicle's streams would use
TEST(ScriptingM4Remote, SendKeepsAFrameForTheVehicle)
{
    EXPECT_EQ(MAVLINK_TX_RESERVE, 280);
    EXPECT_TRUE(mavlink_send_has_room(100 + 280, 100));
    EXPECT_FALSE(mavlink_send_has_room(100 + 279, 100));
    EXPECT_FALSE(mavlink_send_has_room(100, 100));        // the message alone fits: in-process sends
    EXPECT_FALSE(mavlink_send_has_room(0, 0));
}

// the table's fields arrive as values 3..10 (m4_wrappers.cpp); nil leaves a field 0, as in-process
// a missing or mistyped field does. frame is cut to its uint8_t, as lua_tointeger() into it is
TEST(ScriptingM4Remote, CommandIntPacketTakesTheTablesValues)
{
    Call c;
    c.args.nil();            // the GCS self
    c.args.integer(179);     // MAV_CMD_DO_SET_HOME
    c.args.number(1.5f);     // p1
    c.args.nil();            // p2
    c.args.integer(3);       // p3: an integer is a number
    c.args.nil();            // p4
    c.args.integer(-353632621);
    c.args.nil();            // y
    c.args.number(584.5f);   // z
    c.args.integer(262);     // frame
    ASSERT_EQ(c.run(thing_command_int), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].tag, ValueTag::True);
    EXPECT_EQ(test_command.command, 179);
    EXPECT_FLOAT_EQ(test_command.param1, 1.5f);
    EXPECT_FLOAT_EQ(test_command.param2, 0.0f);
    EXPECT_FLOAT_EQ(test_command.param3, 3.0f);
    EXPECT_EQ(test_command.x, -353632621);
    EXPECT_EQ(test_command.y, 0);
    EXPECT_FLOAT_EQ(test_command.z, 584.5f);
    EXPECT_EQ(test_command.frame, 6);
}

// no table: the command is still checked, then nothing is run (in-process returns no result).
// Without the two-value case the call is malformed and the script gets an error
TEST(ScriptingM4Remote, CommandIntWithoutATable)
{
    Call c;
    c.args.nil();
    c.args.integer(179);
    ASSERT_EQ(c.run(thing_command_int), ReplyStatus::Ok);
    EXPECT_EQ(c.values[0].tag, ValueTag::False);
    EXPECT_EQ(test_command.command, 179);
    c.args.nil();
    c.args.integer(70000);   // past uint16_t: get_uint16_t()'s in-process text
    EXPECT_EQ(c.run(thing_command_int), ReplyStatus::ArgError);
    EXPECT_EQ(c.text(0), "out of range");
}

// any other count is a hostile M4 (the wrapper sends 2 or 10): an argument error, never a read
// past the values
TEST(ScriptingM4Remote, CommandIntIsMalformedOtherwise)
{
    for (uint8_t count : { 1, 3, 9, 11 }) {
        Call c;
        for (uint8_t i = 0; i < count; i++) {
            c.args.integer(1);
        }
        EXPECT_EQ(c.run(thing_command_int), ReplyStatus::ArgError) << int(count);
        EXPECT_EQ(c.text(0), "M4: malformed run_command_int") << int(count);
    }
}

// commands whose handler holds the host thread past 900 ms, or crashes the vehicle, are refused.
// Without the refusals command_int_runs_on_the_host() is true for each of these packets
TEST(ScriptingM4Remote, CommandIntRefusesLongAndCrashingCommands)
{
    EXPECT_EQ(COMMAND_INT_DENIED, 2);

    TestCommandInt calibration;
    calibration.command = COMMAND_INT_PREFLIGHT_CALIBRATION;
    EXPECT_EQ(COMMAND_INT_PREFLIGHT_CALIBRATION, 241);
    EXPECT_FALSE(command_int_runs_on_the_host(calibration));

    // the bootloader flash, with its magic x only (without it the handler returns FAILED at once)
    TestCommandInt flash;
    flash.command = COMMAND_INT_FLASH_BOOTLOADER;
    EXPECT_TRUE(command_int_runs_on_the_host(flash));
    flash.x = 290876;
    EXPECT_FALSE(command_int_runs_on_the_host(flash));

    // the trap, with its magic p1 (the handler casts it to uint32_t)
    TestCommandInt trap;
    trap.command = COMMAND_INT_DEBUG_TRAP;
    EXPECT_TRUE(command_int_runs_on_the_host(trap));
    trap.param1 = 32451;
    EXPECT_FALSE(command_int_runs_on_the_host(trap));
    trap.param1 = 32451.9f;
    EXPECT_FALSE(command_int_runs_on_the_host(trap));
    trap.param1 = 32450;
    EXPECT_TRUE(command_int_runs_on_the_host(trap));

    // the crash triggers and DFU: p1/p2/p3 42/24/71, any p4 (93 lockup, 96 parameter wipe, 99 DFU)
    for (float p4 : { 0.0f, 93.0f, 96.0f, 97.0f, 99.0f, 100.0f, 101.0f, 102.0f }) {
        TestCommandInt crash;
        crash.command = COMMAND_INT_PREFLIGHT_REBOOT_SHUTDOWN;
        crash.param1 = 42;
        crash.param2 = 24;
        crash.param3 = 71;
        crash.param4 = p4;
        EXPECT_FALSE(command_int_runs_on_the_host(crash)) << p4;
    }
    // a plain reboot, and a triple that is not the triple, run as in-process
    TestCommandInt reboot;
    reboot.command = COMMAND_INT_PREFLIGHT_REBOOT_SHUTDOWN;
    reboot.param1 = 1;
    EXPECT_TRUE(command_int_runs_on_the_host(reboot));
    reboot.param1 = 42;
    reboot.param2 = 24;
    reboot.param3 = 70;
    reboot.param4 = 93;
    EXPECT_TRUE(command_int_runs_on_the_host(reboot));
    EXPECT_EQ(COMMAND_INT_PREFLIGHT_REBOOT_SHUTDOWN, 246);
    EXPECT_EQ(COMMAND_INT_FLASH_BOOTLOADER, 42650);
    EXPECT_EQ(COMMAND_INT_DEBUG_TRAP, 42700);

    TestCommandInt set_home;
    set_home.command = 179;   // MAV_CMD_DO_SET_HOME
    EXPECT_TRUE(command_int_runs_on_the_host(set_home));
    set_home.command = 42701;   // MAV_CMD_SCRIPTING: it only sets flags
    EXPECT_TRUE(command_int_runs_on_the_host(set_home));
}

// PREFLIGHT_STORAGE refuses the parameter erase (p1 2 and 4: uint8_t(p1) in the handler, so also
// 2.9 and, where the conversion wraps, 258), STORAGE_FORMAT its p1 = p2 = 1. Other p1 values are
// the handler's own (0 and 1 read and write the parameters, 3 is unsupported); a format with
// other params is UNSUPPORTED there. Without the cases each of these returns true
TEST(ScriptingM4Remote, CommandIntRefusesDestructiveStorageCommands)
{
    EXPECT_EQ(COMMAND_INT_PREFLIGHT_STORAGE, 245);
    EXPECT_EQ(COMMAND_INT_STORAGE_FORMAT, 526);
    TestCommandInt storage;
    storage.command = COMMAND_INT_PREFLIGHT_STORAGE;
    for (float p1 : { 2.0f, 2.9f, 4.0f, 4.5f, 258.0f }) {
        storage.param1 = p1;
        EXPECT_FALSE(command_int_runs_on_the_host(storage)) << p1;
    }
    for (float p1 : { 0.0f, 1.0f, 3.0f, 5.0f, 1.99f, -1.0f, 1e20f }) {
        storage.param1 = p1;
        EXPECT_TRUE(command_int_runs_on_the_host(storage)) << p1;
    }
    TestCommandInt format;
    format.command = COMMAND_INT_STORAGE_FORMAT;
    format.param1 = 1;
    format.param2 = 1;
    EXPECT_FALSE(command_int_runs_on_the_host(format));
    format.param1 = 1.0f + FLT_EPSILON;   // the handler's is_equal() takes it for 1
    EXPECT_FALSE(command_int_runs_on_the_host(format));
    format.param2 = 0;
    EXPECT_TRUE(command_int_runs_on_the_host(format));
}

// the refusals at the 42/24/71 boundary: values the handler's is_equal() takes for the triple are
// refused too; one float step further is not the triple
TEST(ScriptingM4Remote, CommandIntTripleBoundary)
{
    TestCommandInt crash;
    crash.command = COMMAND_INT_PREFLIGHT_REBOOT_SHUTDOWN;
    crash.param1 = 42.0f + FLT_EPSILON;
    crash.param2 = 24;
    crash.param3 = 71;
    EXPECT_FALSE(command_int_runs_on_the_host(crash));
    crash.param1 = 42.0001f;
    EXPECT_TRUE(command_int_runs_on_the_host(crash));
    crash.param1 = NAN;
    EXPECT_TRUE(command_int_runs_on_the_host(crash));
}

// the helper the port calls: a refused command returns DENIED and never runs; any other runs once
// and returns its own result. Without the check the refused command runs
TEST(ScriptingM4Remote, CommandIntResultRunsOnlyWhatIsAllowed)
{
    int runs = 0;
    TestCommandInt packet;
    packet.command = COMMAND_INT_PREFLIGHT_CALIBRATION;
    EXPECT_EQ(command_int_result(packet, [&]() { runs++; return 0; }), COMMAND_INT_DENIED);
    EXPECT_EQ(runs, 0);
    packet.command = 179;   // MAV_CMD_DO_SET_HOME
    EXPECT_EQ(command_int_result(packet, [&]() { runs++; return 5; }), 5);
    EXPECT_EQ(runs, 1);
}

// gps:inject_data() takes the GPS semaphore for at most GPS_INJECT_WAIT_MS and runs the driver
// call under it. Without the take the action runs on a busy semaphore (the main loop may be
// deleting the driver); without the give the main loop's GPS update would wait for ever
TEST(ScriptingM4Remote, BoundedTakeRunsUnderTheSemaphoreOrRefuses)
{
    TestGpsSemaphore sem;
    int runs = 0;
    EXPECT_TRUE(run_with_bounded_take(sem, GPS_INJECT_WAIT_MS, [&]() { runs++; }));
    EXPECT_EQ(runs, 1);
    EXPECT_EQ(sem.gives, 1);
    EXPECT_EQ(sem.waited_ms, 30U);
    EXPECT_EQ(GPS_INJECT_WAIT_MS, 30U);

    sem.free = false;
    EXPECT_FALSE(run_with_bounded_take(sem, GPS_INJECT_WAIT_MS, [&]() { runs++; }));
    EXPECT_EQ(runs, 1);
    EXPECT_EQ(sem.gives, 1);
}

// every character AP_Logger_Write() writes passes; any other is named before the logger is asked
// for a message type. Without the check 'd' reaches msg_fmt_for_name(), which takes a type and
// M7 memory for the name before the value fails (and SITL's Write_calc_msg_len() panics on 'x')
TEST(ScriptingM4Remote, LoggerFormatIsCheckedFirst)
{
    EXPECT_EQ(logger_format_unsupported("bhcHCiLefnMBIEQNZ"), nullptr);
    const char *mixed = "ffdx";
    EXPECT_EQ(logger_format_unsupported(mixed), mixed + 2);
    for (const char *format : { "a", "q", "g", "x", " " }) {
        EXPECT_NE(logger_format_unsupported(format), nullptr) << format;
    }
    EXPECT_EQ(logger_format_unsupported(""), nullptr);
}

// names, labels, units and multipliers the logger takes. Without these checks SITL's logger stops
// the host thread (validate_structure(): config_error()) on a repeated, empty or "TimeUS" label
// and on a unit or multiplier it has no entry for; a name is any bytes in-process
TEST(ScriptingM4Remote, LoggerNamesLabelsAndIdsAreChecked)
{
    for (const char *name : { "A", "AB_1", "zZ9" }) {
        EXPECT_TRUE(logger_name_valid(name)) << name;
    }
    for (const char *name : { "", "A-B", "A B", "\x01", "\xC3\xA9" }) {
        EXPECT_FALSE(logger_name_valid(name)) << name;
    }
    EXPECT_TRUE(logger_labels_valid("TimeUS,a,b,Ab"));
    EXPECT_TRUE(logger_labels_valid("TimeUS"));
    for (const char *labels : { "TimeUS,a,a", "TimeUS,TimeUS", "TimeUS,a,", "TimeUS,,a", ",TimeUS",
                                "TimeUS,ab,a,ab" }) {
        EXPECT_FALSE(logger_labels_valid(labels)) << labels;
    }
    EXPECT_EQ(logger_ids_unsupported("s-?mA", LOGGER_UNIT_IDS), nullptr);
    const char *units = "smx";
    EXPECT_EQ(logger_ids_unsupported(units, LOGGER_UNIT_IDS), units + 2);
    EXPECT_EQ(logger_ids_unsupported("F0-?!/", LOGGER_MULTIPLIER_IDS), nullptr);
    const char *mults = "F0H";
    EXPECT_EQ(logger_ids_unsupported(mults, LOGGER_MULTIPLIER_IDS), mults + 2);
}

// log_file_content(): a path the M4's file calls may read, while fewer than 40 files wait. Without
// the bound every call is a new M7 allocation the logger frees at about 640 bytes a second
TEST(ScriptingM4Remote, FileContentNeedsAReadablePathAndRoom)
{
    static const char path[] = "/APM/scripts/test.lua";
    EXPECT_TRUE(file_content_allowed(path, strlen(path), 0));
    EXPECT_TRUE(file_content_allowed(path, strlen(path), FILE_CONTENT_QUEUE_MAX - 1));
    EXPECT_FALSE(file_content_allowed(path, strlen(path), FILE_CONTENT_QUEUE_MAX));
    EXPECT_EQ(FILE_CONTENT_QUEUE_MAX, 40);
    EXPECT_TRUE(file_content_allowed("@SYS/tasks.txt", 14, 0));     // reads of @ files are allowed
    EXPECT_FALSE(file_content_allowed("@SYS/threads.txt", 16, 0));  // it resets the thread statistics
    EXPECT_FALSE(file_content_allowed("../../x", 7, 0));           // above the root
    static const char nul[] = { 'a', 0, 'b' };
    EXPECT_FALSE(file_content_allowed(nul, sizeof(nul), 0));
    std::string long_path(300, 'a');
    EXPECT_FALSE(file_content_allowed(long_path.c_str(), long_path.size(), 0));
}

// the logger's directory, however the path names it. Without the check the log being written can
// be queued, and as it grows faster than FILE logging reads it, it holds the queue for the boot
TEST(ScriptingM4Remote, PathInDirectory)
{
    for (const char *path : { "/APM/LOGS/00000001.BIN", "APM/LOGS/1.BIN", "//APM/./logs/x", "/APM/X/../LOGS/x",
                              "/apm/Logs/sub/x" }) {
        EXPECT_TRUE(path_in_directory(path, "/APM/LOGS")) << path;
    }
    for (const char *path : { "/APM/LOGS", "/APM/LOGS/", "/APM/LOGSX/1.BIN", "/APM/1.BIN", "/APM/LOGS/../1.BIN",
                              "/LOGS/1.BIN", "" }) {
        EXPECT_FALSE(path_in_directory(path, "/APM/LOGS")) << path;
    }
    EXPECT_TRUE(path_in_directory("logs/00000001.BIN", "logs"));
    EXPECT_TRUE(path_in_directory("./logs/1.BIN", "logs"));
    EXPECT_FALSE(path_in_directory("scripts/x.lua", "logs"));
}

// the paths FatFs opens in the logger's directory, read as FatFs reads them: '\' separates, a
// name's trailing spaces and dots are dropped, "0:" names the drive, a control character ends
// the path. Read as plain text, each looked like another directory, and the log being written
// could be queued
TEST(ScriptingM4Remote, PathInDirectoryReadsAsFatFs)
{
    for (const char *path : { "\\APM\\LOGS\\00000001.BIN", "/APM\\LOGS/1.BIN", "/APM/LOGS./1.BIN",
                              "/APM/LOGS /1.BIN", "/APM./LOGS. ./1.BIN", "0:/APM/LOGS/1.BIN", "0:APM/LOGS/1.BIN",
                              "/APM/LOGS/1.BIN\x01/../../x", "/APM/X\\..\\LOGS/1.BIN" }) {
        EXPECT_TRUE(path_in_directory(path, "/APM/LOGS")) << path;
    }
    for (const char *path : { "/APM/ LOGS/1.BIN", "/APM/LO GS/1.BIN", "/APM/LOGS\\..\\1.BIN", "/APM/LOGS.X/1.BIN" }) {
        EXPECT_FALSE(path_in_directory(path, "/APM/LOGS")) << path;
    }
}

// any byte of 0x80 or above: FatFs (CP850) folds accented letters to ASCII capitals in its
// short-name lookup, so "/\x83PM/LOGS" opens /APM/LOGS; macOS SITL opens "log\xC5\xBF" (logſ) as "logs"
TEST(ScriptingM4Remote, PathInDirectoryRefusesNonAscii)
{
    for (const char *path : { "/\x83PM/LOGS/1.BIN", "/APM/L\x93GS/1.BIN", "/\xB5PM/LOGS/1.BIN", "/APM/LO\223S/x",
                              "/APM/scripts/\xE9t\xE9.lua" }) {
        EXPECT_TRUE(path_in_directory(path, "/APM/LOGS")) << path;
    }
    EXPECT_TRUE(path_in_directory("log\xC5\xBF/00000001.BIN", "logs"));
    EXPECT_TRUE(path_in_log_directory("/\x83PM/LOGS/1.BIN", "/APM/LOGS"));
}

// "@MAV_LOG" is AP_Filesystem's alias of the logger's directory: "@MAV_LOG/00000001.BIN" opens
// the log being written
TEST(ScriptingM4Remote, PathInLogDirectoryTakesTheAlias)
{
    for (const char *path : { "@MAV_LOG/00000001.BIN", "/@MAV_LOG/1.BIN", "@MAV_LOG", "@MAV_LOG/", "@mav_log/1.BIN",
                              "@MAV_LOG\\1.BIN", "@MAV_LOG./1.BIN", "@MAV_LOG/../x", "/APM/LOGS/1.BIN" }) {
        EXPECT_TRUE(path_in_log_directory(path, "/APM/LOGS")) << path;
    }
    for (const char *path : { "@MAV_LOGS/1.BIN", "@SYS/x", "/APM/scripts/x.lua", "x/@MAV_LOG/1.BIN" }) {
        EXPECT_FALSE(path_in_log_directory(path, "/APM/LOGS")) << path;
    }
}

// 'Q' values of logger:write take what coerce_to_uint64_t() takes in-process: a boxed uint64_t or
// uint32_t, a non-negative integer, a non-negative number below 2^64. Anything else is its
// in-process error. In-process 2^64 itself passes, and its conversion is undefined
TEST(ScriptingM4Remote, Uint64Coercion)
{
    Call c;
    c.call.uint32_type = UINT32;
    c.call.uint64_type = 13;
    const uint64_t big = 0x123456789ULL;
    c.args.blob(13, &big, sizeof(big));
    ASSERT_EQ(c.run(thing_uint64), ReplyStatus::Ok);
    uint64_t got = 0;
    memcpy(&got, c.values[0].blob, sizeof(got));
    EXPECT_EQ(got, big);
    const uint32_t small = 0xFFFFFFF0U;
    c.args.blob(UINT32, &small, sizeof(small));
    ASSERT_EQ(c.run(thing_uint64), ReplyStatus::Ok);
    memcpy(&got, c.values[0].blob, sizeof(got));
    EXPECT_EQ(got, 0xFFFFFFF0ULL);
    c.args.integer(42);
    ASSERT_EQ(c.run(thing_uint64), ReplyStatus::Ok);
    memcpy(&got, c.values[0].blob, sizeof(got));
    EXPECT_EQ(got, 42U);
    c.args.number(1.0e19f);
    ASSERT_EQ(c.run(thing_uint64), ReplyStatus::Ok);
    memcpy(&got, c.values[0].blob, sizeof(got));
    EXPECT_EQ(got, uint64_t(1.0e19f));
    c.args.number(float(UINT64_MAX));   // 2^64
    EXPECT_EQ(c.run(thing_uint64), ReplyStatus::ArgError);
    c.args.integer(-1);
    EXPECT_EQ(c.run(thing_uint64), ReplyStatus::ArgError);
    EXPECT_EQ(c.text(0), "Unable to coerce to uint64_t");
    c.args.string("x", 1);
    EXPECT_EQ(c.run(thing_uint64), ReplyStatus::ArgError);
}

#if AP_SCRIPTING_M4_ENABLED && HAL_LOGGING_ENABLED
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// the vehicle's logger, without backends: formats are taken, rows go nowhere
static AP_Logger &test_logger()
{
    static AP_Logger logger;
    return logger;
}

static const AP_Logger::log_write_fmt *logger_format(const char *name)
{
    for (const AP_Logger::log_write_fmt *f = test_logger().log_write_fmts; f != nullptr; f = f->next) {
        if (strcmp(f->name, name) == 0) {
            return f;
        }
    }
    return nullptr;
}

// logger:write(name, labels, format, ...) as the M4 sends it: a nil self, then the strings
static void logger_strings(Call &c, std::initializer_list<const char *> strings)
{
    c.args.nil();
    for (const char *text : strings) {
        c.args.string(text, strlen(text));
    }
}

// the identifiers LOGGER_UNIT_IDS and LOGGER_MULTIPLIER_IDS name are the logger's own: a unit
// the logger added and the list lacks would be refused, one it lacks would stop SITL
TEST(ScriptingM4LoggerWrite, IdsAreTheLoggersTables)
{
    std::string units, mults;
    for (const UnitStructure &unit : log_Units) {
        units += unit.ID;
    }
    for (const MultiplierStructure &mult : log_Multipliers) {
        mults += mult.ID;
    }
    EXPECT_EQ(units, LOGGER_UNIT_IDS);
    EXPECT_EQ(mults, LOGGER_MULTIPLIER_IDS);
}

// a row takes its format once; the same call again finds it
TEST(ScriptingM4LoggerWrite, ARowTakesOneFormat)
{
    test_logger();
    Call c;
    for (int i = 0; i < 2; i++) {
        logger_strings(c, { "LWA", "a,b,c", "fiQ", "m-s", "0-F" });
        c.args.number(1.5f);
        c.args.integer(7);
        c.args.integer(9);
        ASSERT_EQ(c.run(AP_Logger_Write), ReplyStatus::Ok) << c.text(0);
        EXPECT_EQ(c.count, 0);
    }
    const AP_Logger::log_write_fmt *f = logger_format("LWA");
    ASSERT_NE(f, nullptr);
    EXPECT_STREQ(f->fmt, "QfiQ");
    EXPECT_STREQ(f->labels, "TimeUS,a,b,c");
    EXPECT_STREQ(f->units, "sm-s");
    EXPECT_STREQ(f->mults, "F0-F");
    EXPECT_EQ(f->msg_len, 3 + 8 + 4 + 4 + 8);
    int named = 0;
    for (const AP_Logger::log_write_fmt *g = test_logger().log_write_fmts; g != nullptr; g = g->next) {
        named += strcmp(g->name, "LWA") == 0;
    }
    EXPECT_EQ(named, 1);
}

// a row longer than 255 bytes is refused before the logger is asked for a message type. Without
// the check msg_fmt_for_name() takes it: it copies the strings (about 121 B), finds the row too
// long, and returns nullptr without freeing the copy, "could not map message type": an M4 loop of
// these calls empties the M7 heap
TEST(ScriptingM4LoggerWrite, TheRowIsMeasuredFirst)
{
    test_logger();
    Call c;
    logger_strings(c, { "LWZ", "a,b,c,d", "ZZZZ", "", "", "", "" });
    EXPECT_EQ(c.run(AP_Logger_Write), ReplyStatus::ArgError);
    EXPECT_EQ(c.text(0), "unknown format");
    EXPECT_EQ(logger_format("LWZ"), nullptr);
    // 3 + 8 + 3 x 64 + 52 = 255 fits
    logger_strings(c, { "LWY", "a,b,c,d", "ZZZN", "", "", "", "" });
    ASSERT_EQ(c.run(AP_Logger_Write), ReplyStatus::Ok) << c.text(0);
    ASSERT_NE(logger_format("LWY"), nullptr);
    EXPECT_EQ(logger_format("LWY")->msg_len, 3 + 8 + 3 * 64 + 16);
}

// what SITL's logger stops the host thread on (config_error()) is refused before it is asked, and
// takes no format. Without each check this test hangs or aborts in msg_fmt_for_name()
TEST(ScriptingM4LoggerWrite, WhatTheLoggerStopsOnIsRefused)
{
    test_logger();
    Call c;
    struct Case {
        std::initializer_list<const char *> strings;
        uint8_t values;
        const char *text;
    };
    const Case cases[] = {
        { { "LWB", "a,a", "ff" }, 2, "labels must be unique and not empty" },
        { { "LWB", "a,", "ff" }, 2, "labels must be unique and not empty" },
        { { "LWB", "TimeUS", "f" }, 1, "labels must be unique and not empty" },
        { { "LWB", "a", "f", "x", "0" }, 1, "x unsupported unit" },
        { { "LWB", "a", "f", "m", "H" }, 1, "H unsupported multiplier" },
        { { "LW-", "a", "f" }, 1, "Name must be letters, digits or _" },
        { { "", "a", "f" }, 1, "Name must be letters, digits or _" },
        { { "LWB", "a,b", "fd" }, 2, "d unsupported format" },
    };
    for (const Case &k : cases) {
        logger_strings(c, k.strings);
        for (uint8_t i = 0; i < k.values; i++) {
            c.args.number(1);
        }
        EXPECT_NE(c.run(AP_Logger_Write), ReplyStatus::Ok) << k.text;
        EXPECT_EQ(c.text(0), k.text);
    }
    EXPECT_EQ(logger_format("LWB"), nullptr);
    EXPECT_EQ(logger_format("LW-"), nullptr);
    EXPECT_EQ(logger_format(""), nullptr);
}

// a name keeps the format of its first row. SITL's logger stops on another
// (assert_same_fmt_for_name()); elsewhere the row would be written under the first one's layout
TEST(ScriptingM4LoggerWrite, ANameKeepsItsFormat)
{
    test_logger();
    Call c;
    logger_strings(c, { "LWR", "a", "f" });
    c.args.number(1);
    ASSERT_EQ(c.run(AP_Logger_Write), ReplyStatus::Ok) << c.text(0);
    const std::initializer_list<const char *> others[] = {
        { "LWR", "a", "i" },
        { "LWR", "b", "f" },
        { "LWR", "a", "f", "m", "0" },
    };
    for (const std::initializer_list<const char *> &other : others) {
        logger_strings(c, other);
        c.args.integer(1);
        EXPECT_EQ(c.run(AP_Logger_Write), ReplyStatus::ArgError);
        EXPECT_EQ(c.text(0), "format differs from an earlier write");
    }
    const AP_Logger::log_write_fmt *f = logger_format("LWR");
    ASSERT_NE(f, nullptr);
    EXPECT_STREQ(f->fmt, "Qf");
    EXPECT_EQ(f->units, nullptr);
}

// every value is read before the logger is asked for a message type. In-process a bad value
// fails after msg_fmt_for_name(), which has taken a type and M7 memory for good
TEST(ScriptingM4LoggerWrite, ValuesAreReadFirst)
{
    test_logger();
    Call c;
    logger_strings(c, { "LWV", "a,b", "fN", "x", "y" });
    EXPECT_EQ(c.run(AP_Logger_Write), ReplyStatus::ArgError);
    EXPECT_EQ(c.text(0), "number expected, got string");
    logger_strings(c, { "LWV", "a,b", "fN" });
    c.args.number(1);
    c.args.string("seventeen chars!!", 17);
    EXPECT_EQ(c.run(AP_Logger_Write), ReplyStatus::Error);
    EXPECT_EQ(c.text(0), "arg 6 too long for N format");
    logger_strings(c, { "LWV", "a,b", "fN" });
    c.args.number(1);
    c.args.boolean(true);
    EXPECT_EQ(c.run(AP_Logger_Write), ReplyStatus::ArgError);
    EXPECT_EQ(logger_format("LWV"), nullptr);
    logger_strings(c, { "LWV", "a,b", "fN" });
    c.args.number(1);
    c.args.string("sixteen chars!!!", 16);
    ASSERT_EQ(c.run(AP_Logger_Write), ReplyStatus::Ok) << c.text(0);
    EXPECT_NE(logger_format("LWV"), nullptr);
}

// the Log quota is charged before the logger is asked for a message type: over it the row is
// dropped and takes nothing. Without the charge a script writes rows as fast as the host serves it
TEST(ScriptingM4LoggerWrite, LogQuotaComesBeforeTheFormat)
{
    test_logger();
    Call c;
    Quota quota;
    quota.reset(0);
    c.call.quota = &quota;
    c.call.now_ms = 0;
    for (int i = 0; i < 500; i++) {
        ASSERT_TRUE(quota.allow(QuotaClass::Log, 0));
    }
    logger_strings(c, { "LWQ", "a", "f" });
    c.args.number(1);
    ASSERT_EQ(c.run(AP_Logger_Write), ReplyStatus::Ok);
    EXPECT_EQ(c.count, 0);
    EXPECT_EQ(quota.take_hits(), 1U);
    EXPECT_EQ(logger_format("LWQ"), nullptr);
    c.call.now_ms = 2;   // one row per 2 ms
    logger_strings(c, { "LWQ", "a", "f" });
    c.args.number(1);
    ASSERT_EQ(c.run(AP_Logger_Write), ReplyStatus::Ok);
    EXPECT_EQ(quota.take_hits(), 0U);
    EXPECT_NE(logger_format("LWQ"), nullptr);
}

#if HAL_LOGGER_FILE_CONTENTS_ENABLED
// logger:log_file_content() of the M4 queues a readable path while fewer than 40 files wait.
// Without the replacement every call queues any path; without its queue count the 41st queues
TEST(ScriptingM4LoggerWrite, FileContentIsCheckedAndBounded)
{
    AP_Logger &logger = test_logger();
    Call c;
    EXPECT_EQ(logger.file_content_queued(UINT8_MAX), 0);
    for (const char *path : { "../../x", "/APM/../../x" }) {
        c.args.nil();
        c.args.string(path, strlen(path));
        ASSERT_EQ(c.run(lua_m4_AP_Logger_log_file_content), ReplyStatus::Ok);
    }
    EXPECT_EQ(logger.file_content_queued(UINT8_MAX), 0);
    // a device ("//dev/zero": SITL's AP_Filesystem drops one leading '/'), a directory, a missing
    // file, and a file in the logger's directory (SITL: "logs"),
    // also by AP_Filesystem's alias of it
    mkdir("logs", 0755);
    FILE *in_logs = fopen("logs/LWTEST.BIN", "w");
    ASSERT_NE(in_logs, nullptr);
    fclose(in_logs);
    for (const char *path : { "//dev/zero", "/", "no_such_file.txt", "logs/LWTEST.BIN", "./logs/../logs/LWTEST.BIN",
                              "@MAV_LOG/LWTEST.BIN", "log\xC5\xBF/LWTEST.BIN" }) {
        c.args.nil();
        c.args.string(path, strlen(path));
        ASSERT_EQ(c.run(lua_m4_AP_Logger_log_file_content), ReplyStatus::Ok);
        EXPECT_EQ(logger.file_content_queued(UINT8_MAX), 0) << path;
    }
    unlink("logs/LWTEST.BIN");
    // a regular file elsewhere is queued
    FILE *regular = fopen("LWTEST.TXT", "w");
    ASSERT_NE(regular, nullptr);
    fclose(regular);
    c.args.nil();
    c.args.string("LWTEST.TXT", 10);
    ASSERT_EQ(c.run(lua_m4_AP_Logger_log_file_content), ReplyStatus::Ok);
    EXPECT_EQ(logger.file_content_queued(UINT8_MAX), 1);
    static const char path[] = "LWTEST.TXT";
    for (int i = 0; i < FILE_CONTENT_QUEUE_MAX + 5; i++) {
        c.args.nil();
        c.args.string(path, strlen(path));
        ASSERT_EQ(c.run(lua_m4_AP_Logger_log_file_content), ReplyStatus::Ok);
    }
    EXPECT_EQ(logger.file_content_queued(UINT8_MAX), FILE_CONTENT_QUEUE_MAX);
    EXPECT_EQ(logger.file_content_queued(3), 3);
    unlink("LWTEST.TXT");
}
#endif  // HAL_LOGGER_FILE_CONTENTS_ENABLED

// M4 scripts take at most 32 of the logger's message types per boot; a name they have keeps
// working. Without the cap a script takes every free type in one burst, and the vehicle's next
// first Write() of a name raises an internal error (logger_mapfailure), which blocks arming.
// Runs last: the types taken here stay taken, as on the vehicle
TEST(ScriptingM4LoggerWrite, ScriptTypesAreCapped)
{
    test_logger();
    Call c;
    auto entries = []() {
        int count = 0;
        for (const AP_Logger::log_write_fmt *f = test_logger().log_write_fmts; f != nullptr; f = f->next) {
            count++;
        }
        return count;
    };
    char name[5];
    int i = 0;
    while (entries() < LOGGER_SCRIPT_TYPES_MAX) {
        snprintf(name, sizeof(name), "C%03d", i++);
        logger_strings(c, { name, "a", "f" });
        c.args.number(1);
        ASSERT_EQ(c.run(AP_Logger_Write), ReplyStatus::Ok) << name << " " << c.text(0);
        ASSERT_LT(i, 100);
    }
    EXPECT_EQ(LOGGER_SCRIPT_TYPES_MAX, 32);
    // one Log row left: the refused name must not take it
    Quota quota;
    quota.reset(0);
    c.call.quota = &quota;
    c.call.now_ms = 0;
    for (int n = 0; n < 499; n++) {
        ASSERT_TRUE(quota.allow(QuotaClass::Log, 0));
    }
    logger_strings(c, { "CNEW", "a", "f" });
    c.args.number(1);
    EXPECT_EQ(c.run(AP_Logger_Write), ReplyStatus::ArgError);
    EXPECT_EQ(c.text(0), "too many log message types");
    EXPECT_EQ(entries(), LOGGER_SCRIPT_TYPES_MAX);
    EXPECT_EQ(logger_format("CNEW"), nullptr);
    // a name already taken still writes, and its other formats are still refused as such
    logger_strings(c, { name, "a", "f" });
    c.args.number(2);
    EXPECT_EQ(c.run(AP_Logger_Write), ReplyStatus::Ok) << c.text(0);
    EXPECT_EQ(quota.take_hits(), 0U);
    logger_strings(c, { name, "a", "i" });
    c.args.integer(2);
    EXPECT_EQ(c.run(AP_Logger_Write), ReplyStatus::ArgError);
    EXPECT_EQ(c.text(0), "format differs from an earlier write");
}
#endif  // AP_SCRIPTING_M4_ENABLED && HAL_LOGGING_ENABLED

AP_GTEST_MAIN()
