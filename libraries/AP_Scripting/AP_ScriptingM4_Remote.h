#pragma once

/*
  The M7 end of the generated bindings. gen-bindings -r writes
  lua_remote_dispatch.cpp: the in-process binding functions, as
  emit_userdata_method() and emit_field() write them, compiled in this
  namespace against this header in place of Lua. A lua_State is one call
  from the M4: its arguments are the decoded values, its results are
  collected in order and encoded when the function returns. Argument errors
  leave the function with ap_longjmp, as luaL_error() does in-process; the
  generated code checks every argument before it takes a semaphore.
  Userdata arrive as blobs (the described fields of a pure-data type, in
  their wire form) or handles (HandleTable). Scripting host thread only.
 */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <type_traits>
// ap_setjmp.h includes the board header inside extern "C"; C++ needs it included first
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/ap_setjmp.h>
#include "AP_ScriptingM4_Rpc.h"
#include "AP_ScriptingM4_Files.h"
#include "AP_ScriptingM4_Handles.h"
#include "AP_ScriptingM4_Objects.h"
#include "AP_ScriptingM4_Quota.h"
#include "AP_ScriptingM4_Wire.h"

namespace ScriptingM4Remote {

// bytes per reply of a manual function that reads "up to" a count the script gives
// (readstring, recv): what fits one reply frame with the values that follow it
static const uint16_t IO_CHUNK = 768;
static_assert(IO_CHUNK + 3 + 8 + 5 <= ScriptingM4::MAX_FRAME - sizeof(ScriptingM4::ReplyHeader),
              "a chunk, a uint32_t blob and an integer fit one reply");

typedef int32_t lua_Integer;   // LUA_32BITS, as on the M4
typedef float lua_Number;
struct RemoteCall;
typedef RemoteCall lua_State;
typedef int (*lua_CFunction)(lua_State *L);

// the blob wire form (AP_ScriptingM4_Wire.h); the generated codecs name them unqualified
using ScriptingM4::WireReader;
using ScriptingM4::WireWriter;

// one generated type, by type id - 1; decode == nullptr: no blob (another kind, or not on this firmware)
struct BlobCodec {
    const char *name;                                       // the __name in type error texts
    uint16_t size;                                          // sizeof the C++ type
    bool (*decode)(WireReader &reader, void *storage);      // constructs the object in storage
    void (*encode)(WireWriter &writer, const void *object);
};

// I2C limits on the M7 host thread (AP_ScriptingM4_ManualI2C.cpp, and the generated methods of
// AP_HAL::I2CDevice, which take the semaphore for at most I2C_SEMAPHORE_WAIT_MS). ChibiOS times
// a transfer out after 1 + 2 * (8e6 / busclock * bytes) / 1000 ms per attempt and makes
// retries + 1 attempts, clearing the bus between them (20 x 10 us, under 1 ms). At the clock floor
// 256 bytes time out after 41 ms; with at most 18 retries (19 attempts) the nominal worst case is
// 30 + 19 * (41 + 1) = 828 ms, below the 850 ms budget of the 900 ms rule. In-process passes any
// clock, length and retry count through (recorded deviation). A request at the floor still sets
// the shared bus to 100 kHz until reboot, as in-process: the floor does not prevent that
static const uint32_t I2C_CLOCK_MIN_HZ = 100000;
static const uint32_t I2C_CLOCK_MAX_HZ = 1000000;
static const uint32_t I2C_TRANSFER_BYTES_MAX = 256;
static const uint32_t I2C_SEMAPHORE_WAIT_MS = 30;
static const uint32_t I2C_RETRIES_MAX = 18;
static const uint32_t I2C_BUS_CLEAR_MS = 1;
static_assert(I2C_SEMAPHORE_WAIT_MS + (I2C_RETRIES_MAX + 1) *
              (1 + 2 * ((8 * 1000000UL / I2C_CLOCK_MIN_HZ) * I2C_TRANSFER_BYTES_MAX) / 1000 + I2C_BUS_CLEAR_MS) <= 850,
              "the worst I2C transfer fits the host thread's budget");

inline uint32_t i2c_clamp_bus_clock(uint32_t hertz)
{
    return hertz < I2C_CLOCK_MIN_HZ ? I2C_CLOCK_MIN_HZ : (hertz > I2C_CLOCK_MAX_HZ ? I2C_CLOCK_MAX_HZ : hertz);
}

// the bus clock argument of i2c:get_device, clamped
uint32_t i2c_bus_clock_argument(lua_State *L, int arg);

inline bool i2c_transfer_fits(size_t send_bytes, size_t receive_bytes)
{
    return send_bytes <= I2C_TRANSFER_BYTES_MAX && receive_bytes <= I2C_TRANSFER_BYTES_MAX &&
           send_bytes + receive_bytes <= I2C_TRANSFER_BYTES_MAX;
}

// CAN limits on the M7 host thread (AP_ScriptingM4_ManualCAN.cpp). write_frame() waits in
// CANSensor::write_frame()'s select() for as long as the script asks; the port clamps the wait
// (recorded deviation: in-process passes any timeout through)
static const uint32_t CAN_WRITE_TIMEOUT_MAX_US = 900000;
// the timeout argument of ScriptingCANBuffer:write_frame, clamped
uint32_t can_write_timeout_argument(lua_State *L, int arg);
// scripting CAN buffers an M4 session may hold per sensor: the sensor never frees one, and its
// CAN thread walks the buffer list recursively. Buffers of earlier sessions are handed out again
static const uint8_t CAN_BUFFERS_PER_SESSION = 8;
static const uint32_t CAN_BUFFER_WAIT_MS = 30;

// method flags (gen-bindings -r); the high nibble is the ScriptingM4::QuotaClass
static const uint8_t METHOD_WRITES_SELF = 0x01;   // the reply ends with the updated self blob
static const uint8_t METHOD_QUOTA_FALSE = 0x02;   // over its quota the call returns false, else it is dropped

struct RemoteCall {
    // set once by the server
    ScriptingM4::HandleTable *handles;
    ScriptingM4::ObjectPool *objects;    // the owned objects (Owned results)
    ScriptingM4::FileTable *files;    // the M4 session's open files (sock:sendfile)
    void (*warn)(void *context, uint8_t severity, const char *text);   // set_and_print_new_error_message()
    void *warn_context;
    ScriptingM4::Quota *quota;           // the quotas a port charges itself (CAN frames)
    // set by dispatch()
    const BlobCodec *codecs;
    uint8_t codec_count;
    uint8_t uint32_type;          // the boxed numerics' type ids: 4 and 8 raw bytes
    uint8_t uint64_type;

    // per call (run())
    uint32_t now_ms;                   // set by the server before dispatch()
    const ScriptingM4::RpcValue *args;
    uint8_t argc;
    struct Result {
        enum Kind : uint8_t { Value, Object, Handle, Owned };
        Kind kind;
        uint8_t type_id;
        ScriptingM4::RpcValue value;   // Value; a string points into the arena
        void *object;                  // Object: in the arena; Handle: an arena slot holding the pointer;
                                       // Owned: the pool slot
    } results[ScriptingM4::MAX_VALUES];
    uint8_t result_count;
    void *self;                        // argument 1 decoded as a blob: written back
    uint8_t self_type;
    uint16_t arena_used;
    alignas(8) uint8_t arena[2048];    // decoded blobs, results, terminated strings
    ScriptingM4::ReplyStatus status;
    uint8_t error_arg;
    char message[96];
    ap_jmp_buf jump;
};

// runs one binding function on decoded arguments: Ok with its results (and, with
// METHOD_WRITES_SELF, the updated self) in results, or the error (text first)
ScriptingM4::ReplyStatus run(RemoteCall &call, const ScriptingM4::RpcValue *args, uint8_t argc,
                             lua_CFunction function, uint8_t flags,
                             ScriptingM4::RpcWriter &results, uint8_t &error_arg);

// the in-process API the generated functions call (generated_cpp_deps.h, lauxlib, lua_boxed_numerics)
int lua_gettop(lua_State *L);
int binding_argcheck(lua_State *L, int expected_arg_count);
int field_argerror(lua_State *L);
bool userdata_zero_arg_check(lua_State *L);
lua_Integer luaL_checkinteger(lua_State *L, int arg);
lua_Number luaL_checknumber(lua_State *L, int arg);
const char *luaL_checklstring(lua_State *L, int arg, size_t *length);
#ifndef luaL_checkstring
inline const char *luaL_checkstring(lua_State *L, int arg) { return luaL_checklstring(L, arg, nullptr); }
#endif
int lua_toboolean(lua_State *L, int arg);
lua_Integer get_integer(lua_State *L, int arg, lua_Integer min_value, lua_Integer max_value);
int8_t get_int8_t(lua_State *L, int arg);
int16_t get_int16_t(lua_State *L, int arg);
uint8_t get_uint8_t(lua_State *L, int arg);
uint16_t get_uint16_t(lua_State *L, int arg);
float get_number(lua_State *L, int arg, float min_value, float max_value);
uint32_t get_uint32(lua_State *L, int arg, uint32_t min_value, uint32_t max_value);
uint32_t coerce_to_uint32_t(lua_State *L, int arg);
// these leave the binding function (ap_longjmp), as in-process
__attribute__((noreturn)) int luaL_argerror(lua_State *L, int arg, const char *message);
__attribute__((noreturn, format(printf, 2, 3))) int luaL_error(lua_State *L, const char *fmt, ...);
void luaL_checkstack(lua_State *L, int size, const char *message);
void lua_pushboolean(lua_State *L, int value);
void lua_pushinteger(lua_State *L, lua_Integer value);
void lua_pushnumber(lua_State *L, lua_Number value);
void lua_pushstring(lua_State *L, const char *text);
void lua_pushlstring(lua_State *L, const char *text, size_t length);   // binary, NULs included
int lua_type(lua_State *L, int arg);   // lua.h's LUA_TNONE (-1), LUA_TNIL (0), ... of an argument
const char *lua_tolstring(lua_State *L, int arg, size_t *length);   // nullptr for a non-string
uint32_t *new_uint32_t(lua_State *L);
uint64_t *new_uint64_t(lua_State *L);   // gps:time_epoch_usec()
uint64_t *check_uint64_t(lua_State *L, int arg);   // DroneCAN_Handle(): a boxed uint64_t argument
__attribute__((noreturn)) int not_supported_error(lua_State *L, int arg, const char *name);
namespace lua_scripts {
void set_and_print_new_error_message(uint8_t severity, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
}
#ifndef LUA_MINSTACK
#define LUA_MINSTACK 20
#endif

// storage of the generated check_<type>()/new_<type>()
void *remote_check_blob(lua_State *L, int arg, uint8_t type_id);
void *remote_new_blob(lua_State *L, uint8_t type_id, size_t size);
void **remote_check_handle(lua_State *L, int arg, uint8_t type_id);
void **remote_new_handle(lua_State *L, uint8_t type_id);
// storage of the generated check_<type>()/new_<type>() of an owned type: the object itself, in
// the pool; a failed call destroys the objects it made (run())
void *remote_check_owned(lua_State *L, int arg, uint8_t type_id);
void *remote_new_owned(lua_State *L, uint8_t type_id, ScriptingM4::ObjectPool::Destroy destroy);

// manual bindings ported to the adapter (AP_ScriptingM4_Manual.cpp; gen-bindings -r remote_ready_manual[])
int lua_millis(lua_State *L);
int lua_micros(lua_State *L);
int lua_new_Parameter(lua_State *L);
// AP_ScriptingM4_ManualSerial.cpp
int lua_serial_find_serial(lua_State *L);
int lua_serial_find_simulated_device(lua_State *L);
int lua_serial_begin(lua_State *L);
int lua_serial_writestring(lua_State *L);
int lua_serial_readstring(lua_State *L);
// AP_ScriptingM4_ManualI2C.cpp
int lua_get_i2c_device(lua_State *L);
int AP_HAL__I2CDevice_read_registers(lua_State *L);
int AP_HAL__I2CDevice_transfer(lua_State *L);
int lua_get_PWMSource(lua_State *L);
int lua_m4_I2CDevice_set_retries(lua_State *L);
// AP_ScriptingM4_ManualCAN.cpp
int lua_get_CAN_device(lua_State *L);
int lua_get_CAN_device2(lua_State *L);
int lua_m4_ScriptingCANBuffer_write_frame(lua_State *L);
int DroneCAN_Handle__new_handle(lua_State *L);
int DroneCAN_Handle__broadcast(lua_State *L);
int DroneCAN_Handle__request(lua_State *L);
int DroneCAN_Handle__check_message(lua_State *L);
int lua_m4_DroneCAN_Handle_subscribe(lua_State *L);
// AP_ScriptingM4_ManualSocket.cpp
int lua_get_SocketAPM(lua_State *L);
int SocketAPM_close(lua_State *L);
int SocketAPM_sendfile(lua_State *L);
int SocketAPM_recv(lua_State *L);
int SocketAPM_accept(lua_State *L);
int SocketAPM_ipv4_addr_to_string(lua_State *L);
int SocketAPM_string_to_ipv4_addr(lua_State *L);
int SocketAPM_send_checked(lua_State *L);
int SocketAPM_sendto_checked(lua_State *L);
int lua_m4_SocketAPM_pollin(lua_State *L);
int lua_m4_SocketAPM_pollout(lua_State *L);
int lua_m4_SocketAPM_set_blocking(lua_State *L);
// the session's sockets are deleted and its sendfile hand-overs counted afresh
// (AP_ScriptingM4_Server::end_session())
void sockets_end_session();
// an owned DroneCAN_Handle's release (the in-process __gc): its subscriber goes
void DroneCAN_Handle__release(void *object);
// M7 replacements of generated methods (gen-bindings -r remote_replaced_methods[])
int lua_m4_Parameter_init_by_info(lua_State *L);
int lua_m4_analog_channel(lua_State *L);
// the analog sources earlier sessions were handed may be handed out again (AP_ScriptingM4_Server::reset())
void analog_new_session();
// the CAN buffers earlier sessions were handed may be handed out again (AP_ScriptingM4_Server::reset())
void can_new_session();

// the members of ScriptingCANBuffer a buffer handed out again resets (a friend of the class)
struct CANBufferAccess {
    // empty, unfiltered and of the depth the new script asked for; false if the CAN thread held
    // the buffer past the wait or the new depth could not be allocated (it then holds no frame).
    // A frame the CAN thread accepted under the old filters just before the reset can still be
    // pushed after it: the new session accepts every frame, so it is a new frame, not an old one
    template <typename Buffer> static bool reuse(Buffer &buffer, uint32_t depth)
    {
        if (!buffer.sem.take(CAN_BUFFER_WAIT_MS)) {
            return false;
        }
        buffer.num_filters = 0;
        const bool resized = buffer.buffer.set_size(depth);
        buffer.sem.give();
        return resized;
    }
};

// CAN:get_device*()'s buffer: one an earlier session was handed, else make()'s, reset for this
// session; nullptr past CAN_BUFFERS_PER_SESSION (before make()) or when the reset fails. A
// failed reset still spends the session's slot; the buffer is handed out again next session
template <typename Buffer, typename Make>
Buffer *take_CAN_buffer(ScriptingM4::SessionRecycler<Buffer, CAN_BUFFERS_PER_SESSION> &buffers, Make make,
                        uint32_t depth)
{
    Buffer *buffer = buffers.take_at_most(make);
    if (buffer == nullptr || !CANBufferAccess::reuse(*buffer, depth)) {
        return nullptr;
    }
    return buffer;
}

// buffer:write_frame(frame, timeout_us)'s call, the timeout clamped. Back-to-back calls on a bus
// that takes no frame keep the host thread busy up to 900 ms each (Bench Validation)
template <typename Buffer, typename Frame> int CAN_write_frame(lua_State *L, Buffer &buffer, Frame &frame, int timeout_arg)
{
    lua_pushboolean(L, buffer.write_frame(frame, can_write_timeout_argument(L, timeout_arg)));
    return 1;
}

// DroneCAN_Handle:check_message()'s results are the payload, the node id (integer, 5 bytes), the
// uint64_t timestamp (blob, 12 bytes) and canfd (1 byte): a longer payload cannot cross in one reply
static const uint16_t DRONECAN_PAYLOAD_MAX = ScriptingM4::MAX_STRING - (5 + 12 + 1);
// the next payload of a subscriber's queue, peeked: the caller pops it once copied. One that
// cannot cross is popped and freed, and false returned, so it never blocks the queue
template <typename Payloads, typename Payload> bool dronecan_peek_payload(Payloads &payloads, Payload &payload)
{
    if (!payloads.peek(payload)) {
        return false;
    }
    if (payload.length > DRONECAN_PAYLOAD_MAX) {
        payloads.pop();
        free(payload.data);
        return false;
    }
    return true;
}

// check_message()'s payload, node id and timestamp pushed and the payload popped and freed, as
// in-process (peek first: a payload stays queued if a push faults); false for none (nil)
template <typename Payloads, typename Payload> bool dronecan_push_payload(lua_State *L, Payloads &payloads, Payload &payload)
{
    if (!dronecan_peek_payload(payloads, payload)) {
        return false;
    }
    lua_pushlstring(L, reinterpret_cast<const char *>(payload.data), payload.length);
    payloads.pop();
    free(payload.data);
    lua_pushinteger(L, payload.node_id);
    *new_uint64_t(L) = payload.timestamp;
    return true;
}

// the CAN frames of a DroneCAN transfer of length payload bytes: one for a single-frame transfer,
// else 7 bytes per frame after the 2-byte transfer CRC. The Can quota counts frames, as a long
// payload is up to 145 frames on the bus
inline uint16_t dronecan_transfer_frames(size_t length)
{
    return length <= 7 ? 1 : uint16_t((length + 2 + 6) / 7);
}
// broadcast()/request(): takes the transfer's frames from the Can quota; false over it
bool can_transfer_allowed(lua_State *L, size_t length);
// request()'s node: an integer 0..127, else an argument error (in-process a float conversion)
uint8_t dronecan_target_node(lua_State *L, int arg);

// the priority of every M4 DroneCAN transfer: CANARD_TRANSFER_PRIORITY_MEDIUM, below ESC and
// actuator traffic (recorded deviation: in-process sends at CANARD_TRANSFER_PRIORITY_HIGHEST)
static const uint8_t DRONECAN_M4_TRANSFER_PRIORITY = 16;
// the fields of a broadcast's or request's Canard::Transfer that come from the handle and payload
template <typename Transfer, typename Handle>
void dronecan_fill_transfer(Transfer &transfer, Handle &handle, const void *data, size_t length)
{
    transfer.data_type_signature = handle.signature;
    transfer.data_type_id = handle.data_type;
    transfer.inout_transfer_id = &handle.transfer_id;
    transfer.priority = DRONECAN_M4_TRANSFER_PRIORITY;
    transfer.payload = data;
    transfer.payload_len = length;
    transfer.timeout_ms = 10;
}

// M4 DroneCAN subscribers live at once: each holds up to 8 payloads of M7 heap and runs on the
// DroneCAN thread for every matching transfer (recorded deviation: in-process has no cap)
static const uint8_t DRONECAN_SUBSCRIBERS_PER_SESSION = 8;
// the live subscribers of this session's handles; the caller holds the canard rx semaphore
class DroneCANSubscribers {
public:
    // slot's subscriber, if any, replaced by make()'s; false (no subscriber) past the cap
    template <typename Subscriber, typename Make> bool replace(Subscriber *&slot, Make make)
    {
        drop(slot);
        if (_live >= DRONECAN_SUBSCRIBERS_PER_SESSION) {
            return false;
        }
        slot = make();
        if (slot == nullptr) {
            return false;
        }
        _live++;
        return true;
    }
    template <typename Subscriber> void drop(Subscriber *&slot)
    {
        if (slot != nullptr) {
            delete slot;
            slot = nullptr;
            _live--;
        }
    }
    // the release of every handle ran at the session end; a new session starts from none
    void new_session() { _live = 0; }
    uint8_t live() const { return _live; }

private:
    uint8_t _live = 0;
};

// Sockets (AP_ScriptingM4_ManualSocket.cpp). The M7 issues every socket call non-blocking, so
// its host thread never waits on the network: a script's blocking socket (the in-process default,
// or set_blocking(true)) is the M4's, which tries a connect or send again until it completes
// (m4/m4_socket_wait.h). pollin()/pollout() wait in select() for at most this long (recorded
// deviation: in-process waits as long as the script asks)
static const uint32_t SOCKET_POLL_TIMEOUT_MAX_MS = 900;
// the timeout argument of SocketAPM:pollin/pollout, clamped
uint32_t socket_poll_timeout_argument(lua_State *L, int arg);
// sock:sendfile() hand-overs per M4 session: each leaves an M7 descriptor and a socket with
// AP_Networking until the file is sent, past the session's end (recorded deviation)
static const uint8_t SOCKET_SENDFILES_PER_SESSION = 2;

// a socket the M7 hands out (Socket(), sock:accept()), made non-blocking; nullptr stays nullptr
template <typename Socket> Socket *socket_nonblocking(Socket *sock)
{
    if (sock != nullptr) {
        (void)sock->set_blocking(false);
    }
    return sock;
}

// sock:set_blocking(blocking)'s call: the socket stays non-blocking whatever the script asks
// (the M4 keeps the script's mode); true if the socket is open, as in-process
template <typename Socket> int socket_set_blocking(lua_State *L, Socket &sock)
{
    (void)lua_toboolean(L, 2);
    lua_pushboolean(L, sock.set_blocking(false));
    return 1;
}

// sock:pollin(timeout_ms)/pollout(timeout_ms)'s call, the timeout clamped
template <typename Socket> int socket_poll(lua_State *L, Socket &sock, bool out)
{
    const uint32_t timeout_ms = socket_poll_timeout_argument(L, 2);
    lua_pushboolean(L, out ? sock.pollout(timeout_ms) : sock.pollin(timeout_ms));
    return 1;
}

// a send's results: the count, as in-process, and whether the call would have blocked, which the
// M4 drops after it tried again (m4_wrappers.cpp). errno is one for all threads on ChibiOS: another
// thread's socket call in between can change it, so a blocking send may end early or try again
// for up to the idle limit; Socket.cpp's own errno checks share this
inline int socket_send_results(lua_State *L, int32_t count)
{
    lua_pushinteger(L, count);
    lua_pushboolean(L, count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK));
    return 2;
}

// sock:send(data, len)'s call, the length bounded by the string: in-process SocketAPM reads len
// bytes from the Lua string, which here would read past the call's copy into M7 memory
template <typename Socket> int socket_send(lua_State *L, Socket &sock)
{
    size_t length = 0;
    const char *data = luaL_checklstring(L, 2, &length);
    const uint32_t requested = coerce_to_uint32_t(L, 3);
    errno = 0;   // a failure that sets none (a closed socket) is no earlier call's EAGAIN
    const auto count = sock.send(data, requested < length ? size_t(requested) : length);
    return socket_send_results(L, int32_t(count));
}

// sock:sendto(data, len, addr, port)'s call, the length bounded as send()'s
template <typename Socket> int socket_sendto(lua_State *L, Socket &sock)
{
    size_t length = 0;
    const char *data = luaL_checklstring(L, 2, &length);
    const uint32_t requested = coerce_to_uint32_t(L, 3);
    const uint32_t address = coerce_to_uint32_t(L, 4);
    const uint16_t port = get_uint16_t(L, 5);
    errno = 0;
    const auto count = sock.sendto(data, requested < length ? size_t(requested) : length, address, port);
    return socket_send_results(L, int32_t(count));
}

// the session's sock:sendfile() hand-overs
class SocketSendfiles {
public:
    bool available() const { return _handed_over < SOCKET_SENDFILES_PER_SESSION; }
    void take() { _handed_over++; }
    void new_session() { _handed_over = 0; }

private:
    uint8_t _handed_over = 0;
};

// AP_Networking::sendfile() waits for the networking semaphore, which its sendfile thread holds
// over a pass of file reads (up to 32 KB each, on the SD card the logger shares): the port takes
// it first for at most this long (recorded deviation: false if it is busy, in-process waits)
static const uint32_t SOCKET_SENDFILE_WAIT_MS = 30;

// sock:sendfile()'s hand-over of descriptor fd: hand_over(fd) only while the session has
// hand-overs left and the semaphore was taken within SOCKET_SENDFILE_WAIT_MS; hand_over takes the
// held, recursive semaphore again at once. One refused spends none. True if handed over
template <typename Semaphore, typename HandOver>
bool socket_sendfile(SocketSendfiles &sendfiles, int fd, Semaphore &sem, HandOver hand_over)
{
    if (fd == -1 || !sendfiles.available() || !sem.take(SOCKET_SENDFILE_WAIT_MS)) {
        return false;
    }
    const bool handed_over = hand_over(fd);
    sem.give();
    if (!handed_over) {
        return false;
    }
    sendfiles.take();
    return true;
}

// what a socket is at the session end, as its descriptor reports it (SO_TYPE, SO_ACCEPTCONN)
enum class SocketKind : uint8_t {
    Stream,      // a TCP socket that is not listening: connected, connecting or never connected
    Listening,   // a TCP socket after listen()
    Other,       // a datagram socket, or one whose kind could not be read
};

// only a non-listening stream socket is reset at the session end. lWIP closes a listening socket
// without waiting, and its close path reads unsent/unacked from the smaller listen PCB, so a
// linger of 0 there would abort a listen PCB; a datagram socket has nothing to linger for
inline bool socket_end_resets(SocketKind kind)
{
    return kind == SocketKind::Stream;
}

// a session's socket at the session end: a stream socket set to close abortively (reset(fd):
// SO_LINGER {1, 0}), then every socket deleted. A graceful close of a stream socket can hold the
// host thread while the network stack has no segment for the FIN (lwIP: up to 20 s and more); the
// dying session's unsent data is dropped (recorded deviation). A script's own sock:close() stays
// graceful, as in-process
template <typename Socket, typename KindOf, typename Reset> void socket_end(Socket *&slot, KindOf kind_of, Reset reset)
{
    if (slot == nullptr) {
        return;
    }
    const int fd = slot->get_read_fd();
    if (fd != -1 && socket_end_resets(kind_of(fd))) {
        reset(fd);
    }
    delete slot;
    slot = nullptr;
}

// generated (lua_remote_dispatch.cpp, M4 builds only)
bool method_available(uint16_t method);
uint8_t method_flags(uint16_t method);   // 0 for an unavailable method
ScriptingM4::ReplyStatus dispatch(uint16_t method, const ScriptingM4::RpcValue *args, uint8_t argc,
                                  RemoteCall &call, ScriptingM4::RpcWriter &results, uint8_t &error_arg);
void binding_info(uint8_t *info);        // BINDING_INFO_SIZE bytes (AP_ScriptingM4_Bindings.h)

}  // namespace ScriptingM4Remote
