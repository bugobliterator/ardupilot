/*
  The socket manual bindings of lua_bindings.cpp, ported to the remote adapter
  (gen-bindings -r remote_ready_manual[], remote_replaced_methods[]). Sockets
  are session objects in AP_Scripting's array, as in-process, deleted by
  AP_ScriptingM4_Server::end_session(). Where the in-process code trusts a
  length or a pointer from Lua (send, sendto, sendfile), the M7 checks it.
  Every socket is non-blocking on the M7, whatever the script asks: the M4
  waits in place of a blocking call (m4/m4_socket_wait.h), and pollin/pollout
  wait at most SOCKET_POLL_TIMEOUT_MAX_MS, so no call holds the serving thread.
 */
#include "AP_Scripting_config.h"

#if AP_SCRIPTING_M4_ENABLED

#include <AP_Networking/AP_Networking_Config.h>

#if AP_NETWORKING_SOCKETS_ENABLED

#include <AP_HAL/utility/Socket.h>
#include <AP_Networking/AP_Networking.h>
#if AP_NETWORKING_BACKEND_CHIBIOS || AP_NETWORKING_BACKEND_PPP
#include <lwip/sockets.h>
#include <lwip/tcpip.h>
#define SOCKET_CALL(x) ::lwip_##x
#else
#include <sys/socket.h>
#define SOCKET_CALL(x) ::x
#endif
#include "AP_Scripting.h"
#include "AP_ScriptingM4_Remote.h"

namespace ScriptingM4Remote {

// generated (lua_remote_dispatch.cpp)
SocketAPM **check_SocketAPM(lua_State *L, int arg);
SocketAPM **new_SocketAPM(lua_State *L);

static SocketSendfiles sendfiles;

// whether the network stack's core is made: lwIP's core lock exists once tcpip_init() ran
static bool stack_core_made()
{
#if AP_NETWORKING_BACKEND_CHIBIOS || AP_NETWORKING_BACKEND_PPP
    return lock_tcpip_core != nullptr;
#else
    return true;
#endif
}

// Socket(datagram): lua_get_SocketAPM(), nil while the network stack does not run
// (socket_stack_ready()). A handle refused at encoding (handle table full) is
// retried once by the M4, which makes a second socket; the first stays in its slot until the
// session ends (recorded)
int lua_get_SocketAPM(lua_State *L)
{
    binding_argcheck(L, 1);
    const uint8_t datagram = get_uint8_t(L, 1);
    auto *scripting = AP::scripting();

    if (!socket_stack_ready(AP::network().is_healthy(), stack_core_made())) {
        return 0;
    }

    auto *sock = socket_nonblocking(NEW_NOTHROW SocketAPM(datagram));
    if (sock == nullptr) {
        return luaL_argerror(L, 1, "SocketAPM device nullptr");
    }
    for (uint8_t i=0; i<SCRIPTING_MAX_NUM_NET_SOCKET; i++) {
        if (scripting->_net_sockets[i] == nullptr) {
            scripting->_net_sockets[i] = sock;
            *new_SocketAPM(L) = scripting->_net_sockets[i];
            return 1;
        }
    }

    // the in-process code leaks sock here; it is deleted, as nothing holds it
    delete sock;
    return luaL_argerror(L, 1, "no sockets available");
}

// sock:close(): SocketAPM_close(). In-process the userdata's pointer becomes null and a later
// call dereferences it; here the handle is forgotten, and a later call is "M4: stale SocketAPM"
int SocketAPM_close(lua_State *L)
{
    binding_argcheck(L, 1);

    SocketAPM *ud = *check_SocketAPM(L, 1);

    auto *scripting = AP::scripting();

    // clear allocated socket
    for (uint8_t i=0; i<SCRIPTING_MAX_NUM_NET_SOCKET; i++) {
        if (scripting->_net_sockets[i] == ud) {
            ud->close();
            delete ud;
            scripting->_net_sockets[i] = nullptr;
            (void)L->handles->forget(ud);
            break;
        }
    }

    return 0;
}

// sock:sendfile(file): SocketAPM_sendfile(). The M4's wrapper (wrap_SocketAPM_sendfile) sends the
// file's id in this session's file table, in place of the Lua file the in-process code reads its
// fd from. On success AP_Networking owns the descriptor: the id leaves the table unclosed, and
// the script's file:close() then fails, as in-process where the fd becomes -1. At most
// SOCKET_SENDFILES_PER_SESSION per session
int SocketAPM_sendfile(lua_State *L)
{
    binding_argcheck(L, 2);

    SocketAPM *ud = *check_SocketAPM(L, 1);

    const int32_t id = luaL_checkinteger(L, 2);
    const int fd = L->files != nullptr ? L->files->lookup(id) : -1;

    const bool ret = socket_sendfile(sendfiles, fd, AP::network().get_semaphore(), [ud](int descriptor) {
        return AP::network().sendfile(ud, descriptor);
    });
    if (ret) {
        (void)L->files->remove(id);
    }

    lua_pushboolean(L, ret);
    return 1;
}

// sock:recv(count): SocketAPM_recv(), into a stack buffer in place of a luaL_Buffer: up to
// IO_CHUNK bytes per call, as in-process it returns up to count (a UDP datagram longer than
// IO_CHUNK is cut, recorded). SocketAPM::recv() never waits: it polls with a timeout of 0
int SocketAPM_recv(lua_State *L)
{
    binding_argcheck(L, 2);

    SocketAPM *ud = *check_SocketAPM(L, 1);

    const uint16_t count = get_uint16_t(L, 2);
    uint8_t data[IO_CHUNK];

    // read up to that number of bytes
    const auto ret = ud->recv(data, MIN(count, uint16_t(sizeof(data))), 0);
    if (ret < 0) {
        return 0; // error, return nil
    }

    int retcount = 1;

    lua_pushlstring(L, reinterpret_cast<const char *>(data), size_t(ret));

    // also push the address and port if available
    uint32_t ip_addr;
    uint16_t port;
    if (ud->last_recv_address(ip_addr, port)) {
        *new_uint32_t(L) = ip_addr;
        lua_pushinteger(L, port);
        retcount += 2;
    }

    return retcount;
}

// sock:accept(): SocketAPM_accept(); the accepted socket is non-blocking on the M7 too. As for
// Socket(), a refused handle's retry accepts another connection (recorded)
int SocketAPM_accept(lua_State *L)
{
    binding_argcheck(L, 1);

    SocketAPM *ud = *check_SocketAPM(L, 1);

    auto *scripting = AP::scripting();

    // find an empty slot
    for (uint8_t i=0; i<SCRIPTING_MAX_NUM_NET_SOCKET; i++) {
        if (scripting->_net_sockets[i] == nullptr) {
            scripting->_net_sockets[i] = socket_nonblocking(ud->accept(0));
            if (scripting->_net_sockets[i] == nullptr) {
                return 0;
            }
            *new_SocketAPM(L) = scripting->_net_sockets[i];
            return 1;
        }
    }

    // out of socket slots, return nil, caller can retry
    return 0;
}

// ipv4_addr_to_string(addr): SocketAPM_ipv4_addr_to_string()
int SocketAPM_ipv4_addr_to_string(lua_State *L)
{
    binding_argcheck(L, 1);
    const uint32_t ip_addr = get_uint32(L, 1, 0, UINT32_MAX);
    char buf[IP4_STR_LEN];
    const char *ret = SocketAPM::inet_addr_to_str(ip_addr, buf, sizeof(buf));
    if (ret == nullptr) {
        return 0;
    }
    lua_pushlstring(L, ret, strlen(ret));
    return 1;
}

// string_to_ipv4_addr(text): SocketAPM_string_to_ipv4_addr()
int SocketAPM_string_to_ipv4_addr(lua_State *L)
{
    binding_argcheck(L, 1);
    const char *str = luaL_checkstring(L, 1);
    *new_uint32_t(L) = SocketAPM::inet_str_to_addr(str);
    return 1;
}

// sock:send(data, len): the generated SocketAPM_send() with the length bounded by the string
// (socket_send()), and whether it would have blocked for the M4's wrapper
int SocketAPM_send_checked(lua_State *L)
{
    binding_argcheck(L, 3);
    SocketAPM *ud = *check_SocketAPM(L, 1);
    return socket_send(L, *ud);
}

// sock:sendto(data, len, addr, port): the generated SocketAPM_sendto() with the length bounded
// by the string (socket_sendto())
int SocketAPM_sendto_checked(lua_State *L)
{
    binding_argcheck(L, 5);
    SocketAPM *ud = *check_SocketAPM(L, 1);
    return socket_sendto(L, *ud);
}

// sock:pollin(timeout_ms): the generated SocketAPM_pollin() with the timeout clamped
int lua_m4_SocketAPM_pollin(lua_State *L)
{
    binding_argcheck(L, 2);
    SocketAPM *ud = *check_SocketAPM(L, 1);
    return socket_poll(L, *ud, false);
}

// sock:pollout(timeout_ms): the generated SocketAPM_pollout() with the timeout clamped
int lua_m4_SocketAPM_pollout(lua_State *L)
{
    binding_argcheck(L, 2);
    SocketAPM *ud = *check_SocketAPM(L, 1);
    return socket_poll(L, *ud, true);
}

// sock:set_blocking(blocking): the socket stays non-blocking (socket_set_blocking()); the M4's
// wrapper keeps the mode the script asked for
int lua_m4_SocketAPM_set_blocking(lua_State *L)
{
    binding_argcheck(L, 2);
    SocketAPM *ud = *check_SocketAPM(L, 1);
    return socket_set_blocking(L, *ud);
}

// what a socket is, as its descriptor reports it. SO_ACCEPTCONN is not available on every stack
// (macOS SITL answers ENOPROTOOPT): then a socket with a peer is a connected stream, and one
// without (listening, or not connected) is left alone
static SocketKind socket_kind(int fd)
{
    int value = 0;
    socklen_t length = sizeof(value);
    if (SOCKET_CALL(getsockopt)(fd, SOL_SOCKET, SO_TYPE, &value, &length) != 0 || value != SOCK_STREAM) {
        return SocketKind::Other;
    }
    value = 0;
    length = sizeof(value);
    if (SOCKET_CALL(getsockopt)(fd, SOL_SOCKET, SO_ACCEPTCONN, &value, &length) == 0) {
        return value != 0 ? SocketKind::Listening : SocketKind::Stream;
    }
    struct sockaddr_storage peer {};
    socklen_t peer_length = sizeof(peer);
    return SOCKET_CALL(getpeername)(fd, reinterpret_cast<struct sockaddr *>(&peer), &peer_length) == 0
        ? SocketKind::Stream : SocketKind::Other;
}

void sockets_end_session()
{
    auto *scripting = AP::scripting();
    // clear allocated sockets, with a reset in place of a FIN the stack may wait for
    for (uint8_t i = 0; i < SCRIPTING_MAX_NUM_NET_SOCKET; i++) {
        socket_end(scripting->_net_sockets[i], socket_kind, [](int fd) {
            struct linger reset {};
            reset.l_onoff = 1;
            reset.l_linger = 0;
            (void)SOCKET_CALL(setsockopt)(fd, SOL_SOCKET, SO_LINGER, &reset, sizeof(reset));
        });
    }
    // a file handed over stays with AP_Networking until it is sent, past the session's end
    // (Bench Validation: FatFs descriptors held across sessions)
    sendfiles.new_session();
}

}  // namespace ScriptingM4Remote

#endif  // AP_NETWORKING_SOCKETS_ENABLED
#endif  // AP_SCRIPTING_M4_ENABLED
