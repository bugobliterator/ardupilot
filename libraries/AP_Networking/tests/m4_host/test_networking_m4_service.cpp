#include <AP_gtest.h>
#include <chrono>
#include <thread>
#include "net_m4.h"
#include "net_sys_arch.h"
#include "AP_Networking_M4_Layout.h"
#include "lwip/api.h"
#include "lwip/sockets.h"

using namespace NetM4;

static Region test_region;
static std::atomic<uint32_t> test_generation;
uint8_t *net_platform_region() { return reinterpret_cast<uint8_t *>(&test_region); }
void net_platform_doorbell_wait(uint32_t timeout_ms) { std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms)); }
uint32_t net_platform_session() { return 1; }
bool net_platform_ppp_open() { return false; }
int32_t net_platform_ppp_read(uint8_t *, uint32_t, uint32_t) { return -1; }
int32_t net_platform_ppp_write(const uint8_t *, uint32_t) { return -1; }

static void wait_for(bool (*done)(), uint32_t ms)
{
    for (uint32_t i = 0; i < ms / 10 && !done(); i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

static bool stack_up()
{
    StatusFields s {};
    return read_status(test_region.status, s) && (s.flags & STATUS_STACK_UP);
}

// before the M4 publishes its header: nothing of the last session is sent or delivered
TEST(NetM4Service, AaSessionStartDropsTheLastSessionsBytes)
{
    reset_region(test_region, test_generation);
    ByteRing m7_tx, m4_rx;
    ASSERT_TRUE(m7_tx.attach(&test_region.ports[2].tx_control, test_region.ports[2].tx, PORT_RING_DATA));
    ASSERT_TRUE(m4_rx.attach(&test_region.ports[2].rx_control, test_region.ports[2].rx, PORT_RING_DATA));
    const uint8_t old_bytes[5] = {1, 2, 3, 4, 5};
    ASSERT_EQ(m7_tx.write(old_bytes, sizeof(old_bytes)), sizeof(old_bytes));   // for the last session's M4
    ASSERT_EQ(m4_rx.write(old_bytes, 3), 3U);                                  // the last session's M4 wrote
    net_m4_session_start();
    EXPECT_EQ(m7_tx.space(), PORT_RING_DATA);   // the M4 consumed nothing: its tail moved to the head
    ByteRing m7_rx;
    ASSERT_TRUE(m7_rx.attach(&test_region.ports[2].rx_control, test_region.ports[2].rx, PORT_RING_DATA));
    EXPECT_EQ(m7_rx.available(), 0U);
    EXPECT_FALSE(m7_rx.desynced());
}

// runs before a valid config starts the stack (gtest runs tests in file order)
TEST(NetM4Service, AbInvalidConfigStartsNothing)
{
    reset_region(test_region, test_generation);
    Config config {};
    config.magic = CONFIG_MAGIC;
    config.enabled = 1;
    config.ports[0].type = 9;                 // no such port type
    net_m4_configure(config);
    config.ports[0].type = 1;
    config.magic = 0;                         // the M7's "networking off" answer
    net_m4_configure(config);
    config.magic = CONFIG_MAGIC;
    config.enabled = 0;
    net_m4_configure(config);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    EXPECT_FALSE(stack_up());
    EXPECT_EQ(net_m4_threads_started(), 0U);
}

TEST(NetM4Service, UdpClientAndTcpServerOverLoopback)
{
    reset_region(test_region, test_generation);
    Config config {};
    config.magic = CONFIG_MAGIC;
    config.enabled = 1;
    config.ports[0] = { 1, 1, 15550, 0x7F000001 };   // UDP client to 127.0.0.1:15550
    config.ports[1] = { 4, 0, 15763, 0 };            // TCP server on 15763
    config.ports[2] = { 3, 0, 15900, 0x7F000001 };   // TCP client to 127.0.0.1:15900 (TcpClient...)
    config.ports[3] = { 2, 1, 15910, 0 };            // UDP server on 15910 (UdpServer...)
    // a peer socket on the same lwIP stack, bound before the service sends
    net_m4_configure(config);
    wait_for(stack_up, 2000);
    ASSERT_TRUE(stack_up());
    EXPECT_EQ(net_m4_threads_started(), 2U);   // tcpip and net_port; no PPP transport
    net_m4_configure(config);                  // once per session
    EXPECT_EQ(net_m4_threads_started(), 2U);
    const int udp = lwip_socket(AF_INET, SOCK_DGRAM, 0);
    const timeval two_seconds {2, 0};   // a lost datagram fails the test instead of hanging it
    lwip_setsockopt(udp, SOL_SOCKET, SO_RCVTIMEO, &two_seconds, sizeof(two_seconds));
    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = lwip_htons(15550);
    addr.sin_addr.s_addr = lwip_htonl(0x7F000001);
    ASSERT_EQ(lwip_bind(udp, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)), 0);

    ByteRing tx, rx;
    ASSERT_TRUE(tx.attach(&test_region.ports[0].tx_control, test_region.ports[0].tx, PORT_RING_DATA));
    ASSERT_TRUE(rx.attach(&test_region.ports[0].rx_control, test_region.ports[0].rx, PORT_RING_DATA));
    const uint8_t datagram[3] = {0xFD, 1, 2};
    ASSERT_TRUE(tx.write_record(datagram, sizeof(datagram)));
    uint8_t got[16];
    sockaddr_in from {};
    socklen_t from_len = sizeof(from);
    EXPECT_EQ(lwip_recvfrom(udp, got, sizeof(got), 0, reinterpret_cast<sockaddr *>(&from), &from_len), 3);
    EXPECT_EQ(lwip_sendto(udp, "back", 4, 0, reinterpret_cast<sockaddr *>(&from), from_len), 4);
    wait_for([] {
        ByteRing r;
        return r.attach(&test_region.ports[0].rx_control, test_region.ports[0].rx, PORT_RING_DATA) && r.available() == 4; }, 2000);
    EXPECT_EQ(rx.available(), 4U);

    const int tcp = lwip_socket(AF_INET, SOCK_STREAM, 0);
    lwip_setsockopt(tcp, SOL_SOCKET, SO_RCVTIMEO, &two_seconds, sizeof(two_seconds));
    addr.sin_port = lwip_htons(15763);
    ASSERT_EQ(lwip_connect(tcp, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)), 0);
    ByteRing tcp_tx;
    ASSERT_TRUE(tcp_tx.attach(&test_region.ports[1].tx_control, test_region.ports[1].tx, PORT_RING_DATA));
    tcp_tx.write(reinterpret_cast<const uint8_t *>("mavlink"), 7);
    EXPECT_EQ(lwip_recv(tcp, got, sizeof(got), 0), 7);
    // the status is written every 100 ms: wait for the connected state rather than read once
    wait_for([] { StatusFields s {}; return read_status(test_region.status, s) && s.port_state[1] == PORT_CONNECTED; }, 1000);
    StatusFields status {};
    ASSERT_TRUE(read_status(test_region.status, status));
    EXPECT_EQ(status.port_state[0], uint32_t(PORT_CONNECTED));   // a datagram both ways
    EXPECT_EQ(status.port_state[1], uint32_t(PORT_CONNECTED));
    EXPECT_EQ(status.port_state[2], uint32_t(PORT_CLOSED));   // nothing listens on 15900 yet
    EXPECT_EQ(status.port_state[3], uint32_t(PORT_OPEN));     // bound, no sender yet
    EXPECT_EQ(status.flags & STATUS_LINK_UP, 0U);   // no PPP: no link, loopback only
    EXPECT_EQ(status.session, 1U);
    EXPECT_EQ(status.magic, STATUS_MAGIC);
    // the heartbeat moves with every write
    const uint32_t heartbeat = status.heartbeat;
    wait_for([] { StatusFields s {}; return read_status(test_region.status, s) && s.heartbeat > 3; }, 1000);
    ASSERT_TRUE(read_status(test_region.status, status));
    EXPECT_GT(status.heartbeat, heartbeat);

    // a full RX ring: the M4 reads datagrams and drops them, counted, and never waits
    uint8_t filler[PORT_RING_DATA];
    memset(filler, 0, sizeof(filler));
    ByteRing m4_rx;   // the M4's own producer end, to fill the ring
    ASSERT_TRUE(m4_rx.attach(&test_region.ports[0].rx_control, test_region.ports[0].rx, PORT_RING_DATA));
    m4_rx.write(filler, m4_rx.space());
    for (uint8_t i = 0; i < 5; i++) {
        // spaced: loopback queues at most LWIP_LOOPBACK_MAX_PBUFS
        EXPECT_EQ(lwip_sendto(udp, "lost!", 5, 0, reinterpret_cast<sockaddr *>(&from), from_len), 5);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    wait_for([] { StatusFields s {}; return read_status(test_region.status, s) && s.port_rx_dropped[0] == 25; }, 1000);
    ASSERT_TRUE(read_status(test_region.status, status));
    EXPECT_EQ(status.port_rx_dropped[0], 25U);
    EXPECT_EQ(rx.available(), PORT_RING_DATA);
    // and the port goes on once the M7 reads
    rx.consumer_resync();
    EXPECT_EQ(lwip_sendto(udp, "again", 5, 0, reinterpret_cast<sockaddr *>(&from), from_len), 5);
    wait_for([] {
        ByteRing r;
        return r.attach(&test_region.ports[0].rx_control, test_region.ports[0].rx, PORT_RING_DATA) && r.available() == 5; }, 2000);
    EXPECT_EQ(rx.available(), 5U);
    lwip_close(udp);
    lwip_close(tcp);
}

static sockaddr_in loopback(uint16_t port)
{
    sockaddr_in addr {};
    addr.sin_len = sizeof(addr);
    addr.sin_family = AF_INET;
    addr.sin_port = lwip_htons(port);
    addr.sin_addr.s_addr = lwip_htonl(0x7F000001);
    return addr;
}

static uint32_t port_state(uint8_t port)
{
    StatusFields s {};
    return read_status(test_region.status, s) ? s.port_state[port] : 0xFFFFFFFFU;
}

// a refused connect is retried every second; once connected, bytes both ways; when the peer
// closes, the client connects again
TEST(NetM4Service, TcpClientRetriesConnectsAndReconnects)
{
    // refused while nothing listens: the port stays closed
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    EXPECT_EQ(port_state(2), uint32_t(PORT_CLOSED));
    const int listener = lwip_socket(AF_INET, SOCK_STREAM, 0);
    const timeval four_seconds {4, 0};
    lwip_setsockopt(listener, SOL_SOCKET, SO_RCVTIMEO, &four_seconds, sizeof(four_seconds));
    sockaddr_in addr = loopback(15900);
    ASSERT_EQ(lwip_bind(listener, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)), 0);
    ASSERT_EQ(lwip_listen(listener, 1), 0);
    // the next retry connects (within RETRY_MS of the socket set up again after the refusal)
    int peer = lwip_accept(listener, nullptr, nullptr);
    ASSERT_GE(peer, 0);
    lwip_setsockopt(peer, SOL_SOCKET, SO_RCVTIMEO, &four_seconds, sizeof(four_seconds));
    wait_for([] { return port_state(2) == PORT_CONNECTED; }, 3000);
    ASSERT_EQ(port_state(2), uint32_t(PORT_CONNECTED));

    ByteRing tx, rx;
    ASSERT_TRUE(tx.attach(&test_region.ports[2].tx_control, test_region.ports[2].tx, PORT_RING_DATA));
    ASSERT_TRUE(rx.attach(&test_region.ports[2].rx_control, test_region.ports[2].rx, PORT_RING_DATA));
    ASSERT_EQ(tx.write(reinterpret_cast<const uint8_t *>("ping"), 4), 4U);
    uint8_t got[16];
    EXPECT_EQ(lwip_recv(peer, got, sizeof(got), 0), 4);
    EXPECT_EQ(memcmp(got, "ping", 4), 0);
    EXPECT_EQ(lwip_send(peer, "pong!", 5, 0), 5);
    wait_for([] {
        ByteRing r;
        return r.attach(&test_region.ports[2].rx_control, test_region.ports[2].rx, PORT_RING_DATA) && r.available() == 5; }, 2000);
    ASSERT_EQ(rx.available(), 5U);
    EXPECT_EQ(rx.read(got, sizeof(got)), 5U);
    EXPECT_EQ(memcmp(got, "pong!", 5), 0);

    // the peer closes: the client closes too, then connects again
    lwip_close(peer);
    wait_for([] { return port_state(2) == PORT_CLOSED; }, 2000);
    EXPECT_EQ(port_state(2), uint32_t(PORT_CLOSED));
    peer = lwip_accept(listener, nullptr, nullptr);
    ASSERT_GE(peer, 0);
    wait_for([] { return port_state(2) == PORT_CONNECTED; }, 3000);
    EXPECT_EQ(port_state(2), uint32_t(PORT_CONNECTED));
    lwip_close(peer);
    lwip_close(listener);
}

// the UDP server answers its first sender; a stray datagram from another sender does not take
// the stream, a sender heard after 3 s of silence from the peer does. Datagrams over 512 B are
// delivered cut and the rest counted
TEST(NetM4Service, UdpServerKeepsItsPeer)
{
    const timeval short_wait {0, 300000};
    int sender[2];
    for (uint8_t i = 0; i < 2; i++) {
        sender[i] = lwip_socket(AF_INET, SOCK_DGRAM, 0);
        lwip_setsockopt(sender[i], SOL_SOCKET, SO_RCVTIMEO, &short_wait, sizeof(short_wait));
        sockaddr_in local = loopback(uint16_t(15920 + i));
        ASSERT_EQ(lwip_bind(sender[i], reinterpret_cast<sockaddr *>(&local), sizeof(local)), 0);
    }
    sockaddr_in server = loopback(15910);
    ByteRing tx, rx;
    ASSERT_TRUE(tx.attach(&test_region.ports[3].tx_control, test_region.ports[3].tx, PORT_RING_DATA));
    ASSERT_TRUE(rx.attach(&test_region.ports[3].rx_control, test_region.ports[3].rx, PORT_RING_DATA));
    uint8_t got[16];
    // which sender a record from the M7 reaches
    auto reply_goes_to = [&]() {
        const uint8_t record[1] = {0x55};
        EXPECT_TRUE(tx.write_record(record, sizeof(record)));
        for (uint8_t i = 0; i < 2; i++) {
            if (lwip_recv(sender[i], got, sizeof(got), 0) == 1) {
                return int(i);
            }
        }
        return -1;
    };
    EXPECT_EQ(lwip_sendto(sender[0], "a", 1, 0, reinterpret_cast<sockaddr *>(&server), sizeof(server)), 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(reply_goes_to(), 0);
    // a stray datagram from sender 1 is delivered, and the stream stays with sender 0
    EXPECT_EQ(lwip_sendto(sender[1], "b", 1, 0, reinterpret_cast<sockaddr *>(&server), sizeof(server)), 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(reply_goes_to(), 0);
    wait_for([] {
        ByteRing r;
        return r.attach(&test_region.ports[3].rx_control, test_region.ports[3].rx, PORT_RING_DATA) && r.available() == 2; }, 1000);
    EXPECT_EQ(rx.available(), 2U);
    // sender 0 silent for 3 s: sender 1 takes over
    std::this_thread::sleep_for(std::chrono::milliseconds(3100));
    EXPECT_EQ(lwip_sendto(sender[1], "b", 1, 0, reinterpret_cast<sockaddr *>(&server), sizeof(server)), 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(reply_goes_to(), 1);
    EXPECT_EQ(port_state(3), uint32_t(PORT_CONNECTED));

    // 600 B: 512 delivered, 88 counted
    rx.consumer_resync();
    StatusFields before {};
    ASSERT_TRUE(read_status(test_region.status, before));
    uint8_t big[600];
    memset(big, 0x3C, sizeof(big));
    EXPECT_EQ(lwip_sendto(sender[1], big, sizeof(big), 0, reinterpret_cast<sockaddr *>(&server), sizeof(server)), int(sizeof(big)));
    wait_for([] {
        ByteRing r;
        return r.attach(&test_region.ports[3].rx_control, test_region.ports[3].rx, PORT_RING_DATA) && r.available() == 512; }, 1000);
    EXPECT_EQ(rx.available(), 512U);
    wait_for([] { StatusFields s {}; return read_status(test_region.status, s) && s.port_rx_dropped[3] >= 88; }, 1000);
    StatusFields after {};
    ASSERT_TRUE(read_status(test_region.status, after));
    EXPECT_EQ(after.port_rx_dropped[3] - before.port_rx_dropped[3], 88U);
    lwip_close(sender[0]);
    lwip_close(sender[1]);
}

// a connection a Lua listener accepts is allocated on the tcpip thread, and must still get a
// 4-slot receive box
TEST(NetM4Service, AcceptedConnectionsGetFourSlotReceiveBoxes)
{
    struct netconn *listener = nullptr;
    std::thread lua([&listener] {
        set_lua_thread(os_self());
        listener = netconn_new(NETCONN_TCP);
        netconn_bind(listener, IP4_ADDR_ANY, 15800);
        netconn_listen_with_backlog(listener, 1);
        struct netconn *udp = netconn_new(NETCONN_UDP);
        EXPECT_EQ(mbox_depth(&udp->recvmbox), int(LUA_UDP_RECV_SLOTS));   // a Lua UDP socket
        netconn_delete(udp);
        set_lua_thread(nullptr);
    });
    lua.join();
    ASSERT_NE(listener, nullptr);
    const int client = lwip_socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_port = lwip_htons(15800);
    addr.sin_addr.s_addr = lwip_htonl(0x7F000001);
    ASSERT_EQ(lwip_connect(client, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)), 0);
    struct netconn *accepted = nullptr;
    ASSERT_EQ(netconn_accept(listener, &accepted), ERR_OK);
    EXPECT_EQ(mbox_depth(&accepted->recvmbox), int(TCP_RECV_SLOTS));
    struct netconn *m7_udp = netconn_new(NETCONN_UDP);                  // not the Lua thread
    EXPECT_EQ(mbox_depth(&m7_udp->recvmbox), int(UDP_RECV_SLOTS));
    netconn_delete(m7_udp);
    netconn_delete(accepted);
    netconn_delete(listener);
    lwip_close(client);
}

// last: the SITL fault path parks the net_port thread, which then leaves the region alone
TEST(NetM4Service, ZzParkStopsTheRegionWrites)
{
    ASSERT_TRUE(stack_up());
    net_m4_park();
    StatusFields before {};
    ASSERT_TRUE(read_status(test_region.status, before));
    std::this_thread::sleep_for(std::chrono::milliseconds(3 * STATUS_PERIOD_MS));
    StatusFields after {};
    ASSERT_TRUE(read_status(test_region.status, after));
    EXPECT_EQ(after.heartbeat, before.heartbeat);
    ByteRing tx;
    ASSERT_TRUE(tx.attach(&test_region.ports[0].tx_control, test_region.ports[0].tx, PORT_RING_DATA));
    const uint8_t datagram[2] = {1, 2};
    ASSERT_TRUE(tx.write_record(datagram, sizeof(datagram)));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_EQ(tx.space(), PORT_RING_DATA - 4);   // not consumed
}

AP_GTEST_MAIN()
