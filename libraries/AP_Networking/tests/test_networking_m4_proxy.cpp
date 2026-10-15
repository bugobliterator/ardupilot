#include <AP_gtest.h>

// networking-on-M4 builds only: the layout needs the CubeFramework headers
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED

#include <AP_Networking/AP_Networking_M4.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

using namespace NetM4;

static Region region;
static std::atomic<uint32_t> generation;

static void rings(ByteRing &m7, ByteRing &m4, bool tx)
{
    reset_region(region, generation);
    RingControl *control = tx ? &region.ports[0].tx_control : &region.ports[0].rx_control;
    uint8_t *data = tx ? region.ports[0].tx : region.ports[0].rx;
    ASSERT_TRUE(m7.attach(control, data, PORT_RING_DATA, &generation));
    ASSERT_TRUE(m4.attach(control, data, PORT_RING_DATA));
}

static uint16_t heartbeat(uint8_t *buf, uint8_t seq)
{
    // a channel status of its own: the test has no GCS for mavlink_get_channel_status()
    mavlink_status_t status {};
    status.current_tx_seq = seq;
    mavlink_heartbeat_t packet {};
    packet.type = MAV_TYPE_QUADROTOR;
    packet.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    mavlink_message_t msg {};
    memcpy(_MAV_PAYLOAD_NON_CONST(&msg), &packet, sizeof(packet));
    msg.msgid = MAVLINK_MSG_ID_HEARTBEAT;
    mavlink_finalize_message_buffer(&msg, 1, 1, &status, MAVLINK_MSG_ID_HEARTBEAT_MIN_LEN,
                                    MAVLINK_MSG_ID_HEARTBEAT_LEN, MAVLINK_MSG_ID_HEARTBEAT_CRC);
    return mavlink_msg_to_send_buffer(buf, &msg);
}

TEST(NetM4Proxy, UdpFlushSendsOneMavlinkFramePerRecord)
{
    ByteRing m7, m4;
    rings(m7, m4, true);
    ByteBuffer writebuffer(4096);
    uint8_t frame[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = heartbeat(frame, 1);
    writebuffer.write(frame, len);
    writebuffer.write(frame, len);
    EXPECT_EQ(flush_tx(writebuffer, m7, true), uint32_t(2 * len));
    // mavlink_packetise(): one frame per datagram, as the socket ports send them
    uint8_t out[MAX_RECORD];
    EXPECT_EQ(m4.read_record(out, sizeof(out)), int32_t(len));
    EXPECT_EQ(m4.read_record(out, sizeof(out)), int32_t(len));
    EXPECT_EQ(memcmp(out, frame, len), 0);
    EXPECT_EQ(writebuffer.available(), 0U);
    // more than the ring holds: every record is one whole frame, the rest waits
    const uint16_t frames = (3 * PORT_RING_DATA) / len;
    for (uint16_t i = 0; i < frames; i++) {
        writebuffer.write(frame, len);
    }
    flush_tx(writebuffer, m7, true);
    uint32_t total = 0;
    int32_t got;
    while ((got = m4.read_record(out, sizeof(out))) > 0) {
        EXPECT_EQ(got, int32_t(len)) << "a record cut a MAVLink frame";
        total += got;
    }
    EXPECT_EQ(writebuffer.available() % len, 0U);
    EXPECT_EQ(total + writebuffer.available(), uint32_t(frames * len));
    EXPECT_GT(total, uint32_t(MAX_RECORD));
}

TEST(NetM4Proxy, UdpFlushWithoutMavlinkSendsWholeRecords)
{
    ByteRing m7, m4;
    rings(m7, m4, true);
    ByteBuffer writebuffer(2048);
    uint8_t bytes[700];
    for (uint16_t i = 0; i < sizeof(bytes); i++) {
        bytes[i] = uint8_t(i);   // holds MAVLink start bytes: no frame cutting applies
    }
    writebuffer.write(bytes, sizeof(bytes));
    EXPECT_EQ(flush_tx(writebuffer, m7, true, false), uint32_t(sizeof(bytes)));
    uint8_t out[MAX_RECORD];
    EXPECT_EQ(m4.read_record(out, sizeof(out)), int32_t(MAX_RECORD));
    EXPECT_EQ(m4.read_record(out, sizeof(out)), int32_t(MAX_RECORD));
    EXPECT_EQ(m4.read_record(out, sizeof(out)), 100);
    EXPECT_EQ(out[0], uint8_t(600));
}

TEST(NetM4Proxy, UdpFlushStopsWhenTheRecordDoesNotFit)
{
    ByteRing m7, m4;
    rings(m7, m4, true);
    ByteBuffer writebuffer(4096);
    uint8_t bytes[1500] {};
    writebuffer.write(bytes, sizeof(bytes));
    // three 302 B records fit 1024 B; the fourth waits whole
    EXPECT_EQ(flush_tx(writebuffer, m7, true, false), uint32_t(3 * MAX_RECORD));
    EXPECT_EQ(writebuffer.available(), sizeof(bytes) - 3 * MAX_RECORD);
}

TEST(NetM4Proxy, TcpFlushMovesWhatFits)
{
    ByteRing m7, m4;
    rings(m7, m4, true);
    ByteBuffer writebuffer(2048);
    uint8_t bytes[1500] {};
    writebuffer.write(bytes, sizeof(bytes));
    EXPECT_EQ(flush_tx(writebuffer, m7, false), PORT_RING_DATA);
    EXPECT_EQ(writebuffer.available(), sizeof(bytes) - PORT_RING_DATA);   // the rest waits, nothing lost here
}

TEST(NetM4Proxy, DrainResyncsAHostileRing)
{
    ByteRing m7, m4;
    rings(m7, m4, false);
    const uint8_t hello[5] = {'h', 'e', 'l', 'l', 'o'};
    m4.write(hello, sizeof(hello));
    ByteBuffer readbuffer(2048);
    EXPECT_EQ(drain_rx(m7, readbuffer), 5U);
    region.ports[0].rx_control.head.value.store(1500);         // the M4 claims more bytes than fit
    EXPECT_EQ(drain_rx(m7, readbuffer), 0U);
    EXPECT_FALSE(m7.desynced());                               // drain_rx resynced it
    EXPECT_EQ(readbuffer.available(), 5U);
}

TEST(NetM4Proxy, DrainStopsAtAFullReadBuffer)
{
    ByteRing m7, m4;
    rings(m7, m4, false);
    uint8_t bytes[600] {};
    m4.write(bytes, sizeof(bytes));
    ByteBuffer readbuffer(512);
    const uint32_t room = readbuffer.space();
    EXPECT_EQ(drain_rx(m7, readbuffer), room);
    EXPECT_EQ(m7.available(), sizeof(bytes) - room);   // the rest stays in the ring
}

TEST(NetM4Proxy, RegionResetDropsQueuedBytes)
{
    ByteRing m7, m4;
    rings(m7, m4, true);
    ByteBuffer readbuffer(2048), writebuffer(2048);
    uint8_t bytes[100] {};
    writebuffer.write(bytes, sizeof(bytes));
    readbuffer.write(bytes, 40);
    flush_tx(writebuffer, m7, false);
    writebuffer.write(bytes, 10);
    // a fault restart: nothing queued before it may be delivered after it
    EXPECT_EQ(reset_port(readbuffer, writebuffer), 50U);
    reset_region(region, generation);
    EXPECT_EQ(readbuffer.available(), 0U);
    EXPECT_EQ(writebuffer.available(), 0U);
    EXPECT_EQ(m4.available(), 0U);
}

static StatusFields status_of(uint32_t session, uint32_t change_count, uint32_t flags)
{
    StatusFields fields {};
    fields.magic = STATUS_MAGIC;
    fields.session = session;
    fields.flags = flags;
    fields.ip = 0x0A000002;
    fields.netmask = 0xFFFFFF00;
    fields.gateway = 0x0A000001;
    fields.change_count = change_count;
    return fields;
}

TEST(NetM4Proxy, StatusOfTheSessionGivesAddressesAndLinkUp)
{
    StatusBlock block {};
    StatusTracker tracker {};
    StatusFields out {};
    write_status(block, status_of(3, 1, STATUS_STACK_UP | STATUS_LINK_UP));
    EXPECT_EQ(track_status(tracker, block, 3, out), STATUS_NEW_SESSION | STATUS_ADDRESSES | STATUS_LINK_CAME_UP);
    EXPECT_EQ(out.ip, 0x0A000002U);
    EXPECT_EQ(out.gateway, 0x0A000001U);
    // unchanged: nothing to announce
    EXPECT_EQ(track_status(tracker, block, 3, out), 0);
    write_status(block, status_of(3, 2, STATUS_STACK_UP | STATUS_LINK_UP));
    EXPECT_EQ(track_status(tracker, block, 3, out), STATUS_ADDRESSES);
}

TEST(NetM4Proxy, StatusOfAnotherSessionIsIgnored)
{
    StatusBlock block {};
    StatusTracker tracker {};
    StatusFields out {};
    write_status(block, status_of(3, 1, STATUS_LINK_UP));
    EXPECT_EQ(track_status(tracker, block, 3, out), STATUS_NEW_SESSION | STATUS_ADDRESSES | STATUS_LINK_CAME_UP);
    // the supervisor granted session 4; the block still holds session 3's
    StatusFields stale {};
    EXPECT_EQ(track_status(tracker, block, 4, stale), STATUS_NEW_SESSION);
    EXPECT_EQ(stale.ip, 0U);
    EXPECT_FALSE(tracker.have_status);
    // the new session publishes: its link comes up again
    write_status(block, status_of(4, 1, STATUS_LINK_UP));
    EXPECT_EQ(track_status(tracker, block, 4, out), STATUS_ADDRESSES | STATUS_LINK_CAME_UP);
}

TEST(NetM4Proxy, TornOrBadStatusKeepsTheLastCopy)
{
    StatusBlock block {};
    StatusTracker tracker {};
    StatusFields out {};
    write_status(block, status_of(1, 1, STATUS_LINK_UP));
    track_status(tracker, block, 1, out);
    block.seq.store(block.seq.load() | 1U);   // the M4 is writing, forever
    StatusFields torn {};
    EXPECT_EQ(track_status(tracker, block, 1, torn), 0);
    EXPECT_EQ(torn.ip, 0U);
    write_status(block, status_of(1, 2, 1U << 7));   // an unknown flag
    EXPECT_EQ(track_status(tracker, block, 1, torn), 0);
    EXPECT_TRUE(tracker.have_status);
    EXPECT_EQ(tracker.change_count, 1U);
}

// the frontend's reset path, through the singleton
static AP_Networking networking;

// AP_Networking's ports and counters (friend of AP_Networking and of its Port)
struct NetM4TestAccess {
    static AP_HAL::UARTDriver &uart(AP_Networking &net, uint8_t i) { return net.ports[i]; }
    static void open(AP_Networking &net, uint8_t i, AP_Networking::NetworkPortType type,
                     AP_SerialManager::SerialProtocol protocol)
    {
        AP_Networking::Port &port = net.ports[i];
        port.type = type;
        port.state.idx = AP_SERIALMANAGER_NET_PORT_1 + i;
        port.state.protocol.set(int8_t(protocol));
        ASSERT_TRUE(port.init_buffers(2048, 2048));
        port.m4_open();
    }
    static void udp_mavlink_and_tcp(AP_Networking &net)
    {
        open(net, 0, AP_Networking::NetworkPortType::UDP_CLIENT, AP_SerialManager::SerialProtocol_MAVLink2);
        open(net, 1, AP_Networking::NetworkPortType::TCP_CLIENT, AP_SerialManager::SerialProtocol_None);
        net.ports[2].type = AP_Networking::NetworkPortType::NONE;
        net.ports[3].type = AP_Networking::NetworkPortType::NONE;
    }
    static uint32_t tx_stats(AP_Networking &net, uint8_t i) { return net.ports[i].tx_stats_bytes; }
    static uint32_t rx_stats(AP_Networking &net, uint8_t i) { return net.ports[i].rx_stats_bytes; }
    static uint32_t doorbells(AP_Networking &net) { return net._m4_doorbells; }
    static uint32_t bytes_dropped(AP_Networking &net) { return net._m4_bytes_dropped; }
    static uint32_t tx_desyncs(AP_Networking &net) { return net._m4_tx_desyncs; }
    static bool held(AP_Networking &net) { return net._m4_held; }
    static uint32_t attached_session(AP_Networking &net) { return net._m4_attached_session; }
};

static Region shared;

// a region, the two test ports, and a confirmed session
static AP_Networking &started(uint32_t session)
{
    AP_Networking &net = AP::network();
    NetM4TestAccess::udp_mavlink_and_tcp(net);
    net.m4_region_reset(reinterpret_cast<uint8_t *>(&shared));
    net.m4_io_update(session, true);
    return net;
}

// the M4's end of port i's rings
static void peer(uint8_t i, ByteRing &tx, ByteRing &rx)
{
    ASSERT_TRUE(tx.attach(&shared.ports[i].tx_control, shared.ports[i].tx, PORT_RING_DATA));
    ASSERT_TRUE(rx.attach(&shared.ports[i].rx_control, shared.ports[i].rx, PORT_RING_DATA));
}

TEST(NetM4Proxy, RegionResetClearsEveryPortAndRing)
{
    AP_Networking &net = started(0);
    for (uint8_t i = 0; i < NUM_PORTS; i++) {
        ByteRing tx;
        ASSERT_TRUE(tx.attach(&shared.ports[i].tx_control, shared.ports[i].tx, PORT_RING_DATA));
        const uint8_t stale[4] = {1, 2, 3, 4};
        tx.write(stale, sizeof(stale));
    }
    const uint8_t queued[3] = {7, 8, 9};
    NetM4TestAccess::uart(net, 1).write(queued, sizeof(queued));
    EXPECT_GT(net.m4_queued_bytes_for_test(), 0U);
    net.m4_region_reset(reinterpret_cast<uint8_t *>(&shared));   // what grant_restart() calls first
    net.m4_io_update(1, true);                                  // the new session confirmed
    EXPECT_EQ(net.m4_queued_bytes_for_test(), 0U);
    for (uint8_t i = 0; i < NUM_PORTS; i++) {
        ByteRing rx;
        ASSERT_TRUE(rx.attach(&shared.ports[i].tx_control, shared.ports[i].tx, PORT_RING_DATA));
        EXPECT_EQ(rx.available(), 0U) << unsigned(i);
    }
}

TEST(NetM4Proxy, PortsFlushByTheirType)
{
    AP_Networking &net = started(0);
    ByteRing udp_tx, udp_rx, tcp_tx, tcp_rx;
    peer(0, udp_tx, udp_rx);
    peer(1, tcp_tx, tcp_rx);
    uint8_t frame[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = heartbeat(frame, 3);
    AP_HAL::UARTDriver &udp = NetM4TestAccess::uart(net, 0);
    AP_HAL::UARTDriver &tcp = NetM4TestAccess::uart(net, 1);
    EXPECT_EQ(udp.write(frame, len), len);
    EXPECT_EQ(udp.write(frame, len), len);
    const uint8_t bytes[5] = {'b', 'y', 't', 'e', 's'};
    EXPECT_EQ(tcp.write(bytes, sizeof(bytes)), sizeof(bytes));
    const uint32_t udp_sent = NetM4TestAccess::tx_stats(net, 0);
    const uint32_t tcp_sent = NetM4TestAccess::tx_stats(net, 1);
    net.m4_io_update(0, true);
    // MAVLink UDP: one frame per record
    uint8_t out[MAX_RECORD];
    EXPECT_EQ(udp_tx.read_record(out, sizeof(out)), int32_t(len));
    EXPECT_EQ(udp_tx.read_record(out, sizeof(out)), int32_t(len));
    EXPECT_EQ(udp_tx.read_record(out, sizeof(out)), RECORD_NONE);
    // TCP: the bytes as they are, no record header
    EXPECT_EQ(tcp_tx.available(), sizeof(bytes));
    EXPECT_EQ(tcp_tx.read(out, sizeof(out)), sizeof(bytes));
    EXPECT_EQ(memcmp(out, bytes, sizeof(bytes)), 0);
    EXPECT_EQ(NetM4TestAccess::tx_stats(net, 0) - udp_sent, uint32_t(2 * len));
    EXPECT_EQ(NetM4TestAccess::tx_stats(net, 1) - tcp_sent, sizeof(bytes));
    // RX: into the port's read buffer, counted
    const uint32_t tcp_received = NetM4TestAccess::rx_stats(net, 1);
    tcp_rx.write(bytes, sizeof(bytes));
    net.m4_io_update(0, true);
    EXPECT_EQ(tcp.available(), sizeof(bytes));
    EXPECT_EQ(tcp.read(out, sizeof(out)), ssize_t(sizeof(bytes)));
    EXPECT_EQ(NetM4TestAccess::rx_stats(net, 1) - tcp_received, sizeof(bytes));
}

TEST(NetM4Proxy, DoorbellOnlyWhenATxRingGetsDataFromEmpty)
{
    AP_Networking &net = started(0);
    ByteRing tcp_tx, tcp_rx;
    peer(1, tcp_tx, tcp_rx);
    AP_HAL::UARTDriver &tcp = NetM4TestAccess::uart(net, 1);
    const uint8_t bytes[4] = {1, 2, 3, 4};
    const uint32_t before = NetM4TestAccess::doorbells(net);
    net.m4_io_update(0, true);
    EXPECT_EQ(NetM4TestAccess::doorbells(net), before);          // nothing sent
    tcp.write(bytes, sizeof(bytes));
    net.m4_io_update(0, true);
    EXPECT_EQ(NetM4TestAccess::doorbells(net), before + 1);      // empty -> data
    tcp.write(bytes, sizeof(bytes));
    net.m4_io_update(0, true);
    EXPECT_EQ(NetM4TestAccess::doorbells(net), before + 1);      // the M4 has not read yet
    uint8_t out[16];
    EXPECT_EQ(tcp_tx.read(out, sizeof(out)), 8U);
    tcp.write(bytes, sizeof(bytes));
    net.m4_io_update(0, true);
    EXPECT_EQ(NetM4TestAccess::doorbells(net), before + 2);
}

TEST(NetM4Proxy, SessionChangeDropsThePortsBytesOnceConfirmed)
{
    AP_Networking &net = started(0);
    AP_HAL::UARTDriver &tcp = NetM4TestAccess::uart(net, 1);
    ByteRing tcp_tx, tcp_rx;
    peer(1, tcp_tx, tcp_rx);
    const uint8_t bytes[10] {};
    tcp_rx.write(bytes, 6);
    net.m4_io_update(0, true);
    EXPECT_EQ(tcp.available(), 6U);
    // the grant moves on: the port holds, nothing is reset yet
    net.m4_io_update(1, false);
    EXPECT_EQ(tcp.available(), 0U);
    EXPECT_EQ(tcp.write(bytes, sizeof(bytes)), 0U);
    EXPECT_EQ(tcp.txspace(), 0U);
    const uint32_t dropped = NetM4TestAccess::bytes_dropped(net);
    // confirmed: the held read buffer is dropped and counted
    net.m4_io_update(1, true);
    EXPECT_EQ(NetM4TestAccess::bytes_dropped(net) - dropped, 6U);
    EXPECT_EQ(tcp.available(), 0U);
    EXPECT_EQ(tcp.write(bytes, sizeof(bytes)), sizeof(bytes));
}

TEST(NetM4Proxy, OldPeerAfterTheGrantNeverReachesTheNewSession)
{
    AP_Networking &net = started(0);
    AP_HAL::UARTDriver &tcp = NetM4TestAccess::uart(net, 1);
    ByteRing old_tx, old_rx;   // the M4 of session 0, still running after the grant
    peer(1, old_tx, old_rx);
    const uint8_t old_bytes[4] = {'o', 'l', 'd', '!'};
    old_rx.write(old_bytes, sizeof(old_bytes));
    net.m4_io_update(0, true);
    net.m4_io_update(1, false);   // grant 1, not running yet
    for (uint8_t pass = 0; pass < 3; pass++) {
        old_rx.write(old_bytes, sizeof(old_bytes));
        const uint8_t new_out[3] = {'n', 'e', 'w'};
        EXPECT_EQ(tcp.write(new_out, sizeof(new_out)), 0U);   // held: no new-session TX at all
        net.m4_io_update(1, false);
        uint8_t taken[16];
        EXPECT_EQ(old_tx.read(taken, sizeof(taken)), 0U);     // the old peer gets nothing
        EXPECT_EQ(tcp.available(), 0U);
    }
    // session 1 confirmed; its M4 writes its own bytes
    net.m4_io_update(1, true);
    ByteRing new_tx, new_rx;
    peer(1, new_tx, new_rx);
    const uint8_t new_in[5] = {'f', 'r', 'e', 's', 'h'};
    new_rx.write(new_in, sizeof(new_in));
    const uint8_t new_out[3] = {'n', 'e', 'w'};
    EXPECT_EQ(tcp.write(new_out, sizeof(new_out)), sizeof(new_out));
    net.m4_io_update(1, true);
    uint8_t in[32];
    ASSERT_EQ(tcp.available(), sizeof(new_in));
    EXPECT_EQ(tcp.read(in, sizeof(in)), ssize_t(sizeof(new_in)));
    EXPECT_EQ(memcmp(in, new_in, sizeof(new_in)), 0);
    uint8_t out[32];
    EXPECT_EQ(new_tx.read(out, sizeof(out)), sizeof(new_out));
    EXPECT_EQ(memcmp(out, new_out, sizeof(new_out)), 0);
}

TEST(NetM4Proxy, DesyncedTxRingIsCountedAndLeftToTheM4)
{
    AP_Networking &net = started(0);
    AP_HAL::UARTDriver &tcp = NetM4TestAccess::uart(net, 1);
    // the M4 stores a tail more than a ring's length behind the head
    shared.ports[1].tx_control.tail.value.store(500);
    const uint32_t head = shared.ports[1].tx_control.head.value.load();
    const uint32_t desyncs = NetM4TestAccess::tx_desyncs(net);
    const uint8_t bytes[4] = {1, 2, 3, 4};
    EXPECT_EQ(tcp.write(bytes, sizeof(bytes)), sizeof(bytes));
    for (uint8_t pass = 0; pass < 3; pass++) {
        net.m4_io_update(0, true);
    }
    // counted once, the head untouched, the bytes waiting in the port
    EXPECT_EQ(NetM4TestAccess::tx_desyncs(net) - desyncs, 1U);
    EXPECT_EQ(shared.ports[1].tx_control.head.value.load(), head);
    // the M4's consumer resyncs: the bytes flow
    ByteRing m4_tx, m4_rx;
    peer(1, m4_tx, m4_rx);
    m4_tx.consumer_resync();
    net.m4_io_update(0, true);
    EXPECT_FALSE(m4_tx.desynced());
    EXPECT_EQ(m4_tx.available(), sizeof(bytes));
}

// a grant source whose grant moves on at a chosen point of the pass
struct RacingGrant {
    mutable uint32_t grant;
    uint32_t confirmed_grant;     // the grant the M4 confirmed
    bool bump_after_current;      // the grant lands between the two reads
    uint32_t current() const
    {
        const uint32_t now = grant;
        if (bump_after_current) {
            grant++;
        }
        return now;
    }
    bool confirmed(uint32_t session) const { return session == confirmed_grant; }
};

TEST(NetM4Proxy, GrantLandingDuringAPassNeverRunsItUnheld)
{
    AP_Networking &net = started(0);
    ByteRing old_tx, old_rx;
    peer(1, old_tx, old_rx);
    AP_HAL::UARTDriver &tcp = NetM4TestAccess::uart(net, 1);
    // grant 1 lands after the pass read grant 0: the pass is one more ordinary pass for session 0
    RacingGrant during {0, 0, true};
    net.m4_io_update_from(during);
    EXPECT_FALSE(NetM4TestAccess::held(net));
    EXPECT_EQ(NetM4TestAccess::attached_session(net), 0U);
    // the next pass reads grant 1, which only session 0 is confirmed for: held, nothing moves
    const uint8_t old_bytes[3] = {'o', 'l', 'd'};
    old_rx.write(old_bytes, sizeof(old_bytes));
    RacingGrant after {during.grant, 0, false};
    net.m4_io_update_from(after);
    EXPECT_TRUE(NetM4TestAccess::held(net));
    EXPECT_EQ(NetM4TestAccess::attached_session(net), 0U);
    EXPECT_EQ(tcp.available(), 0U);
    EXPECT_EQ(old_rx.space(), PORT_RING_DATA - sizeof(old_bytes));   // not drained
}

#endif  // AP_NETWORKING_M4_ENABLED

AP_GTEST_MAIN()
