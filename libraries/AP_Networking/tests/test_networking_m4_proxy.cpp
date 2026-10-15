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

TEST(NetM4Proxy, RegionResetClearsEveryPortAndRing)
{
    static Region shared;
    AP_Networking &net = AP::network();
    net.m4_region_reset(reinterpret_cast<uint8_t *>(&shared));
    for (uint8_t i = 0; i < NUM_PORTS; i++) {
        ByteRing tx;
        ASSERT_TRUE(tx.attach(&shared.ports[i].tx_control, shared.ports[i].tx, PORT_RING_DATA));
        const uint8_t stale[4] = {1, 2, 3, 4};
        tx.write(stale, sizeof(stale));
    }
    EXPECT_GT(net.m4_queued_bytes_for_test(), 0U);
    net.m4_region_reset(reinterpret_cast<uint8_t *>(&shared));   // what grant_restart() calls first
    EXPECT_EQ(net.m4_queued_bytes_for_test(), 0U);
    for (uint8_t i = 0; i < NUM_PORTS; i++) {
        ByteRing rx;
        ASSERT_TRUE(rx.attach(&shared.ports[i].tx_control, shared.ports[i].tx, PORT_RING_DATA));
        EXPECT_EQ(rx.available(), 0U) << unsigned(i);
    }
}

TEST(NetM4Proxy, SessionChangeDropsTheRingsWithoutAReset)
{
    static Region shared;
    AP_Networking &net = AP::network();
    net.m4_region_reset(reinterpret_cast<uint8_t *>(&shared));
    net.m4_io_update(0);
    // the M4 of session 0 left bytes for the M7, and the M7 queued bytes it never took
    ByteRing m4_rx, m7_tx;
    ASSERT_TRUE(m4_rx.attach(&shared.ports[1].rx_control, shared.ports[1].rx, PORT_RING_DATA));
    ASSERT_TRUE(m7_tx.attach(&shared.ports[2].tx_control, shared.ports[2].tx, PORT_RING_DATA));
    const uint8_t stale[6] = {1, 2, 3, 4, 5, 6};
    m4_rx.write(stale, sizeof(stale));
    m7_tx.write(stale, sizeof(stale));
    net.m4_io_update(0);
    EXPECT_EQ(net.m4_queued_bytes_for_test(), 2 * sizeof(stale));
    // a restart that kept the region: session 1 starts with the M7's ends empty
    net.m4_io_update(1);
    EXPECT_EQ(net.m4_queued_bytes_for_test(), 0U);
    EXPECT_EQ(shared.ports[1].rx_control.tail.value.load(), shared.ports[1].rx_control.head.value.load());
    EXPECT_EQ(shared.ports[2].tx_control.head.value.load(), shared.ports[2].tx_control.tail.value.load());
}

#endif  // AP_NETWORKING_M4_ENABLED

AP_GTEST_MAIN()
