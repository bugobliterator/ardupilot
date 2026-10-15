#include <AP_gtest.h>

// networking-on-M4 builds only: the layout needs the CubeFramework headers
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED

#include <AP_Networking/AP_Networking_M4_Layout.h>
#include <AP_Networking/AP_Networking_M4_Config.h>
#include <thread>

using namespace NetM4;

static Region region;
static std::atomic<uint32_t> generation;

static void attach_pair(ByteRing &producer, ByteRing &consumer)
{
    reset_region(region, generation);
    producer.attach(&region.ports[0].tx_control, region.ports[0].tx, PORT_RING_DATA);
    consumer.attach(&region.ports[0].tx_control, region.ports[0].tx, PORT_RING_DATA);
}

TEST(NetM4Rings, LayoutMatchesTheBaseline)
{
    // offsets from addresses, not offsetof (Region holds std::atomic: -Winvalid-offsetof)
    const uintptr_t base = reinterpret_cast<uintptr_t>(&region);
    EXPECT_EQ(sizeof(Region), 16384U);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&region.ports[0]) - base, 256U);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&region.dds_tx) - base, 8960U);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&region.can_tx) - base, 13184U);
    EXPECT_EQ(reinterpret_cast<uintptr_t>(&region.spare) - base, 15360U);
    EXPECT_EQ(sizeof(RingControl), 64U);
}

TEST(NetM4Rings, BytesRoundTripAcrossTheWrap)
{
    ByteRing tx, rx;
    attach_pair(tx, rx);
    uint8_t in[700], out[700];
    for (uint32_t pass = 0; pass < 10; pass++) {
        for (uint32_t i = 0; i < sizeof(in); i++) {
            in[i] = uint8_t(pass * 7 + i);
        }
        ASSERT_EQ(tx.write(in, sizeof(in)), sizeof(in));
        ASSERT_EQ(rx.available(), sizeof(in));
        ASSERT_EQ(rx.read(out, sizeof(out)), sizeof(out));
        ASSERT_EQ(memcmp(in, out, sizeof(in)), 0);
    }
}

TEST(NetM4Rings, FullRingWritesWhatFits)
{
    ByteRing tx, rx;
    attach_pair(tx, rx);
    uint8_t big[1500] {};
    EXPECT_EQ(tx.write(big, sizeof(big)), PORT_RING_DATA);
    EXPECT_EQ(tx.space(), 0U);
    EXPECT_FALSE(tx.write_record(big, 1));
}

TEST(NetM4Rings, RecordsKeepTheirBoundaries)
{
    ByteRing tx, rx;
    attach_pair(tx, rx);
    const uint8_t a[3] = {1, 2, 3}, b[300] = {9};
    ASSERT_TRUE(tx.write_record(a, sizeof(a)));
    ASSERT_TRUE(tx.write_record(b, sizeof(b)));
    uint8_t out[300];
    EXPECT_EQ(rx.read_record(out, sizeof(out)), 3);
    EXPECT_EQ(rx.read_record(out, sizeof(out)), 300);
    EXPECT_EQ(rx.read_record(out, sizeof(out)), RECORD_NONE);
}

TEST(NetM4Rings, HostileHeadIsDetectedAndResynced)
{
    ByteRing tx, rx;
    attach_pair(tx, rx);
    // the peer writes a head that claims more than the ring holds
    region.ports[0].tx_control.head.value.store(1500);
    EXPECT_TRUE(rx.desynced());
    EXPECT_EQ(rx.available(), 0U);
    uint8_t out[16];
    EXPECT_EQ(rx.read(out, sizeof(out)), 0U);
    rx.consumer_resync();
    EXPECT_FALSE(rx.desynced());
    EXPECT_EQ(rx.available(), 0U);
    // a head outside the index range cannot be fixed by the consumer: the ring reads nothing
    // until the next region reset (a hostile M4 only silences its own ports)
    region.ports[0].tx_control.head.value.store(0xFFFFFFF0);
    rx.consumer_resync();
    EXPECT_TRUE(rx.desynced());
    EXPECT_EQ(rx.read(out, sizeof(out)), 0U);
}

TEST(NetM4Rings, HostileRecordLengthIsBad)
{
    ByteRing tx, rx;
    attach_pair(tx, rx);
    const uint8_t lie[2] = {0xFF, 0x7F};   // a record header claiming 32767 bytes
    ASSERT_EQ(tx.write(lie, sizeof(lie)), 2U);
    uint8_t out[300];
    EXPECT_EQ(rx.read_record(out, sizeof(out)), RECORD_BAD);
}

TEST(NetM4Rings, StatusSeqlock)
{
    StatusBlock block {};
    StatusFields in {}, out {};
    in.magic = STATUS_MAGIC;
    in.flags = STATUS_STACK_UP | STATUS_LINK_UP;
    in.ip = 0x0A000E02;
    write_status(block, in);
    ASSERT_TRUE(read_status(block, out));
    EXPECT_EQ(out.ip, 0x0A000E02U);
    // a writer that faulted mid-update leaves the sequence odd: never read as valid
    block.seq.store(block.seq.load() + 1);
    EXPECT_FALSE(read_status(block, out));
}

TEST(NetM4Rings, StatusValuesAreChecked)
{
    StatusBlock block {};
    StatusFields in {}, out {};
    in.magic = STATUS_MAGIC;
    in.port_state[2] = 7;            // no such state
    write_status(block, in);
    EXPECT_FALSE(read_status(block, out));
    in.port_state[2] = PORT_CONNECTED;
    in.flags = 0x80000000;           // an unknown flag
    write_status(block, in);
    EXPECT_FALSE(read_status(block, out));
    in.flags = 0;
    in.magic = 0;
    write_status(block, in);
    EXPECT_FALSE(read_status(block, out));
}

TEST(NetM4Rings, ConfigIsChecked)
{
    Config config {};
    config.magic = CONFIG_MAGIC;
    config.ports[0].type = 4;
    EXPECT_TRUE(config_valid(config));
    config.ports[0].type = 5;
    EXPECT_FALSE(config_valid(config));
    config.ports[0].type = 1;
    config.can_mcast_mask = 0x04;    // only buses 1 and 2 bridge
    EXPECT_FALSE(config_valid(config));
}

// a ring's data area with guard bytes on both sides
struct GuardedRing {
    RingControl control;
    uint8_t before[64];
    uint8_t data[PORT_RING_DATA];
    uint8_t after[64];
};
static const uint8_t GUARD = 0xA5;

static bool guards_intact(const GuardedRing &ring)
{
    for (uint32_t i = 0; i < sizeof(ring.before); i++) {
        if (ring.before[i] != GUARD || ring.after[i] != GUARD) {
            return false;
        }
    }
    return true;
}

TEST(NetM4Rings, HostileIndicesNeverCopy)
{
    // over the index range, or claiming more than the ring holds
    const uint32_t hostile[][2] = {
        {2048, 0}, {0, 2048}, {0xFFFFFFFF, 0}, {0, 0xFFFFFFF0}, {1500, 0}, {0, 1023}, {1025, 0},
    };
    static GuardedRing ring;
    for (const auto &indices : hostile) {
        memset(&ring.before, GUARD, sizeof(ring.before));
        memset(&ring.after, GUARD, sizeof(ring.after));
        memset(ring.data, 0x5A, sizeof(ring.data));
        ByteRing tx, rx;
        tx.attach(&ring.control, ring.data, PORT_RING_DATA);
        rx.attach(&ring.control, ring.data, PORT_RING_DATA);
        ring.control.head.value.store(indices[0]);
        ring.control.tail.value.store(indices[1]);
        uint8_t big[2048];
        memset(big, 0x11, sizeof(big));
        EXPECT_EQ(tx.space(), 0U);
        EXPECT_EQ(tx.write(big, sizeof(big)), 0U);
        EXPECT_FALSE(tx.write_record(big, 10));
        EXPECT_EQ(rx.available(), 0U);
        EXPECT_EQ(rx.read(big, sizeof(big)), 0U);
        EXPECT_EQ(rx.read_record(big, MAX_RECORD), RECORD_BAD);
        EXPECT_TRUE(guards_intact(ring));
        for (uint32_t i = 0; i < sizeof(ring.data); i++) {
            ASSERT_EQ(ring.data[i], 0x5A);
        }
        // the indices are the peer's to fix: nothing was stored
        EXPECT_EQ(ring.control.head.value.load(), indices[0]);
        EXPECT_EQ(ring.control.tail.value.load(), indices[1]);
    }
}

TEST(NetM4Rings, RacingPeerNeverOverflows)
{
    static GuardedRing ring;
    memset(&ring.before, GUARD, sizeof(ring.before));
    memset(&ring.after, GUARD, sizeof(ring.after));
    ring.control.head.value.store(0);
    ring.control.tail.value.store(0);
    ByteRing tx, rx;
    tx.attach(&ring.control, ring.data, PORT_RING_DATA);
    rx.attach(&ring.control, ring.data, PORT_RING_DATA);
    std::atomic<bool> stop {false};
    // the peer rewrites both indices with anything, as fast as it can
    std::thread peer([&]() {
        uint32_t x = 0x12345678;
        while (!stop.load()) {
            x = x * 1664525U + 1013904223U;
            ring.control.head.value.store((x >> 8) % 4100);
            x = x * 1664525U + 1013904223U;
            ring.control.tail.value.store((x & 1) ? (x >> 8) % 4100 : x);
        }
    });
    uint8_t buf[2048];
    memset(buf, 0x22, sizeof(buf));
    for (uint32_t i = 0; i < 200000; i++) {
        EXPECT_LE(tx.write(buf, sizeof(buf)), PORT_RING_DATA);
        tx.write_record(buf, MAX_RECORD);
        EXPECT_LE(rx.read(buf, sizeof(buf)), PORT_RING_DATA);
        EXPECT_LE(rx.read_record(buf, MAX_RECORD), int32_t(MAX_RECORD));
    }
    stop.store(true);
    peer.join();
    EXPECT_TRUE(guards_intact(ring));
}

TEST(NetM4Rings, RecordHeaderOverLimitsIsBad)
{
    ByteRing tx, rx;
    attach_pair(tx, rx);
    // a header claiming MAX_RECORD + 1, with that many bytes behind it
    uint8_t lie[2 + MAX_RECORD + 1] {};
    lie[0] = uint8_t(MAX_RECORD + 1);
    lie[1] = uint8_t((MAX_RECORD + 1) >> 8);
    ASSERT_EQ(tx.write(lie, sizeof(lie)), sizeof(lie));
    uint8_t out[1024];
    EXPECT_EQ(rx.read_record(out, sizeof(out)), RECORD_BAD);
    // a header claiming 200 bytes with 10 behind it
    attach_pair(tx, rx);
    uint8_t short_record[12] = {200, 0};
    ASSERT_EQ(tx.write(short_record, sizeof(short_record)), sizeof(short_record));
    EXPECT_EQ(rx.read_record(out, sizeof(out)), RECORD_BAD);
    // a header within the limits but over the reader's capacity
    attach_pair(tx, rx);
    const uint8_t record[20] {};
    ASSERT_TRUE(tx.write_record(record, sizeof(record)));
    EXPECT_EQ(rx.read_record(out, 19), RECORD_BAD);
}

TEST(NetM4Rings, ResetBumpsTheGeneration)
{
    std::atomic<uint32_t> owner {7};
    region.header.generation.store(1000);   // the M4's copy is never read back
    EXPECT_EQ(reset_region(region, owner), 8U);
    EXPECT_EQ(owner.load(), 8U);
    EXPECT_EQ(region.header.generation.load(), 8U);
    EXPECT_EQ(reset_region(region, owner), 9U);
    EXPECT_EQ(region.header.generation.load(), 9U);
}

TEST(NetM4Rings, RingAttachedBeforeAResetIsStale)
{
    std::atomic<uint32_t> owner {0};
    reset_region(region, owner);
    ByteRing tx, rx;
    tx.attach(&region.ports[1].rx_control, region.ports[1].rx, PORT_RING_DATA, &owner);
    rx.attach(&region.ports[1].rx_control, region.ports[1].rx, PORT_RING_DATA, &owner);
    const uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    ASSERT_EQ(tx.write(data, sizeof(data)), sizeof(data));
    reset_region(region, owner);
    // a new session's indices: the old attachment refuses them
    region.ports[1].rx_control.head.value.store(4);
    EXPECT_TRUE(tx.stale());
    EXPECT_TRUE(rx.stale());
    uint8_t out[16];
    EXPECT_EQ(tx.space(), 0U);
    EXPECT_EQ(tx.write(data, sizeof(data)), 0U);
    EXPECT_FALSE(tx.write_record(data, sizeof(data)));
    EXPECT_EQ(rx.available(), 0U);
    EXPECT_EQ(rx.read(out, sizeof(out)), 0U);
    EXPECT_EQ(rx.read_record(out, sizeof(out)), RECORD_BAD);
    rx.consumer_resync();
    tx.producer_reset();
    EXPECT_EQ(region.ports[1].rx_control.head.value.load(), 4U);
    EXPECT_EQ(region.ports[1].rx_control.tail.value.load(), 0U);
    // attached again: the new session's data
    rx.attach(&region.ports[1].rx_control, region.ports[1].rx, PORT_RING_DATA, &owner);
    EXPECT_FALSE(rx.stale());
    EXPECT_EQ(rx.available(), 4U);
}

#endif  // AP_NETWORKING_M4_ENABLED

AP_GTEST_MAIN()
