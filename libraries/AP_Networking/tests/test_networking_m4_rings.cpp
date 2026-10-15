#include <AP_gtest.h>

// networking-on-M4 builds only: the layout needs the CubeFramework headers
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED

#include <AP_Networking/AP_Networking_M4_Layout.h>
#include <AP_Networking/AP_Networking_M4_Config.h>

using namespace NetM4;

static Region region;

static void attach_pair(ByteRing &producer, ByteRing &consumer)
{
    reset_region(region);
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

#endif  // AP_NETWORKING_M4_ENABLED

AP_GTEST_MAIN()
