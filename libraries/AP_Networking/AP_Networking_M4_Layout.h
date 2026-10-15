#pragma once

/*
  The network region at the top of the M4 shared window (AP_NETWORKING_M4_RING_BASE, 16 KB):
  the status block the M4 writes, the NET_P byte rings, the DDS datagram pipe and the CAN
  frame ring. Shared by the M7 proxy (AP_Networking_M4), the M4 network service and the
  SITL M4 process. Everything the M4 writes is untrusted on the M7.
 */

#include <atomic>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <CubePilotFW/SpscRing.h>

namespace NetM4 {

static const uint8_t NUM_PORTS = 4;
static const uint32_t PORT_RING_DATA = 1024;
static const uint32_t DDS_RING_DATA = 2048;
static const uint32_t CAN_RING_DATA = 1024;
// a UDP port's TX ring carries records: uint16_t length, then the datagram
static const uint16_t MAX_RECORD = 300;
static const int32_t RECORD_NONE = -1;
static const int32_t RECORD_BAD = -2;
// a well-formed record longer than the reader's capacity: skipped and counted, read again
static const int32_t RECORD_DROPPED = -3;

struct alignas(32) RingIndex {
    std::atomic<uint32_t> value;
    uint8_t pad[28];
};
struct RingControl {
    RingIndex head;   // producer only
    RingIndex tail;   // consumer only
};
static_assert(sizeof(RingControl) == sizeof(CubePilotFW::SpscRing::Control), "one control layout");

// SPSC byte ring; indices count in [0, 2 * size) (mirrored), size a power of two. Every
// operation loads head and tail once, checks them and uses only those copies: the peer can
// rewrite either index at any time
class ByteRing {
public:
    // generation: on the M7, its own region generation (reset_region() bumps it); a ring attached
    // under one generation refuses every operation after a reset until it is attached again.
    // nullptr on the M4. False, and every operation refuses, when size is not a power of two
    // in [1, 2^31]
    bool attach(RingControl *control, uint8_t *data, uint32_t size,
                const std::atomic<uint32_t> *generation = nullptr);
    // producer
    uint32_t space() const;
    uint32_t write(const uint8_t *src, uint32_t len);
    bool write_record(const uint8_t *src, uint16_t len);
    void producer_reset();
    // consumer
    uint32_t available() const;
    uint32_t read(uint8_t *dst, uint32_t len);
    // a record's length, RECORD_NONE when no whole header is queued, RECORD_BAD when the ring
    // is desynced, stale or the header claims more than MAX_RECORD or than is queued (resync),
    // RECORD_DROPPED when a well-formed record is longer than capacity (skipped and counted)
    int32_t read_record(uint8_t *dst, uint16_t capacity);
    uint32_t records_dropped() const { return _records_dropped; }
    void consumer_resync();
    // the indices are out of range, or the ring is stale
    bool desynced() const;
    // the region was reset after attach()
    bool stale() const;

private:
    uint32_t used(uint32_t head, uint32_t tail) const { return (head - tail) & _index_mask; }
    bool valid(uint32_t head, uint32_t tail) const {
        return head <= _index_mask && tail <= _index_mask && used(head, tail) <= _size;
    }
    // attached with a valid size, and not stale
    bool usable() const;
    // one load of each index (acquire on the peer's, relaxed on ours); false: stale or out of range
    bool producer_snapshot(uint32_t &head, uint32_t &tail) const;
    bool consumer_snapshot(uint32_t &head, uint32_t &tail) const;
    void copy_in(uint32_t index, const uint8_t *src, uint32_t len);
    void copy_out(uint32_t index, uint8_t *dst, uint32_t len) const;
    RingControl *_control = nullptr;   // nullptr until attached with a valid size
    uint8_t *_data = nullptr;
    uint32_t _size = 0;
    uint32_t _index_mask = 0;   // 2 * size - 1
    const std::atomic<uint32_t> *_generation = nullptr;
    uint32_t _attached_generation = 0;
    uint32_t _records_dropped = 0;
};

static const uint32_t STATUS_MAGIC = 0x5453344E;   // "N4ST"
static const uint32_t STATUS_STACK_UP = 1U << 0;   // lwIP running
static const uint32_t STATUS_LINK_UP = 1U << 1;    // a netif with an address
static const uint32_t STATUS_DHCP = 1U << 2;       // the address came from DHCP
static const uint32_t STATUS_KNOWN_FLAGS = STATUS_STACK_UP | STATUS_LINK_UP | STATUS_DHCP;
enum PortState : uint32_t { PORT_CLOSED = 0, PORT_OPEN = 1, PORT_CONNECTED = 2 };

struct StatusFields {
    uint32_t magic;
    uint32_t session;          // the M4's restart count (PeerHeader restarts)
    uint32_t flags;
    uint32_t ip;               // host byte order
    uint32_t netmask;
    uint32_t gateway;
    uint32_t change_count;     // bumped when ip, netmask, gateway or a flag changes
    uint32_t port_state[NUM_PORTS];
    uint32_t port_rx_dropped[NUM_PORTS];   // bytes the M4 could not put in a full RX ring
    uint32_t dds_tx, dds_rx, can_tx, can_rx;
    uint32_t pbuf_free;        // lwIP PBUF_POOL elements free
    uint32_t mem_free;         // lwIP heap bytes free
    uint32_t reserved[10];
};
struct StatusBlock {
    std::atomic<uint32_t> seq;   // odd while the M4 writes
    StatusFields fields;
};
static_assert(sizeof(StatusBlock) == 128, "status block layout");

// M4 only
void write_status(StatusBlock &block, const StatusFields &fields);
// M7: false if torn three times, or a value is out of range; the caller keeps its last copy
bool read_status(const StatusBlock &block, StatusFields &out);

template <uint32_t N>
struct RingArea {
    RingControl control;
    uint8_t data[N];
};
struct PortRings {
    RingControl tx_control;   // M7 -> M4
    uint8_t tx[PORT_RING_DATA];
    RingControl rx_control;   // M4 -> M7
    uint8_t rx[PORT_RING_DATA];
};
struct RegionHeader {
    // a copy of the M7's region generation, for the M4; the M7 never reads it back
    std::atomic<uint32_t> generation;
    uint8_t reserved[124];
};
struct Region {
    StatusBlock status;
    RegionHeader header;
    PortRings ports[NUM_PORTS];
    RingArea<DDS_RING_DATA> dds_tx;   // SpscRing, M7 -> M4
    RingArea<DDS_RING_DATA> dds_rx;   // SpscRing, M4 -> M7
    RingArea<CAN_RING_DATA> can_tx;   // SpscRing, M7 -> M4
    RingArea<CAN_RING_DATA> can_rx;   // SpscRing, M4 -> M7
    uint8_t spare[1024];
};
static_assert(sizeof(Region) == 16384, "the network region is 16 KB");

/*
  M7 only, while the M4 is held or halted (AP_Networking::m4_region_reset()): every ring empty,
  the status block zero, then the next generation published.

  Session contract for the M7 users of the region (AP_Networking's proxy):
  - generation is the M7's own counter, never the region's copy, which the M4 can write;
  - each user holds the same lock as the reset (AP_Networking::_m4_sem) around every ring
    operation, so a reset never runs during one;
  - rings are attached with &generation; after a reset they return 0 / RECORD_BAD until
    attached again, and the user drops what it cached for the old session (buffered bytes,
    port state, the last status copy) when it attaches again.
 */
inline uint32_t reset_region(Region &region, std::atomic<uint32_t> &generation)
{
    memset(static_cast<void *>(&region), 0, sizeof(region));
    const uint32_t next = generation.load(std::memory_order_relaxed) + 1U;
    generation.store(next, std::memory_order_release);
    region.header.generation.store(next, std::memory_order_release);
    return next;
}

}  // namespace NetM4
