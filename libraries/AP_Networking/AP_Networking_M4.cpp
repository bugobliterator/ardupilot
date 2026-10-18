#include "AP_Networking.h"

#if AP_NETWORKING_ENABLED && AP_NETWORKING_M4_ENABLED

#include "AP_Networking_M4.h"
#include <AP_Scripting/AP_ScriptingM4_Placement.h>
#include <AP_HAL/utility/packetise.h>
#include <AP_HAL/utility/Socket.h>
#include <AP_Scripting/AP_Scripting.h>
#include <GCS_MAVLink/GCS.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <hal.h>   // __CORTEX_M, for HSEM.h
#include <CubePilotFW/pal/HSEM.h>
#endif

extern const AP_HAL::HAL& hal;

static_assert(sizeof(NetM4::Region) == AP_NETWORKING_M4_RING_SIZE, "the region fills the ring window");

static_assert(ARRAY_SIZE(NetM4::Config::ports) == NetM4::NUM_PORTS, "one config per port ring");
#if AP_NETWORKING_REGISTER_PORT_ENABLED
static_assert(AP_NETWORKING_NUM_PORTS <= NetM4::NUM_PORTS, "every NET_P port fits the M4 config");
#endif

bool AP_Networking::m4_config(NetM4::Config &out) const
{
    memset(&out, 0, sizeof(out));
    if (!param.enabled) {
        return false;
    }
    NetM4::Config config {};
    config.magic = NetM4::CONFIG_MAGIC;
    config.enabled = 1;
    // 0 where the M7 backends do not control them (PPP only: IPCP assigns the addresses);
    // Ethernet addressing on the M4 comes with Ethernet on the M4
    config.dhcp = get_dhcp_enabled() ? 1 : 0;
    config.ip = get_ip_param();
    config.netmask = get_netmask_param();
    config.gateway = get_gateway_param();
#if AP_NETWORKING_PPP_GATEWAY_ENABLED
    config.remote_ppp_ip = param.remote_ppp_ip.get_uint32();
#endif
    config.options = uint32_t(param.options.get());
#if AP_NETWORKING_CAN_MCAST_ENABLED
    config.can_mcast_mask = (option_is_set(OPTION::CAN1_MCAST_ENDPOINT) ? 1U : 0U) |
                            (option_is_set(OPTION::CAN2_MCAST_ENDPOINT) ? 2U : 0U);
#endif
#if AP_NETWORKING_REGISTER_PORT_ENABLED
    for (uint8_t i = 0; i < ARRAY_SIZE(ports) && i < ARRAY_SIZE(config.ports); i++) {
        const auto &port = ports[i];
        // the type the port was opened with at boot (NET_Pn_TYPE needs a reboot): the M7 end
        // frames by it (m4_io_update()), so a later M4 session must too. A port without
        // buffers has no M7 end
        const int32_t type = (port.readbuffer == nullptr || port.writebuffer == nullptr)
                             ? int32_t(NetworkPortType::NONE) : int32_t(port.type);
        const int32_t number = port.port.get();
        if (type < int32_t(NetworkPortType::NONE) || type > int32_t(NetworkPortType::TCP_SERVER) ||
            number < 0 || number > UINT16_MAX) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NET: NET_P%u invalid, M4 network off", unsigned(i + 1));
            return false;
        }
        config.ports[i].type = uint8_t(type);
        config.ports[i].packetise = (type == int32_t(NetworkPortType::UDP_CLIENT) ||
                                     type == int32_t(NetworkPortType::UDP_SERVER)) ? 1 : 0;
        config.ports[i].port = uint16_t(number);
        config.ports[i].ip = port.ip.get_uint32();
    }
#endif
    // DDS needs no entry: each datagram on the pipe carries its destination
    if (!NetM4::config_valid(config)) {
        return false;
    }
    out = config;
    return true;
}

uint32_t NetM4::flush_tx(ByteBuffer &writebuffer, ByteRing &ring, bool packetise, bool mavlink_frames)
{
    uint32_t moved = 0;
    uint8_t buf[MAX_RECORD];
    while (writebuffer.available() > 0) {
        uint32_t len = MIN(writebuffer.available(), uint32_t(MAX_RECORD));
        if (packetise) {
#if AP_MAVLINK_PACKETISE_ENABLED
            if (mavlink_frames) {
                len = mavlink_packetise(writebuffer, len);
            }
#else
            (void)mavlink_frames;
#endif
            if (len == 0 || writebuffer.peekbytes(buf, len) != len || !ring.write_record(buf, uint16_t(len))) {
                break;
            }
            writebuffer.advance(len);
            moved += len;
        } else {
            len = writebuffer.peekbytes(buf, len);
            const uint32_t written = ring.write(buf, len);
            writebuffer.advance(written);
            moved += written;
            if (written < len || written == 0) {
                break;
            }
        }
    }
    return moved;
}

uint32_t NetM4::drain_rx(ByteRing &ring, ByteBuffer &readbuffer)
{
    if (ring.desynced()) {
        ring.consumer_resync();
        return 0;
    }
    uint32_t moved = 0;
    uint8_t buf[256];
    while (true) {
        const uint32_t len = MIN(readbuffer.space(), uint32_t(sizeof(buf)));
        const uint32_t got = len > 0 ? ring.read(buf, len) : 0;
        if (got == 0) {
            return moved;
        }
        readbuffer.write(buf, got);
        moved += got;
    }
}

uint32_t NetM4::reset_port(ByteBuffer &readbuffer, ByteBuffer &writebuffer)
{
    const uint32_t dropped = readbuffer.available() + writebuffer.available();
    readbuffer.clear();
    writebuffer.clear();
    return dropped;
}

uint8_t NetM4::track_status(StatusTracker &tracker, const StatusBlock &block, uint32_t session, uint32_t now_ms,
                            StatusFields &out)
{
    uint8_t events = 0;
    if (session != tracker.session) {
        tracker = StatusTracker {};
        tracker.session = session;
        events |= STATUS_NEW_SESSION;
    }
    StatusFields status;
    const bool fresh = read_status(block, status) && status.session == session &&
                       (!tracker.have_status || status.heartbeat != tracker.heartbeat);
    if (!fresh) {
        // the last good copy stands (a torn or bad block, the last session's, an unchanged one)
        // until this session's heartbeat has stood still for STATUS_STALE_MS: e.g. the M4 is off
        // for good, or stopped inside a write
        if (tracker.have_status && !tracker.stale && now_ms - tracker.heartbeat_ms >= STATUS_STALE_MS) {
            tracker.stale = true;
            tracker.link_up = false;
            events |= STATUS_WENT_STALE;
        }
        return events;
    }
    if (!tracker.have_status || tracker.stale || status.change_count != tracker.change_count) {
        events |= STATUS_ADDRESSES;
    }
    const bool link_up = (status.flags & STATUS_LINK_UP) != 0;
    if (link_up && !tracker.link_up) {
        events |= STATUS_LINK_CAME_UP;
    }
    tracker.have_status = true;
    tracker.stale = false;
    tracker.heartbeat = status.heartbeat;
    tracker.heartbeat_ms = now_ms;
    tracker.change_count = status.change_count;
    tracker.link_up = link_up;
    out = status;
    return events;
}

/*
  the M4 network region: the scripting supervisor clears it while the M4 cannot run, before
  every start and every restart grant that follows an M4 fault or exit. The ports hold until
  the next session is confirmed
 */
void AP_Networking::m4_region_reset(uint8_t *region)
{
    WITH_SEMAPHORE(_m4_sem);
    // the region is AP_NETWORKING_M4_RING_SIZE aligned
    _m4_region = static_cast<NetM4::Region *>(static_cast<void *>(region));
    NetM4::reset_region(*_m4_region, _m4_region_generation);
    _m4_attached = false;
    m4_hold_ports();
}

uint32_t AP_Networking::m4_session()
{
    const AP_Scripting *scripting = AP::scripting();
    return scripting != nullptr ? scripting->m4_session() : 0;
}

bool AP_Networking::m4_session_confirmed(uint32_t session)
{
    const AP_Scripting *scripting = AP::scripting();
    return scripting != nullptr && scripting->m4_session_confirmed(session);
}

namespace {
// the scripting supervisor's grant, for m4_io_update_from()
struct SupervisorGrant {
    uint32_t current() const { return AP_Networking::m4_session(); }
    bool confirmed(uint32_t session) const { return AP_Networking::m4_session_confirmed(session); }
};
}

/*
  the M4 of the last session may still run: no port reads or writes, and no ring access, until
  the next session is confirmed. _m4_sem held
 */
void AP_Networking::m4_hold_ports()
{
    _m4_held = true;
#if AP_NETWORKING_REGISTER_PORT_ENABLED
    for (auto &port : ports) {
        WITH_SEMAPHORE(port.sem);
        port.m4_held = true;
    }
#endif
}

/*
  a confirmed new M4 session (after a region reset, or a restart that kept the region): the
  rings attached again, the M7's ends of them reset and every port's buffered bytes dropped, so
  nothing queued in one session reaches the next; then the ports run. _m4_sem held
 */
void AP_Networking::m4_attach(uint32_t session)
{
    _m4_attached = true;
    _m4_attached_generation = _m4_region_generation.load(std::memory_order_acquire);
    _m4_attached_session = session;
    for (uint8_t i = 0; i < NetM4::NUM_PORTS; i++) {
        NetM4::PortRings &rings = _m4_region->ports[i];
        if (!_m4_tx[i].attach(&rings.tx_control, rings.tx, NetM4::PORT_RING_DATA, &_m4_region_generation) ||
            !_m4_rx[i].attach(&rings.rx_control, rings.rx, NetM4::PORT_RING_DATA, &_m4_region_generation)) {
            _m4_attached = false;
            continue;
        }
        _m4_tx[i].producer_reset();
        // what the new session wrote before this pass, after it published its header
        _m4_session_rx_dropped += _m4_rx[i].available();
        _m4_rx[i].consumer_resync();
    }
    if (!_m4_attached) {
        return;
    }
    _m4_held = false;
    _m4_doorbell_pending = false;
#if AP_NETWORKING_REGISTER_PORT_ENABLED
    for (auto &port : ports) {
        WITH_SEMAPHORE(port.sem);
        if (port.readbuffer != nullptr && port.writebuffer != nullptr) {
            _m4_bytes_dropped += NetM4::reset_port(*port.readbuffer, *port.writebuffer);
        }
        port.m4_held = false;
    }
#endif
}

/*
  the NET_P ports' data through the rings, registered with the IO thread. Waits only on M7
  threads: the supervisor's reset and the ports' writers
 */
void AP_Networking::m4_io_update()
{
    m4_io_update_from(SupervisorGrant {});
}

void AP_Networking::m4_io_update(uint32_t session, bool confirmed)
{
    WITH_SEMAPHORE(_m4_sem);
    if (_m4_region == nullptr) {
        return;
    }
    if (!confirmed) {
        if (!_m4_held) {
            m4_hold_ports();
        }
        return;
    }
    if (_m4_held || !_m4_attached || session != _m4_attached_session ||
        _m4_region_generation.load(std::memory_order_acquire) != _m4_attached_generation) {
        m4_attach(session);
        if (!_m4_attached) {
            return;
        }
    }
#if AP_NETWORKING_REGISTER_PORT_ENABLED
    for (uint8_t i = 0; i < ARRAY_SIZE(ports); i++) {
        auto &port = ports[i];
        if (port.type == NetworkPortType::NONE || port.readbuffer == nullptr || port.writebuffer == nullptr) {
            continue;
        }
        // a bad tail from the M4: its consumer resyncs the ring; until then nothing fits and the
        // port's bytes wait in its write buffer
        const bool desynced = _m4_tx[i].desynced();
        if (desynced && !_m4_tx_desynced[i]) {
            _m4_tx_desyncs++;
        }
        _m4_tx_desynced[i] = desynced;
        WITH_SEMAPHORE(port.sem);
        const bool was_empty = _m4_tx[i].space() == NetM4::PORT_RING_DATA;
        // UDP ports carry records (NetM4::Config packetise), MAVLink ones ending on frames
        const bool records = port.type == NetworkPortType::UDP_CLIENT || port.type == NetworkPortType::UDP_SERVER;
        const uint32_t sent = NetM4::flush_tx(*port.writebuffer, _m4_tx[i], records, port.packetise);
        port.tx_stats_bytes += sent;
        port.rx_stats_bytes += NetM4::drain_rx(_m4_rx[i], *port.readbuffer);
        if (was_empty && sent > 0) {
            _m4_doorbell_pending = true;
        }
    }
    if (_m4_doorbell_pending) {
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        // the M4 sleeps until a doorbell; a busy take is retried on the next pass
        if (CubePilotFW::HwSemaphore::takeTx(AP_NETWORKING_M4_DOORBELL_GROUP)) {
            CubePilotFW::HwSemaphore::releaseTx(AP_NETWORKING_M4_DOORBELL_GROUP);
            _m4_doorbell_pending = false;
            _m4_doorbells++;
        }
#else
        // the SITL M4 process polls
        _m4_doorbell_pending = false;
        _m4_doorbells++;
#endif
    }
#endif  // AP_NETWORKING_REGISTER_PORT_ENABLED
}

uint8_t AP_Networking::m4_track_status(NetM4::StatusTracker &tracker, NetM4::StatusFields &out)
{
    WITH_SEMAPHORE(_m4_sem);
    if (_m4_region == nullptr) {
        return 0;
    }
    return NetM4::track_status(tracker, _m4_region->status, m4_session(), AP_HAL::millis(), out);
}

uint32_t AP_Networking::m4_queued_bytes_for_test()
{
    WITH_SEMAPHORE(_m4_sem);
    uint32_t total = 0;
#if AP_NETWORKING_REGISTER_PORT_ENABLED
    for (auto &port : ports) {
        WITH_SEMAPHORE(port.sem);
        if (port.readbuffer != nullptr && port.writebuffer != nullptr) {
            total += port.readbuffer->available() + port.writebuffer->available();
        }
    }
#endif
    if (_m4_region != nullptr) {
        // the bytes in the region, whatever the proxy's own ring ends
        for (uint8_t i = 0; i < NetM4::NUM_PORTS; i++) {
            NetM4::ByteRing tx, rx;
            if (tx.attach(&_m4_region->ports[i].tx_control, _m4_region->ports[i].tx, NetM4::PORT_RING_DATA) &&
                rx.attach(&_m4_region->ports[i].rx_control, _m4_region->ports[i].rx, NetM4::PORT_RING_DATA)) {
                total += rx.available() + (NetM4::PORT_RING_DATA - tx.space());
            }
        }
    }
    return total;
}

bool AP_Networking_M4::init()
{
    hal.scheduler->register_io_process(FUNCTOR_BIND(&frontend, &AP_Networking::m4_io_update, void));
    return true;
}

/*
  the addresses and link state the M4 publishes, at 10Hz
 */
void AP_Networking_M4::update()
{
    NetM4::StatusFields status;
    const uint8_t events = frontend.m4_track_status(tracker, status);
    if ((events & (NetM4::STATUS_NEW_SESSION | NetM4::STATUS_WENT_STALE)) &&
        (activeSettings.ip != 0 || activeSettings.nm != 0 || activeSettings.gw != 0)) {
        activeSettings.ip = 0;
        activeSettings.nm = 0;
        activeSettings.gw = 0;
        activeSettings.last_change_ms = AP_HAL::millis();
    }
    if (events & NetM4::STATUS_ADDRESSES) {
        activeSettings.ip = status.ip;
        activeSettings.nm = status.netmask;
        activeSettings.gw = status.gateway;
        activeSettings.last_change_ms = AP_HAL::millis();
    }
    if (events & NetM4::STATUS_LINK_CAME_UP) {
        char ipstr[IP4_STR_LEN];
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Net: M4 up, ppp0 %s",
                      SocketAPM::inet_addr_to_str(status.ip, ipstr, sizeof(ipstr)));
    }
}

#endif  // AP_NETWORKING_ENABLED && AP_NETWORKING_M4_ENABLED
