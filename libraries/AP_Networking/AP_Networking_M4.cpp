#include "AP_Networking.h"

#if AP_NETWORKING_ENABLED && AP_NETWORKING_M4_ENABLED

#include "AP_Networking_M4.h"
#include <AP_Scripting/AP_ScriptingM4_Placement.h>
#include <AP_HAL/utility/packetise.h>
#include <GCS_MAVLink/GCS.h>

static_assert(sizeof(NetM4::Region) == AP_NETWORKING_M4_RING_SIZE, "the region fills the ring window");

void AP_Networking::m4_region_reset(uint8_t *region)
{
    WITH_SEMAPHORE(_m4_sem);
    // the region is AP_NETWORKING_M4_RING_SIZE aligned
    NetM4::reset_region(*static_cast<NetM4::Region *>(static_cast<void *>(region)), _m4_region_generation);
}

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
        const int32_t type = int32_t(port.type_param.get());
        const int32_t number = port.port.get();
        if (type < int32_t(NetworkPortType::NONE) || type > int32_t(NetworkPortType::TCP_SERVER) ||
            number < 0 || number > UINT16_MAX) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NET: NET_P%u invalid, M4 network off", unsigned(i + 1));
            return false;
        }
        config.ports[i].type = uint8_t(type);
        config.ports[i].packetise = (port.type_param == NetworkPortType::UDP_CLIENT ||
                                     port.type_param == NetworkPortType::UDP_SERVER) ? 1 : 0;
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

uint8_t NetM4::track_status(StatusTracker &tracker, const StatusBlock &block, uint32_t session, StatusFields &out)
{
    uint8_t events = 0;
    if (session != tracker.session) {
        tracker = StatusTracker {};
        tracker.session = session;
        events |= STATUS_NEW_SESSION;
    }
    StatusFields status;
    if (!read_status(block, status) || status.session != session) {
        // the last good copy stands: a torn or bad block, or the last session's
        return events;
    }
    if (!tracker.have_status || status.change_count != tracker.change_count) {
        events |= STATUS_ADDRESSES;
    }
    const bool link_up = (status.flags & STATUS_LINK_UP) != 0;
    if (link_up && !tracker.link_up) {
        events |= STATUS_LINK_CAME_UP;
    }
    tracker.have_status = true;
    tracker.change_count = status.change_count;
    tracker.link_up = link_up;
    out = status;
    return events;
}

#endif  // AP_NETWORKING_ENABLED && AP_NETWORKING_M4_ENABLED
