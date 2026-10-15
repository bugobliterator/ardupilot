#include "AP_Networking.h"

#if AP_NETWORKING_ENABLED && AP_NETWORKING_M4_ENABLED

#include "AP_Networking_M4_Layout.h"
#include <AP_Scripting/AP_ScriptingM4_Placement.h>
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

#endif  // AP_NETWORKING_ENABLED && AP_NETWORKING_M4_ENABLED
