#include "AP_Networking.h"

#if AP_NETWORKING_ENABLED && AP_NETWORKING_M4_ENABLED

#include "AP_Networking_M4_Layout.h"
#include <AP_Scripting/AP_ScriptingM4_Placement.h>

static_assert(sizeof(NetM4::Region) == AP_NETWORKING_M4_RING_SIZE, "the region fills the ring window");

void AP_Networking::m4_region_reset(uint8_t *region)
{
    WITH_SEMAPHORE(_m4_sem);
    // the region is AP_NETWORKING_M4_RING_SIZE aligned
    NetM4::reset_region(*static_cast<NetM4::Region *>(static_cast<void *>(region)), _m4_region_generation);
}

bool AP_Networking::m4_config(NetM4::Config &out) const
{
    memset(&out, 0, sizeof(out));
    if (!param.enabled) {
        return false;
    }
    out.magic = NetM4::CONFIG_MAGIC;
    out.enabled = 1;
    out.dhcp = get_dhcp_enabled() ? 1 : 0;
    out.ip = get_ip_param();
    out.netmask = get_netmask_param();
    out.gateway = get_gateway_param();
    out.options = uint32_t(param.options.get());
#if AP_NETWORKING_CAN_MCAST_ENABLED
    out.can_mcast_mask = (option_is_set(OPTION::CAN1_MCAST_ENDPOINT) ? 1U : 0U) |
                         (option_is_set(OPTION::CAN2_MCAST_ENDPOINT) ? 2U : 0U);
#endif
#if AP_NETWORKING_REGISTER_PORT_ENABLED
    for (uint8_t i = 0; i < ARRAY_SIZE(ports) && i < NetM4::NUM_PORTS; i++) {
        const auto &port = ports[i];
        out.ports[i].type = uint8_t(port.type_param.get());
        out.ports[i].packetise = (port.type_param == NetworkPortType::UDP_CLIENT ||
                                  port.type_param == NetworkPortType::UDP_SERVER) ? 1 : 0;
        out.ports[i].port = uint16_t(port.port.get());
        out.ports[i].ip = port.ip.get_uint32();
    }
#endif
    // DDS needs no entry: each datagram on the pipe carries its destination
    return NetM4::config_valid(out);
}

#endif  // AP_NETWORKING_ENABLED && AP_NETWORKING_M4_ENABLED
