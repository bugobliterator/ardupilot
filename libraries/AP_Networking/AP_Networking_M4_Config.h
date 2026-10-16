#pragma once

// NET_* as the M4 needs them, sent once per M4 session (MsgType::NetConfig)

#include <stdint.h>

namespace NetM4 {

static const uint32_t CONFIG_MAGIC = 0x4643344E;   // "N4CF"
// the meaning of the status block's words (AP_Networking_M4_Layout.h StatusFields), in the scripting
// contract hash: bump it when a field's meaning changes, even at the same size. 2: heartbeat and
// port_tx_dropped taken from reserved
static const uint32_t STATUS_LAYOUT_VERSION = 2;

struct __attribute__((packed)) PortConfig {
    uint8_t type;        // AP_Networking::NetworkPortType: 0 off, 1 UDP client, 2 UDP server, 3 TCP client, 4 TCP server
    uint8_t packetise;   // 1: the TX ring carries records (UDP ports)
    uint16_t port;
    uint32_t ip;         // host byte order
};

struct __attribute__((packed)) Config {
    uint32_t magic;
    uint8_t enabled;
    uint8_t dhcp;
    uint8_t can_mcast_mask;   // bit 0: CAN1, bit 1: CAN2
    uint8_t reserved0;
    uint32_t ip;
    uint32_t netmask;
    uint32_t gateway;
    uint32_t remote_ppp_ip;   // NET_REMPPP_IP, 0 where the PPP gateway is not built
    uint32_t options;         // NET_OPTIONS
    PortConfig ports[4];
};
// part of the scripting contract hash (AP_ScriptingM4_Rpc.h): a layout change is a deliberate edit
static_assert(sizeof(Config) == 60, "NetConfig layout");

inline bool config_valid(const Config &config)
{
    if (config.magic != CONFIG_MAGIC || (config.can_mcast_mask & ~0x03U) != 0) {
        return false;
    }
    for (const PortConfig &port : config.ports) {
        if (port.type > 4) {
            return false;
        }
    }
    return true;
}

}  // namespace NetM4
