#include "AP_Networking.h"

#if AP_NETWORKING_ENABLED && AP_NETWORKING_M4_ENABLED

#include "AP_Networking_M4_Layout.h"

void AP_Networking::m4_region_reset(uint8_t *region)
{
    // the region is AP_NETWORKING_M4_RING_SIZE aligned
    NetM4::reset_region(*static_cast<NetM4::Region *>(static_cast<void *>(region)));
}

#endif  // AP_NETWORKING_ENABLED && AP_NETWORKING_M4_ENABLED
