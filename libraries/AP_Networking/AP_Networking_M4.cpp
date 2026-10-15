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

#endif  // AP_NETWORKING_ENABLED && AP_NETWORKING_M4_ENABLED
