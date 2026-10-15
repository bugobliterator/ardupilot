#pragma once

#include "AP_Networking_Config.h"

#if AP_NETWORKING_ENABLED && AP_NETWORKING_M4_ENABLED

#include "AP_Networking_Backend.h"
#include "AP_Networking_M4_Layout.h"
#include <AP_HAL/utility/RingBuffer.h>

namespace NetM4 {

// a NET_P port's data path between its buffers and its rings, under the port's lock.
// TX, packetise (UDP ports): whole records of at most MAX_RECORD, each ending on a MAVLink frame
// when mavlink_frames; otherwise bytes. As many as fit, the rest waits in writebuffer
uint32_t flush_tx(ByteBuffer &writebuffer, ByteRing &ring, bool packetise, bool mavlink_frames = true);
// RX: as many bytes as readbuffer takes; a desynced ring is resynced and read next time
uint32_t drain_rx(ByteRing &ring, ByteBuffer &readbuffer);
// both buffers emptied; returns the bytes dropped
uint32_t reset_port(ByteBuffer &readbuffer, ByteBuffer &writebuffer);

// the status block as the proxy follows it, one M4 session at a time
struct StatusTracker {
    uint32_t session;
    uint32_t change_count;
    bool have_status;   // a status of this session was taken
    bool link_up;
};
enum StatusEvent : uint8_t {
    STATUS_NEW_SESSION = 1U << 0,   // the session changed: the last session's addresses are void
    STATUS_ADDRESSES = 1U << 1,     // out holds this session's addresses, first seen or changed
    STATUS_LINK_CAME_UP = 1U << 2,
};
// session: the supervisor's current grant (AP_ScriptingM4::m4_session()). A torn or out of range
// block, or one of another session, leaves the tracker's copy as it was
uint8_t track_status(StatusTracker &tracker, const StatusBlock &block, uint32_t session, StatusFields &out);

}  // namespace NetM4

// the M7 side of networking on the M4: the NET_P ports through the rings (AP_Networking::m4_io_update()),
// the addresses from the status block
class AP_Networking_M4 : public AP_Networking_Backend
{
public:
    using AP_Networking_Backend::AP_Networking_Backend;

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Networking_M4);

    bool init() override;
    void update() override;

private:
    NetM4::StatusTracker tracker;
};

#endif  // AP_NETWORKING_ENABLED && AP_NETWORKING_M4_ENABLED
