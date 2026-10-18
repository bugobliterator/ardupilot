#pragma once

/*
  A stalled M4 network: the net_port thread writes the status block every STATUS_PERIOD_MS, and
  it stops when the tcpip thread blocks or spins (net_port waits on the tcpip core lock) while
  the service thread still beats the M4 heartbeat. Once this session's status heartbeat has been
  seen, standing still for STALL_MS is a network failure. A session that never writes a status
  (no network configured) never stalls. C++11, no ArduPilot dependencies (host unit tests).
 */

#include <stdint.h>

namespace ScriptingM4 {

class NetStall {
public:
    // above the M7's STATUS_STALE_MS (1 s), which only voids the addresses
    static const uint32_t STALL_MS = 3000;

    void session_started() { _seen = false; }
    // read: this session's status block was read whole, with heartbeat; a torn or another
    // session's block is no movement. True while a seen heartbeat stands still for STALL_MS
    bool stalled(bool read, uint32_t heartbeat, uint32_t now_ms)
    {
        if (read && (!_seen || heartbeat != _heartbeat)) {
            _seen = true;
            _heartbeat = heartbeat;
            _moved_ms = now_ms;
            return false;
        }
        return _seen && now_ms - _moved_ms >= STALL_MS;
    }

private:
    uint32_t _heartbeat = 0;
    uint32_t _moved_ms = 0;
    bool _seen = false;
};

}  // namespace ScriptingM4
