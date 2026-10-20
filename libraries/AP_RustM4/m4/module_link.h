#pragma once

/*
  One call between the module thread and the privileged M4 service. The
  request is a whole frame (its seq is set by the service), the reply the
  whole frame the M7 answered it with.
 */

#include <atomic>
#include <stdint.h>
#include <AP_RustM4_Protocol.h>

namespace RustM4 {

struct ModuleLink {
    enum State : uint8_t {
        Idle = 0,     // the module thread may post
        Posted = 1,   // request filled in by the module thread
        Sent = 2,     // on the ring, waiting for the reply
        Done = 3,     // result (and reply) filled in by the service
    };
    std::atomic<uint8_t> state;
    std::atomic<uint8_t> fault_request;   // for the module thread, FaultKind::None once taken
    uint16_t request_len;
    int32_t result;                       // reply length, or -errno
    uint8_t request[MAX_FRAME];
    uint8_t reply[MAX_FRAME];
};

}  // namespace RustM4
