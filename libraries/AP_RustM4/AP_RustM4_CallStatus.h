#pragma once

/*
  The PR's binding status codes (generator/host_api.json) and what a module's
  binding_call returns for a reply from the M7. Output is copied only on OK
  with the expected size: host errors leave Rust values unchanged.
 */

#include <stdint.h>
#include "AP_RustM4_Protocol.h"

namespace RustM4 {

enum : int32_t {
    STATUS_OK = 0,
    STATUS_STOPPED = 1,
    STATUS_INVALID = -1,
    STATUS_NO_MEMORY = -2,
    STATUS_UNAVAILABLE = -3,
    STATUS_HOST_ERROR = -4,
    STATUS_BUSY = -5,
};

struct CallOutcome {
    int32_t status;
    bool copy_output;
};

// transport_result: the call's port result, > 0 the reply length, < 0 -errno (no reply)
inline CallOutcome call_outcome(int32_t transport_result, const BindingReplyHeader &reply, uint16_t requested_out)
{
    if (transport_result <= 0) {
        return { STATUS_HOST_ERROR, false };
    }
    if (reply.transport == uint8_t(ReplyStatus::UnknownMethod)) {
        return { STATUS_INVALID, false };
    }
    if (reply.transport != uint8_t(ReplyStatus::Ok) || reply.status < STATUS_BUSY || reply.status > STATUS_STOPPED) {
        return { STATUS_HOST_ERROR, false };
    }
    if (reply.status != STATUS_OK) {
        return { reply.status, false };
    }
    if (reply.out_size != requested_out) {
        return { STATUS_HOST_ERROR, false };
    }
    return { STATUS_OK, true };
}

// a Ping's reply: OK, an M7 that does not know Ping (INVALID), or anything else (HOST_ERROR)
inline int32_t ping_reply_status(const uint8_t *frame, uint16_t len)
{
    SystemReplyHeader reply;
    if (len != sizeof(reply)) {
        return STATUS_HOST_ERROR;
    }
    memcpy(&reply, frame, sizeof(reply));
    if (reply.header.type != uint8_t(MsgType::SystemReply) || reply.header.length != len ||
        reply.method != uint16_t(SystemMethod::Ping)) {
        return STATUS_HOST_ERROR;
    }
    if (reply.transport == uint8_t(ReplyStatus::UnknownMethod)) {
        return STATUS_INVALID;
    }
    return reply.transport == uint8_t(ReplyStatus::Ok) && reply.result == 0 ? STATUS_OK : STATUS_HOST_ERROR;
}

}  // namespace RustM4
