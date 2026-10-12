#pragma once

/*
  A blocking socket on the M4 (m4_wrappers.cpp). The M7 issues every socket
  call non-blocking, so its host thread never waits on the network. A script
  whose socket blocks (the in-process default, or set_blocking(true)) gets a
  connect() and send() that wait here instead, on the Lua thread: the call is
  tried again every SOCKET_RETRY_MS until it completes or fails, or until it
  has made no progress for SOCKET_IDLE_LIMIT_MS. Each try is one short M7
  call, well inside the 1 s call timeout. As in-process, the VM's instruction
  budget does not run while a binding waits; the idle limit is the wait's own
  budget (recorded deviation: in-process waits as long as the network stack).
  C++11, no dependencies (host unit tests).
 */

#include <stddef.h>
#include <stdint.h>

namespace ScriptingM4 {

static const uint32_t SOCKET_RETRY_MS = 10;
static const uint32_t SOCKET_IDLE_LIMIT_MS = 5000;

enum class SocketStep : uint8_t {
    Done,       // completed or failed: stop trying
    Progress,   // part of it was done: try again, the idle time starts over
    Wait,       // it would have blocked: try again
};

// tries step() until it is Done, sleeping SOCKET_RETRY_MS before each try; false if it gave up
// after SOCKET_IDLE_LIMIT_MS without progress
template <typename Step, typename Sleep, typename Millis>
bool socket_wait(Step step, Sleep sleep_ms, Millis millis)
{
    uint32_t progress_ms = millis();
    for (;;) {
        sleep_ms(SOCKET_RETRY_MS);
        switch (step()) {
        case SocketStep::Done:
            return true;
        case SocketStep::Progress:
            progress_ms = millis();
            break;
        case SocketStep::Wait:
            if (millis() - progress_ms >= SOCKET_IDLE_LIMIT_MS) {
                return false;
            }
            break;
        }
    }
}

// a blocking send's progress over its M7 calls: what each sent, and whether it would have blocked
class SocketSend {
public:
    explicit SocketSend(size_t wanted) : _wanted(wanted) {}

    SocketStep add(int32_t count, bool would_block)
    {
        if (count > 0) {
            // the M7 sends at most what it was given; a larger count is not trusted past it
            _sent += size_t(count) < remaining() ? size_t(count) : remaining();
            return remaining() == 0 ? SocketStep::Done : SocketStep::Progress;
        }
        if (would_block || (count == 0 && remaining() > 0)) {
            return SocketStep::Wait;
        }
        return SocketStep::Done;   // sent all of nothing, or failed
    }
    size_t sent() const { return _sent; }
    size_t remaining() const { return _wanted - _sent; }
    // what the blocking send returns: the bytes sent, else the last call's count
    int32_t result(int32_t last_count) const { return _sent > 0 ? int32_t(_sent) : last_count; }

private:
    size_t _wanted;
    size_t _sent = 0;
};

}  // namespace ScriptingM4
