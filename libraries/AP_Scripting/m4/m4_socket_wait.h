#pragma once

/*
  A blocking socket on the M4 (m4_wrappers.cpp). The M7 issues every socket
  call non-blocking, so its host thread never waits on the network. A script
  whose socket blocks (the in-process default, or set_blocking(true)) gets a
  connect() and send() that wait here instead, on the Lua thread: the call is
  tried again every SOCKET_RETRY_MS until it completes or fails, or until it
  has made no progress for SOCKET_IDLE_LIMIT_MS. Each try is one short M7
  call, well inside the 1 s call timeout. As in-process, the VM's instruction
  budget does not run while a binding waits; the idle limit, and for a send
  SOCKET_SEND_LIMIT_MS (30 s) in all, are the wait's own budget (recorded
  deviation: in-process waits as long as the network stack).
  C++11, no dependencies (host unit tests).
 */

#include <stddef.h>
#include <stdint.h>

namespace ScriptingM4 {

static const uint32_t SOCKET_RETRY_MS = 10;
static const uint32_t SOCKET_IDLE_LIMIT_MS = 5000;
// a blocking send gives up after this long in all, however slowly the peer takes the data
static const uint32_t SOCKET_SEND_LIMIT_MS = 30000;
// the most of a stream send one M7 call carries (m4_wrappers.cpp checks it fits a call frame): a
// send of more is cut into pieces of this size, so it is not refused for its length. A datagram
// is never cut: it is one call, and one too long for a frame fails, as one datagram it must stay
static const size_t SOCKET_SEND_CHUNK = 768;

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

// sock:send()/sendto() of length bytes over its M7 calls: call(offset, piece, first, would_block)
// sends piece bytes from offset and returns the call's count (first: the call carries the script's
// own length argument, so the M7 checks the arguments before anything is sent; it sends at most
// the piece). wanted() is the length the script asked for, read once the first call succeeded.
// - a stream send goes in pieces of SOCKET_SEND_CHUNK, the next at once after a whole piece;
// - a datagram goes in one piece;
// - non-blocking: it ends at the first piece that is short, would block or fails;
// - blocking: such a piece is tried again every SOCKET_RETRY_MS, until all is sent, a call fails,
//   SOCKET_IDLE_LIMIT_MS pass without progress or SOCKET_SEND_LIMIT_MS in all.
// The result is the bytes sent if any, else the last call's count, as one in-process send returns
template <typename Call, typename Wanted, typename Sleep, typename Millis>
int32_t socket_send_all(size_t length, bool datagram, bool blocking, Call call, Wanted wanted, Sleep sleep_ms,
                        Millis millis)
{
    const uint32_t start_ms = millis();
    uint32_t progress_ms = start_ms;
    size_t piece = (datagram || length < SOCKET_SEND_CHUNK) ? length : SOCKET_SEND_CHUNK;
    bool would_block = false;
    int32_t count = call(0, piece, true, would_block);
    SocketSend progress(wanted());
    for (;;) {
        const bool whole = count >= 0 && (size_t(count) >= piece || size_t(count) >= progress.remaining());
        const SocketStep step = progress.add(count, would_block);
        if (step == SocketStep::Done) {
            break;
        }
        const uint32_t now_ms = millis();
        if (step == SocketStep::Progress) {
            progress_ms = now_ms;
        }
        if (!whole) {
            // a short piece, or one that would block or failed
            if (!blocking || now_ms - progress_ms >= SOCKET_IDLE_LIMIT_MS || now_ms - start_ms >= SOCKET_SEND_LIMIT_MS) {
                break;
            }
            sleep_ms(SOCKET_RETRY_MS);
        }
        const size_t rest = progress.remaining();
        piece = (datagram || rest < SOCKET_SEND_CHUNK) ? rest : SOCKET_SEND_CHUNK;
        would_block = false;
        count = call(progress.sent(), piece, false, would_block);
    }
    return progress.result(count);
}

}  // namespace ScriptingM4
