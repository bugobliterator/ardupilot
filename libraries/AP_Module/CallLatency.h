#pragma once

/*
  The binding call latency probe: one call's stage times, taken from a single
  microsecond counter (TIM5 on the STM32H757, read by both cores), the ring of
  the last calls and the summary math. No ArduPilot dependencies: the host
  unit tests include it. The collector around it is CallLatencyLog.
 */

#include <stdint.h>

namespace AP_Module {

/*
  The stamps of one call, in the order they are taken. In-process calls have
  no frame, ring or second core: every stamp from svc_entry to dispatch is the
  dispatch entry, and reply_sent and reply_seen are adapter_done.
 */
struct CallStamps {
    uint32_t svc_entry;     // t0: the module's binding call enters the host (M4: the SVC body)
    uint32_t pre_send;      // the frame is checked and about to go on the ring (M4)
    uint32_t sent;          // t1: on the ring, doorbell rung (M4)
    uint32_t m7_pass;       // the M7 host's serve pass started (its wake, after any duty-cap pause)
    uint32_t m7_received;   // t2: the M7 host took the frame off the ring
    uint32_t dispatch;      // t3: dispatch starts, on the lane's thread
    uint32_t lock_wait;     // the adapter's lock is about to be taken (probe pre-take)
    uint32_t adapter;       // t4: the adapter is entered, its lock held
    uint32_t adapter_done;  // t5: the adapter returned
    uint32_t reply_sent;    // t6: the reply is on the ring, doorbell rung (M7)
    uint32_t reply_seen;    // t7: the M4 service took the reply off the ring
    uint32_t svc_return;    // t8: the binding call returns to the module
};

// the logged stages; Pass lies inside Receive, the others add up to Total
enum class CallStage : uint8_t {
    Pre,       // svc_entry -> pre_send: host checks, framing, service lock
    Send,      // pre_send -> sent: ring write, CRC, doorbell
    Receive,   // sent -> m7_received: until the M7 host takes the frame
    Pass,      // m7_pass -> m7_received: the host's pass before this frame
    Handoff,   // m7_received -> dispatch: to the slow worker, or none
    Check,     // dispatch -> lock_wait: frame checks, quota, input copy
    Lock,      // lock_wait -> adapter: waiting for the adapter's lock
    Adapter,   // adapter -> adapter_done
    Reply,     // adapter_done -> reply_sent: reply framing, ring write, doorbell
    Back,      // reply_sent -> reply_seen: until the M4 service takes the reply
    Return,    // reply_seen -> svc_return: module thread wake, reply copy
    Total,     // svc_entry -> svc_return
};
static const uint8_t CALL_STAGES = 12;
static const char *const CALL_STAGE_NAMES[CALL_STAGES] = {
    "Pre", "Snd", "Rcv", "Pas", "Hof", "Chk", "Sem", "Adp", "Rpl", "Bck", "Ret", "Tot",
};

// CallTrace flags
static const uint16_t CALL_LANE_MASK = 0x3;           // CallLane
static const uint16_t CALL_M4_SLEPT = 1U << 2;        // the module thread slept for its reply
static const uint16_t CALL_M4_POLL_WAKE = 1U << 3;    // the M4 service's wait before the reply timed out (no doorbell)
static const uint16_t CALL_M4_SENT_BY_STEP = 1U << 4; // the service's step sent it, not the module thread
static const uint16_t CALL_CAP_PAUSED = 1U << 5;      // the pass that served it followed a duty-cap pause
static const uint16_t CALL_REFUSED = 1U << 6;         // refused unrun (duty-cap debt): no adapter stages
static const uint16_t CALL_QUOTA = 1U << 7;           // over its quota: answered unrun, no adapter stages
static const uint16_t CALL_LOCK_TAKEN = 1U << 8;      // the probe took the adapter's lock first: Lock is its wait
static const uint16_t CALL_LOCK_UNKNOWN = 1U << 9;    // the adapter takes a lock the probe cannot: inside Adapter
static const uint16_t CALL_M4_MISSING = 1U << 10;     // no M4 completion arrived: Back and Return are 0
static const uint8_t CALL_POSITION_SHIFT = 12;        // frames served before it in its pass, up to 15

enum class CallLane : uint8_t { InProcess = 0, Fast = 1, Slow = 2 };

struct CallTrace {
    uint64_t time_us;   // when the M7 recorded it (log TimeUS)
    uint32_t seq;       // frame seq; in-process: a call counter
    uint16_t id;        // BindingId
    uint16_t flags;
    int32_t stage_us[CALL_STAGES];
};

// counter differences (the counter wraps): a stage that runs backwards across cores reads negative
inline int32_t stamp_delta(uint32_t from, uint32_t to)
{
    return int32_t(to - from);
}

inline void call_stages(const CallStamps &s, int32_t (&stage_us)[CALL_STAGES])
{
    stage_us[uint8_t(CallStage::Pre)] = stamp_delta(s.svc_entry, s.pre_send);
    stage_us[uint8_t(CallStage::Send)] = stamp_delta(s.pre_send, s.sent);
    stage_us[uint8_t(CallStage::Receive)] = stamp_delta(s.sent, s.m7_received);
    stage_us[uint8_t(CallStage::Pass)] = stamp_delta(s.m7_pass, s.m7_received);
    stage_us[uint8_t(CallStage::Handoff)] = stamp_delta(s.m7_received, s.dispatch);
    stage_us[uint8_t(CallStage::Check)] = stamp_delta(s.dispatch, s.lock_wait);
    stage_us[uint8_t(CallStage::Lock)] = stamp_delta(s.lock_wait, s.adapter);
    stage_us[uint8_t(CallStage::Adapter)] = stamp_delta(s.adapter, s.adapter_done);
    stage_us[uint8_t(CallStage::Reply)] = stamp_delta(s.adapter_done, s.reply_sent);
    stage_us[uint8_t(CallStage::Back)] = stamp_delta(s.reply_sent, s.reply_seen);
    stage_us[uint8_t(CallStage::Return)] = stamp_delta(s.reply_seen, s.svc_return);
    stage_us[uint8_t(CallStage::Total)] = stamp_delta(s.svc_entry, s.svc_return);
}

// the in-process form of CallStamps: dispatch entry, lock wait, adapter, adapter done, return
inline CallStamps in_process_stamps(uint32_t entry, uint32_t lock_wait, uint32_t adapter, uint32_t adapter_done,
                                    uint32_t returned)
{
    return CallStamps { entry, entry, entry, entry, entry, entry, lock_wait, adapter, adapter_done,
                        adapter_done, adapter_done, returned };
}

/*
  The last N calls. Each is flushed (logged) at most once, oldest first; calls
  overwritten before their turn are counted as skipped.
 */
template <uint16_t N>
class CallTraceRing {
public:
    static const uint16_t SIZE = N;

    explicit CallTraceRing(CallTrace *storage) : _traces(storage) {}

    void push(const CallTrace &trace)
    {
        _traces[_written % N] = trace;
        _written++;
    }

    // the oldest call not yet flushed; false if none
    bool next_unflushed(CallTrace &trace)
    {
        if (_written - _flushed > N) {
            _skipped += (_written - _flushed) - N;
            _flushed = _written - N;
        }
        if (_flushed == _written) {
            return false;
        }
        trace = _traces[_flushed % N];
        _flushed++;
        return true;
    }

    // calls in the ring, newest last: index 0 is the oldest still held
    uint16_t count() const { return _written < N ? uint16_t(_written) : N; }
    const CallTrace &at(uint16_t index) const
    {
        return _traces[(_written - count() + index) % N];
    }
    uint32_t written() const { return _written; }
    uint32_t skipped() const { return _skipped; }

private:
    CallTrace *_traces;
    uint32_t _written = 0;
    uint32_t _flushed = 0;
    uint32_t _skipped = 0;
};

// ascending, in place (shell sort: no recursion, no allocation)
inline void sort_values(int32_t *values, uint16_t count)
{
    static const uint16_t gaps[] = { 301, 132, 57, 23, 10, 4, 1 };
    for (uint16_t gap : gaps) {
        for (uint16_t i = gap; i < count; i++) {
            const int32_t value = values[i];
            uint16_t j = i;
            while (j >= gap && values[j - gap] > value) {
                values[j] = values[j - gap];
                j -= gap;
            }
            values[j] = value;
        }
    }
}

// nearest rank of sorted values: the smallest value with at least percent % of them at or below it
inline int32_t sorted_percentile(const int32_t *sorted, uint16_t count, uint8_t percent)
{
    if (count == 0) {
        return 0;
    }
    uint32_t rank = (uint32_t(count) * percent + 99U) / 100U;
    if (rank == 0) {
        rank = 1;
    }
    return sorted[rank - 1];
}

struct StageSummary {
    uint16_t count;
    int32_t p50_us;
    int32_t p99_us;
    int32_t max_us;
};

// one stage of the calls of id in ring (all ids if id is ANY_ID), using scratch (ring size) for the sort
static const uint32_t ANY_ID = 0xFFFFFFFFU;
template <uint16_t N>
inline StageSummary summarize_stage(const CallTraceRing<N> &ring, uint32_t id, CallStage stage, int32_t *scratch)
{
    uint16_t count = 0;
    for (uint16_t i = 0; i < ring.count(); i++) {
        const CallTrace &trace = ring.at(i);
        if (id == ANY_ID || trace.id == id) {
            scratch[count++] = trace.stage_us[uint8_t(stage)];
        }
    }
    sort_values(scratch, count);
    StageSummary summary { count, sorted_percentile(scratch, count, 50), sorted_percentile(scratch, count, 99),
                           count != 0 ? scratch[count - 1] : 0 };
    return summary;
}

/*
  The probed call of an M4 frame waits for its M4 completion (sent, reply seen,
  return, M4 flags), which comes in the trailer of the module's next probed call.
 */
struct M4Completion {
    uint32_t seq;
    uint32_t sent;
    uint32_t reply_seen;
    uint32_t svc_return;
    uint16_t flags;
};

class PendingCall {
public:
    // the M7 part of a call is done; returns true with the previous pending call in evicted when
    // it never completed (it is to be recorded with CALL_M4_MISSING)
    bool hold(const CallTrace &partial, const CallStamps &stamps, CallTrace &evicted)
    {
        const bool had = _valid;
        if (had) {
            evicted = missing();
        }
        _trace = partial;
        _stamps = stamps;
        _valid = true;
        return had;
    }

    // the completion of the pending call: true with the whole trace if its seq matches
    bool complete(const M4Completion &done, CallTrace &trace)
    {
        if (!_valid || done.seq != _trace.seq) {
            return false;
        }
        _stamps.sent = done.sent;
        _stamps.reply_seen = done.reply_seen;
        _stamps.svc_return = done.svc_return;
        trace = _trace;
        trace.flags |= done.flags;
        call_stages(_stamps, trace.stage_us);
        _valid = false;
        return true;
    }

    // a session ended: the pending call, if any, as missing
    bool take_missing(CallTrace &trace)
    {
        if (!_valid) {
            return false;
        }
        trace = missing();
        _valid = false;
        return true;
    }

    bool valid() const { return _valid; }

private:
    // the M4 stages of a call whose completion never came: sent as pre_send, back and return 0
    CallTrace missing() const
    {
        CallStamps stamps = _stamps;
        stamps.sent = stamps.pre_send;
        stamps.reply_seen = stamps.reply_sent;
        stamps.svc_return = stamps.reply_sent;
        CallTrace trace = _trace;
        trace.flags |= CALL_M4_MISSING;
        call_stages(stamps, trace.stage_us);
        trace.stage_us[uint8_t(CallStage::Total)] = 0;
        return trace;
    }

    CallTrace _trace {};
    CallStamps _stamps {};
    bool _valid = false;
};

}  // namespace AP_Module
