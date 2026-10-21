#pragma once

/*
  The binding call latency probe: one call's stage times, taken from a single
  microsecond counter (TIM5 on the STM32H757, read by both cores), the ring of
  the last calls and the summary math. No ArduPilot dependencies: the host
  unit tests include it. The collector around it is CallLatencyLog.
 */

#include <stdint.h>

namespace AP_Rust {

/*
  The stamps of one call, in the order they are taken. In-process calls have
  no frame, ring or second core: svc_entry is the host's binding call entry,
  every stamp from pre_send to dispatch is the dispatch entry, and reply_sent
  and reply_seen are adapter_done.
 */
struct CallStamps {
    uint32_t svc_entry;     // t0: the module's binding call enters the host (M4: the SVC body)
    uint32_t pre_send;      // the frame is checked and about to go on the ring (M4)
    uint32_t sent;          // t1: on the ring, doorbell rung (M4)
    uint32_t m7_pass;       // the M7 host's serve pass started (its wake)
    uint32_t m7_received;   // t2: the M7 host took the frame off the ring
    uint32_t dispatch;      // t3: dispatch starts, on the lane's thread
    uint32_t lock_wait;     // the adapter's lock is about to be taken (probe pre-take)
    uint32_t adapter;       // t4: the adapter is entered, its lock held
    uint32_t adapter_done;  // t5: the adapter returned
    uint32_t reply_sent;    // t6: the reply is on the ring, doorbell rung (M7)
    uint32_t reply_seen;    // t7: the M4 service took the reply off the ring
    uint32_t svc_return;    // t8: the binding call returns to the module
    // the sub-stage stamps, each valid only with its CALL_STAMP_* bit in taken
    uint32_t chk_parsed;          // M7: frame parsed, module, id and lane checked
    uint32_t chk_quota;           // M7: quota checked
    uint32_t chk_copied;          // M7: serve buffers cleared, input copied
    uint32_t m7_doorbell;         // M7: the doorbell interrupt that woke the pass (first frame of the pass)
    uint32_t m4_packed;           // M4: frame packed, before the platform call
    uint32_t m4_posted;           // M4: the link posted, before forward_posted()
    uint32_t m4_forward_returned; // M4: forward_posted() returned with the call sent
    uint32_t m4_reply_isr;        // M4: the doorbell interrupt before the reply was seen
    uint32_t m4_reply_wake;       // M4: the service thread resumed after that doorbell
    uint32_t m4_signalled;        // M4: complete() signalled the module thread
    uint32_t m4_resumed;          // M4: the module thread resumed after its wait
    uint32_t taken;               // CALL_STAMP_*
};

// CallStamps::taken
static const uint32_t CALL_STAMP_CHK_PARSED = 1U << 0;
static const uint32_t CALL_STAMP_CHK_QUOTA = 1U << 1;
static const uint32_t CALL_STAMP_CHK_COPIED = 1U << 2;
static const uint32_t CALL_STAMP_M7_DOORBELL = 1U << 3;
static const uint32_t CALL_STAMP_M4_PACKED = 1U << 4;
static const uint32_t CALL_STAMP_M4_POSTED = 1U << 5;
static const uint32_t CALL_STAMP_M4_FORWARD_RETURNED = 1U << 6;
static const uint32_t CALL_STAMP_M4_REPLY_WAKE = 1U << 7;   // m4_reply_isr and m4_reply_wake
static const uint32_t CALL_STAMP_M4_SIGNALLED = 1U << 8;
static const uint32_t CALL_STAMP_M4_RESUMED = 1U << 9;
// the M4's bits, as its trailer carries them
static const uint32_t CALL_STAMPS_M4 = CALL_STAMP_M4_PACKED | CALL_STAMP_M4_POSTED | CALL_STAMP_M4_FORWARD_RETURNED |
                                       CALL_STAMP_M4_REPLY_WAKE | CALL_STAMP_M4_SIGNALLED | CALL_STAMP_M4_RESUMED;

/*
  Sub-stages of Chk, Rcv, Pre, Send, Back and Return, from the sub-stage stamps. A sub-stage
  counts only for calls that took both of its stamps; the three of Rcv, Pre, Back and Return add
  up to their stage for the calls that have all three.
 */
enum class CallSubstage : uint8_t {
    ChkParse,    // dispatch -> chk_parsed: parse, module, id and lane checks
    ChkQuota,    // chk_parsed -> chk_quota
    ChkCopy,     // chk_quota -> chk_copied: clearing the serve buffers, copying the input
    ChkRest,     // chk_copied -> lock_wait: the M7 exec timer, the lock lookup
    RcvIsr,      // sent -> m7_doorbell: M4 send to the M7 doorbell interrupt
    RcvWake,     // m7_doorbell -> m7_pass: interrupt to the host thread's pass
    RcvTake,     // m7_pass -> m7_received: the pass until the frame is off the ring
    PrePack,     // svc_entry -> m4_packed: checks and framing
    PrePost,     // m4_packed -> m4_posted: copy to the link, post
    PreForward,  // m4_posted -> pre_send: service lock, frame checks
    SndUnlock,   // sent -> m4_forward_returned: from the send to forward_posted()'s return
    BckIsr,      // reply_sent -> m4_reply_isr: M7 send to the M4 doorbell interrupt
    BckWake,     // m4_reply_isr -> m4_reply_wake: interrupt to the service thread
    BckDrain,    // m4_reply_wake -> reply_seen: the service step until the reply is off the ring
    RetSignal,   // reply_seen -> m4_signalled: reply copy to the link, signalling the module thread
    RetSwitch,   // m4_signalled -> m4_resumed: until the module thread runs
    RetCopy,     // m4_resumed -> svc_return: reply checks, output copy, return
};
static const uint8_t CALL_SUBSTAGES = 17;
static const char *const CALL_SUBSTAGE_NAMES[CALL_SUBSTAGES] = {
    "ChkParse", "ChkQuota", "ChkCopy", "ChkRest", "RcvIsr", "RcvWake", "RcvTake", "PrePack", "PrePost",
    "PreForward", "SndUnlock", "BckIsr", "BckWake", "BckDrain", "RetSignal", "RetSwitch", "RetCopy",
};

// the sub-stages of one call; returns the mask of those it took both stamps of (bit per CallSubstage)
inline uint32_t call_substages(const CallStamps &s, int32_t (&sub_us)[CALL_SUBSTAGES])
{
    struct Span {
        uint32_t from, to, needs;
    };
    const Span spans[CALL_SUBSTAGES] = {
        { s.dispatch, s.chk_parsed, CALL_STAMP_CHK_PARSED },
        { s.chk_parsed, s.chk_quota, CALL_STAMP_CHK_PARSED | CALL_STAMP_CHK_QUOTA },
        { s.chk_quota, s.chk_copied, CALL_STAMP_CHK_QUOTA | CALL_STAMP_CHK_COPIED },
        { s.chk_copied, s.lock_wait, CALL_STAMP_CHK_COPIED },
        { s.sent, s.m7_doorbell, CALL_STAMP_M7_DOORBELL },
        { s.m7_doorbell, s.m7_pass, CALL_STAMP_M7_DOORBELL },
        { s.m7_pass, s.m7_received, CALL_STAMP_M7_DOORBELL },
        { s.svc_entry, s.m4_packed, CALL_STAMP_M4_PACKED },
        { s.m4_packed, s.m4_posted, CALL_STAMP_M4_PACKED | CALL_STAMP_M4_POSTED },
        { s.m4_posted, s.pre_send, CALL_STAMP_M4_POSTED },
        { s.sent, s.m4_forward_returned, CALL_STAMP_M4_FORWARD_RETURNED },
        { s.reply_sent, s.m4_reply_isr, CALL_STAMP_M4_REPLY_WAKE },
        { s.m4_reply_isr, s.m4_reply_wake, CALL_STAMP_M4_REPLY_WAKE },
        { s.m4_reply_wake, s.reply_seen, CALL_STAMP_M4_REPLY_WAKE },
        { s.reply_seen, s.m4_signalled, CALL_STAMP_M4_SIGNALLED },
        { s.m4_signalled, s.m4_resumed, CALL_STAMP_M4_SIGNALLED | CALL_STAMP_M4_RESUMED },
        { s.m4_resumed, s.svc_return, CALL_STAMP_M4_RESUMED },
    };
    uint32_t valid = 0;
    for (uint8_t i = 0; i < CALL_SUBSTAGES; i++) {
        const bool taken = (s.taken & spans[i].needs) == spans[i].needs;
        sub_us[i] = taken ? int32_t(spans[i].to - spans[i].from) : 0;
        if (taken) {
            valid |= 1U << i;
        }
    }
    return valid;
}

// count, sum and max of each sub-stage of one binding
struct SubstageTotals {
    uint32_t count[CALL_SUBSTAGES];
    int64_t sum_us[CALL_SUBSTAGES];
    int32_t max_us[CALL_SUBSTAGES];

    void add(const int32_t (&sub_us)[CALL_SUBSTAGES], uint32_t valid)
    {
        for (uint8_t i = 0; i < CALL_SUBSTAGES; i++) {
            if ((valid & (1U << i)) == 0) {
                continue;
            }
            if (count[i] == 0 || sub_us[i] > max_us[i]) {
                max_us[i] = sub_us[i];
            }
            count[i]++;
            sum_us[i] += sub_us[i];
        }
    }

    // mean rounded to the nearest microsecond; 0 without samples
    int32_t mean_us(uint8_t i) const
    {
        if (count[i] == 0) {
            return 0;
        }
        const int64_t half = sum_us[i] >= 0 ? int64_t(count[i] / 2) : -int64_t(count[i] / 2);
        return int32_t((sum_us[i] + half) / int64_t(count[i]));
    }
};

/*
  The core cycles of the parts of a probed call (DWT), counted without the probe's clock reads in
  between: they split the stages finer than the stamps. The M4's come in its CallProbeTrailer (the
  values of RustM4::M4CyclePart), the M7's are counted on the M7. A part is summed only for calls
  that took it.
 */
enum class M4CyclePart : uint8_t {
    SvcEntry,      // SVC handler entry to host_binding_call(): the argument copy
    Checks,        // the host's checks after the entry stamp, up to begin_probed_call()
    BeginProbe,    // begin_probed_call(): the trailer
    Pack,          // pack_binding_call()
    FwdLock,       // forward_posted(): the service lock
    FwdRing,       // Channel::send() with the M7 awake: the ring write
    FwdBell,       // Channel::send() with the M7 asleep: the ring write and the doorbell
    FwdUnlock,     // forward_posted(): the service unlock
    IsrToThread,   // doorbell interrupt entry to the module thread's return from its wait
    DrainLock,     // the reply's drain step: the service lock
    DrainRing,     // Channel::receive() of the reply
    DrainParse,    // header and reply checks until the copy to the link
    DrainCopy,     // the reply copied to the link
    PortTail,      // after that copy to the platform call's return: complete(), unlock, wait end
    RetCopy,       // reply checks and the copy to the module's output
    SvcReturn,     // host_binding_call()'s end to the end of the SVC body
};
static const uint8_t M4_CYCLE_PARTS = 16;
static const char *const M4_CYCLE_PART_NAMES[M4_CYCLE_PARTS] = {
    "SvcEntry", "Checks", "BeginProbe", "Pack", "FwdLock", "FwdRing", "FwdBell", "FwdUnlock",
    "IsrToThread", "DrainLock", "DrainRing", "DrainParse", "DrainCopy", "PortTail", "RetCopy", "SvcReturn",
};

enum class M7CyclePart : uint8_t {
    IsrToThread,   // doorbell interrupt entry to the host thread's return from its wait (first frame of a pass)
    Wake,          // Channel::wake() after that wait (first frame of a pass)
    CapWait,       // after the wake to serve(): the pass's stamps (first frame of a pass)
    Take,          // Channel::receive() of the frame
    Parse,         // serve entry: the header, module, id and quota checks
    Zero,          // clearing the serve buffers
    InCopy,        // the input copy
    Adapter,       // the lock lookup and take, the adapter, the give
    ReplyPack,     // the reply framed, up to its send
    ReplyRing,     // Channel::send() with the M4 awake: the ring write
    ReplyBell,     // Channel::send() with the M4 asleep: the ring write and the doorbell
    PassEnd,       // after the probe's record to the end of serve() (last frame of a pass)
};
static const uint8_t M7_CYCLE_PARTS = 12;
static const char *const M7_CYCLE_PART_NAMES[M7_CYCLE_PARTS] = {
    "IsrToThread", "Wake", "CapWait", "Take", "Parse", "Zero", "InCopy", "Adapter", "ReplyPack", "ReplyRing",
    "ReplyBell", "PassEnd",
};

// count, sum and max of each cycle part of one binding
template <uint8_t N>
struct CycleTotals {
    uint32_t count[N];
    uint64_t sum[N];
    uint32_t max[N];

    void add(uint8_t part, uint32_t cycles)
    {
        if (part >= N) {
            return;
        }
        sum[part] += cycles;
        if (count[part] == 0 || cycles > max[part]) {
            max[part] = cycles;
        }
        count[part]++;
    }

    // the parts with their bit in taken
    void add(const uint32_t (&cycles)[N], uint32_t taken)
    {
        for (uint8_t i = 0; i < N; i++) {
            if ((taken & (1U << i)) != 0) {
                add(i, cycles[i]);
            }
        }
    }

    // mean rounded to the nearest cycle; 0 without samples
    uint32_t mean(uint8_t i) const
    {
        return count[i] != 0 ? uint32_t((sum[i] + count[i] / 2) / count[i]) : 0;
    }
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
static const uint16_t CALL_M4_POLL_WAKE = 1U << 3;    // the M4 wait before the reply was drained timed out (no doorbell)
static const uint16_t CALL_M4_SENT_BY_STEP = 1U << 4; // a later drain sent it, not forward_posted()
// bits 5 and 6 are unused, kept clear so older logs read the same
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

// the in-process form of CallStamps: host entry, dispatch entry, lock wait, adapter, adapter done, return
inline CallStamps in_process_stamps(uint32_t host_entry, uint32_t entry, uint32_t lock_wait, uint32_t adapter,
                                    uint32_t adapter_done, uint32_t returned)
{
    return CallStamps { host_entry, entry, entry, entry, entry, entry, lock_wait, adapter, adapter_done,
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
    // the M4's sub-stage stamps, valid per CALL_STAMP_M4_* bit in taken
    uint32_t taken;
    uint32_t packed;
    uint32_t posted;
    uint32_t forward_returned;
    uint32_t reply_isr;
    uint32_t reply_wake;
    uint32_t signalled;
    uint32_t resumed;
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

    // the completion of the pending call: true with the whole trace (and its stamps) if its seq matches
    bool complete(const M4Completion &done, CallTrace &trace, CallStamps *stamps = nullptr)
    {
        if (!_valid || done.seq != _trace.seq) {
            return false;
        }
        _stamps.sent = done.sent;
        _stamps.reply_seen = done.reply_seen;
        _stamps.svc_return = done.svc_return;
        _stamps.m4_packed = done.packed;
        _stamps.m4_posted = done.posted;
        _stamps.m4_forward_returned = done.forward_returned;
        _stamps.m4_reply_isr = done.reply_isr;
        _stamps.m4_reply_wake = done.reply_wake;
        _stamps.m4_signalled = done.signalled;
        _stamps.m4_resumed = done.resumed;
        _stamps.taken = (_stamps.taken & ~CALL_STAMPS_M4) | (done.taken & CALL_STAMPS_M4);
        if (stamps != nullptr) {
            *stamps = _stamps;
        }
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

}  // namespace AP_Rust
