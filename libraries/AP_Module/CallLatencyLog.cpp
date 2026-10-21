#include "CallLatencyLog.h"

#if AP_MODULE_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/ExpandingString.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <AP_Module/BindingM4Policy_generated.h>
#include <AP_Scheduler/AP_Scheduler.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <hal.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <time.h>
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && defined(STM32_ST_USE_TIMER) && STM32_ST_USE_TIMER == 5 && \
    CH_CFG_ST_RESOLUTION == 32 && CH_CFG_ST_FREQUENCY == 1000000
#define CALL_LATENCY_CLOCK_TIM5 1
#else
#define CALL_LATENCY_CLOCK_TIM5 0
#endif

extern const AP_HAL::HAL& hal;

namespace AP_Module {

CallLatencyLog &CallLatencyLog::get()
{
    static CallLatencyLog log;
    return log;
}

uint32_t CallLatencyLog::clock_us()
{
#if CALL_LATENCY_CLOCK_TIM5
    // AP_HAL::micros() reads the same count through the 64-bit system time, under a lock
    return TIM5->CNT;
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint32_t(uint64_t(ts.tv_sec) * 1000000U + uint64_t(ts.tv_nsec) / 1000U);
#else
    return AP_HAL::micros();
#endif
}

uint32_t CallLatencyLog::cycles()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    return DWT->CYCCNT;
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint32_t(uint64_t(ts.tv_sec) * 1000000000U + uint64_t(ts.tv_nsec));
#else
    return 0;
#endif
}

uint32_t CallLatencyLog::cycles_per_us()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    return STM32_SYS_CK / 1000000U;
#else
    return 1000;   // SITL: nanoseconds
#endif
}

AP_HAL::Semaphore *CallLatencyLog::adapter_lock(uint32_t id, uint16_t &flags)
{
    if (id >= M4_BINDING_COUNT) {
        return nullptr;
    }
    switch (M4_BINDING_LOCK[id]) {
    case M4Lock::None:
        return nullptr;
    case M4Lock::Scheduler: {
        AP_Scheduler *scheduler = AP_Scheduler::get_singleton();
        if (scheduler == nullptr) {
            return nullptr;
        }
        flags |= CALL_LOCK_TAKEN;
        return &scheduler->get_semaphore();
    }
    case M4Lock::Ahrs: {
        AP_AHRS *ahrs = AP_AHRS::get_singleton();
        if (ahrs == nullptr) {
            return nullptr;
        }
        flags |= CALL_LOCK_TAKEN;
        return &ahrs->get_semaphore();
    }
    case M4Lock::Other:
        break;
    }
    flags |= CALL_LOCK_UNKNOWN;
    return nullptr;
}

bool CallLatencyLog::start()
{
    if (_ring != nullptr) {
        return true;
    }
    auto *traces = static_cast<CallTrace *>(calloc(CALL_TRACE_COUNT, sizeof(CallTrace)));
    auto *scratch = static_cast<int32_t *>(calloc(CALL_TRACE_COUNT, sizeof(int32_t)));
    auto *substages = static_cast<SubstageTotals *>(calloc(SUMMARY_IDS, sizeof(SubstageTotals)));
    auto *m4_pre = static_cast<M4PreCycleTotals *>(calloc(SUMMARY_IDS, sizeof(M4PreCycleTotals)));
    auto *ring = traces != nullptr ? NEW_NOTHROW CallTraceRing<CALL_TRACE_COUNT>(traces) : nullptr;
    if (scratch == nullptr || ring == nullptr || substages == nullptr || m4_pre == nullptr) {
        free(traces);
        free(scratch);
        free(substages);
        free(m4_pre);
        delete ring;
        return false;
    }
    // the cost of one stamp, and of the take and give the probe adds around a locked adapter
    static const uint8_t READS = 64;
    uint32_t start = cycles();
    for (uint8_t i = 0; i < READS; i++) {
        (void)clock_us();
    }
    const uint32_t stamp_cycles = (cycles() - start) / READS;
    HAL_Semaphore lock;
    start = cycles();
    for (uint8_t i = 0; i < READS; i++) {
        lock.take_blocking();
        lock.give();
    }
    const uint32_t lock_pair_cycles = (cycles() - start) / READS;
    WITH_SEMAPHORE(_sem);
    _stamp_cycles = stamp_cycles;
    _lock_pair_cycles = lock_pair_cycles;
    _scratch = scratch;
    _substages = substages;
    _m4_pre = m4_pre;
    _log_tokens_milli = LOG_ROWS_BURST * 1000U;
    _log_refill_ms = AP_HAL::millis();
    _ring = ring;
    return true;
}

int8_t CallLatencyLog::id_slot(uint32_t id)
{
    for (uint8_t i = 0; i < _id_count; i++) {
        if (_ids[i].id == id) {
            return int8_t(i);
        }
    }
    if (_id_count == SUMMARY_IDS) {
        return -1;
    }
    _ids[_id_count] = IdTotals { id, 0, INT32_MIN, 0 };
    return int8_t(_id_count++);
}

void CallLatencyLog::note_id(const CallTrace &trace)
{
    const int8_t index = id_slot(trace.id);
    if (index < 0) {
        return;
    }
    IdTotals *slot = &_ids[index];
    slot->count++;
    slot->flags = trace.flags;
    if ((trace.flags & CALL_M4_MISSING) == 0) {
        slot->max_total_us = MAX(slot->max_total_us, trace.stage_us[uint8_t(CallStage::Total)]);
    }
}

void CallLatencyLog::record(const CallTrace &trace)
{
    WITH_SEMAPHORE(_sem);
    if (_ring == nullptr) {
        return;
    }
    _ring->push(trace);
    note_id(trace);
}

void CallLatencyLog::record_substages(uint32_t id, const CallStamps &stamps)
{
    int32_t sub_us[CALL_SUBSTAGES];
    const uint32_t valid = call_substages(stamps, sub_us);
    WITH_SEMAPHORE(_sem);
    if (_ring == nullptr || valid == 0) {
        return;
    }
    const int8_t index = id_slot(id);
    if (index >= 0) {
        _substages[index].add(sub_us, valid);
    }
}

void CallLatencyLog::record_m4_pre_cycles(uint32_t id, const uint16_t (&cycles)[M4_PRE_PARTS])
{
    WITH_SEMAPHORE(_sem);
    if (_ring == nullptr) {
        return;
    }
    const int8_t index = id_slot(id);
    if (index >= 0) {
        _m4_pre[index].add(cycles);
    }
}

void CallLatencyLog::add_overhead(uint32_t overhead_cycles)
{
    WITH_SEMAPHORE(_sem);
    _overhead_cycles += overhead_cycles;
    _overhead_calls++;
    _overhead_max_cycles = MAX(_overhead_max_cycles, overhead_cycles);
}

void CallLatencyLog::flush(uint32_t now_ms)
{
#if HAL_LOGGING_ENABLED
    if (_ring == nullptr) {
        return;
    }
    _log_tokens_milli = MIN(uint32_t(LOG_ROWS_BURST) * 1000U,
                            _log_tokens_milli + (now_ms - _log_refill_ms) * LOG_ROWS_PER_SECOND);
    _log_refill_ms = now_ms;
    while (_log_tokens_milli >= 1000U) {
        CallTrace trace;
        {
            WITH_SEMAPHORE(_sem);
            if (!_ring->next_unflushed(trace)) {
                return;
            }
            _logged++;
        }
        _log_tokens_milli -= 1000U;
        const int32_t *s = trace.stage_us;
        // @LoggerMessage: M4LT
        // @Description: Rust binding call latency probe: the stages of one call from one microsecond counter (TIM5 on hardware), at most 50 rows a second (MOD_M4_OPTS bit 2 on the M4, MOD_DEBUG_OPTS bit 3 in process)
        // @Field: TimeUS: Time since system startup, when the M7 recorded the call
        // @Field: Id: BindingId (the SRPC rows of the same log name it)
        // @Field: Fl: flags: bits 0-1 lane (0 in process, 1 fast, 2 slow worker), 2 M4 module thread slept for the reply, 3 M4 wait before the reply was drained timed out, 4 sent by a later M4 drain, 5 pass followed a duty-cap pause, 6 refused unrun (duty cap), 7 over quota (unrun), 8 probe took the adapter's lock first, 9 adapter takes a lock the probe cannot, 10 M4 completion missing, 12-15 frames before it in its pass
        // @Field: Pre: M4 binding call entry to frame ready to send (in process: host binding call entry to the dispatch)
        // @Field: Snd: M4 ring write and doorbell
        // @Field: Rcv: frame on the ring until the M7 host took it
        // @Field: Pas: part of Rcv: the M7 host's pass before this frame
        // @Field: Hof: hand-off to the slow worker (0 on the fast lane)
        // @Field: Chk: frame checks, quota and input copy before the lock
        // @Field: Sem: wait for the adapter's lock (scheduler or AHRS semaphore)
        // @Field: Adp: adapter
        // @Field: Rpl: M7 reply framing, ring write and doorbell
        // @Field: Bck: reply on the ring until the M4 service took it
        // @Field: Ret: M4 module thread wake and reply copy to the return of the call
        // @Field: Tot: binding call entry to return
        // @Field: Seq: frame sequence number (in process: a call counter)
        AP::logger().WriteStreaming("M4LT", "TimeUS,Id,Fl,Pre,Snd,Rcv,Pas,Hof,Chk,Sem,Adp,Rpl,Bck,Ret,Tot,Seq",
                                    "s--ssssssssssss-", "F--FFFFFFFFFFFF-", "QHHiiiiiiiiiiiiI",
                                    trace.time_us, trace.id, trace.flags, s[0], s[1], s[2], s[3], s[4], s[5], s[6],
                                    s[7], s[8], s[9], s[10], s[11], trace.seq);
    }
#endif
}

void CallLatencyLog::summary(ExpandingString &str)
{
    WITH_SEMAPHORE(_sem);
    if (_ring == nullptr) {
        str.printf("call latency probe off (MOD_M4_OPTS bit 2, or MOD_DEBUG_OPTS bit 3 in process)\n");
        return;
    }
    const uint32_t per_us = MAX(cycles_per_us(), 1U);
    str.printf("calls %lu, window %u (last calls), logged %lu, not logged %lu\n",
               (unsigned long)_ring->written(), unsigned(_ring->count()), (unsigned long)_logged,
               (unsigned long)_ring->skipped());
    str.printf("probe cost: M7 stamp %lu cycles, lock take+give %lu cycles, M4 stamp %lu cycles, "
               "M7 per call mean %lu max %lu cycles (%lu cycles/us)\n",
               (unsigned long)_stamp_cycles, (unsigned long)_lock_pair_cycles, (unsigned long)_m4_stamp_cycles,
               (unsigned long)(_overhead_calls != 0 ? _overhead_cycles / _overhead_calls : 0),
               (unsigned long)_overhead_max_cycles, (unsigned long)per_us);
    str.printf("memset cost: M7 4096 B of the serve input %lu cycles, M4 64 B %u.%02u and 4 KB %u.%02u cycles/byte "
               "(0: not measured)\n",
               (unsigned long)_m7_memset_cycles, unsigned(_m4_memset_small_centicycles / 100U),
               unsigned(_m4_memset_small_centicycles % 100U), unsigned(_m4_memset_large_centicycles / 100U),
               unsigned(_m4_memset_large_centicycles % 100U));
    static const char *const lanes[] = { "in-process", "fast", "slow", "?" };
    for (uint8_t i = 0; i < _id_count; i++) {
        const IdTotals &totals = _ids[i];
        str.printf("id %lu %s: calls %lu, max Tot %ld us; window p50/p99/max us, sub mean/max us and m4cyc "
                   "mean/max M4 cycles since start:\n",
                   (unsigned long)totals.id, lanes[totals.flags & CALL_LANE_MASK], (unsigned long)totals.count, long(totals.max_total_us));
        for (uint8_t stage = 0; stage < CALL_STAGES; stage++) {
            const StageSummary s = summarize_stage(*_ring, totals.id, CallStage(stage), _scratch);
            if (s.count == 0) {
                continue;
            }
            str.printf("  %s n %u %ld %ld %ld\n", CALL_STAGE_NAMES[stage], unsigned(s.count), long(s.p50_us),
                       long(s.p99_us), long(s.max_us));
        }
        // since start, not the window
        const SubstageTotals &sub = _substages[i];
        for (uint8_t substage = 0; substage < CALL_SUBSTAGES; substage++) {
            if (sub.count[substage] == 0) {
                continue;
            }
            str.printf("  sub %s n %lu mean %ld max %ld\n", CALL_SUBSTAGE_NAMES[substage],
                       (unsigned long)sub.count[substage], long(sub.mean_us(substage)), long(sub.max_us[substage]));
        }
        // M4 core cycles, since start
        const M4PreCycleTotals &pre = _m4_pre[i];
        for (uint8_t part = 0; part < M4_PRE_PARTS && pre.count != 0; part++) {
            str.printf("  m4cyc %s n %lu mean %lu max %u\n", M4_PRE_PART_NAMES[part], (unsigned long)pre.count,
                       (unsigned long)pre.mean(part), unsigned(pre.max[part]));
        }
    }
}

}  // namespace AP_Module

#endif  // AP_MODULE_ENABLED
