#pragma once

#include "AP_Rust_config.h"

#if AP_SCRIPTING_RUST_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include "CallLatency.h"

class ExpandingString;

namespace AP_Rust {

/*
  Collector of the binding call latency probe (RUST_DEBUG_OPTS bit 3 in
  process, RUST_M4_OPTS bit 2 on the M4): the last CALL_TRACE_COUNT calls,
  logged as M4LT rows at most LOG_ROWS_PER_SECOND, and the @SYS/rust_latency.txt
  summary. Its memory is allocated by the first start(): none while the
  probe is off.
 */
class CallLatencyLog {
public:
    static const uint16_t CALL_TRACE_COUNT = 512;
    static const uint8_t LOG_ROWS_PER_SECOND = 50;

    static CallLatencyLog &get();

    // the stage counter both cores read: TIM5's count on the STM32 (the M7's system timer, which
    // the M4 reads too), CLOCK_MONOTONIC in SITL, where the M4 is another process
    static uint32_t clock_us();
    // a cycle count for the probe's own cost (ChibiOS: DWT); cycles_per_us() of them a microsecond
    static uint32_t cycles();
    static uint32_t cycles_per_us();

    // the lock the generated adapter of id takes, for the probe to take first (recursive locks:
    // the adapter's own take then waits for nothing); null if none, or one the probe cannot take,
    // which is flagged CALL_LOCK_UNKNOWN
    static AP_HAL::Semaphore *adapter_lock(uint32_t id, uint16_t &flags);

    // allocates the ring at the first call and measures the cost of a stamp; false if out of memory
    bool start();
    bool active() const { return _ring != nullptr; }

    void record(const CallTrace &trace);
    // cycles the probe spent on one call besides its stamps' waits
    void add_overhead(uint32_t cycles);
    // the sub-stages of a recorded call (its stamps), summed per binding for the summary
    void record_substages(uint32_t id, const CallStamps &stamps);
    // the M4's cost of one stamp, as its trailers report it
    void set_m4_stamp_cycles(uint32_t cycles) { _m4_stamp_cycles = cycles; }
    // the M4's memset cost in hundredths of a cycle per byte (64 B on its stack, 4 KB of its link
    // buffers), as its trailers report it; 0 until measured
    void set_m4_memset(uint16_t small_centicycles, uint16_t large_centicycles)
    {
        _m4_memset_small_centicycles = small_centicycles;
        _m4_memset_large_centicycles = large_centicycles;
    }
    // cycles of one memset of the M7 server's 4096 B input buffer
    void set_m7_memset_cycles(uint32_t cycles) { _m7_memset_cycles = cycles; }
    // logs the calls not yet logged, within LOG_ROWS_PER_SECOND (a burst of LOG_ROWS_BURST)
    void flush(uint32_t now_ms);
    // @SYS/rust_latency.txt
    void summary(ExpandingString &str);

private:
    static const uint8_t LOG_ROWS_BURST = 10;
    static const uint8_t SUMMARY_IDS = 32;

    struct IdTotals {
        uint32_t id;
        uint32_t count;
        int32_t max_total_us;
        uint16_t flags;   // of its last call: the lane
    };

    // the IdTotals slot of id (claimed if new), or -1 when every slot is taken
    int8_t id_slot(uint32_t id);
    void note_id(const CallTrace &trace);

    HAL_Semaphore _sem;
    CallTraceRing<CALL_TRACE_COUNT> *_ring;
    int32_t *_scratch;
    IdTotals _ids[SUMMARY_IDS];
    SubstageTotals *_substages;    // SUMMARY_IDS of them, by id slot
    uint8_t _id_count;
    uint32_t _logged;
    uint32_t _log_tokens_milli;
    uint32_t _log_refill_ms;
    uint32_t _stamp_cycles;        // one clock_us() read on this core
    uint32_t _lock_pair_cycles;    // an uncontended take and give
    uint32_t _m4_stamp_cycles;
    uint16_t _m4_memset_small_centicycles;
    uint16_t _m4_memset_large_centicycles;
    uint32_t _m7_memset_cycles;
    uint64_t _overhead_cycles;
    uint32_t _overhead_calls;
    uint32_t _overhead_max_cycles;
};

}  // namespace AP_Rust

#endif  // AP_SCRIPTING_RUST_ENABLED
