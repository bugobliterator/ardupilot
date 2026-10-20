#pragma once

#include <stdint.h>

namespace RustM4 {

/*
  The M7's AP_HAL::millis() kept on the M4. TIM5, the M7's 1 MHz 32-bit
  ChibiOS system timer, counts the low 32 bits of the M7's micros64(), so the
  M4 reads micros() from it directly. millis() is uint32_t(micros64() / 1000)
  and also needs the count of TIM5 wraps (one per 71.6 min), which TIM5 does
  not hold: the M4 takes it once from one M7 millis() value, then follows the
  wraps itself. A gap between reads longer than one wrap is found from the
  M4's own millisecond clock, which runs from the same crystal. That clock
  stops while the M4 core is halted (SWD) and TIM5 does not: a halt longer
  than one wrap (71.6 min) leaves millis() behind by whole wraps until the
  next M4 restart.
 */
struct M7Clock {
    bool calibrated;
    uint32_t counter;      // TIM5 at the last read
    uint32_t local_ms;     // the M4's own millisecond clock at that read
    uint64_t micros64;     // the M7's micros64() at that read, modulo M7_MILLIS_PERIOD_US
};

// micros64() values M7_MILLIS_PERIOD_US apart give the same uint32_t millis()
static const uint64_t M7_MILLIS_PERIOD_US = 1000ULL << 32;
static const uint64_t M7_COUNTER_PERIOD_US = 1ULL << 32;

// m7_millis: an M7 millis() value read before counter (by less than half a counter wrap)
inline void m7_clock_calibrate(M7Clock &clock, uint32_t m7_millis, uint32_t counter, uint32_t local_ms)
{
    // the counter wraps that put counter nearest to m7_millis * 1000, plus one period so that
    // nothing goes negative
    const uint64_t estimate = uint64_t(m7_millis) * 1000U + M7_MILLIS_PERIOD_US;
    const uint64_t wraps = (estimate - counter + M7_COUNTER_PERIOD_US / 2U) >> 32;
    clock.micros64 = ((wraps << 32) + counter) % M7_MILLIS_PERIOD_US;
    clock.counter = counter;
    clock.local_ms = local_ms;
    clock.calibrated = true;
}

// a later read: counter and local_ms taken together, less than 49.7 days of local_ms after the last
inline void m7_clock_advance(M7Clock &clock, uint32_t counter, uint32_t local_ms)
{
    const uint32_t delta = counter - clock.counter;
    const uint64_t elapsed_us = uint64_t(uint32_t(local_ms - clock.local_ms)) * 1000U;
    // whole counter wraps the delta missed, rounded: the two clocks agree far better than half a wrap
    const uint64_t missed = elapsed_us > delta ? (elapsed_us - delta + M7_COUNTER_PERIOD_US / 2U) >> 32 : 0;
    clock.micros64 = (clock.micros64 + delta + (missed << 32)) % M7_MILLIS_PERIOD_US;
    clock.counter = counter;
    clock.local_ms = local_ms;
}

inline uint32_t m7_clock_millis(const M7Clock &clock)
{
    return uint32_t(clock.micros64 / 1000U);
}

}  // namespace RustM4
