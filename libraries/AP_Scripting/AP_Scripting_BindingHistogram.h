#pragma once

#include <stdint.h>
#include <string.h>

// Log2 histogram of durations in microseconds. Bucket 0 holds 0 us; bucket i (1..15) holds
// [2^(i-1), 2^i) us, the last bucket also everything above. Percentiles report the bucket's
// upper bound, so "p99 = 64" reads "99% of calls took under 64 us".
class BindingHistogram {
public:
    static constexpr uint8_t BUCKETS = 16;

    void reset() { memset(this, 0, sizeof(*this)); }

    void add(uint32_t us) {
        uint8_t i = 0;
        if (us != 0) {
            i = 32 - __builtin_clz(us);
            if (i >= BUCKETS) {
                i = BUCKETS - 1;
            }
        }
        _bucket[i]++;
        _count++;
        if (us > _max_us) {
            _max_us = us;
        }
    }

    uint32_t percentile(uint8_t pct) const {
        if (_count == 0) {
            return 0;
        }
        const uint64_t target = (uint64_t(_count) * pct + 99) / 100;
        uint64_t seen = 0;
        for (uint8_t i = 0; i < BUCKETS; i++) {
            seen += _bucket[i];
            if (seen >= target) {
                return i == 0 ? 1 : (1U << i);
            }
        }
        return 1U << (BUCKETS - 1);
    }

    uint32_t count() const { return _count; }
    uint32_t max_us() const { return _max_us; }

private:
    uint32_t _bucket[BUCKETS];
    uint32_t _count;
    uint32_t _max_us;
};
