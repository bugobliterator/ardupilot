#pragma once

/*
  The per-window call statistics of the M4 service as plain rows, and the steps
  that pick the busiest rows and fold the rest into one. No ArduPilot or
  CubeFramework dependencies: the M4 service and the host unit tests include it.
 */

#include <stdint.h>

namespace RustM4 {

// Stats rows the M4 sends per one-second window: one for each of the STATS_REPORTED methods with
// the most round-trip time, then m4:other and m4:sysother
static const uint8_t STATS_REPORTED = 8;
static const uint8_t STATS_ROWS_PER_WINDOW = STATS_REPORTED + 2;

struct StatsRow {
    uint16_t method;
    uint32_t count;
    uint32_t p50_us;
    uint32_t p99_us;
    uint32_t max_us;
    uint32_t total_us;
    uint32_t m7_max_us;
    uint32_t lost;
};

// order[0..count) = indexes of totals[0..count), largest total first; equal totals keep their index order
inline void stats_rank(const uint32_t *totals, uint8_t count, uint8_t *order)
{
    for (uint8_t i = 0; i < count; i++) {
        uint8_t j = i;
        while (j > 0 && totals[order[j - 1]] < totals[i]) {
            order[j] = order[j - 1];
            j--;
        }
        order[j] = i;
    }
}

// counts and totals add up; p50, p99, max and m7_max are the largest, so upper bounds
inline void stats_add(StatsRow &sum, const StatsRow &row)
{
    sum.count += row.count;
    sum.total_us += row.total_us;
    sum.lost += row.lost;
    sum.p50_us = row.p50_us > sum.p50_us ? row.p50_us : sum.p50_us;
    sum.p99_us = row.p99_us > sum.p99_us ? row.p99_us : sum.p99_us;
    sum.max_us = row.max_us > sum.max_us ? row.max_us : sum.max_us;
    sum.m7_max_us = row.m7_max_us > sum.m7_max_us ? row.m7_max_us : sum.m7_max_us;
}

}  // namespace RustM4
