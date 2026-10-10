#include "AP_Scripting_BindingStats.h"

#if AP_SCRIPTING_ENABLED && AP_SCRIPTING_BINDING_STATS_ENABLED

#include "AP_Scripting_BindingHistogram.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

extern const char *const binding_stats_names[];
extern const uint16_t binding_stats_count;

namespace AP_Scripting_BindingStats {

struct Entry {
    BindingHistogram hist;
    uint64_t total_us;
    uint32_t sem_max_us;
};

// one Entry per binding (~80 B each); allocated on first use so builds that never call a
// binding pay nothing
static Entry *entries;

static Entry *entry(uint16_t id)
{
    if (id >= binding_stats_count) {
        return nullptr;
    }
    if (entries == nullptr) {
        entries = NEW_NOTHROW Entry[binding_stats_count];
        if (entries == nullptr) {
            return nullptr;
        }
        reset_window();
    }
    return &entries[id];
}

int call(lua_State *L, uint16_t id, lua_fn fn)
{
    const uint32_t t0 = AP_HAL::micros();
    const int ret = fn(L);
    const uint32_t dt = AP_HAL::micros() - t0;
    Entry *e = entry(id);
    if (e != nullptr) {
        e->hist.add(dt);
        e->total_us += dt;
    }
    return ret;
}

void record_sem_wait(uint16_t id, uint32_t wait_us)
{
    Entry *e = entry(id);
    if (e != nullptr && wait_us > e->sem_max_us) {
        e->sem_max_us = wait_us;
    }
}

void reset_window()
{
    if (entries == nullptr) {
        return;
    }
    for (uint16_t i = 0; i < binding_stats_count; i++) {
        entries[i].hist.reset();
        entries[i].total_us = 0;
        entries[i].sem_max_us = 0;
    }
}

uint8_t top(Snapshot *out, uint8_t max)
{
    if (entries == nullptr || max == 0) {
        return 0;
    }
    uint8_t n = 0;
    for (uint16_t i = 0; i < binding_stats_count; i++) {
        const Entry &e = entries[i];
        if (e.hist.count() == 0) {
            continue;
        }
        // insertion into out[], kept sorted by total_us descending
        uint8_t pos = n;
        while (pos > 0 && out[pos-1].total_us < e.total_us) {
            if (pos < max) {
                out[pos] = out[pos-1];
            }
            pos--;
        }
        if (pos >= max) {
            continue;
        }
        out[pos] = Snapshot{ i, e.hist.count(), e.hist.percentile(50), e.hist.percentile(99),
                             e.hist.max_us(), e.sem_max_us, e.total_us };
        if (n < max) {
            n++;
        }
    }
    return n;
}

const char *name(uint16_t id)
{
    return id < binding_stats_count ? binding_stats_names[id] : "?";
}

}

#endif  // AP_SCRIPTING_BINDING_STATS_ENABLED
