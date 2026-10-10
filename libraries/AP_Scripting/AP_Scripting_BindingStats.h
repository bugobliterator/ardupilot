#pragma once

#include "AP_Scripting_config.h"

#if AP_SCRIPTING_ENABLED && AP_SCRIPTING_BINDING_STATS_ENABLED

#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/system.h>

struct lua_State;

namespace AP_Scripting_BindingStats {

typedef int (*lua_fn)(lua_State *L);

// Runs fn and records its duration under id. A Lua error raised inside fn longjmps past
// the recording, so failed calls are not counted.
int call(lua_State *L, uint16_t id, lua_fn fn);
void record_sem_wait(uint16_t id, uint32_t wait_us);

struct Snapshot {
    uint16_t id;
    uint32_t count;
    uint32_t p50_us;
    uint32_t p99_us;
    uint32_t max_us;
    uint32_t sem_max_us;
    uint64_t total_us;
};
// fills out[] with up to max entries of the current window, largest total_us first
uint8_t top(Snapshot *out, uint8_t max);
void reset_window();
const char *name(uint16_t id);

}

template <uint16_t ID, AP_Scripting_BindingStats::lua_fn FN>
static int binding_stats_wrap(lua_State *L)
{
    return AP_Scripting_BindingStats::call(L, ID, FN);
}

#define BINDING_FN(id, fn) binding_stats_wrap<id, fn>
#define BINDING_SEM_WAIT_BEGIN() const uint32_t binding_sem_t0 = AP_HAL::micros()
#define BINDING_SEM_WAIT_END(id) AP_Scripting_BindingStats::record_sem_wait(id, AP_HAL::micros() - binding_sem_t0)

#else

#define BINDING_FN(id, fn) fn
#define BINDING_SEM_WAIT_BEGIN()
#define BINDING_SEM_WAIT_END(id)

#endif  // AP_SCRIPTING_BINDING_STATS_ENABLED
