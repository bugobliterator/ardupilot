/*
  SITL side of the M4 Lua thread: a pthread of the M4 process, over the same
  LuaLink and M4Service as the M4 image. There is no privilege split on a
  host; Lua-thread faults are signals, recorded by main.cpp's handler.
 */
#include "lua_port.h"

#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <AP_ScriptingM4_Placement.h>
#include "../m4_lua.h"
#include "../m4_port.h"

using namespace ScriptingM4;

static LuaLink *lua_link;
static bool (*service_forward_posted)();
static pthread_mutex_t done_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t done_cond = PTHREAD_COND_INITIALIZER;

static uint64_t monotonic_us()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * 1000000U + uint64_t(ts.tv_nsec) / 1000U;
}

int32_t ScriptingM4::port_call(const uint8_t *request, uint16_t request_len, uint8_t *reply, uint16_t reply_cap)
{
    if (request_len > MAX_FRAME) {
        return -EMSGSIZE;
    }
    LuaLink &link = *lua_link;
    memcpy(link.request, request, request_len);
    link.request_len = request_len;
    link.state.store(LuaLink::Posted);
    // sent from here when the service could; otherwise its next step (1 ms poll) sends it
    (void)service_forward_posted();
    // the service completes every posted call within CALL_TIMEOUT_MS of posting
    pthread_mutex_lock(&done_mutex);
    while (link.state.load() != LuaLink::Done) {
        struct timespec deadline;
        clock_gettime(CLOCK_REALTIME, &deadline);
        deadline.tv_nsec += 10 * 1000000L;
        if (deadline.tv_nsec >= 1000000000L) {
            deadline.tv_sec++;
            deadline.tv_nsec -= 1000000000L;
        }
        pthread_cond_timedwait(&done_cond, &done_mutex, &deadline);
    }
    pthread_mutex_unlock(&done_mutex);
    int32_t result = link.result;
    if (result > 0) {
        if (uint32_t(result) > reply_cap) {
            result = -EMSGSIZE;
        } else {
            memcpy(reply, link.reply, size_t(result));
        }
    }
    link.state.store(LuaLink::Idle);
    return result;
}

uint32_t ScriptingM4::port_millis()
{
    return uint32_t(monotonic_us() / 1000U);
}

uint32_t ScriptingM4::port_micros()
{
    return uint32_t(monotonic_us());
}

// the M7 is another process here: millis() and micros() stay remote calls
bool ScriptingM4::port_m7_time_available()
{
    return false;
}

M7Counter ScriptingM4::port_m7_counter()
{
    return M7Counter { 0, port_millis() };
}

void ScriptingM4::port_sleep_ms(uint32_t ms)
{
    struct timespec ts { time_t(ms / 1000), long(ms % 1000) * 1000000L };
    nanosleep(&ts, nullptr);
}

FaultKind ScriptingM4::port_fault_request()
{
    return FaultKind(lua_link->fault_request.exchange(0));
}

__attribute__((noinline)) static uint32_t overflow_stack(uint32_t depth)
{
    volatile uint8_t frame[1024];
    frame[0] = uint8_t(depth);
    if (depth == UINT32_MAX) {
        return frame[0];   // never reached: the stack runs out first
    }
    return overflow_stack(depth + 1) + frame[0];   // not a tail call
}

void ScriptingM4::port_lua_fault(FaultKind kind)
{
    switch (kind) {
    case FaultKind::LuaAircr:
    case FaultKind::LuaRccWrite:
        // stand-in: SITL has neither AIRCR nor RCC; a stray write is a SIGSEGV/SIGBUS
        *reinterpret_cast<volatile uint32_t *>(uintptr_t(8)) = 0;
        break;
    case FaultKind::LuaStackOverflow:
        (void)overflow_stack(0);
        break;
    default:
        break;
    }
    for (;;) {
        __builtin_trap();
    }
}

uint32_t ScriptingM4::port_heap_capacity()
{
    // the heap the M4 image guarantees, charged with the model below: a script that
    // fits here fits there
    return AP_SCRIPTING_M4_LUA_HEAP_MIN;
}

/*
  A model of newlib-nano's malloc on the host allocator. Each block costs
  heap_block_cost() of the size it was allocated at (at least newlib-nano's
  chunk) and, as newlib-nano's realloc does, keeps that cost while realloc
  finds it big enough. The header in front of the block holds the cost.
 */
static const size_t BLOCK_HEADER = 16;   // keeps the block 16-aligned

static uint32_t &charged_cost(void *block)
{
    return *reinterpret_cast<uint32_t *>(static_cast<uint8_t *>(block) - BLOCK_HEADER);
}

void *ScriptingM4::port_heap_realloc(void *block, size_t size)
{
    // the model's usable size is its cost less the 8 bytes of chunk overhead
    if (block != nullptr && size <= charged_cost(block) - 8) {
        return block;
    }
    const uint32_t cost = heap_block_cost(size);
    void *base = block != nullptr ? static_cast<uint8_t *>(block) - BLOCK_HEADER : nullptr;
    uint8_t *grown = static_cast<uint8_t *>(realloc(base, BLOCK_HEADER + cost - 8));
    if (grown == nullptr) {
        return nullptr;
    }
    charged_cost(grown + BLOCK_HEADER) = cost;
    return grown + BLOCK_HEADER;
}

void ScriptingM4::port_heap_free(void *block)
{
    if (block != nullptr) {
        free(static_cast<uint8_t *>(block) - BLOCK_HEADER);
    }
}

uint32_t ScriptingM4::port_heap_block_cost(void *block)
{
    return charged_cost(block);
}

void lua_port_wake()
{
    pthread_mutex_lock(&done_mutex);
    pthread_cond_broadcast(&done_cond);
    pthread_mutex_unlock(&done_mutex);
}

static void *lua_thread(void *)
{
    // fault handlers run on an alternate stack (per thread): a stack overflow is still recorded
    static uint8_t alternate_stack[64 * 1024];
    stack_t ss {};
    ss.ss_sp = alternate_stack;
    ss.ss_size = sizeof(alternate_stack);
    sigaltstack(&ss, nullptr);
    lua_main();
}

bool lua_port_start(LuaLink &link, bool (*forward_posted)())
{
    lua_link = &link;
    service_forward_posted = forward_posted;
    // the restart and Off signal (SIGUSR1) is the service thread's alone, as the restart
    // doorbell interrupts only the service on the M4: a Lua thread that took it would
    // restart a locked-up service. The mask is inherited, so the thread never has it unblocked.
    sigset_t restart;
    sigset_t previous;
    sigemptyset(&restart);
    sigaddset(&restart, SIGUSR1);
    pthread_sigmask(SIG_BLOCK, &restart, &previous);
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 256 * 1024);
    pthread_t thread;
    const bool started = pthread_create(&thread, &attr, lua_thread, nullptr) == 0;
    pthread_attr_destroy(&attr);
    pthread_sigmask(SIG_SETMASK, &previous, nullptr);
    return started;
}
