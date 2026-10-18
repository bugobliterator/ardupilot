/*
  M4 image side of the Lua thread (spec §4). The thread starts privileged on
  a small stack in privileged RAM and drops to unprivileged mode on its Lua
  RAM stack (PORT_USE_SYSCALL). From then on privileged code runs for it
  only in the six SVCs below, and pointers it passes are checked against
  Lua RAM before privileged code copies through them. This object's statics
  are privileged (scripting_m4.ld) except the two placed in Lua RAM here.
 */
#include "ch.h"

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <AP_ScriptingM4_Layout.h>
#include "../m4_lua.h"
#include "../m4_port.h"
#include "lua_port.h"

using namespace ScriptingM4;

// SVC numbers; r0-r3 carry the arguments, r0 the result
enum : uint32_t {
    SYS_CALL = 0,           // (request, request_len, reply, reply_cap) -> reply length or -errno
    SYS_MILLIS = 1,         // () -> ms
    SYS_MICROS = 2,         // () -> us
    SYS_SLEEP_MS = 3,       // (ms)
    SYS_FAULT_REQUEST = 4,  // () -> FaultKind
    SYS_M7_TIME = 5,        // () -> TIM5 count (the M7's micros()) in r0, this core's ms in r1
    SYS_COUNT = 6,
};
static_assert(SYS_COUNT == 6, "port_syscall compares against the handler count");

static const uint32_t STACK_GUARD_SIZE = 32;

extern "C" uint8_t __ram1_end__[], __heap_base__[], __heap_end__[];

// the Lua thread's stack, first in Lua RAM (scripting_m4.ld .lua_stack): an
// overflow runs into the guard, then into privileged RAM, and faults
alignas(32) static uint8_t lua_stack[AP_SCRIPTING_M4_LUA_STACK_SIZE] __attribute__((section(".bss.lua_stack")));
static_assert(AP_SCRIPTING_M4_LUA_STACK_SIZE % 32 == 0, "the guard needs a 32-byte aligned stack");

// newlib's heap break: _sbrk runs on the Lua thread (scripting_m4.ld .lua_bss)
static uint8_t *heap_break __attribute__((section(".bss.m4lua")));

// the Lua thread's copy of m7_counter_running, set before it starts
static bool m7_time_available __attribute__((section(".bss.m4lua")));

// privileged: the thread's syscall stack and what the SVCs use
static THD_WORKING_AREA(lua_wa, 2048);
static LuaLink *lua_link;
static binary_semaphore_t *service_wake;
static bool (*service_forward_posted)();
static binary_semaphore_t lua_done;
static uint32_t micros_last_cycles;
static uint64_t micros_cycles;
static bool m7_counter_running;

// Lua RAM above the stack guard
static bool in_lua_ram(uint32_t address, uint32_t length)
{
    const uint32_t base = uint32_t(uintptr_t(lua_stack)) + STACK_GUARD_SIZE;
    const uint32_t end = uint32_t(uintptr_t(__ram1_end__));
    return address >= base && address <= end && length <= end - address;
}

static void sys_call(struct port_extctx *ctx)
{
    const uint32_t request = ctx->r0;
    const uint32_t request_len = ctx->r1;
    const uint32_t reply = ctx->r2;
    const uint32_t reply_cap = ctx->r3;
    if (request_len > MAX_FRAME || reply_cap > MAX_FRAME ||
        !in_lua_ram(request, request_len) || !in_lua_ram(reply, reply_cap)) {
        ctx->r0 = uint32_t(-EFAULT);
        return;
    }
    LuaLink &link = *lua_link;
    memcpy(link.request, reinterpret_cast<const void *>(uintptr_t(request)), request_len);
    link.request_len = uint16_t(request_len);
    link.state.store(LuaLink::Posted);
    // sent from here when the service could; otherwise the service's next step sends it
    if (!service_forward_posted()) {
        chBSemSignal(service_wake);
    }
    // the service completes every posted call; sent ones within CALL_TIMEOUT_MS
    while (link.state.load() != LuaLink::Done) {
        (void)chBSemWaitTimeout(&lua_done, TIME_MS2I(10));
    }
    int32_t result = link.result;
    if (result > 0) {
        if (uint32_t(result) > reply_cap) {
            result = -EMSGSIZE;
        } else {
            memcpy(reinterpret_cast<void *>(uintptr_t(reply)), link.reply, uint32_t(result));
        }
    }
    link.state.store(LuaLink::Idle);
    ctx->r0 = uint32_t(result);
}

static void sys_millis(struct port_extctx *ctx)
{
    ctx->r0 = uint32_t(TIME_I2MS(chVTGetSystemTimeX()));
}

// DWT cycles carried in 64 bits: differences between calls less than the 21 s
// CYCCNT wrap apart (a script's run time) are exact
static void sys_micros(struct port_extctx *ctx)
{
    const uint32_t cycles = DWT->CYCCNT;
    micros_cycles += cycles - micros_last_cycles;
    micros_last_cycles = cycles;
    ctx->r0 = uint32_t(micros_cycles / (AP_SCRIPTING_M4_CORE_CLOCK_HZ / 1000000U));
}

/*
  TIM5 is the M7's ChibiOS system timer (1 MHz, 32 bits): its count is the M7's
  AP_HAL::micros(). TIM5 and RCC lie outside every MPU region; privileged code
  reads them through the default memory map (PRIVDEFENA) for one read, with the
  kernel locked so that nothing else runs meanwhile. The Lua thread never gets
  that map: PRIVDEFENA applies to privileged accesses only.
 */
enum class OutsideRegister : uint8_t {
    Tim5Count,
    Tim5Control,
    Tim5AutoReload,
    RccC1Apb1L,
};

// the registers are fixed here: no caller can pass an address
static uint32_t read_outside_regions(OutsideRegister reg)
{
    // no default: -Wswitch flags a register without a case
    const volatile uint32_t *address = nullptr;
    switch (reg) {
    case OutsideRegister::Tim5Count:
        address = &TIM5->CNT;
        break;
    case OutsideRegister::Tim5Control:
        address = &TIM5->CR1;
        break;
    case OutsideRegister::Tim5AutoReload:
        address = &TIM5->ARR;
        break;
    case OutsideRegister::RccC1Apb1L:
        address = &RCC_C1->APB1LENR;
        break;
    }
    chSysLock();
    const uint32_t control = MPU->CTRL;
    MPU->CTRL = control | MPU_CTRL_PRIVDEFENA_Msk;
    __DSB();
    __ISB();
    const uint32_t value = *address;
    MPU->CTRL = control;
    __DSB();
    __ISB();
    chSysUnlock();
    return value;
}

// the M7 enables TIM5's clock (RCC_C1) and starts it in halInit() as its 32-bit system timer
// (full-range ARR), before it releases this core. Any other use of TIM5 (another M7 image or
// board, PWM) keeps millis()/micros() remote calls
static bool m7_counter_started()
{
    return (read_outside_regions(OutsideRegister::RccC1Apb1L) & RCC_APB1LENR_TIM5EN) != 0 &&
           (read_outside_regions(OutsideRegister::Tim5Control) & TIM_CR1_CEN) != 0 &&
           read_outside_regions(OutsideRegister::Tim5AutoReload) == 0xFFFFFFFFU;
}

// this core's ticks are milliseconds (sys_millis without TIME_I2MS's 64-bit division)
static_assert(CH_CFG_ST_FREQUENCY == 1000, "sys_m7_time returns ticks as milliseconds");
static void sys_m7_time(struct port_extctx *ctx)
{
    ctx->r1 = uint32_t(chVTGetSystemTimeX());
    ctx->r0 = m7_counter_running ? read_outside_regions(OutsideRegister::Tim5Count) : 0;
}

static void sys_sleep_ms(struct port_extctx *ctx)
{
    const uint32_t ms = ctx->r0 < 60000U ? ctx->r0 : 60000U;
    if (ms == 0) {
        chThdYield();
    } else {
        chThdSleepMilliseconds(ms);
    }
    ctx->r0 = 0;
}

static void sys_fault_request(struct port_extctx *ctx)
{
    ctx->r0 = lua_link->fault_request.exchange(uint8_t(FaultKind::None));
}

typedef void (*syscall_handler)(struct port_extctx *ctx);

extern "C" {

extern const syscall_handler m4_syscalls[SYS_COUNT];
__attribute__((used)) const syscall_handler m4_syscalls[SYS_COUNT] = {
    sys_call, sys_millis, sys_micros, sys_sleep_ms, sys_fault_request, sys_m7_time,
};

__attribute__((used)) void m4_syscall_invalid(struct port_extctx *ctx)
{
    ctx->r0 = uint32_t(-ENOSYS);
}

/*
  Replaces the port's weak port_syscall(). The SVC handler enters it in thread
  mode, privileged, on the Lua thread's privileged stack, with the caller's
  frame in r0 and the SVC number in r1; "svc 1" from privileged mode then
  returns to the caller, unprivileged (ARMv7-M chcore.c SVC_Handler). No stack
  use here: that handler finds its link context right above the frame "svc 1"
  pushes (as ChibiOS os/sb/host/compilers/GCC/sbexc.S).
 */
__attribute__((naked)) void port_syscall(struct port_extctx *, uint32_t)
{
    __asm volatile(
        "cmp    r1, #6\n"
        "bhs    1f\n"
        "movw   r3, #:lower16:m4_syscalls\n"
        "movt   r3, #:upper16:m4_syscalls\n"
        "ldr    r3, [r3, r1, lsl #2]\n"
        "blx    r3\n"
        "svc    1\n"
        "1:\n"
        "bl     m4_syscall_invalid\n"
        "svc    1\n");
}

// the Lua thread's entry, unprivileged
void lua_entry(void)
{
    ScriptingM4::lua_main();
}

// newlib malloc (the Lua heap) grows into the rest of Lua RAM; runs on the Lua thread
void *_sbrk(ptrdiff_t increment)
{
    if (heap_break == nullptr) {
        heap_break = __heap_base__;
    }
    if (increment < 0 || increment > __heap_end__ - heap_break) {
        errno = ENOMEM;
        return reinterpret_cast<void *>(-1);
    }
    uint8_t *previous = heap_break;
    heap_break += increment;
    return previous;
}

// newlib's abort() and assert end in nosys _exit's spin: fault instead, so the M7 restarts the M4
void abort(void)
{
    ScriptingM4::port_lua_fault(FaultKind::HardFault);
}

void __assert_func(const char *, int, const char *, const char *)
{
    ScriptingM4::port_lua_fault(FaultKind::HardFault);
}

}  // extern "C"

// r0-r3 are clobbered, so the compiler keeps no argument in them before the moves
template <uint32_t N>
static inline uint32_t svc(uint32_t a0 = 0, uint32_t a1 = 0, uint32_t a2 = 0, uint32_t a3 = 0)
{
    uint32_t result;
    __asm volatile(
        "mov    r0, %[a0]\n"
        "mov    r1, %[a1]\n"
        "mov    r2, %[a2]\n"
        "mov    r3, %[a3]\n"
        "svc    %[number]\n"
        "mov    %[result], r0\n"
        : [result] "=r"(result)
        : [number] "i"(N), [a0] "r"(a0), [a1] "r"(a1), [a2] "r"(a2), [a3] "r"(a3)
        : "r0", "r1", "r2", "r3", "memory");
    return result;
}

int32_t ScriptingM4::port_call(const uint8_t *request, uint16_t request_len, uint8_t *reply, uint16_t reply_cap)
{
    return int32_t(svc<SYS_CALL>(uint32_t(uintptr_t(request)), request_len, uint32_t(uintptr_t(reply)), reply_cap));
}

uint32_t ScriptingM4::port_millis()
{
    return svc<SYS_MILLIS>();
}

uint32_t ScriptingM4::port_micros()
{
    return svc<SYS_MICROS>();
}

void ScriptingM4::port_sleep_ms(uint32_t ms)
{
    (void)svc<SYS_SLEEP_MS>(ms);
}

FaultKind ScriptingM4::port_fault_request()
{
    return FaultKind(svc<SYS_FAULT_REQUEST>());
}

bool ScriptingM4::port_m7_time_available()
{
    return m7_time_available;
}

M7Counter ScriptingM4::port_m7_counter()
{
    uint32_t counter;
    uint32_t local_ms;
    __asm volatile(
        "svc    %[number]\n"
        "mov    %[counter], r0\n"
        "mov    %[local_ms], r1\n"
        : [counter] "=r"(counter), [local_ms] "=r"(local_ms)
        : [number] "i"(SYS_M7_TIME)
        : "r0", "r1", "r2", "r3", "memory");
    return M7Counter { counter, local_ms };
}

uint32_t ScriptingM4::port_heap_capacity()
{
    return uint32_t(__heap_end__ - __heap_base__);   // linker symbols: no memory access
}

__attribute__((noinline)) static uint32_t overflow_stack(uint32_t depth)
{
    volatile uint8_t frame[256];
    frame[0] = uint8_t(depth);
    return overflow_stack(depth + 1) + frame[0];   // not a tail call
}

// unprivileged: each of these faults; none may reset the system or reach RCC
void ScriptingM4::port_lua_fault(FaultKind kind)
{
    switch (kind) {
    case FaultKind::LuaAircr:
        // AIRCR = VECTKEY | SYSRESETREQ: the PPB is privileged-only, so a precise BusFault
        *reinterpret_cast<volatile uint32_t *>(0xE000ED0CU) = 0x05FA0004U;
        break;
    case FaultKind::LuaStackOverflow:
        (void)overflow_stack(0);   // MemManage at the guard (region 6) or privileged RAM below it
        break;
    case FaultKind::LuaRccWrite:
        *reinterpret_cast<volatile uint32_t *>(0x580244A0U) = 0;   // RCC_GCR: no region, MemManage
        break;
    default:
        break;
    }
    for (;;) {
        __builtin_trap();   // UDF: escalates to HardFault (UsageFault is not enabled)
    }
}

static THD_FUNCTION(lua_thread, arg)
{
    (void)arg;
    // unprivileged from here on; the exception return wants the entry without the Thumb bit
    port_unprivileged_jump(uint32_t(uintptr_t(&lua_entry)) & ~1U,
                           uint32_t(uintptr_t(&lua_stack[sizeof(lua_stack)])));
}

void lua_port_start(LuaLink &link, binary_semaphore_t &wake, bool (*forward_posted)())
{
    lua_link = &link;
    service_wake = &wake;
    service_forward_posted = forward_posted;
    m7_counter_running = m7_counter_started();
    m7_time_available = m7_counter_running;
    chBSemObjectInit(&lua_done, true);
    // below the service (the main thread, NORMALPRIO): the heartbeat waits for Lua only for the
    // service mutex, held for one forward_call() with priority inheritance
    chThdCreateStatic(lua_wa, sizeof(lua_wa), NORMALPRIO - 1, lua_thread, nullptr);
}

void lua_port_wake()
{
    chBSemSignal(&lua_done);
}

uint32_t lua_stack_guard()
{
    return uint32_t(uintptr_t(lua_stack));
}
