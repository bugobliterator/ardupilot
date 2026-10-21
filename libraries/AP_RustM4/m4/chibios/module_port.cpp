/*
  The module thread on the M4 image. It starts privileged on its syscall
  stack in privileged RAM and drops to unprivileged mode on the module's
  stack (PORT_USE_SYSCALL). From then on privileged code runs for it only
  in the SVC handlers below, in thread mode on that syscall stack, which
  call the AP_RustM4_HostCalls.h bodies. MPU region 6 is the no-access
  guard below the module stack, region 7 the module's code (RO+X).

  Local binding ids are answered inside SYS_BINDING_CALL, on this thread's
  syscall stack: newlib's float libm writes errno (impure_data, privileged
  RAM) there. That state belongs to privileged code only (modules link no
  newlib and the MPU keeps them out of ram0), and nothing on the M4 reads
  errno.
 */
#include "ch.h"

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <AP_Rust/Bindings.h>
#include <AP_RustM4_M7Clock.h>
#include "../module_port.h"
#include "module_svc.h"

using namespace RustM4;

extern "C" void module_entry_stub(void);
extern "C" int32_t m4_selftest_start(const AP_RustHostApi *api, void *context);
extern "C" uint8_t __m4_selftest_start__[], __m4_selftest_end__[];
extern "C" uint8_t __module_area_base__[], __module_area_end__[];

// what the entry stub reads at the module's stack pointer
struct LaunchRecord {
    uint32_t entry;   // with the Thumb bit
    const AP_RustHostApi *api;
    void *context;
    uint32_t rw_base;
};
static_assert(sizeof(LaunchRecord) == 16, "the stub reads the record by offset");

// privileged (rust_m4.ld ram0): the module thread's syscall stack and its state
static THD_WORKING_AREA(module_wa, AP_RUST_M4_SYSCALL_STACK_SIZE);
static thread_t *module_thread;
static ModuleContext *active;
static uint32_t module_stack_top;
static int32_t module_result;
static bool module_has_returned;
static ModuleLink *call_link_of;
static M4Service *service_of;
static void (*wake_service_of)();
static binary_semaphore_t call_done;
static binary_semaphore_t stop_wake;   // sleep_ms waits on it; a stop request signals it
static semaphore_t parked;             // nobody signals it: a panicked module waits here
static M7Clock m7_clock;
static bool m7_counter_running;
// the M7 millis() of the session's Config and this core's ticks when it arrived (set by the service)
static bool m7_millis_pending;
static uint32_t m7_millis_given;
static uint32_t m7_millis_given_ms;
static uint32_t local_last_cycles;
static uint64_t local_cycles;

// the built-in selftest's context and heap bitmaps
static ModuleContext module_context;
static uint32_t selftest_heap_used[2], selftest_heap_starts[2];
static const uint32_t SELFTEST_HEAP_SIZE = 1024;

static int32_t port_call(void *, const uint8_t *request, uint16_t len, uint8_t *reply, uint16_t cap)
{
    ModuleLink &link = *call_link_of;
    if (len > MAX_FRAME) {
        return -EMSGSIZE;
    }
    if (request != link.request) {
        memcpy(link.request, request, len);
    }
    link.request_len = len;
    link.state.store(ModuleLink::Posted);
    // sent from here when the service could; otherwise the service's next step sends it
    if (!service_of->forward_posted()) {
        wake_service_of();
    }
    // the service completes every posted call; sent ones within CALL_TIMEOUT_MS
    while (link.state.load() != ModuleLink::Done) {
        (void)chBSemWaitTimeout(&call_done, TIME_MS2I(10));
    }
    int32_t result = link.result;
    if (result > 0) {
        if (uint32_t(result) > cap) {
            result = -EMSGSIZE;
        } else if (reply != link.reply) {
            memcpy(reply, link.reply, uint32_t(result));
        }
    }
    link.state.store(ModuleLink::Idle);
    return result;
}

static void port_notify(void *platform, const uint8_t *frame, uint16_t len)
{
    (void)port_call(platform, frame, len, call_link_of->reply, MAX_FRAME);
}

static const HostPlatform host_platform {
    port_call, port_notify, AP_Rust::binding_local_wire, AP_Rust::binding_local_dispatch, nullptr
};

/*
  TIM5 is the M7's ChibiOS system timer (1 MHz, 32 bits): its count is the
  low word of the M7's micros64(). TIM5 and RCC lie outside every MPU region;
  privileged code reads them through the default memory map (PRIVDEFENA)
  for one read, with the kernel locked so that nothing else runs meanwhile.
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

// the M7 starts TIM5 in halInit() as its 32-bit system timer (full-range ARR) before it releases
// this core; any other use of TIM5 leaves time_us on this core's own cycle counter
static bool m7_counter_started()
{
    return (read_outside_regions(OutsideRegister::RccC1Apb1L) & RCC_APB1LENR_TIM5EN) != 0 &&
           (read_outside_regions(OutsideRegister::Tim5Control) & TIM_CR1_CEN) != 0 &&
           read_outside_regions(OutsideRegister::Tim5AutoReload) == 0xFFFFFFFFU;
}

static_assert(CH_CFG_ST_FREQUENCY == 1000, "M7Clock takes this core's ticks as milliseconds");

// the M7's micros64(), its wraps followed from the first read. The wraps are taken from the M7's
// millis() in the session's Config; before that, the first read assumes the M7 ran less than one
// TIM5 wrap (71.6 min)
static uint64_t time_us()
{
    if (!m7_counter_running) {
        const uint32_t cycles = DWT->CYCCNT;
        local_cycles += cycles - local_last_cycles;
        local_last_cycles = cycles;
        return local_cycles / (AP_RUST_M4_CORE_CLOCK_HZ / 1000000U);
    }
    const uint32_t local_ms = uint32_t(chVTGetSystemTimeX());
    const uint32_t counter = read_outside_regions(OutsideRegister::Tim5Count);
    chSysLock();
    const bool given = m7_millis_pending;
    // that millis() moved on with this core's ticks since it arrived
    const uint32_t m7_millis_now = m7_millis_given + (local_ms - m7_millis_given_ms);
    m7_millis_pending = false;
    chSysUnlock();
    if (given) {
        m7_clock_calibrate(m7_clock, m7_millis_now, counter, local_ms);
    } else if (!m7_clock.calibrated) {
        m7_clock_calibrate(m7_clock, counter / 1000U, counter, local_ms);
    } else {
        m7_clock_advance(m7_clock, counter, local_ms);
    }
    return m7_clock.micros64;
}

static bool own_context(uint32_t context)
{
    return active != nullptr && reinterpret_cast<void *>(uintptr_t(context)) == active->token;
}

// a panicked module's thread keeps its stack and waits here until the M4 restarts
__attribute__((noreturn)) static void park_module()
{
    chThdSetPriority(LOWPRIO);
    for (;;) {
        (void)chSemWait(&parked);
    }
}

static void sys_allocate(struct port_extctx *ctx)
{
    ctx->r0 = own_context(ctx->r0) ? uint32_t(uintptr_t(host_allocate(*active, ctx->r1, ctx->r2))) : 0;
}

static void sys_deallocate(struct port_extctx *ctx)
{
    if (!own_context(ctx->r0) ||
        host_deallocate(*active, reinterpret_cast<uint8_t *>(uintptr_t(ctx->r1)))) {
        return;
    }
    // the PR's Module::deallocate text
    static const char text[] = "invalid deallocation";
    memcpy(active->panic_text, text, sizeof(text));
    host_panic_report(*active, host_platform);
    park_module();
}

static void sys_time_us(struct port_extctx *ctx)
{
    const uint64_t us = time_us();
    ctx->r0 = uint32_t(us);
    ctx->r1 = uint32_t(us >> 32);
}

static void sys_sleep_ms(struct port_extctx *ctx)
{
    if (!own_context(ctx->r0)) {
        ctx->r0 = uint32_t(STATUS_INVALID);
        return;
    }
    if (!active->stop.load()) {
        const uint32_t ms = ctx->r1 < 60000U ? ctx->r1 : 60000U;
        if (ms == 0) {
            chThdYield();
        } else {
            (void)chBSemWaitTimeout(&stop_wake, TIME_MS2I(ms));
        }
    }
    ctx->r0 = uint32_t(active->stop.load() ? STATUS_STOPPED : STATUS_OK);
}

static void sys_stop_requested(struct port_extctx *ctx)
{
    ctx->r0 = own_context(ctx->r0) ? uint32_t(active->stop.load()) : 1U;
}

static void sys_text(struct port_extctx *ctx)
{
    if (!own_context(ctx->r0) || ctx->r1 > 0xFFU) {
        ctx->r0 = uint32_t(STATUS_INVALID);
        return;
    }
    ctx->r0 = uint32_t(host_text(*active, uint8_t(ctx->r1), reinterpret_cast<const uint8_t *>(uintptr_t(ctx->r2)),
                                 ctx->r3, host_platform));
}

static void sys_panic(struct port_extctx *ctx)
{
    if (active == nullptr) {
        park_module();
    }
    // a foreign context still ends this module's thread
    const uint32_t message = own_context(ctx->r0) ? ctx->r1 : 0;
    host_panic_record(*active, reinterpret_cast<const uint8_t *>(uintptr_t(message)), ctx->r2, host_platform);
    park_module();
}

static void sys_binding_call(struct port_extctx *ctx)
{
    // the arguments lie on the module stack: checked there, then copied before use
    if (active == nullptr || !active->ranges.stack.contains(ctx->r0, sizeof(BindingCallArgs))) {
        ctx->r0 = uint32_t(STATUS_INVALID);
        return;
    }
    BindingCallArgs args;
    memcpy(&args, reinterpret_cast<const void *>(uintptr_t(ctx->r0)), sizeof(args));
    ctx->r0 = uint32_t(host_binding_call(*active, args, host_platform));
}

static void sys_module_returned(struct port_extctx *ctx)
{
    module_result = int32_t(ctx->r0);
    if (active != nullptr) {
        active->state.store(uint8_t(ModuleStateKind::Returned));
        ModuleStateNotify notify {};
        fill_header(notify.call.header, MsgType::SystemCall, sizeof(notify), 0);
        notify.call.method = uint16_t(SystemMethod::ModuleState);
        notify.call.flags = CALL_FLAG_NOTIFY;
        notify.module = active->index;
        notify.state = uint8_t(ModuleStateKind::Returned);
        notify.result = module_result;
        port_notify(nullptr, reinterpret_cast<const uint8_t *>(&notify), sizeof(notify));
    }
    module_has_returned = true;
    chThdExit(MSG_OK);
}

static void sys_fault_request(struct port_extctx *ctx)
{
    ctx->r0 = call_link_of->fault_request.exchange(uint8_t(FaultKind::None));
}

typedef void (*syscall_handler)(struct port_extctx *ctx);

extern "C" {

extern const syscall_handler m4_syscalls[SYS_COUNT];
__attribute__((used)) const syscall_handler m4_syscalls[SYS_COUNT] = {
    sys_allocate, sys_deallocate, sys_time_us, sys_sleep_ms, sys_stop_requested, sys_text, sys_panic,
    sys_binding_call, sys_module_returned, sys_fault_request,
};

__attribute__((used)) void m4_syscall_invalid(struct port_extctx *ctx)
{
    ctx->r0 = uint32_t(-ENOSYS);
}

/*
  Replaces the port's weak port_syscall(). The SVC handler enters it in thread
  mode, privileged, on the module thread's syscall stack, with the caller's
  frame in r0 and the SVC number in r1; "svc 1" from privileged mode then
  returns to the caller, unprivileged (ARMv7-M chcore.c SVC_Handler). No stack
  use here: that handler finds its link context right above the frame "svc 1"
  pushes (as ChibiOS os/sb/host/compilers/GCC/sbexc.S).
 */
__attribute__((naked)) void port_syscall(struct port_extctx *, uint32_t)
{
    __asm volatile(
        "cmp    r1, #10\n"
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

}  // extern "C"
static_assert(SYS_COUNT == 10, "port_syscall compares against the handler count");

// RASR: XN(28) AP(26:24) TEX/S/C/B(21:16) SIZE(5:1) ENABLE(0); as main.cpp's mpu_setup()
static constexpr uint32_t rasr(uint32_t size_field, uint32_t access, bool execute, uint32_t attributes)
{
    return (execute ? 0U : (1U << 28)) | (access << 24) | attributes | (size_field << 1) | 1U;
}
static const uint32_t AP_NONE = 0U;
static const uint32_t AP_ALL_RO = 6U;
static const uint32_t NORMAL_NON_CACHEABLE = 1U << 19;

static void set_region(uint32_t number, uint32_t base, uint32_t attributes)
{
    __DSB();
    MPU->RNR = number;
    MPU->RBAR = base;
    MPU->RASR = attributes;
    __DSB();
    __ISB();
}

static void publish_code(const CodeWindow &w)
{
    set_region(7, w.base, rasr(w.size_field, AP_ALL_RO, true, NORMAL_NON_CACHEABLE) |
               (uint32_t(w.subregion_disable) << 8));
}

static THD_FUNCTION(module_thread_main, arg)
{
    (void)arg;
    // unprivileged from here on; the exception return wants the entry without the Thumb bit
    port_unprivileged_jump(uint32_t(uintptr_t(&module_entry_stub)) & ~1U, module_stack_top - sizeof(LaunchRecord));
}

void RustM4::module_port_init(ModuleLink &link, M4Service &service, void (*wake_service)())
{
    call_link_of = &link;
    service_of = &service;
    wake_service_of = wake_service;
    chBSemObjectInit(&call_done, true);
    chBSemObjectInit(&stop_wake, true);
    chSemObjectInit(&parked, 0);
    m7_counter_running = m7_counter_started();
}

void RustM4::module_port_set_m7_millis(uint32_t m7_millis)
{
    chSysLock();
    m7_millis_given = m7_millis;
    m7_millis_given_ms = uint32_t(chVTGetSystemTimeX());
    m7_millis_pending = true;
    chSysUnlock();
}

void RustM4::module_port_wake()
{
    chBSemSignal(&call_done);
}

void RustM4::module_port_start(ModuleContext &context, const LoadLayout &layout, uint32_t entry, uint32_t rw_base)
{
    if (module_thread != nullptr && !chThdTerminatedX(module_thread)) {
        return;   // one module thread at a time
    }
    active = nullptr;
    module_has_returned = false;
    module_result = 0;
    context.token = module_token(context.index);
    context.stop.store(false);
    chBSemReset(&stop_wake, true);

    // region 7 off while the window's tail is cleared: stale RAM above the code would be mapped
    // executable once it is published
    set_region(7, 0, 0);
    const CodeWindow &w = layout.window;
    const uintptr_t window_end = uintptr_t(w.base) + w.size;
    uintptr_t code_end = w.rx_base;
    const Range &rx = context.ranges.rx;
    if (rx.base >= w.rx_base && rx.base <= window_end && rx.size <= window_end - rx.base) {
        code_end = rx.base + rx.size;
    }
    memset(reinterpret_cast<void *>(code_end), 0, window_end - code_end);
    set_region(6, layout.guard_base, rasr(4, AP_NONE, false, NORMAL_NON_CACHEABLE));
    publish_code(w);

    module_stack_top = layout.stack_top;
    auto *record = reinterpret_cast<LaunchRecord *>(uintptr_t(layout.stack_top - sizeof(LaunchRecord)));
    *record = LaunchRecord { entry, &module_host_api, context.token, rw_base };
    context.state.store(uint8_t(ModuleStateKind::Started));
    active = &context;
    // below the service (the main thread, NORMALPRIO): the heartbeat waits for the module only
    // for the service mutex, held for one forward_call() with priority inheritance
    // painted, so that the syscall stack's high-water mark can be read (CH_DBG_FILL_THREADS is off)
    memset(module_wa, CH_DBG_STACK_FILL_VALUE, sizeof(module_wa));
    module_thread = chThdCreateStatic(module_wa, sizeof(module_wa), NORMALPRIO - 1, module_thread_main, nullptr);
}

void RustM4::module_port_request_stop(ModuleContext &context)
{
    context.stop.store(true);
    chBSemSignal(&stop_wake);
}

bool RustM4::module_port_returned(const ModuleContext &context, int32_t &result)
{
    if (active != &context || !module_has_returned) {
        return false;
    }
    result = module_result;
    return true;
}

// RUST_M4_INJECT 17, on the service thread: the built-in module, from flash, with a module's
// stack and heap in the module area and an empty code window at its top
void RustM4::module_port_start_selftest()
{
    if (module_thread != nullptr && !chThdTerminatedX(module_thread)) {
        return;
    }
    const uint32_t area_base = uint32_t(uintptr_t(__module_area_base__));
    const uint32_t area_size = uint32_t(__module_area_end__ - __module_area_base__);
    const LoadRequest request { 0, CODE_WINDOW_MIN, 32, 0, 4, MODULE_STACK_MIN, SELFTEST_HEAP_SIZE };
    LoadLayout layout;
    if (load_layout(area_base, area_size, request, layout) != LoadFit::Ok) {
        return;
    }
    ModuleContext &m = module_context;
    m.index = 0;
    m.request = call_link_of->request;
    m.reply = call_link_of->reply;
    m.panic_text[0] = 0;
    m.ranges.rx = { uintptr_t(__m4_selftest_start__), uint32_t(__m4_selftest_end__ - __m4_selftest_start__) };
    m.ranges.rw = { 0, 0 };
    m.ranges.stack = { layout.stack_base, layout.stack_top - layout.stack_base };
    m.ranges.heap = { layout.heap_base, layout.heap_end - layout.heap_base };
    if (!m.heap.init(layout.heap_base, layout.heap_end - layout.heap_base, selftest_heap_used,
                     selftest_heap_starts, 2)) {
        return;
    }
    module_port_start(m, layout, uint32_t(uintptr_t(&m4_selftest_start)), 0);
}
