/*
  Lua scripting Cortex-M4 image: ChibiOS RT (no HAL: the M4 owns no
  peripherals), its own MPU, the CubeFramework channel to the M7, the
  privileged M4Service on the main thread and the Lua runtime on an
  unprivileged thread (lua_port.cpp). The M7 released this core with
  RCC_GCR.BOOT_C2 after zeroing the shared window and writing the
  supervisor block.
 */
#include "ch.h"

#include <string.h>
#include <CubePilotFW/Channel.h>
#include <CubePilotFW/PAL.h>
#include <CubePilotFW/PeerHeader.h>
#include <CubePilotFW/pal/HSEM.h>
#include <AP_ScriptingM4_Layout.h>

#include "../m4_service.h"
#include "lua_port.h"
#include "m4_fault.h"

using namespace ScriptingM4;
using CubePilotFW::Channel;
using CubePilotFW::HwSemaphore;
namespace PeerHeader = CubePilotFW::PeerHeader;

static Channel channel;
static M4Service service;
static binary_semaphore_t doorbell;   // also wakes the service for a posted Lua call
static LuaLink call_link;   // not `link`: <unistd.h> declares link()
static mutex_t service_mutex;   // M4Platform::lock: the service and the Lua thread's SVC

static uint8_t *shared_window()
{
    return reinterpret_cast<uint8_t *>(AP_SCRIPTING_M4_WINDOW_BASE);
}

// SCR_M4_INJECT 11: IRQ 147 is reserved on the STM32H757, so only the service's pend raises it.
// Its handler faults, so the fault handler runs inside another exception (ICSR.RETTOBASE 0),
// where m4_warm_restart() cannot return to thread mode and parks
static const uint32_t TEST_IRQ = 147;
static_assert(0x40 + 4 * TEST_IRQ == 0x28C, "Vector28C is IRQ 147's vector");

extern "C" {

void *__dso_handle = nullptr;

CH_IRQ_HANDLER(SysTick_Handler)
{
    CH_IRQ_PROLOGUE();
    chSysLockFromISR();
    chSysTimerHandlerI();
    chSysUnlockFromISR();
    CH_IRQ_EPILOGUE();
}

// HSEM2, this core's HSEM interrupt (CubeFramework's own vector needs the ChibiOS HAL)
CH_IRQ_HANDLER(Vector238)
{
    CH_IRQ_PROLOGUE();
    HwSemaphore::handleIrq();
    CH_IRQ_EPILOGUE();
}

CH_IRQ_HANDLER(Vector28C)
{
    __builtin_trap();   // UDF; UsageFault is not enabled, so it escalates to HardFault
}

// CubeFramework PAL for the bare-metal HSEM build
static unsigned critical_depth;
static syssts_t critical_outer_status;

uint32_t cubefw_pal_timeNowUs(void)
{
    return uint32_t(chVTGetSystemTimeX()) * (1000000U / CH_CFG_ST_FREQUENCY);
}

void cubefw_pal_sleepUs(uint32_t us)
{
    chThdSleepMicroseconds(us);
}

void cubefw_pal_criticalEnter(void)
{
    const syssts_t status = chSysGetStatusAndLockX();
    if (critical_depth++ == 0) {
        critical_outer_status = status;
    }
}

void cubefw_pal_criticalExit(void)
{
    if (critical_depth != 0 && --critical_depth == 0) {
        chSysRestoreStatusX(critical_outer_status);
    }
}

void cubefw_pal_shMemcpy(void *dst, const void *src, size_t len)
{
    memcpy(dst, src, len);
}

// Lua RAM (scripting_m4.ld .lua_data/.lua_bss): crt0 initialises only .data and .bss,
// which are privileged; this hook runs after them and before the C++ constructors
void __late_init(void)
{
    extern uint32_t __lua_data_load__[], __lua_data_base__[], __lua_data_end__[];
    extern uint32_t __lua_bss_base__[], __lua_bss_end__[];
    const uint32_t *src = __lua_data_load__;
    for (uint32_t *dst = __lua_data_base__; dst < __lua_data_end__; dst++) {
        *dst = *src++;
    }
    for (uint32_t *dst = __lua_bss_base__; dst < __lua_bss_end__; dst++) {
        *dst = 0;
    }
}

}  // extern "C"

static void doorbell_callback(uint8_t group)
{
    (void)group;
    chSysLockFromISR();
    chBSemSignalI(&doorbell);
    chSysUnlockFromISR();
}

static void restart_callback(uint8_t group)
{
    (void)group;
    if (supervisor_block(shared_window())->run_permit.load() == 0) {
        m4_park_forever();   // the M7 turned scripting off
    }
    // the M7 stores the grant before ringing; a late or repeated ring is ignored
    if (m4_restart_granted()) {
        m4_warm_restart();   // e.g. the service loop stopped beating
    }
}

static uint32_t millis_now()
{
    return uint32_t(TIME_I2MS(chVTGetSystemTimeX()));
}

static uint32_t cycles_now()
{
    return DWT->CYCCNT;   // enabled by the ARMv7-M port_init()
}

static void lock_service()
{
    chMtxLock(&service_mutex);
}

static void unlock_service()
{
    chMtxUnlock(&service_mutex);
}

static bool forward_posted()
{
    return service.forward_posted();
}

// faults the service performs; the Lua thread's own are in lua_port.cpp
static void inject_fault(FaultKind kind)
{
    switch (kind) {
    case FaultKind::HardFault:
        __builtin_trap();   // UDF; UsageFault is not enabled, so it escalates
    case FaultKind::MemManage:
        // RCC_GCR lies outside every M4 MPU region and PRIVDEFENA=0
        *reinterpret_cast<volatile uint32_t *>(0x580244A0U) = 0;
        break;
    case FaultKind::Stall:
        for (;;) {
            __NOP();        // interrupts on: only the restart doorbell gets out of here
        }
    case FaultKind::Lockup:
        __disable_irq();
        for (;;) {
            __NOP();
        }
    case FaultKind::HandlerFault:
        NVIC_SetPriority(IRQn_Type(TEST_IRQ), 2);
        NVIC_EnableIRQ(IRQn_Type(TEST_IRQ));
        NVIC_SetPendingIRQ(IRQn_Type(TEST_IRQ));
        break;
    default:
        break;
    }
}

// RASR: XN(28) AP(26:24) TEX/S/C/B(21:16) SIZE(5:1) ENABLE(0); region size = 2^(SIZE+1)
static constexpr uint32_t rasr(uint32_t size_field, uint32_t access, bool execute, uint32_t attributes)
{
    return (execute ? 0U : (1U << 28)) | (access << 24) | attributes | (size_field << 1) | 1U;
}
static const uint32_t AP_NONE = 0U;                    // no access
static const uint32_t AP_PRIV_RW = 1U;                 // privileged RW, unprivileged none
static const uint32_t AP_FULL_RW = 3U;                 // privileged and unprivileged RW
static const uint32_t AP_ALL_RO = 6U;                  // privileged and unprivileged RO
static const uint32_t NORMAL_WRITE_THROUGH = 1U << 17; // TEX=000 C=1 B=0
static const uint32_t NORMAL_NON_CACHEABLE = 1U << 19; // TEX=001 C=0 B=0
static const uint32_t SHAREABLE = 1U << 18;
static const uint32_t SHARED_DEVICE = 1U << 16;        // TEX=000 C=0 B=1

static_assert(AP_SCRIPTING_M4_FLASH_SIZE == (1U << (17 + 1)), "flash region size field");
static_assert(AP_SCRIPTING_M4_RAM_SIZE == (1U << (17 + 1)), "RAM region size field");
static_assert(AP_SCRIPTING_M4_PRIV_RAM_SIZE ==
              (1U << (AP_SCRIPTING_M4_PRIV_REGION_MPU_SIZE + 1)) / 8U * (8U - __builtin_popcount(AP_SCRIPTING_M4_PRIV_REGION_SRD)),
              "privileged RAM is the enabled subregions of region 2");
static_assert(AP_SCRIPTING_M4_WINDOW_SIZE == (1U << (AP_SCRIPTING_M4_WINDOW_MPU_SIZE + 1)), "window size field");

/*
  Spec §4. The Lua thread (unprivileged) reaches the image (RO+X) and Lua
  RAM; privileged code also reaches its own RAM, the window and the HSEM.
  PRIVDEFENA=0: everything else (RCC, PWR, FLASH, AXI SRAM) faults for both,
  except one privileged read of TIM5 or RCC at a time (lua_port.cpp);
  the PPB (SCB, MPU, NVIC) is privileged-only in any case. Where regions
  overlap the higher number wins.
 */
static void mpu_setup()
{
    extern uint8_t __main_thread_stack_base__[];
    const struct {
        uint32_t base;
        uint32_t attributes;
    } regions[] = {
        // 0: the image
        { AP_SCRIPTING_M4_FLASH_BASE, rasr(17, AP_ALL_RO, true, NORMAL_WRITE_THROUGH) },
        // 1: SRAM1/2, the Lua RAM part of it for the Lua thread
        { AP_SCRIPTING_M4_RAM_BASE, rasr(17, AP_FULL_RW, false, NORMAL_NON_CACHEABLE) },
        // 2: privileged RAM: kernel, service, stacks; SRD (bits 15:8) disables the subregions above it
        { AP_SCRIPTING_M4_RAM_BASE, rasr(AP_SCRIPTING_M4_PRIV_REGION_MPU_SIZE, AP_PRIV_RW, false,
                                         NORMAL_NON_CACHEABLE | (AP_SCRIPTING_M4_PRIV_REGION_SRD << 8)) },
        // 3: the shared window
        { AP_SCRIPTING_M4_WINDOW_BASE, rasr(AP_SCRIPTING_M4_WINDOW_MPU_SIZE, AP_PRIV_RW, false,
                                            NORMAL_NON_CACHEABLE | SHAREABLE) },
        // 4: HSEM, 1 KB
        { HSEM_BASE, rasr(9, AP_PRIV_RW, false, SHARED_DEVICE | SHAREABLE) },
        // 5: guard below the main thread's stack, above the exception stack (32 B)
        { uint32_t(uintptr_t(__main_thread_stack_base__)), rasr(4, AP_NONE, false, NORMAL_NON_CACHEABLE) },
#if !(defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED)
        // 6: guard at the bottom of the Lua thread's stack (32 B)
        { lua_stack_guard(), rasr(4, AP_NONE, false, NORMAL_NON_CACHEABLE) },
#endif
    };
    const uint32_t count = sizeof(regions) / sizeof(regions[0]);
    MPU->CTRL = 0;
    for (uint32_t i = 0; i < 8; i++) {
        MPU->RNR = i;
        MPU->RBAR = i < count ? regions[i].base : 0;
        MPU->RASR = i < count ? regions[i].attributes : 0;
    }
    MPU->CTRL = MPU_CTRL_ENABLE_Msk;   // PRIVDEFENA=0
    // BusFault reported as such (an unprivileged PPB access), not escalated
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk;
    __DSB();
    __ISB();
}

// The ART caches the M4's fetches from one 1 MB flash page: the one holding this image (RM0399).
// RCC and ART lie outside every MPU region, so this runs while the MPU is off (reset, warm restart)
static_assert((AP_SCRIPTING_M4_FLASH_BASE >> 20) == ((AP_SCRIPTING_M4_FLASH_BASE + AP_SCRIPTING_M4_FLASH_SIZE - 1U) >> 20),
              "the image must lie inside the ART's one cacheable page");
static void art_enable()
{
    RCC_C2->AHB1ENR |= RCC_AHB1ENR_ARTEN;
    (void)RCC_C2->AHB1ENR;
    // EN off while PCACHEADDR changes (RM0399 6.3.2). Stale code is not possible: the image is
    // rewritten only by SWD or the bootloader, and both end in a system reset, which clears CTR
    ART->CTR &= ~ART_CTR_EN;
    ART->CTR = (ART->CTR & ~ART_CTR_PCACHEADDR) |
               ((AP_SCRIPTING_M4_FLASH_BASE >> 20) << ART_CTR_PCACHEADDR_Pos);
    ART->CTR |= ART_CTR_EN;
    __DSB();
    __ISB();
}

int main(void)
{
    // before anything can fault: the fault handler waits for a change from this value
    const uint32_t restarts = m4_latch_restart_grant();
    art_enable();
    mpu_setup();
    SysTick->LOAD = AP_SCRIPTING_M4_CORE_CLOCK_HZ / CH_CFG_ST_FREQUENCY - 1U;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
    NVIC_SetPriority(SysTick_IRQn, 8);
    chSysInit();
    // with PORT_USE_SYSCALL, port_init() enabled the MPU with PRIVDEFENA: back to the map above
    mpu_setup();
    chBSemObjectInit(&doorbell, true);
    chMtxObjectInit(&service_mutex);

    uint8_t *const window = shared_window();
    SupervisorBlock *const block = supervisor_block(window);
    if (block->magic.load() != SUPERVISOR_MAGIC || block->run_permit.load() != 1) {
        m4_park_forever();
    }
    // ruling order on the M4: HwSemaphore::init() -> enableNvic()
    HwSemaphore::init();
    HwSemaphore::setRxCallback(DOORBELL_GROUP, doorbell_callback);
    HwSemaphore::setRxCallback(RESTART_GROUP, restart_callback);
    HwSemaphore::enableRxIrq(DOORBELL_GROUP, true);
    HwSemaphore::enableRxIrq(RESTART_GROUP, true);
    // lowest priority: the restart doorbell interrupts thread mode only (ICSR.RETTOBASE)
    HwSemaphore::enableNvic(CORTEX_MINIMUM_PRIORITY);
    if (channel.init(window, CHANNEL_WINDOW_SIZE, Channel::Core::M4, DOORBELL_GROUP) != 0) {
        m4_park_forever();
    }
    channel.resetAfterRestart();
    PeerHeader::publish(window, PeerHeader::Core::M4, PROTOCOL_VERSION, CONTRACT_HASH, restarts);
    // SCR_M4_INJECT 12: the session the M7 marked faults before the M7 reads its header
    if (restarts != 0 && block->fault_after_publish_grant.load() == restarts) {
        inject_fault(FaultKind::HardFault);
    }

    const M4Platform platform { millis_now, cycles_now, AP_SCRIPTING_M4_CORE_CLOCK_HZ / 1000000U,
                                inject_fault, lua_port_wake, lock_service, unlock_service };
    service.init(channel, window, call_link, platform);
    lua_port_start(call_link, doorbell, forward_posted);
    while (true) {
        if (block->run_permit.load() != 1) {
            m4_park_forever();
        }
        service.step();
        // bounded sleep (Channel contract): a doorbell can be deferred while the HSEM is busy
        if (channel.prepareSleep()) {
            (void)chBSemWaitTimeout(&doorbell, TIME_MS2I(5));
        }
        channel.wake();
    }
}
