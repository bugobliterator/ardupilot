/*
  Lua scripting Cortex-M4 image, M4 Boot & Supervisor phase: ChibiOS RT (no
  HAL: the M4 owns no peripherals), its own MPU, the CubeFramework channel to
  the M7 and M4Service. The M7 released this core with RCC_GCR.BOOT_C2 after
  zeroing the shared window and writing the supervisor block.
 */
#include "ch.h"

#include <string.h>
#include <CubePilotFW/Channel.h>
#include <CubePilotFW/PAL.h>
#include <CubePilotFW/PeerHeader.h>
#include <CubePilotFW/pal/HSEM.h>
#include <AP_ScriptingM4_Layout.h>

#include "../m4_service.h"
#include "m4_fault.h"

using namespace ScriptingM4;
using CubePilotFW::Channel;
using CubePilotFW::HwSemaphore;
namespace PeerHeader = CubePilotFW::PeerHeader;

static Channel channel;
static M4Service service;
static binary_semaphore_t doorbell;
static LuaLink call_link;   // not `link`: <unistd.h> declares link()

// nothing posts a call until the Lua thread exists (Unprivileged Lua Image task)
static void wake_lua()
{
}

static uint8_t *shared_window()
{
    return reinterpret_cast<uint8_t *>(AP_SCRIPTING_M4_WINDOW_BASE);
}

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
    default:
        break;
    }
}

// RASR: XN(28) AP(26:24) TEX/S/C/B(21:16) SIZE(5:1) ENABLE(0)
static constexpr uint32_t rasr(uint32_t size_field, uint32_t access, bool execute, uint32_t attributes)
{
    return (execute ? 0U : (1U << 28)) | (access << 24) | attributes | (size_field << 1) | 1U;
}
static const uint32_t AP_PRIV_RW = 1U;                 // privileged RW, unprivileged none
static const uint32_t AP_PRIV_RO = 5U;                 // privileged RO, unprivileged none
static const uint32_t NORMAL_WRITE_THROUGH = 1U << 17; // TEX=000 C=1 B=0
static const uint32_t NORMAL_NON_CACHEABLE = 1U << 19; // TEX=001 C=0 B=0
static const uint32_t SHAREABLE = 1U << 18;
static const uint32_t SHARED_DEVICE = 1U << 16;        // TEX=000 C=0 B=1

static_assert(AP_SCRIPTING_M4_FLASH_SIZE == (1U << (17 + 1)), "flash region size field");
static_assert(AP_SCRIPTING_M4_RAM_SIZE == (1U << (17 + 1)), "RAM region size field");
static_assert(AP_SCRIPTING_M4_WINDOW_SIZE == (1U << (AP_SCRIPTING_M4_WINDOW_MPU_SIZE + 1)), "window size field");

// spec §4: the M4 reaches only its image, its RAM, the window and the HSEM;
// PRIVDEFENA=0 makes privileged code fault elsewhere too (RCC, PWR, FLASH, AXI SRAM)
static void mpu_setup()
{
    static const struct {
        uint32_t base;
        uint32_t attributes;
    } regions[] = {
        { AP_SCRIPTING_M4_FLASH_BASE, rasr(17, AP_PRIV_RO, true, NORMAL_WRITE_THROUGH) },
        { AP_SCRIPTING_M4_RAM_BASE, rasr(17, AP_PRIV_RW, false, NORMAL_NON_CACHEABLE) },
        { AP_SCRIPTING_M4_WINDOW_BASE, rasr(AP_SCRIPTING_M4_WINDOW_MPU_SIZE, AP_PRIV_RW, false,
                                            NORMAL_NON_CACHEABLE | SHAREABLE) },
        { HSEM_BASE, rasr(9, AP_PRIV_RW, false, SHARED_DEVICE | SHAREABLE) },   // 1 KB
    };
    const uint32_t count = sizeof(regions) / sizeof(regions[0]);
    MPU->CTRL = 0;
    for (uint32_t i = 0; i < 8; i++) {
        MPU->RNR = i;
        MPU->RBAR = i < count ? regions[i].base : 0;
        MPU->RASR = i < count ? regions[i].attributes : 0;
    }
    MPU->CTRL = MPU_CTRL_ENABLE_Msk;
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
    __DSB();
    __ISB();
}

int main(void)
{
    // before anything can fault: the fault handler waits for a change from this value
    const uint32_t restarts = m4_latch_restart_grant();
    mpu_setup();
    SysTick->LOAD = AP_SCRIPTING_M4_CORE_CLOCK_HZ / CH_CFG_ST_FREQUENCY - 1U;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
    NVIC_SetPriority(SysTick_IRQn, 8);
    chSysInit();
    chBSemObjectInit(&doorbell, true);

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

    const M4Platform platform { millis_now, cycles_now, AP_SCRIPTING_M4_CORE_CLOCK_HZ / 1000000U, inject_fault, wake_lua };
    service.init(channel, window, call_link, platform);
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
