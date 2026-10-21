#include "AP_ModuleM4.h"

#if AP_MODULE_M4_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <hal.h>
#include <string.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_ChibiOS/AP_HAL_ChibiOS.h>
#include <AP_HAL_ChibiOS/Scheduler.h>
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>
#include <GCS_MAVLink/GCS.h>
#include <CubePilotFW/Channel.h>
#include <CubePilotFW/PeerHeader.h>
#include <CubePilotFW/pal/HSEM.h>
#include "AP_ModuleM4_Layout.h"
#include "AP_ModuleM4_ServeWait.h"

#if !defined(STM32H757xx)
#error "Modules on the M4 need a dual-core STM32H757"
#endif

// the M4 reads this core's AP_HAL::micros() as TIM5's count (AP_ModuleM4_M7Clock.h): the
// system timer must be TIM5, 32 bits at 1 MHz, with no start offset
static_assert(STM32_ST_USE_TIMER == 5, "M4 local micros() reads TIM5: the system timer must be TIM5");
static_assert(CH_CFG_ST_RESOLUTION == 32 && CH_CFG_ST_FREQUENCY == 1000000,
              "M4 local micros() reads TIM5 as a 32-bit 1 MHz count");
#ifdef AP_BOARD_START_TIME
#error "M4 local micros() is TIM5's count: AP_BOARD_START_TIME would offset the M7's micros() from it"
#endif

extern const AP_HAL::HAL& hal;

using CubePilotFW::HwSemaphore;
namespace PeerHeader = CubePilotFW::PeerHeader;

// the host thread runs at UART priority + 1, the interrupt only signals it
#ifndef AP_MODULE_M4_HSEM_IRQ_PRIORITY
#define AP_MODULE_M4_HSEM_IRQ_PRIORITY 12
#endif
static const uint8_t HSEM_IRQ_PRIORITY = AP_MODULE_M4_HSEM_IRQ_PRIORITY;
// the ISR signals a semaphore, so it must not preempt the kernel's critical
// zones: ChibiOS ARMv7-M chcore.h, CORTEX_MAX_KERNEL_PRIORITY (SVCALL + 1)
static_assert(AP_MODULE_M4_HSEM_IRQ_PRIORITY >= CORTEX_MAX_KERNEL_PRIORITY &&
              AP_MODULE_M4_HSEM_IRQ_PRIORITY <= CORTEX_MIN_KERNEL_PRIORITY,
              "HSEM IRQ priority outside the kernel's interrupt range");
#if AP_MODULE_M4_PRIORITY_TEST
static constexpr int HOST_PRIORITY = APM_MAIN_PRIORITY_BOOST + ModuleM4::HOST_PRIORITY_OFFSET;
static constexpr int WORKER_PRIORITY = APM_MAIN_PRIORITY_BOOST + ModuleM4::WORKER_PRIORITY_OFFSET;
static_assert(HOST_PRIORITY == 183 && WORKER_PRIORITY == 182 && HOST_PRIORITY > APM_TIMER_PRIORITY &&
              WORKER_PRIORITY > APM_TIMER_PRIORITY && HOST_PRIORITY <= HIGHPRIO,
              "priority test: host and worker above the main loop and the timer thread");
#else
static constexpr int HOST_PRIORITY = APM_UART_PRIORITY + ModuleM4::HOST_PRIORITY_OFFSET;
static constexpr int WORKER_PRIORITY = APM_IO_PRIORITY + ModuleM4::WORKER_PRIORITY_OFFSET;
static_assert(HOST_PRIORITY > APM_STORAGE_PRIORITY && HOST_PRIORITY > APM_IO_PRIORITY &&
              HOST_PRIORITY > APM_NET_PRIORITY && HOST_PRIORITY > APM_UART_PRIORITY && HOST_PRIORITY < APM_I2C_PRIORITY,
              "the M4 host thread sits above the I/O threads and below vehicle timing");
// below io - 1, which the OA path planner, the mount POI thread and SmartAudio use; no other
// thread runs at io - 2 (56)
static_assert(WORKER_PRIORITY < APM_IO_PRIORITY - 1 && WORKER_PRIORITY < APM_STORAGE_PRIORITY &&
              WORKER_PRIORITY < APM_UART_PRIORITY && WORKER_PRIORITY > APM_STARTUP_PRIORITY && WORKER_PRIORITY > LOWPRIO,
              "the M4 slow worker sits below the I/O threads and their io - 1 peers");
#endif

bool AP_ModuleM4::platform_start(char *reason, size_t reason_len)
{
    if (stm32_was_cm4_reset()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Module: last reset came from the M4");
    }
    // the M4 must be held at reset (BCM4=0) and boot from its image; the option
    // bytes are set from a controlled path (bootloader), never on a normal boot
    if ((FLASH->OPTSR_CUR & FLASH_OPTSR_BCM4) != 0) {
        hal.util->snprintf(reason, reason_len, "boot opts BCM4=1");
        return false;
    }
    const uint32_t boot_address = (FLASH->BOOT4_CUR & FLASH_BOOT4_BCM4_ADD0) << 16;
    if (boot_address != AP_MODULE_M4_FLASH_BASE) {
        hal.util->snprintf(reason, reason_len, "boot addr %08X", unsigned(boot_address));
        return false;
    }
    const uint32_t *vectors = reinterpret_cast<const uint32_t *>(AP_MODULE_M4_FLASH_BASE);
    const uint32_t initial_sp = vectors[0];
    const uint32_t reset_handler = vectors[1];
    if (initial_sp <= AP_MODULE_M4_RAM_BASE || initial_sp > AP_MODULE_M4_RAM_BASE + AP_MODULE_M4_RAM_SIZE ||
        (reset_handler & 1U) == 0 || reset_handler < AP_MODULE_M4_FLASH_BASE ||
        reset_handler >= AP_MODULE_M4_FLASH_BASE + AP_MODULE_M4_FLASH_SIZE) {
        hal.util->snprintf(reason, reason_len, "image missing");
        return false;
    }
    _shared = reinterpret_cast<uint8_t *>(AP_MODULE_M4_WINDOW_BASE);
    memset(_shared, 0, ModuleM4::WINDOW_SIZE);
    ModuleM4::SupervisorBlock *block = ModuleM4::supervisor_block(_shared);
    block->run_permit.store(1);
    block->restart_grant.store(0);
    block->fault_after_publish_grant.store(0);
    block->magic.store(ModuleM4::SUPERVISOR_MAGIC);
    // ruling order on the M7: enableClock() -> init() -> enableNvic()
    HwSemaphore::enableClock();
    HwSemaphore::init();
    HwSemaphore::setRxCallback(ModuleM4::DOORBELL_GROUP, doorbell_isr);
    HwSemaphore::enableRxIrq(ModuleM4::DOORBELL_GROUP, true);
    HwSemaphore::enableNvic(HSEM_IRQ_PRIORITY);
    if (_channel->init(_shared, ModuleM4::CHANNEL_WINDOW_SIZE, CubePilotFW::Channel::Core::M7,
                       ModuleM4::DOORBELL_GROUP) != 0) {
        hal.util->snprintf(reason, reason_len, "channel init");
        return false;
    }
    _channel->resetAfterRestart();
    PeerHeader::publish(_shared, PeerHeader::Core::M7, ModuleM4::PROTOCOL_VERSION, ModuleM4::CONTRACT_HASH, 0);
    // ES0445 2.3.1: a WWDG reset of either core resets the whole system; set
    // here too, as stm32_watchdog_init() may not run (BRD_OPTIONS) or run later
    RCC->GCR |= RCC_GCR_WW1RSC | RCC_GCR_WW2RSC;
    __DSB();
    RCC->GCR |= RCC_GCR_BOOT_C2;
    return true;
}

void AP_ModuleM4::platform_request_restart()
{
    // freeing our restart semaphore raises the M4's HSEM interrupt; a parked M4 polls restart_grant
    if (HwSemaphore::takeTx(ModuleM4::RESTART_GROUP)) {
        HwSemaphore::releaseTx(ModuleM4::RESTART_GROUP);
    }
}

bool AP_ModuleM4::platform_m4_exited()
{
    return false;
}

void AP_ModuleM4::platform_stop()
{
    // there is no per-core reset (ES0445 2.3.1): with run_permit=0 the restart
    // doorbell parks an M4 that is stalled with interrupts on
    if (_shared != nullptr) {
        platform_request_restart();
    }
}

uint32_t AP_ModuleM4::platform_wait_us() const
{
    return 10000;
}

#if CH_DBG_STATISTICS != TRUE
#error "the M4 host duty cap charges ChibiOS thread statistics: define HAL_ENABLE_THREAD_STATISTICS"
#endif

bool AP_ModuleM4::platform_thread_cpu(uint32_t &cpu)
{
    // a reset of this thread's statistics sets best to all ones: chTMObjectInit() from a thread
    // reading @SYS/threads.txt, or Util::thread_info() on this thread. The M4 cannot reach that
    // file (module_path_allowed()); an interval with a reset is charged the CPU since it plus
    // RESET_ALLOWANCE_US (interval_cpu_us())
    time_measurement_t &stats = chThdGetSelfX()->stats;
    volatile rtcnt_t &last = stats.last;
    // the low word of cumulative (little endian), one 32-bit load
    volatile uint32_t &cumulative = *reinterpret_cast<volatile uint32_t *>(&stats.cumulative);
    volatile rtcnt_t &best = stats.best;
    // only this thread's switches change its statistics, and a reset happens only while it is
    // switched out: a switch-in rewrites last, so a reading with last unchanged around it is whole
    // without the kernel lock
    rtcnt_t started, now;
    uint32_t finished;
    bool reset;
    do {
        started = last;
        finished = cumulative;
        reset = best != 0;
        now = chSysGetRealtimeCounterX();
    } while (last != started);
    if (!reset) {
        cpu = finished + (now - started);
        return false;
    }
    // reset: best is cleared under the lock, so a reset after the check is not lost
    chSysLock();
    ModuleM4::ThreadStatsClock clock { uint32_t(stats.cumulative), stats.last, stats.best };
    cpu = ModuleM4::read_thread_cycles(clock, chSysGetRealtimeCounterX(), reset);
    stats.best = clock.best;
    chSysUnlock();
    return reset;
}

uint32_t AP_ModuleM4::platform_timeout_ms() const
{
    return AP_HAL::millis();
}

#endif  // AP_MODULE_M4_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
