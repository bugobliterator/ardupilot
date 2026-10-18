#include "AP_ScriptingM4.h"

#if AP_SCRIPTING_M4_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

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
#include "AP_ScriptingM4_Layout.h"
#include "AP_ScriptingM4_ServeWait.h"
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
#include <AP_Networking/AP_Networking.h>
#endif

#if !defined(STM32H757xx)
#error "M4 scripting needs a dual-core STM32H757"
#endif

// the M4 reads this core's AP_HAL::micros() as TIM5's count (m4/chibios/lua_port.cpp): the
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
static const uint8_t HSEM_IRQ_PRIORITY = 12;
static constexpr int HOST_PRIORITY = APM_UART_PRIORITY + ScriptingM4::HOST_PRIORITY_OFFSET;
static constexpr int WORKER_PRIORITY = APM_IO_PRIORITY + ScriptingM4::WORKER_PRIORITY_OFFSET;
static_assert(HOST_PRIORITY > APM_STORAGE_PRIORITY && HOST_PRIORITY > APM_IO_PRIORITY &&
              HOST_PRIORITY > APM_NET_PRIORITY && HOST_PRIORITY > APM_UART_PRIORITY && HOST_PRIORITY < APM_I2C_PRIORITY,
              "the M4 host thread sits above the I/O threads and below vehicle timing");
// below io - 1, which the OA path planner, the mount POI thread and SmartAudio use; no other
// thread runs at io - 2 (56)
static_assert(WORKER_PRIORITY < APM_IO_PRIORITY - 1 && WORKER_PRIORITY < APM_STORAGE_PRIORITY &&
              WORKER_PRIORITY < APM_UART_PRIORITY && WORKER_PRIORITY > APM_STARTUP_PRIORITY && WORKER_PRIORITY > LOWPRIO,
              "the M4 slow worker sits below the I/O threads and their io - 1 peers");

bool AP_ScriptingM4::platform_start(char *reason, size_t reason_len)
{
    if (stm32_was_cm4_reset()) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Scripting: last reset came from the M4");
    }
    // the M4 must be held at reset (BCM4=0) and boot from its image; the option
    // bytes are set from a controlled path (bootloader), never on a normal boot
    if ((FLASH->OPTSR_CUR & FLASH_OPTSR_BCM4) != 0) {
        hal.util->snprintf(reason, reason_len, "boot opts BCM4=1");
        return false;
    }
    const uint32_t boot_address = (FLASH->BOOT4_CUR & FLASH_BOOT4_BCM4_ADD0) << 16;
    if (boot_address != AP_SCRIPTING_M4_FLASH_BASE) {
        hal.util->snprintf(reason, reason_len, "boot addr %08X", unsigned(boot_address));
        return false;
    }
    const uint32_t *vectors = reinterpret_cast<const uint32_t *>(AP_SCRIPTING_M4_FLASH_BASE);
    const uint32_t initial_sp = vectors[0];
    const uint32_t reset_handler = vectors[1];
    if (initial_sp <= AP_SCRIPTING_M4_RAM_BASE || initial_sp > AP_SCRIPTING_M4_RAM_BASE + AP_SCRIPTING_M4_RAM_SIZE ||
        (reset_handler & 1U) == 0 || reset_handler < AP_SCRIPTING_M4_FLASH_BASE ||
        reset_handler >= AP_SCRIPTING_M4_FLASH_BASE + AP_SCRIPTING_M4_FLASH_SIZE) {
        hal.util->snprintf(reason, reason_len, "image missing");
        return false;
    }
    _shared = reinterpret_cast<uint8_t *>(AP_SCRIPTING_M4_WINDOW_BASE);
    ScriptingM4::clear_channel_region(_shared);
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
    AP::network().m4_region_reset(_shared + ScriptingM4::RING_REGION_OFFSET);
#endif
    ScriptingM4::SupervisorBlock *block = ScriptingM4::supervisor_block(_shared);
    block->run_permit.store(1);
#if defined(AP_NETWORKING_M4_ENABLED) && AP_NETWORKING_M4_ENABLED
    store_lua_permit();
    _lua_permit.session_granted();
#endif
    block->restart_grant.store(0);
    block->fault_after_publish_grant.store(0);
    block->magic.store(ScriptingM4::SUPERVISOR_MAGIC);
    // ruling order on the M7: enableClock() -> init() -> enableNvic()
    HwSemaphore::enableClock();
    HwSemaphore::init();
    HwSemaphore::setRxCallback(ScriptingM4::DOORBELL_GROUP, doorbell_isr);
    HwSemaphore::enableRxIrq(ScriptingM4::DOORBELL_GROUP, true);
    HwSemaphore::enableNvic(HSEM_IRQ_PRIORITY);
    if (_channel->init(_shared, ScriptingM4::CHANNEL_WINDOW_SIZE, CubePilotFW::Channel::Core::M7,
                       ScriptingM4::DOORBELL_GROUP) != 0) {
        hal.util->snprintf(reason, reason_len, "channel init");
        return false;
    }
    _channel->resetAfterRestart();
    PeerHeader::publish(_shared, PeerHeader::Core::M7, ScriptingM4::PROTOCOL_VERSION, ScriptingM4::CONTRACT_HASH, 0);
    // ES0445 2.3.1: a WWDG reset of either core resets the whole system; set
    // here too, as stm32_watchdog_init() may not run (BRD_OPTIONS) or run later
    RCC->GCR |= RCC_GCR_WW1RSC | RCC_GCR_WW2RSC;
    __DSB();
    RCC->GCR |= RCC_GCR_BOOT_C2;
    return true;
}

void AP_ScriptingM4::platform_request_restart()
{
    // freeing our restart semaphore raises the M4's HSEM interrupt; a parked M4 polls restart_grant
    if (HwSemaphore::takeTx(ScriptingM4::RESTART_GROUP)) {
        HwSemaphore::releaseTx(ScriptingM4::RESTART_GROUP);
    }
}

bool AP_ScriptingM4::platform_m4_exited()
{
    return false;
}

void AP_ScriptingM4::platform_stop()
{
    // there is no per-core reset (ES0445 2.3.1): with run_permit=0 the restart
    // doorbell parks an M4 that is stalled with interrupts on
    if (_shared != nullptr) {
        platform_request_restart();
    }
}

uint32_t AP_ScriptingM4::platform_wait_us() const
{
    return 10000;
}

#if CH_DBG_STATISTICS != TRUE
#error "the M4 host duty cap charges ChibiOS thread statistics: define HAL_ENABLE_THREAD_STATISTICS"
#endif

bool AP_ScriptingM4::platform_thread_cpu_us(uint64_t &cpu_us)
{
    // a reset of this thread's statistics sets best to all ones: chTMObjectInit() from a thread
    // reading @SYS/threads.txt, or Util::thread_info() on this thread. The M4 cannot reach that
    // file (check_script_path(), fs:* path arguments); an interval with a reset is charged the CPU
    // since it plus RESET_ALLOWANCE_US (interval_cpu_us())
    chSysLock();
    thread_t *self = chThdGetSelfX();
    ScriptingM4::ThreadStatsClock clock { self->stats.cumulative, self->stats.last, self->stats.best };
    bool reset;
    const uint64_t cycles = ScriptingM4::read_thread_cycles(clock, chSysGetRealtimeCounterX(), reset);
    self->stats.best = clock.best;
    chSysUnlock();
    cpu_us = cycles / (HAL_EXPECTED_SYSCLOCK / 1000000U);
    return reset;
}

uint32_t AP_ScriptingM4::platform_timeout_ms() const
{
    return AP_HAL::millis();
}

#endif  // AP_SCRIPTING_M4_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
