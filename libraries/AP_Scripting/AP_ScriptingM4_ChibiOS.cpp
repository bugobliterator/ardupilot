#include "AP_ScriptingM4.h"

#if AP_SCRIPTING_M4_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

#include <hal.h>
#include <string.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>
#include <GCS_MAVLink/GCS.h>
#include <CubePilotFW/Channel.h>
#include <CubePilotFW/PeerHeader.h>
#include <CubePilotFW/pal/HSEM.h>
#include "AP_ScriptingM4_Layout.h"

#if !defined(STM32H757xx)
#error "M4 scripting needs a dual-core STM32H757"
#endif

extern const AP_HAL::HAL& hal;

using CubePilotFW::HwSemaphore;
namespace PeerHeader = CubePilotFW::PeerHeader;

// the host thread runs at PRIORITY_SCRIPTING, the interrupt only signals it
static const uint8_t HSEM_IRQ_PRIORITY = 12;

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
    memset(_shared, 0, ScriptingM4::WINDOW_SIZE);
    ScriptingM4::SupervisorBlock *block = ScriptingM4::supervisor_block(_shared);
    block->run_permit.store(1);
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

uint32_t AP_ScriptingM4::platform_timeout_ms() const
{
    return AP_HAL::millis();
}

#endif  // AP_SCRIPTING_M4_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
