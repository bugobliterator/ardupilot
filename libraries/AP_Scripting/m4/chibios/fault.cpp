/*
  Fault handlers of the M4 scripting image: record the fault in this core's
  CubeFramework peer header, wait for the M7 to grant a restart, then warm
  restart. HardFault runs with the MPU off (HFNMIENA=0); MemManage, BusFault
  and UsageFault run with it on and touch only SCB, SysTick, the stacked frame
  (SRAM1/2) and the shared window.
 */
#include "ch.h"

#include <CubePilotFW/PeerHeader.h>
#include <AP_ScriptingM4_Layout.h>

#include "m4_fault.h"

using namespace ScriptingM4;
namespace PeerHeader = CubePilotFW::PeerHeader;

static uint32_t restart_grant_at_start;
static bool restart_grant_latched;

static uint8_t *shared_window()
{
    return reinterpret_cast<uint8_t *>(AP_SCRIPTING_M4_WINDOW_BASE);
}

// one SysTick period: with PRIMASK set a pending interrupt still ends WFI
static void sleep_one_tick()
{
    __WFI();
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
}

// busy-wait for the next SysTick wrap: inside a fault handler the SysTick
// interrupt has too low a priority to end a WFI
static void wait_one_tick()
{
    if ((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) == 0) {
        // faulted before main() started SysTick
        SysTick->LOAD = AP_SCRIPTING_M4_CORE_CLOCK_HZ / CH_CFG_ST_FREQUENCY - 1U;
        SysTick->VAL = 0;
        SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    }
    // COUNTFLAG clears on read
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0) {
    }
}

extern "C" uint32_t m4_latch_restart_grant(void)
{
    restart_grant_at_start = supervisor_block(shared_window())->restart_grant.load();
    restart_grant_latched = true;
    return restart_grant_at_start;
}

extern "C" bool m4_restart_granted(void)
{
    return restart_grant_latched &&
           supervisor_block(shared_window())->restart_grant.load() != restart_grant_at_start;
}

extern "C" void m4_park_forever(void)
{
    __disable_irq();
    for (;;) {
        sleep_one_tick();
    }
}

extern "C" void m4_warm_restart(void)
{
    __disable_irq();
    if (__get_IPSR() == 0 || (SCB->ICSR & SCB_ICSR_RETTOBASE_Msk) == 0) {
        // thread mode, or another exception is active: the exception return would fault
        m4_park_forever();
    }
    SysTick->CTRL = 0;
    for (uint32_t i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFU;
        NVIC->ICPR[i] = 0xFFFFFFFFU;
    }
    SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk | SCB_ICSR_PENDSTCLR_Msk;
    FPU->FPCCR &= ~FPU_FPCCR_LSPACT_Msk;
    MPU->CTRL = 0;   // main() programs it again
    __DSB();
    __ISB();
    const uint32_t *vectors = reinterpret_cast<const uint32_t *>(AP_SCRIPTING_M4_FLASH_BASE);
    register uint32_t initial_sp asm("r0") = vectors[0];
    register uint32_t reset_handler asm("r1") = vectors[1];
    // a basic exception frame on the image's initial main stack, returning to Reset_Handler
    __asm volatile(
        "mov   r2, #0\n"
        "msr   control, r2\n"       // privileged, no FP context
        "isb\n"
        "sub   r0, r0, #32\n"
        "str   r2, [r0, #0]\n"      // r0
        "str   r2, [r0, #4]\n"      // r1
        "str   r2, [r0, #8]\n"      // r2
        "str   r2, [r0, #12]\n"     // r3
        "str   r2, [r0, #16]\n"     // r12
        "mvn   r3, #0\n"
        "str   r3, [r0, #20]\n"     // lr
        "bic   r1, r1, #1\n"
        "str   r1, [r0, #24]\n"     // pc: Reset_Handler without the Thumb bit
        "mov   r3, #0x01000000\n"
        "str   r3, [r0, #28]\n"     // xPSR: Thumb
        "msr   msp, r0\n"
        "mvn   lr, #6\n"            // EXC_RETURN 0xFFFFFFF9: thread mode, main stack, basic frame
        "bx    lr\n"
        : "+r"(initial_sp), "+r"(reset_handler)
        :
        : "r2", "r3", "lr", "memory");
    __builtin_unreachable();
}

// frame: stacked r0-r3, r12, lr, pc, xpsr of the faulting context
extern "C" __attribute__((used, noreturn)) void m4_fault_entry(const uint32_t *frame)
{
    const uint32_t cfsr = SCB->CFSR;
    const uint32_t hfsr = SCB->HFSR;
    SupervisorBlock *block = supervisor_block(shared_window());
    // a grant is a change from the value seen at start-up, so a grant written
    // before this handler reads the block is not missed
    const uint32_t granted = restart_grant_latched ? restart_grant_at_start : block->restart_grant.load();
    const bool frame_valid = (cfsr & (SCB_CFSR_MSTKERR_Msk | SCB_CFSR_STKERR_Msk)) == 0;
    PeerHeader::recordFault(shared_window(), PeerHeader::Core::M4, cfsr, hfsr,
                            frame_valid ? frame[6] : 0, frame_valid ? frame[5] : 0);
    SCB->CFSR = cfsr;   // write one to clear
    SCB->HFSR = hfsr;
    __disable_irq();
    // the M7 grants a restart within its budget; past the budget it withdraws run_permit
    for (;;) {
        if (block->run_permit.load() == 0) {
            m4_park_forever();
        }
        if (block->restart_grant.load() != granted) {
            break;
        }
        wait_one_tick();
    }
    m4_warm_restart();
}

extern "C" __attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "tst   lr, #4\n"
        "ite   eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "b     m4_fault_entry\n");
}

extern "C" void MemManage_Handler(void) noexcept __attribute__((alias("HardFault_Handler")));
extern "C" void BusFault_Handler(void) noexcept __attribute__((alias("HardFault_Handler")));
extern "C" void UsageFault_Handler(void) noexcept __attribute__((alias("HardFault_Handler")));
