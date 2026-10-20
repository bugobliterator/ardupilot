// Records how the M4 was started, then counts, in backup SRAM.
#include <stdint.h>

#define PROBE ((volatile uint32_t *)0x38800F00u)
#define SCB_CPUID (*(volatile uint32_t *)0xE000ED00u)
#define SCB_VTOR (*(volatile uint32_t *)0xE000ED08u)
#define PROBE_UP 0x5055344Du  // "M4UP"

int main(void)
{
    uint32_t stack;
    __asm volatile("mrs %0, msp" : "=r"(stack));
    PROBE[1] = SCB_CPUID;
    PROBE[2] = SCB_VTOR;
    PROBE[3] = stack;
    PROBE[0] = PROBE_UP;
    for (uint32_t count = 1;; ++count) {
        PROBE[4] = count;
    }
}
