#include <stdint.h>

#define RCC_GCR (*(volatile uint32_t *)0x580244A0u)
#define RCC_GCR_BOOT_C2 (1u << 3)

int main(void)
{
    RCC_GCR |= RCC_GCR_BOOT_C2;
    for (;;) {
        __asm volatile("wfi");
    }
}
