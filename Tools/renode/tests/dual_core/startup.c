// Minimal Cortex-M7/M4 start-up for the Renode dual-core test images.
#include <stdint.h>

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
extern void (*__init_array_start[])(void);
extern void (*__init_array_end[])(void);
int main(void);

void Reset_Handler(void);
void Default_Handler(void);
void HSEM_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

#define SCB_CPACR (*(volatile uint32_t *)0xE000ED88u)

void Default_Handler(void)
{
    for (;;) {
    }
}

__attribute__((section(".vectors"), used))
void (*const vector_table[16 + 150])(void) = {
    [0] = (void (*)(void))&_estack,
    [1] = Reset_Handler,
    [2 ... 16 + 149] = Default_Handler,
    [16 + HSEM_IRQ_NUMBER] = HSEM_IRQHandler,
};

void Reset_Handler(void)
{
    uint32_t *source = &_sidata;
    for (uint32_t *destination = &_sdata; destination < &_edata;) {
        *destination++ = *source++;
    }
    for (uint32_t *destination = &_sbss; destination < &_ebss;) {
        *destination++ = 0;
    }
    SCB_CPACR |= 0xFu << 20;  // CP10/CP11 full access: the images use the hard-float ABI
    __asm volatile("dsb\n isb" ::: "memory");
    for (void (**constructor)(void) = __init_array_start; constructor < __init_array_end; ++constructor) {
        (*constructor)();
    }
    main();
    for (;;) {
    }
}
