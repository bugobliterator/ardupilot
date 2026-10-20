#include "hsem_probe.h"

void HSEM_IRQHandler(void)
{
    const uint32_t status = HSEM_C2MISR;
    HSEM_RESULT[RESULT_M4_IRQ_MASKED_STATUS] = status;
    HSEM_C2ICR = status;
    HSEM_RESULT[RESULT_M4_IRQ_COUNT] += 1;
}

int main(void)
{
    HSEM_R(5) = 0x80000100u;                               // held by the M7: no change
    HSEM_RESULT[RESULT_M4_LOCK_WHILE_HELD] = HSEM_R(5);     // 0x80000300
    HSEM_R(5) = 0x00000100u;                               // free by a non-owner: ignored
    HSEM_RESULT[RESULT_M4_FOREIGN_FREE] = HSEM_R(5);        // 0x80000300
    HSEM_RESULT[RESULT_M4_READ_LOCK] = HSEM_RLR(7);         // 1-step lock as COREID 1: 0x80000100
    HSEM_C2IER = 1u << 9;
    NVIC_ISER(126 / 32) = 1u << (126 % 32);
    HSEM_R(7) = 0x00000100u;                               // free: interrupts the M7
    HSEM_RESULT[RESULT_M4_STEP] = 1;
    while (HSEM_RESULT[RESULT_M7_STEP] < 2 || HSEM_RESULT[RESULT_M4_IRQ_COUNT] == 0) {
    }
    HSEM_RESULT[RESULT_M4_STEP] = 2;
    for (;;) {
    }
}
