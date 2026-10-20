#include "hsem_probe.h"

void HSEM_IRQHandler(void)
{
    const uint32_t status = HSEM_C1MISR;
    HSEM_RESULT[RESULT_M7_IRQ_MASKED_STATUS] = status;
    HSEM_C1ICR = status;
    HSEM_RESULT[RESULT_M7_IRQ_COUNT] += 1;
}

int main(void)
{
    for (int word = 0; word < RESULT_WORDS; ++word) {
        HSEM_RESULT[word] = 0;
    }
    HSEM_R(5) = 0x80000300u;                               // 2-step lock as COREID 3
    HSEM_RESULT[RESULT_M7_TWO_STEP_LOCK] = HSEM_R(5);       // 0x80000300
    HSEM_R(6) = 0x80000100u;                               // COREID 1 written by the M7
    HSEM_RESULT[RESULT_M7_WRONG_COREID] = HSEM_R(6);        // 0: ignored
    HSEM_C1IER = 1u << 7;
    NVIC_ISER(125 / 32) = 1u << (125 % 32);
    HSEM_RESULT[RESULT_M7_STEP] = 1;
    RCC_GCR |= 1u << 3;                                    // BOOT_C2
    while (HSEM_RESULT[RESULT_M4_STEP] < 1 || HSEM_RESULT[RESULT_M7_IRQ_COUNT] == 0) {
    }
    HSEM_RESULT[RESULT_M7_CORE2_RAW_STATUS] = HSEM_C2ISR;   // bit 7: a free flags both cores
    (void)HSEM_RLR(9);                                     // 1-step lock as COREID 3
    HSEM_R(9) = 0x00000300u;                               // free: the M4 has C2IER bit 9
    HSEM_RESULT[RESULT_M7_STEP] = 2;
    while (HSEM_RESULT[RESULT_M4_STEP] < 2) {
    }
    HSEM_R(10) = 0x80000300u;
    HSEM_R(11) = 0x80000300u;
    HSEM_KEYR = 0x12340000u;
    HSEM_CR = (0x4321u << 16) | (3u << 8);                 // wrong key: nothing freed
    HSEM_RESULT[RESULT_M7_WRONG_KEY] = HSEM_R(10);          // 0x80000300
    HSEM_CR = (0x1234u << 16) | (3u << 8);                 // free every COREID 3 semaphore
    HSEM_RESULT[RESULT_M7_CLEARED_10] = HSEM_R(10);
    HSEM_RESULT[RESULT_M7_CLEARED_11] = HSEM_R(11);
    HSEM_RESULT[RESULT_M7_CLEARED_5] = HSEM_R(5);
    HSEM_RESULT[RESULT_M7_STEP] = 3;
    for (;;) {
    }
}
