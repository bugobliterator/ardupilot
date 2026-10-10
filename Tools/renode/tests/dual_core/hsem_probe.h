#pragma once
#include <stdint.h>

#define HSEM_ADDRESS 0x58026400u
#define HSEM_R(n) (*(volatile uint32_t *)(HSEM_ADDRESS + 4u * (n)))
#define HSEM_RLR(n) (*(volatile uint32_t *)(HSEM_ADDRESS + 0x80u + 4u * (n)))
#define HSEM_C1IER (*(volatile uint32_t *)(HSEM_ADDRESS + 0x100u))
#define HSEM_C1ICR (*(volatile uint32_t *)(HSEM_ADDRESS + 0x104u))
#define HSEM_C1MISR (*(volatile uint32_t *)(HSEM_ADDRESS + 0x10Cu))
#define HSEM_C2IER (*(volatile uint32_t *)(HSEM_ADDRESS + 0x110u))
#define HSEM_C2ICR (*(volatile uint32_t *)(HSEM_ADDRESS + 0x114u))
#define HSEM_C2ISR (*(volatile uint32_t *)(HSEM_ADDRESS + 0x118u))
#define HSEM_C2MISR (*(volatile uint32_t *)(HSEM_ADDRESS + 0x11Cu))
#define HSEM_CR (*(volatile uint32_t *)(HSEM_ADDRESS + 0x140u))
#define HSEM_KEYR (*(volatile uint32_t *)(HSEM_ADDRESS + 0x144u))
#define RCC_GCR (*(volatile uint32_t *)0x580244A0u)
#define NVIC_ISER(n) (*(volatile uint32_t *)(0xE000E100u + 4u * (n)))
#define HSEM_RESULT ((volatile uint32_t *)0x38001000u)

// Word order is read by test_dual_core.py (HSEM_FIELDS).
enum hsem_result_word {
    RESULT_M7_TWO_STEP_LOCK,
    RESULT_M7_WRONG_COREID,
    RESULT_M4_LOCK_WHILE_HELD,
    RESULT_M4_FOREIGN_FREE,
    RESULT_M4_READ_LOCK,
    RESULT_M7_IRQ_MASKED_STATUS,
    RESULT_M7_IRQ_COUNT,
    RESULT_M7_CORE2_RAW_STATUS,
    RESULT_M4_IRQ_MASKED_STATUS,
    RESULT_M4_IRQ_COUNT,
    RESULT_M7_WRONG_KEY,
    RESULT_M7_CLEARED_10,
    RESULT_M7_CLEARED_11,
    RESULT_M7_CLEARED_5,
    RESULT_M4_STEP,
    RESULT_M7_STEP,
    RESULT_WORDS,
};
