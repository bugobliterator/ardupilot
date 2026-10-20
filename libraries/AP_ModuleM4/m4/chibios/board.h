#pragma once

/*
  STM32H757 Cortex-M4 running native Rust modules. The M4 owns no peripherals: no
  pins, no clocks (the M7 configured both); only the device selection that
  ChibiOS's cmparams.h needs.
 */
#if !defined(STM32H757xx)
#define STM32H757xx
#endif
