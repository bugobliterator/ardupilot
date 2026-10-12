#pragma once
/*
  M4 Lua build: what AP_Math (ftype.h) and AP_Common/Location take from
  AP_HAL. AP_HAL::panic is reached only by a programming error (fill_nanf,
  SITL-only checks): on the M4 it traps, which the fault handler contains.
  AP_Common as the real AP_HAL.h: the AP_Math headers use WARN_IF_UNUSED.
 */
#include "AP_HAL_Boards.h"
#include <stdint.h>
#include <AP_Common/AP_Common.h>

namespace AP_HAL {
[[noreturn]] inline void panic(const char *, ...)
{
    __builtin_trap();
}
}
