#pragma once
/*
  M4 Lua build: what AP_Math (ftype.h) and AP_Common/Location take from
  AP_HAL: the board facts and, as the real AP_HAL.h, AP_Common (the AP_Math
  headers use WARN_IF_UNUSED). Their AP_HAL::panic calls are all SITL-only and
  compiled out (AP_HAL_Boards.h), so there is no AP_HAL::panic here.
 */
#include "AP_HAL_Boards.h"
#include <stdint.h>
#include <AP_Common/AP_Common.h>
