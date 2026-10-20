#pragma once
// M4 build: single-precision FPU, so the double maths functions are refused as on
// single-precision boards
#include "AP_HAL_Boards.h"
#include <math.h>
#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 0
