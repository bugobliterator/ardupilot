#pragma once
/*
  M4 Lua build: the board facts AP_Math and AP_Common/Location read, as the
  CubeOrangePlus-M4Lua M7 has them, so the M4's local pure-data types compute
  what the M7 computes in-process (double ftype and position types). The SITL
  M4 process uses the same values and follows the hardware image: SITL-only
  checks in Location.cpp are compiled out.
 */
#define HAL_BOARD_SITL     3
#define HAL_BOARD_LINUX    7
#define HAL_BOARD_CHIBIOS  10
#define HAL_BOARD_ESP32    12
#define HAL_BOARD_QURT     13
#define HAL_BOARD_EMPTY    99
#define HAL_BOARD_SUBTYPE_NONE -1
// SITL's waf defines these on the command line for the M7
#undef CONFIG_HAL_BOARD
#define CONFIG_HAL_BOARD HAL_BOARD_CHIBIOS
#undef CONFIG_HAL_BOARD_SUBTYPE
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_NONE
#define HAL_HAVE_HARDWARE_DOUBLE 1
#define HAL_WITH_EKF_DOUBLE 1
#define HAL_WITH_POSTYPE_DOUBLE 1
// SITL force-includes ap_config.h, which defines it
#undef HAL_NUM_CAN_IFACES
#define HAL_NUM_CAN_IFACES 0
#ifndef AP_SIM_ENABLED
#define AP_SIM_ENABLED 0
#endif
// the M7 passes -D__AP_LINE__=__LINE__ (constrain_float)
#ifndef __AP_LINE__
#define __AP_LINE__ __LINE__
#endif
