#pragma once
/*
  M4 build: the board facts AP_Math and AP_Common/Location read, as the
  CubeOrangePlus-SimOnHardWare-M4Module M7 has them, so the M4's local pure-data types compute
  what the M7 computes in-process (double ftype and position types). The SITL
  M4 process uses the same values and follows the hardware image: SITL-only
  checks in Location.cpp are compiled out.

  Deliberate differences from the M7's AP_Math and Location:
  - INTERNAL_ERROR is a no-op (AP_InternalError.h): the M7 latches the
    internal-error flag and continues with the same result (AP_Math.cpp:317,421,
    quaternion.cpp:395,673,803, vector3.cpp:273)
  - ROTATION_CUSTOM_1/2 have no custom rotation: Vector3::rotate and
    Quaternion::from_rotation leave the value unchanged. Methods with a Rotation
    argument stay remote, so this is never reached (AP_CustomRotations.h)
  - Location methods that read home, the EKF origin or terrain are not built
    (AP_AHRS.h, AP_Terrain.h); they stay remote
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
#ifndef AP_SIM_ENABLED
#define AP_SIM_ENABLED 0
#endif
// the M7 passes -D__AP_LINE__=__LINE__ (constrain_float)
#ifndef __AP_LINE__
#define __AP_LINE__ __LINE__
#endif
