#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_SerialManager/AP_SerialManager_config.h>
#include <AP_Vehicle/AP_Vehicle_config.h>

#ifndef AP_SCRIPTING_ENABLED
#define AP_SCRIPTING_ENABLED (HAL_PROGRAM_SIZE_LIMIT_KB > 1024)
#endif

// time every binding call and semaphore wait; logs SRPC (measurement builds only)
#ifndef AP_SCRIPTING_BINDING_STATS_ENABLED
#define AP_SCRIPTING_BINDING_STATS_ENABLED 0
#endif

#ifndef AP_SCRIPTING_HEAP_EXPANSION_ALWAYS_ENABLED
#define AP_SCRIPTING_HEAP_EXPANSION_ALWAYS_ENABLED 0
#endif

#if AP_SCRIPTING_ENABLED
    #include <AP_Filesystem/AP_Filesystem_config.h>
    // enumerate all of the possible places we can read a script from.
    #if !AP_FILESYSTEM_POSIX_ENABLED && !AP_FILESYSTEM_FATFS_ENABLED && !AP_FILESYSTEM_ESP32_ENABLED && !AP_FILESYSTEM_ROMFS_ENABLED && !AP_FILESYSTEM_LITTLEFS_ENABLED
        #error "Scripting requires a filesystem"
    #endif
#endif

#ifndef AP_SCRIPTING_SERIALDEVICE_ENABLED
#define AP_SCRIPTING_SERIALDEVICE_ENABLED AP_SERIALMANAGER_REGISTER_ENABLED && (HAL_PROGRAM_SIZE_LIMIT_KB>1024)
#endif

// bindings configuration
#ifndef AP_SCRIPTING_BINDING_MOTORS_ENABLED
#define AP_SCRIPTING_BINDING_MOTORS_ENABLED (AP_SCRIPTING_ENABLED && AP_VEHICLE_ENABLED)
#endif

#ifndef AP_SCRIPTING_BINDING_VEHICLE_ENABLED
#define AP_SCRIPTING_BINDING_VEHICLE_ENABLED 1
#endif  // AP_SCRIPTING_BINDING_VEHICLE_ENABLED

// Lua on the Cortex-M4 of an STM32H757 (board CubeOrangePlus-M4Lua, or SITL
// with --enable-scripting-m4): the M7 keeps the scripting host and the
// supervisor, the in-process interpreter is not started
#ifndef AP_SCRIPTING_M4_ENABLED
#define AP_SCRIPTING_M4_ENABLED 0
#endif

#if AP_SCRIPTING_M4_ENABLED && !(defined(AP_CUBEFRAMEWORK_ENABLED) && AP_CUBEFRAMEWORK_ENABLED)
#error "AP_SCRIPTING_M4_ENABLED needs AP_CUBEFRAMEWORK_ENABLED"
#endif

// M4 request statistics (spec §11): SRPC rows for the M4's requests and the M4ST supervisor log
#ifndef AP_SCRIPTING_RPC_STATS_ENABLED
#define AP_SCRIPTING_RPC_STATS_ENABLED AP_SCRIPTING_M4_ENABLED
#endif

// test-only SCR_M4_INJECT parameter; to be turned off before any release
#ifndef AP_SCRIPTING_M4_FAULT_INJECTION_ENABLED
#define AP_SCRIPTING_M4_FAULT_INJECTION_ENABLED AP_SCRIPTING_M4_ENABLED
#endif
