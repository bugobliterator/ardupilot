
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#if defined(STM32H7) || defined(STM32F7)
//CubeOrange or CubeOrange
#define SENSOR_MASK 0x1FF
#elif defined(STM32F4)
//CubeBlack
#define SENSOR_MASK 0x3FF
#endif

// Global parameter class.
//
class Parameters {
public:
    static const uint16_t k_format_version = 5;

    enum {
        // Layout version number, always key zero.
        //
        k_param_format_version = 0,
        k_param_num_cycles,
        k_param_num_fails,
        k_param_loop_sensor_health,
        k_param_setup_sensor_health
    };

    static AP_Int16 format_version;
    static AP_Int16 num_cycles;
    static AP_Int16 num_fails;
    static AP_Int16 loop_sensor_health;
    static AP_Int16 setup_sensor_health;

    Parameters() {}
    void load_parameters(void);
private:
    
    // setup the var_info table
    AP_Param param_loader{var_info};

    static const AP_Param::Info var_info[];
};

extern const AP_Param::Info var_info[];
