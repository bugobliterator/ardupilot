#pragma once

#include "AP_ModuleM4_config.h"

#if AP_MODULE_ENABLED && AP_MODULE_M4_ENABLED
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Mission/AP_Mission_config.h>
#include <AP_Param/AP_Param.h>

namespace AP_Module
{

// M4 variants: the in-process Runtime's interface, parameters and mission queue; modules run on
// the Cortex-M4 (AP_ModuleM4)
class Runtime
{
public:
#if AP_MISSION_ENABLED
    struct MissionCommand {
        uint16_t p1;
        float content_p1;
        float content_p2;
        float content_p3;
        uint32_t time_ms;
    };
#endif
    struct Config {
        int8_t enable;
        int32_t heap_size;
        int32_t code_size;
        int32_t stack_size;
        int8_t max_modules;
        int16_t restart;
        int32_t loaded_required;
        int32_t running_required;
        int8_t debug_options;
    };

    Runtime();
    CLASS_NO_COPY(Runtime);
    static Runtime *get_singleton()
    {
        return _singleton;
    }
    static const AP_Param::GroupInfo var_info[];
    void init();
    void update();
    bool arming_checks(size_t length, char *message) const;
#if AP_MISSION_ENABLED
    void handle_mission_command(uint16_t p1, float content_p1, float content_p2, float content_p3);
    bool pop_mission_command(MissionCommand &command);
#endif
    Config config() const;

private:
    static Runtime *_singleton;
    AP_Int8 _enable;
    AP_Int32 _heap_size;
    AP_Int32 _code_size;
    AP_Int32 _stack_size;
    AP_Int8 _max_modules;
    AP_Int16 _restart;
    AP_Int32 _loaded_required;
    AP_Int32 _running_required;
    AP_Int8 _debug_options;
#if AP_MISSION_ENABLED
    static constexpr uint8_t MISSION_QUEUE_SIZE = 5;
    HAL_Semaphore _mission_sem;
    MissionCommand _mission_commands[MISSION_QUEUE_SIZE] {};
    uint8_t _mission_head{};
    uint8_t _mission_count{};
#endif
};

}
#endif
