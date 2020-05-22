#include "Parameters.h"

extern const AP_HAL::HAL &hal;

/*
 *  AP_Periph parameter definitions
 *
 */

#define GSCALAR(v, name, def) { v.vtype, name, Parameters::k_param_ ## v, &v, {def_value : def} }

AP_Int16 Parameters::format_version;
AP_Int16 Parameters::num_cycles;
AP_Int16 Parameters::num_fails;
AP_Int16 Parameters::loop_sensor_health;
AP_Int16 Parameters::setup_sensor_health;


const AP_Param::Info Parameters::var_info[] = {
    GSCALAR(format_version,         "FORMAT_VERSION", 0),
    GSCALAR(num_cycles,         "NUM_CYCLES", 0),
    GSCALAR(num_fails,         "NUM_FAILS", 0),
    GSCALAR(loop_sensor_health,     "SENSOR_HMASK", SENSOR_MASK),
    GSCALAR(setup_sensor_health,     "SENSOR_HMASK", SENSOR_MASK),
    AP_VAREND
};


void Parameters::load_parameters(void)
{
    AP_Param::setup_sketch_defaults();

    if (!AP_Param::check_var_info()) {
        hal.console->printf("Bad parameter table\n");
        AP_HAL::panic("Bad parameter table");
    }
    if (!format_version.load() ||
        format_version != Parameters::k_format_version) {
        // erase all parameters
        StorageManager::erase();
        AP_Param::erase_all();

        // save the current format version
        format_version.set_and_save(Parameters::k_format_version);
    }

    // Load all auto-loaded EEPROM variables
    AP_Param::load_all();
}
