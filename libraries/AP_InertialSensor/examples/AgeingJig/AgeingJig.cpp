//
// Simple test for the AP_InertialSensor driver.
//

#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include "UAVCAN_handler.h"
#include <GCS_MAVLink/GCS_Dummy.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();
int ownserial = -1;

void setup();
void loop();

#define UAVCAN_NODE_POOL_SIZE 8192
#ifdef UAVCAN_NODE_POOL_BLOCK_SIZE
#undef UAVCAN_NODE_POOL_BLOCK_SIZE
#endif
#define UAVCAN_NODE_POOL_BLOCK_SIZE 256


// board specific config
static AP_BoardConfig BoardConfig;
static AP_InertialSensor ins;
static AP_Baro baro;

void setup(void);
void loop(void);

void setup(void)
{
    // setup any board specific drivers
    BoardConfig.init();
    AP::ins().init(100);

    // initialize the barometer
    AP::baro().init();
    AP::baro().calibrate();

    UAVCAN_handler::init();
}

#define ACC_HEALTH_OFF  0
#define GYR_HEALTH_OFF 3
#define BARO_HEALTH_OFF 6

void loop()
{
    UAVCAN_handler::loop();

    AP::ins().update();

    AP::baro().update();
}

const struct AP_Param::GroupInfo        GCS_MAVLINK::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

AP_HAL_MAIN();
