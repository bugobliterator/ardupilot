//
// Simple test for the AP_InertialSensor driver.
//

#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include "UAVCAN_handler.h"
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>

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
    hal.uartA->begin(AP_SERIALMANAGER_CONSOLE_BAUD, 32, 128);
    hal.uartB->begin(115200, 32, 128);

    AP::ins().init(100);
    // initialize the barometer
    AP::baro().init();
    AP::baro().calibrate();
    hal.scheduler->delay(2000);
    hal.console->printf("Starting UAVCAN\n");
    UAVCAN_handler::init();
}

#define IMU_HIGH_TEMP 65
#define IMU_LOW_TEMP  50

static uint16_t _sensor_health_mask = 0xFF;
static int8_t _heater_target_temp = IMU_HIGH_TEMP;
static uint32_t _hold_start_ms;
static uint8_t _heater_state;

void loop()
{
    AP::ins().update();
    AP::baro().update();
    for (uint8_t i = 0; i < 3; i++) {
        if (!AP::ins().get_accel_health(i)) {
            _sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::ACCEL_HEALTH_OFF) << i);
        }
        if (!AP::ins().get_gyro_health(i)) {
            _sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::GYRO_HEALTH_OFF) << i);
        }
    }
    for (uint8_t i = 0; i < 2; i++) {
        if (!AP::baro().healthy(i)) {
            _sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::BARO_HEALTH_OFF) << i);
        }
    }

    if ((_heater_target_temp - AP::ins().get_temperature(0)) > 3.0f) {
        _heater_state = com::hex::equipment::jig::Status::HEATER_STATE_HEATING;
        _hold_start_ms = AP_HAL::millis();
    } else if ((_heater_target_temp - AP::ins().get_temperature(0)) < -3.0f) {
        _heater_state = com::hex::equipment::jig::Status::HEATER_STATE_COOLING;
        _hold_start_ms = AP_HAL::millis();
    } else {
        _heater_state = com::hex::equipment::jig::Status::HEATER_STATE_HOLDING;
    }
    //hal.console->printf("Temp Delta: %d %d %f\n", _heater_state, AP_HAL::millis() - _hold_start_ms, (_heater_target_temp - AP::ins().get_temperature(0)));

    if ((_heater_state == com::hex::equipment::jig::Status::HEATER_STATE_HOLDING) && ((AP_HAL::millis() - _hold_start_ms) >= 10000)) {
        if (_heater_target_temp == IMU_HIGH_TEMP) {
            _heater_target_temp = IMU_LOW_TEMP;
        } else if (_heater_target_temp == IMU_LOW_TEMP) {
            _heater_target_temp = IMU_HIGH_TEMP;
        }
    }

    hal.util->set_imu_target_temp(&_heater_target_temp);
    UAVCAN_handler::set_sensor_states(_sensor_health_mask, _heater_state);
    UAVCAN_handler::loop_all();
}

GCS_Dummy _gcs;

extern mavlink_system_t mavlink_system;

const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};

AP_HAL_MAIN();
