//
// Simple test for the AP_InertialSensor driver.
//

#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include "UAVCAN_handler.h"
#include <GCS_MAVLink/GCS_Dummy.h>
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>
#include <AP_Param/AP_Param.h>
#include <AP_Logger/AP_Logger.h>
#include "Parameters.h"
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>

static Parameters g;

const AP_HAL::HAL &hal = AP_HAL::get_HAL();
int ownserial = -1;

void setup();
void loop();

#define UAVCAN_NODE_POOL_SIZE 8192
#ifdef UAVCAN_NODE_POOL_BLOCK_SIZE
#undef UAVCAN_NODE_POOL_BLOCK_SIZE
#endif
#define UAVCAN_NODE_POOL_BLOCK_SIZE 256
const struct LogStructure log_structure[] = {
    LOG_COMMON_STRUCTURES
};

// board specific config
static AP_BoardConfig BoardConfig;
static AP_InertialSensor ins;
static AP_Baro baro;
static Compass compass;
static AP_Int32 unused;
static AP_Logger logger{unused};
static AP_GPS gps;
static AP_AHRS_DCM ahrs;

void log_sensor_health(uint16_t sensor_health);
void setup(void);
void loop(void);

static uint16_t _setup_sensor_health_mask = SENSOR_MASK;
static uint16_t _loop_sensor_health_mask = SENSOR_MASK;
static uint16_t _last_sensor_health_mask = SENSOR_MASK;
static uint16_t _instant_sensor_health_mask = SENSOR_MASK;
static uint32_t _last_accel_error_count[3];
static uint32_t _last_gyro_error_count[3];
static bool fault_recorded;
static int loop_cycle = 0;
char sensor_pos[][5] = {"ACC0", "ACC1", "ACC2", "GYR0", "GYR1", "GYR2", "BAR0", "BAR1", "COM0", "COM1"};

void log_sensor_health(uint16_t sensor_health)
{
    // @LoggerMessage: SENH
    // @Description: Sensor health
    // @Field: TimeUS: Time since system startup
    // @Field: COM1: Compass 1 health
    // @Field: COM1: Compass 0 health
    // @Field: BAR1: Baro 1 health
    // @Field: BAR0: Baro 0 health
    // @Field: GYR2: Gyro 2 health
    // @Field: GYR1: Gyro 1 health
    // @Field: GYR0: Gyro 0 health
    // @Field: ACC0: Accel 2 health
    // @Field: ACC1: Accel 1 health
    // @Field: ACC2: Accel 0 health
    AP::logger().Write("SENH", "TimeUS,COM1,COM0,BAR1,BAR0,GYR2,GYR1,GYR0,ACC2,ACC1,ACC0", "QHHHHHHHHHH",
                                            AP_HAL::micros64(),
                                            (sensor_health >> 9) & 1,
                                            (sensor_health >> 8) & 1,
                                            (sensor_health >> 7) & 1,
                                            (sensor_health >> 6) & 1,
                                            (sensor_health >> 5) & 1,
                                            (sensor_health >> 4) & 1,
                                            (sensor_health >> 3) & 1,
                                            (sensor_health >> 2) & 1,
                                            (sensor_health >> 1) & 1,
                                            (sensor_health) & 1);
}

void setup(void)
{
    unused = -1;
    BoardConfig.init();
    // setup any board specific drivers
    hal.uartA->begin(AP_SERIALMANAGER_CONSOLE_BAUD, 32, 128);
    hal.uartB->begin(115200, 32, 128);
    hal.uartC->begin(9600, 32, 128);

    ins.init(100);
    // initialize the barometer
    baro.init();
    baro.calibrate();
    compass.init();
    hal.scheduler->delay(2000);
    hal.console->printf("Starting UAVCAN\n");
    hal.uartC->printf("Testing firmware updated on 5/8/2020 1645\n");
    hal.uartC->printf("Starting UAVCAN\n");
    hal.gpio->pinMode(0, HAL_GPIO_OUTPUT);
    UAVCAN_handler::init();
    g.load_parameters();
    g.num_cycles.set_and_save(g.num_cycles.get()+1);
    logger.Init(log_structure, ARRAY_SIZE(log_structure));

    //setup test
    hal.scheduler->delay(3000);
    AP::logger().Write_Message("Setup Test Started");
    AP::ins().update();
    AP::baro().update();
    AP::compass().read();
    for (uint8_t i = 0; i < 3; i++) {
        if (!AP::ins().get_accel_health(i)) {
            _setup_sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::ACCEL_HEALTH_OFF) << i);
        }
        if (!AP::ins().get_gyro_health(i)) {
            _setup_sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::GYRO_HEALTH_OFF) << i);
        }
    }
    for (uint8_t i = 0; i < 2; i++) {
        if (!AP::baro().healthy(i)) {
            _setup_sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::BARO_HEALTH_OFF) << i);
        }
    }
    for (uint8_t i = 0; i < 2; i++) {
        if (!AP::compass().healthy(i)) {
            _setup_sensor_health_mask &= ~((1 << 8) << i);
        }
    }

    if (_setup_sensor_health_mask != SENSOR_MASK) {
        g.num_fails.set_and_save(g.num_fails.get()+1);
        g.setup_sensor_health.set_and_save(g.setup_sensor_health.get()&_setup_sensor_health_mask);
    }
    _last_sensor_health_mask = _setup_sensor_health_mask;
    AP::logger().Write_Message("Setup Test Ended");
    AP::logger().Write_MessageF("Healthy Flag: 0x%x", SENSOR_MASK);
    AP::logger().Write_MessageF("Setup Test Flag: 0x%x", _setup_sensor_health_mask);
    AP::logger().Write_MessageF("Accumulated Test Flag: 0x%x", g.loop_sensor_health.get());
    
    for (uint8_t i = 0; i < ((SENSOR_MASK >> 9) + 9); i++) {
        if (((_setup_sensor_health_mask >> i) & 1) == 0) {
            AP::logger().Write_MessageF("bad %s in setup", sensor_pos[i]);
        }
    }
    log_sensor_health(_setup_sensor_health_mask);

    hal.console->printf("Log: %d\n", AP::logger().find_last_log());
    hal.uartC->printf("Log: %d\n", AP::logger().find_last_log());
    
}

#define IMU_HIGH_TEMP 70
// #define IMU_LOW_TEMP  55

static int8_t _heater_target_temp = IMU_HIGH_TEMP;
static uint32_t _hold_start_ms;
static uint8_t _heater_state;
static uint32_t _led_blink_ms;
static uint32_t _led_blink_state;

void loop()
{
    if (loop_cycle == 0)
        AP::logger().Write_Message("Loop Test Started");
    
    //loop test
    AP::ins().update();
    AP::baro().update();
    AP::compass().read();

    _instant_sensor_health_mask = SENSOR_MASK;

    for (uint8_t i = 0; i < 3; i++) {
        if (!AP::ins().get_accel_health(i)) {
            _loop_sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::ACCEL_HEALTH_OFF) << i);
            _instant_sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::ACCEL_HEALTH_OFF) << i);
            
        }
        if (!AP::ins().get_gyro_health(i)) {
            _loop_sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::GYRO_HEALTH_OFF) << i);
            _instant_sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::GYRO_HEALTH_OFF) << i);
        }
    }
    for (uint8_t i = 0; i < 2; i++) {
        if (!AP::baro().healthy(i)) {
            _loop_sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::BARO_HEALTH_OFF) << i);
            _instant_sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::BARO_HEALTH_OFF) << i);
        }
    }
    for (uint8_t i = 0; i < 2; i++) {
        if (!AP::compass().healthy(i)) {
            _loop_sensor_health_mask &= ~((1 << 8) << i);
            _instant_sensor_health_mask &= ~((1 << 8) << i);
        }
    }

    if (_loop_sensor_health_mask != SENSOR_MASK) {
        if (!fault_recorded) {
            fault_recorded = true;
            g.num_fails.set_and_save(g.num_fails.get()+1);
            g.loop_sensor_health.set_and_save(g.loop_sensor_health.get()&_loop_sensor_health_mask);
        }
    }
    loop_cycle++;

    // Log sensor failure event
    if (_instant_sensor_health_mask != _last_sensor_health_mask) {
         for (uint8_t i = 0; i < ((SENSOR_MASK >> 9) + 9); i++)
            if (((_instant_sensor_health_mask >> i) & 1) > ((_last_sensor_health_mask >> i) & 1)) {
                AP::logger().Write_MessageF("%s regain at loop %d\n", sensor_pos[i], loop_cycle);
                hal.console->printf("[EVENT]\t%s regain at loop %d\n", sensor_pos[i], loop_cycle);
                hal.uartC->printf("[EVENT]\t%s regain at loop %d\n", sensor_pos[i], loop_cycle);
            }
            else if (((_instant_sensor_health_mask >> i) & 1) < ((_last_sensor_health_mask >> i) & 1)) {
                AP::logger().Write_MessageF("%s lost at loop %d\n", sensor_pos[i], loop_cycle);
                hal.console->printf("[EVENT]\t%s lost at loop %d\n", sensor_pos[i], loop_cycle);
                hal.uartC->printf("[EVENT]\t%s lost at loop %d\n", sensor_pos[i], loop_cycle);
            }
    }

    //hal.uartC->printf("health:0x%x\n", _loop_sensor_health_mask);
    log_sensor_health(_instant_sensor_health_mask);
    _last_sensor_health_mask = _instant_sensor_health_mask;  

    //Write IMU Data to Log
    logger.Write_IMU();  

    // Do LED Patterns
    if ((AP_HAL::millis() - _led_blink_ms) > 2000) {
        _led_blink_state = 0;
        _led_blink_ms = AP_HAL::millis();

        if (_loop_sensor_health_mask ^ g.loop_sensor_health.get()) {
            hal.console->printf("Fail sensor changed in this run. Log: %d\n", AP::logger().find_last_log());
            hal.uartC->printf("Fail sensor changed in this run. Log: %d\n", AP::logger().find_last_log());
        }

        hal.uartC->printf("com order: [0]%lu [1]%lu [2]%lu ", AP::compass().get_dev_id(0), AP::compass().get_dev_id(1), AP::compass().get_dev_id(2));
        hal.uartC->printf("gyr order: [0]%lu [1]%lu [2]%lu ", AP::ins().get_accel_id(0), AP::ins().get_accel_id(1), AP::ins().get_accel_id(2));
        hal.uartC->printf("acc order: [0]%lu [1]%lu [2]%lu \n", AP::ins().get_gyro_id(0), AP::ins().get_gyro_id(1), AP::ins().get_gyro_id(2));
        //hal.uartC->printf("Baro1(fmu) on %u; Baro0(imu)0 on %u\n", AP::baro().get_baro_bus(1), AP::baro().get_baro_bus(0));
        hal.console->printf("SENSOR_MASK: 0x%x NUM_RUNS: %d NUM_FAILS: %d LOOP_TEST_FLAGS: 0x%x SETUP_TEST_FLAGS: 0x%x\n", SENSOR_MASK, g.num_cycles.get(), g.num_fails.get(), g.loop_sensor_health.get(), g.setup_sensor_health.get());
        hal.uartC->printf("SENSOR_MASK: 0x%x NUM_RUNS: %d NUM_FAILS: %d LOOP_TEST_FLAGS: 0x%x SETUP_TEST_FLAGS: 0x%x\n", SENSOR_MASK, g.num_cycles.get(), g.num_fails.get(), g.loop_sensor_health.get(), g.setup_sensor_health.get());
        hal.uartC->printf("INSTANT SENSOR_MASK: 0x%x\n", _instant_sensor_health_mask);
        
        //for (uint8_t i = 0; i < 3; i++) {
        //    if (!AP::ins().get_accel_health(i)) {
        //        hal.uartC->printf("[ERR]\t%s error count:\t%lu\n", sensor_pos[i], AP::ins().get_accel_error_count(i));
        //        hal.uartC->printf("[ERR]\t%s last error count:\t%lu\n", sensor_pos[i], _last_accel_error_count[i]);
        //    }
        //    if (!AP::ins().get_gyro_health(i)) {
        //        hal.uartC->printf("[ERR]\t%s error count:\t%lu\n", sensor_pos[i], AP::ins().get_gyro_error_count(i));
        //        hal.uartC->printf("[ERR]\t%s last error count:\t%lu\n", sensor_pos[i], _last_gyro_error_count[i]);
        //    }
        //}

        //reset zero count or health bool
        for (uint8_t i = 0; i < 3; i++) {
            //if error count stop increasing
            if (AP::ins().get_accel_error_count(i) > 0) {
                if (AP::ins().get_accel_error_count(i) <= _last_accel_error_count[i]) {
                    AP::ins().reset_accel_health(i);
                    _last_accel_error_count[i] = 0;
                }
            }
            if (AP::ins().get_gyro_error_count(i) > 0) {
                if (AP::ins().get_gyro_error_count(i) <= _last_gyro_error_count[i]) {
                    AP::ins().reset_gyro_health(i);
                    _last_gyro_error_count[i] = 0;
                }
            }
            _last_accel_error_count[i] = AP::ins().get_accel_error_count(i);
            _last_gyro_error_count[i] = AP::ins().get_gyro_error_count(i);
        }
    }

    if ((_led_blink_state < (g.num_fails.get()*2)) && 
        ((AP_HAL::millis() - _led_blink_ms) > (_led_blink_state*30))) {
        _led_blink_state++;
        hal.gpio->toggle(0);
    }


    if ((_heater_target_temp - AP::ins().get_temperature(0)) > 0.5f) {
        _heater_state = com::hex::equipment::jig::Status::HEATER_STATE_HEATING;
        _hold_start_ms = AP_HAL::millis();
    } else {
        _heater_state = com::hex::equipment::jig::Status::HEATER_STATE_HOLDING;
    }

    //hal.console->printf("Temp Delta: %d %d %f\n", _heater_state, AP_HAL::millis() - _hold_start_ms, (_heater_target_temp - AP::ins().get_temperature(0)));

    // if ((_heater_state == com::hex::equipment::jig::Status::HEATER_STATE_HOLDING) && ((AP_HAL::millis() - _hold_start_ms) >= 10000)) {
    //     if (_heater_target_temp == IMU_HIGH_TEMP) {
    //         _heater_target_temp = IMU_LOW_TEMP;
    //     } else if (_heater_target_temp == IMU_LOW_TEMP) {
    //         _heater_target_temp = IMU_HIGH_TEMP;
    //     }
    // }

    BoardConfig.set_target_temp(_heater_target_temp);
    if ((_setup_sensor_health_mask & _loop_sensor_health_mask) == SENSOR_MASK) {
        UAVCAN_handler::set_sensor_states(0x3FF, _heater_state);
    } else {
        UAVCAN_handler::set_sensor_states((_setup_sensor_health_mask & _loop_sensor_health_mask), _heater_state);
    }

    UAVCAN_handler::loop_all();

    /**
    // print console received bit
    if (hal.console->available() > 0) {
        buff[len_before_reboot] = hal.console->read();
        len_before_reboot++;
        hal.uartC->printf("Received bit: ");
        hal.console->printf("Received bit: ");
        hal.uartC->printf(buff);
        hal.console->printf(buff);
        hal.uartC->printf("\n");
        hal.console->printf("\n");
    }
    **/
    
    // auto-reboot for --upload
    if (hal.console->available() > 10) {
        hal.console->printf("rebooting\n");
        hal.uartC->printf("rebooting\n");
        while (hal.console->available()) {
            hal.console->read();
        }
        hal.scheduler->reboot(true);
    }
}

GCS_Dummy _gcs;

extern mavlink_system_t mavlink_system;

const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};

AP_HAL_MAIN();
