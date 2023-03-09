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
#include <AP_Baro/AP_Baro_Backend.h>
#include <AP_Compass/AP_Compass_Backend.h>
#include <AP_InertialSensor/AP_InertialSensor_Backend.h>

static Parameters g;

#define RELEASE_INFO "15/1/2021 1521"

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

enum stype{T_COM, T_BAR, T_INS, T_GYR, T_ACC};

void log_sensor_health(uint16_t sensor_health);
char* get_sensor_name(enum stype type, uint8_t devtype);
uint8_t lock_flag_bit(enum stype type, uint8_t devtype, uint8_t bus, uint8_t instance);
void setup(void);
void loop(void);

static uint16_t _setup_sensor_health_mask;
static uint16_t _loop_sensor_health_mask;
static uint16_t _last_sensor_health_mask;
static uint16_t _instant_sensor_health_mask;
static uint16_t _local_sensor_health_mask = SENSOR_MASK;
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

// char* get_sensor_name(enum stype type, uint8_t devtype) 
// {
//    switch (type) 
//    {
//         case T_COM : 
//             switch (devtype) 
//             {
//                 case AP_Compass_Backend::DEVTYPE_LSM303D:   return (char*)"LSM303D";
//                 case AP_Compass_Backend::DEVTYPE_AK8963:    return (char*)"AK8963";
//                 case AP_Compass_Backend::DEVTYPE_AK09916:   return (char*)"AK09916";
//                 case AP_Compass_Backend::DEVTYPE_ICM20948:  return (char*)"ICM20948";
//                 default:    return (char*)"none";
//             }
//         case T_BAR:
//             switch (devtype) 
//             {
//                 case AP_Baro_Backend::DEVTYPE_BARO_MS5611:  return (char*)"MS5611";
//                 default:    return (char*)"none";
//             }
//         case T_INS:
//             switch (devtype) 
//             {
//                 case AP_InertialSensor_Backend::DEVTYPE_ACC_LSM303D:    return (char*)"LSM303D";
//                 case AP_InertialSensor_Backend::DEVTYPE_ACC_MPU9250:    return (char*)"MPU9250";
//                 case AP_InertialSensor_Backend::DEVTYPE_GYR_L3GD20:     return (char*)"L3GD20";
//                 case AP_InertialSensor_Backend::DEVTYPE_GYR_MPU9250:    return (char*)"MPU9250";
//                 case AP_InertialSensor_Backend::DEVTYPE_INS_ICM20948:   return (char*)"ICM20948";
//                 case AP_InertialSensor_Backend::DEVTYPE_INS_ICM20649:   return (char*)"ICM20649";
//                 case AP_InertialSensor_Backend::DEVTYPE_INS_ICM20602:   return (char*)"ICM20602";
//                 case AP_InertialSensor_Backend::DEVTYPE_INS_ADIS1647X:  return (char*)"ADIS1647X";
//                 default:    return (char*)"none";
//             }
//         default:
//             return (char*)"none";
        
//    }
// }

uint8_t lock_flag_bit(enum stype type, uint8_t devtype, uint8_t bus, uint8_t instance)
{
    switch (type) 
    {
        case T_COM:
            switch (bus)
            {
                case 0:
                    switch (devtype)
                    {
                        case AP_Compass_Backend::DEVTYPE_AK09916:  return 0;
                    }
                    break;
                case 1:
                    switch (devtype)
                    {
                        case AP_Compass_Backend::DEVTYPE_AK8963:    return 1;
                    }
                    break;
                case 4:
                    switch (devtype)
                    {
                        case AP_Compass_Backend::DEVTYPE_LSM303D:   return 0;
                        case AP_Compass_Backend::DEVTYPE_AK09916:   return 0;
                    }
                    break;
            }
            break;
        case T_BAR:
            switch (bus)
            {
                case 1:
                    switch (devtype)
                    {
                        case AP_Baro_Backend::DEVTYPE_BARO_MS5611:  return 1;
                    }
                    break;
                case 4:
                    switch (devtype)
                    {
                        case AP_Baro_Backend::DEVTYPE_BARO_MS5611:  return 0;
                    }
                    break;
            }
            break;
        case T_INS:
            return 16;
        case T_GYR:
            switch (bus)
            {
                case 1:
                    switch (devtype)
                    {
                        case AP_InertialSensor_Backend::DEVTYPE_GYR_MPU9250:    return 2;
                        case AP_InertialSensor_Backend::DEVTYPE_INS_ICM20649:   return 2;   //ICM20649
                    }
                    break;
                case 4:
                    switch (devtype)
                    {
                        case AP_InertialSensor_Backend::DEVTYPE_GYR_MPU9250:    return 0;   //MPU9250
                        case AP_InertialSensor_Backend::DEVTYPE_GYR_L3GD20:     return 1;   //L3GD20
                        case AP_InertialSensor_Backend::DEVTYPE_INS_ICM20948:   return 1;   //ICM20948
                        case AP_InertialSensor_Backend::DEVTYPE_INS_ICM20602:   return 0;   //ICM20602
                        case AP_InertialSensor_Backend::DEVTYPE_INS_ICM42688:   return instance;   //ICM42688
                    }
                    break;
            }
            break;
        case T_ACC:
            switch (bus)
            {
                case 1:
                    switch (devtype)
                    {
                        case AP_InertialSensor_Backend::DEVTYPE_ACC_MPU9250:   return 2;   //MPU9250
                        case AP_InertialSensor_Backend::DEVTYPE_INS_ICM20649:  return 2;   //ICM20649
                    }
                    break;
                case 4:
                    switch (devtype)
                    {
                        case AP_InertialSensor_Backend::DEVTYPE_ACC_MPU9250:   return 0;   //MPU9250
                        case AP_InertialSensor_Backend::DEVTYPE_ACC_LSM303D:   return 1;   //LSM303D
                        case AP_InertialSensor_Backend::DEVTYPE_INS_ICM20948:  return 1;   //ICM20948
                        case AP_InertialSensor_Backend::DEVTYPE_INS_ICM20602:  return 0;   //ICM20602
                        case AP_InertialSensor_Backend::DEVTYPE_INS_ICM42688: hal.console->printf("ICM42688\n"); return instance;   //ICM42688

                    }
                    break;
            }
            break;
    }
    return 16;
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
    // hal.scheduler->delay(2000);
    hal.console->printf("Testing firmware updated on %s\n", RELEASE_INFO);
    // hal.console->printf("Starting UAVCAN\n");
    hal.uartC->printf("Testing firmware updated on %s\n", RELEASE_INFO);
    // hal.uartC->printf("Starting UAVCAN\n");
    hal.gpio->pinMode(0, HAL_GPIO_OUTPUT);
    UAVCAN_handler::init();
    g.load_parameters();
    g.num_cycles.set_and_save(g.num_cycles.get()+1);
    logger.Init(log_structure, ARRAY_SIZE(log_structure));

    //setup test
    hal.scheduler->delay(1000);
    AP::logger().Write_Message("Setup Test Started");
    AP::ins().update();
    AP::baro().update();
    AP::compass().read();

    //turn off the bit for sensors missing on boot
    for (uint8_t i = 0; i < 3; i++) {
            _local_sensor_health_mask &= ~(1 << com::hex::equipment::jig::Status::ACCEL_HEALTH_OFF << lock_flag_bit(T_ACC, AP_HAL::Device::devid_get_devtype(AP::ins().get_accel_id(i)), AP_HAL::Device::devid_get_bus(AP::ins().get_accel_id(i)), i));
            _local_sensor_health_mask &= ~(1 << com::hex::equipment::jig::Status::GYRO_HEALTH_OFF << lock_flag_bit(T_GYR, AP_HAL::Device::devid_get_devtype(AP::ins().get_gyro_id(i)), AP_HAL::Device::devid_get_bus(AP::ins().get_gyro_id(i)), i));
    }
    for (uint8_t i = 0; i < 2; i++) {
            _local_sensor_health_mask &= ~(1 << com::hex::equipment::jig::Status::BARO_HEALTH_OFF << lock_flag_bit(T_BAR, AP_HAL::Device::devid_get_devtype(AP::baro().get_bus_id(i)), AP_HAL::Device::devid_get_bus(AP::baro().get_bus_id(i)), i));
    }
    for (uint8_t i = 0; i < 2; i++) {
            _local_sensor_health_mask &= ~(1 << 8 << lock_flag_bit(T_COM, AP_HAL::Device::devid_get_devtype(AP::compass().get_dev_id(i)), AP_HAL::Device::devid_get_bus(AP::compass().get_dev_id(i)), i));
    }
    _local_sensor_health_mask ^= SENSOR_MASK;

    _setup_sensor_health_mask = _local_sensor_health_mask;
    _loop_sensor_health_mask = _local_sensor_health_mask;
    _last_sensor_health_mask = _local_sensor_health_mask;
    _instant_sensor_health_mask = _local_sensor_health_mask;

    for (uint8_t i = 0; i < 3; i++) {
        if (!AP::ins().get_accel_health(i)) {
            _setup_sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::ACCEL_HEALTH_OFF) << lock_flag_bit(T_ACC, AP_HAL::Device::devid_get_devtype(AP::ins().get_accel_id(i)), AP_HAL::Device::devid_get_bus(AP::ins().get_accel_id(i)), i));
        }
        if (!AP::ins().get_gyro_health(i)) {
            _setup_sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::GYRO_HEALTH_OFF) << lock_flag_bit(T_GYR, AP_HAL::Device::devid_get_devtype(AP::ins().get_gyro_id(i)), AP_HAL::Device::devid_get_bus(AP::ins().get_gyro_id(i)), i));
        }
    }
    for (uint8_t i = 0; i < 2; i++) {
        if (!AP::baro().healthy(i)) {
            _setup_sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::BARO_HEALTH_OFF) << lock_flag_bit(T_BAR, AP_HAL::Device::devid_get_devtype(AP::baro().get_bus_id(i)), AP_HAL::Device::devid_get_bus(AP::baro().get_bus_id(i)), i));
        }
    }
    for (uint8_t i = 0; i < 2; i++) {
        if (!AP::compass().healthy(i)) {
            _setup_sensor_health_mask &= ~((1 << 8) << lock_flag_bit(T_COM, AP_HAL::Device::devid_get_devtype(AP::compass().get_dev_id(i)), AP_HAL::Device::devid_get_bus(AP::compass().get_dev_id(i)), i));
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
static uint32_t _log_ms;

void loop()
{
    if (loop_cycle == 0)
        AP::logger().Write_Message("Loop Test Started");
    
    //loop test
    AP::ins().update();
    AP::baro().update();
    AP::compass().read();

    _instant_sensor_health_mask = _local_sensor_health_mask;

    for (uint8_t i = 0; i < 3; i++) {
        if (!AP::ins().get_accel_health(i)) {
            _loop_sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::ACCEL_HEALTH_OFF) << lock_flag_bit(T_ACC, AP_HAL::Device::devid_get_devtype(AP::ins().get_accel_id(i)), AP_HAL::Device::devid_get_bus(AP::ins().get_accel_id(i)), i));
            _instant_sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::ACCEL_HEALTH_OFF) << lock_flag_bit(T_ACC, AP_HAL::Device::devid_get_devtype(AP::ins().get_accel_id(i)), AP_HAL::Device::devid_get_bus(AP::ins().get_accel_id(i)), i));
            
        }
        if (!AP::ins().get_gyro_health(i)) {
            _loop_sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::GYRO_HEALTH_OFF) << lock_flag_bit(T_GYR, AP_HAL::Device::devid_get_devtype(AP::ins().get_gyro_id(i)), AP_HAL::Device::devid_get_bus(AP::ins().get_gyro_id(i)), i));
            _instant_sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::GYRO_HEALTH_OFF) << lock_flag_bit(T_GYR, AP_HAL::Device::devid_get_devtype(AP::ins().get_gyro_id(i)), AP_HAL::Device::devid_get_bus(AP::ins().get_gyro_id(i)), i));
        }
    }
    for (uint8_t i = 0; i < 2; i++) {
        if (!AP::baro().healthy(i)) {
            _loop_sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::BARO_HEALTH_OFF) << lock_flag_bit(T_BAR, AP_HAL::Device::devid_get_devtype(AP::baro().get_bus_id(i)), AP_HAL::Device::devid_get_bus(AP::baro().get_bus_id(i)), i));
            _instant_sensor_health_mask &= ~((1 << com::hex::equipment::jig::Status::BARO_HEALTH_OFF) << lock_flag_bit(T_BAR, AP_HAL::Device::devid_get_devtype(AP::baro().get_bus_id(i)), AP_HAL::Device::devid_get_bus(AP::baro().get_bus_id(i)), i));
        }
    }
    for (uint8_t i = 0; i < 2; i++) {
        if (!AP::compass().healthy(i)) {
            _loop_sensor_health_mask &= ~((1 << 8) << lock_flag_bit(T_COM, AP_HAL::Device::devid_get_devtype(AP::compass().get_dev_id(i)), AP_HAL::Device::devid_get_bus(AP::compass().get_dev_id(i)), i));
            _instant_sensor_health_mask &= ~((1 << 8) << lock_flag_bit(T_COM, AP_HAL::Device::devid_get_devtype(AP::compass().get_dev_id(i)), AP_HAL::Device::devid_get_bus(AP::compass().get_dev_id(i)), i));
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
        //log instant of error
        log_sensor_health(_instant_sensor_health_mask);
        logger.Write_IMU();
        logger.Write_Baro();
        logger.Write_Compass();
        logger.Write_Power();
    }

    _last_sensor_health_mask = _instant_sensor_health_mask;  

    //Write IMU Data to Log
    if ((AP_HAL::millis() - _log_ms) > 10000) {
        _log_ms = AP_HAL::millis();
        log_sensor_health(_instant_sensor_health_mask);
        logger.Write_IMU();
        logger.Write_Baro();
        logger.Write_Compass();
        logger.Write_Power();
    }

    // Do LED Patterns
    if ((AP_HAL::millis() - _led_blink_ms) > 2000) {
        _led_blink_state = 0;
        _led_blink_ms = AP_HAL::millis();

        if (_loop_sensor_health_mask ^ g.loop_sensor_health.get()) {
            hal.console->printf("Fail sensor changed in this run. Log: %d\n", AP::logger().find_last_log());
            hal.uartC->printf("Fail sensor changed in this run. Log: %d\n", AP::logger().find_last_log());
        }
        hal.console->printf("Testing firmware updated on %s\n", RELEASE_INFO);
        hal.uartC->printf("Testing firmware updated on %s\n", RELEASE_INFO);
        hal.console->printf("SENSOR_MASK: 0x%x NUM_RUNS: %d NUM_FAILS: %d LOOP_TEST_FLAGS: 0x%x SETUP_TEST_FLAGS: 0x%x\n", SENSOR_MASK, g.num_cycles.get(), g.num_fails.get(), g.loop_sensor_health.get(), g.setup_sensor_health.get());
        hal.uartC->printf("SENSOR_MASK: 0x%x NUM_RUNS: %d NUM_FAILS: %d LOOP_TEST_FLAGS: 0x%x SETUP_TEST_FLAGS: 0x%x\n", SENSOR_MASK, g.num_cycles.get(), g.num_fails.get(), g.loop_sensor_health.get(), g.setup_sensor_health.get());
        
        //hal.uartC->printf("_local_sensor_health_mask: 0x%x \n", _local_sensor_health_mask);
        hal.uartC->printf("INSTANT: 0x%x\n", _instant_sensor_health_mask);
        
        //hal.uartC->printf("com order: [1]%s on bus %u; [0]%s on bus %u \n", get_sensor_name(T_COM, AP_HAL::Device::devid_get_devtype(AP::compass().get_dev_id(1))), 
        //                                                                    AP_HAL::Device::devid_get_bus(AP::compass().get_dev_id(1)), 
        //                                                                    get_sensor_name(T_COM, AP_HAL::Device::devid_get_devtype(AP::compass().get_dev_id(0))), 
        //                                                                    AP_HAL::Device::devid_get_bus(AP::compass().get_dev_id(0)));
        //hal.uartC->printf("baro order: [1]%s on bus %u; [0]%s on bus %u \n", get_sensor_name(T_BAR, AP_HAL::Device::devid_get_devtype(AP::baro().get_bus_id(1))), 
        //                                                                    AP_HAL::Device::devid_get_bus(AP::baro().get_bus_id(1)),
        //                                                                    get_sensor_name(T_BAR, AP_HAL::Device::devid_get_devtype(AP::baro().get_bus_id(0))),
        //                                                                    AP_HAL::Device::devid_get_bus(AP::baro().get_bus_id(0)));
        //hal.uartC->printf("gyr order: [2]%s on bus %u; [1]%s  on bus %u; [0]%s on bus %u \n", get_sensor_name(T_INS, AP_HAL::Device::devid_get_devtype(AP::ins().get_accel_id(2))), 
        //                                                                                    AP_HAL::Device::devid_get_bus(AP::ins().get_accel_id(2)), 
        //                                                                                    get_sensor_name(T_INS, AP_HAL::Device::devid_get_devtype(AP::ins().get_accel_id(1))), 
        //                                                                                    AP_HAL::Device::devid_get_bus(AP::ins().get_accel_id(1)), 
        //                                                                                    get_sensor_name(T_INS, AP_HAL::Device::devid_get_devtype(AP::ins().get_accel_id(0))), 
        //                                                                                    AP_HAL::Device::devid_get_bus(AP::ins().get_accel_id(0)));
        //hal.uartC->printf("acc order: [2]%s on bus %u; [1]%s  on bus %u; [0]%s on bus %u \n", get_sensor_name(T_INS, AP_HAL::Device::devid_get_devtype(AP::ins().get_gyro_id(2))), 
        //                                                                                    AP_HAL::Device::devid_get_bus(AP::ins().get_gyro_id(2)),
        //                                                                                    get_sensor_name(T_INS, AP_HAL::Device::devid_get_devtype(AP::ins().get_gyro_id(1))), 
        //                                                                                    AP_HAL::Device::devid_get_bus(AP::ins().get_gyro_id(1)),
        //                                                                                    get_sensor_name(T_INS, AP_HAL::Device::devid_get_devtype(AP::ins().get_gyro_id(0))), 
        //                                                                                    AP_HAL::Device::devid_get_bus(AP::ins().get_gyro_id(0)));

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
    // if (hal.console->available() > 10) {
    //     hal.console->printf("rebooting\n");
    //     hal.uartC->printf("rebooting\n");
    //     while (hal.console->available()) {
    //         hal.console->read();
    //     }
    //     hal.scheduler->reboot(true);
    // }
}

GCS_Dummy _gcs;

extern mavlink_system_t mavlink_system;

const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};

AP_HAL_MAIN();
