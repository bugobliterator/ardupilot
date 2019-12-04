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
#include <AP_Param/AP_Param.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_IOMCU/AP_IOMCU.h>
const AP_HAL::HAL &hal = AP_HAL::get_HAL();
int ownserial = -1;

void setup();
void loop();

#define UAVCAN_NODE_POOL_SIZE 8192
#ifdef UAVCAN_NODE_POOL_BLOCK_SIZE
#undef UAVCAN_NODE_POOL_BLOCK_SIZE
#endif
#define UAVCAN_NODE_POOL_BLOCK_SIZE 256

#define MAX_SAFETY_TEMPERATURE 70

// board specific config
static AP_BoardConfig BoardConfig;
static AP_InertialSensor ins;
static AP_Baro baro;
static AP_Int32 unused;
static AP_Logger logger(unused);

const char shutdown_cmd[] = "STX,0,1,E,END";
const char shutdown_ret[] = "STX,0,1,E,END";
const char cool_cmd[] = "STX,0,1,S,cool.pgm,END"; 
const char cool_ret[] = "STX,1,0,S,cool.pgm,END";
const char heat_cmd[] = "STX,0,1,S,heat.pgm,END"; 
const char heat_ret[] = "STX,1,0,S,heat.pgm,END"; 
const char monitor_cmd[] = "STX,0,1,A,END";

void setup(void);
void loop(void);

void set_cube_power(bool turn_on)
{
    if (turn_on) {
        hal.rcout->write(0, 2000);
        hal.rcout->write(1, 2000);
    } else {
        hal.rcout->write(0, 1000);
        hal.rcout->write(1, 1000);
    }
}

void do_shutdown_cmd() {
    //always turn off
    set_cube_power(false);
    hal.uartB->printf(shutdown_cmd);
    hal.uartB->wait_timeout(13,1000);
    hal.console->printf("Shutting Down:\n");
    hal.scheduler->delay(1);
    while(hal.uartB->available()) {
        hal.console->printf("%c",(uint8_t)hal.uartB->read());
        hal.scheduler->delay(1);
    }
    hal.console->printf("\n");
}

void start_cool_cmd() {
    //always turn off
    do_shutdown_cmd();
    hal.uartB->printf(cool_cmd);
    hal.uartB->wait_timeout(13,1000);
    hal.console->printf("Starting Coolling:\n");
    hal.scheduler->delay(1);
    while(hal.uartB->available()) {
        hal.console->printf("%c",(uint8_t)hal.uartB->read());
        hal.scheduler->delay(1);
    }
    hal.console->printf("\n");
}

void start_heat_cmd() {
    //always turn off 
    do_shutdown_cmd();
    hal.uartB->printf(heat_cmd);
    hal.uartB->wait_timeout(13,1000);
    hal.console->printf("Starting Heating:\n");
    hal.scheduler->delay(1);
    while(hal.uartB->available()) {
        hal.console->printf("%c",(uint8_t)hal.uartB->read());
        hal.scheduler->delay(1);
    }
    hal.console->printf("\n");
}

bool is_chamber_healthy() {
    uint16_t total_read=0;
    char read_bytes[200], fields[3][20];
    hal.uartB->printf(monitor_cmd);
    hal.uartB->wait_timeout(23,1000);
    hal.console->printf("Checking Heater State:\n");
    while(hal.uartB->available() && total_read < 199) {
        read_bytes[total_read++] = hal.uartB->read();
    }
    read_bytes[total_read] = '\0';
    hal.console->printf(read_bytes);
    uint16_t offset = 0, field = 0;
    char *parse_bytes = strstr(&read_bytes[offset], "STX, 1, 0,A,");
    if (parse_bytes == nullptr) {
        return false;
    } else {
        parse_bytes += strlen("STX, 1, 0,A,");
    }
    while (parse_bytes[0] != '\0' && offset < 20 && field < 3) {
        if (parse_bytes[0] == ',') {
            fields[field][offset] = '\0';
            field++;
            offset = 0;
            parse_bytes++;
        }
        fields[field][offset++] = parse_bytes[0];
        parse_bytes++;
    }
    uint16_t status = atoi(fields[1]);
    int16_t temp = atoi(fields[2]);
    hal.console->printf("\nCHAMBER: STATUS: %s/%d TEMP: %s/%d\n", fields[1], status, fields[2], temp);
    if (status != 1) {
        return false;
    }
    if (temp >= MAX_SAFETY_TEMPERATURE) {
        return false;
    }
    return true;
}


uint16_t num_runs;
#define RUN_TIME_MS 3600000
#define NUM_CYCLES 96

uint32_t _run_start_ms, _last_status_print_ms;
bool toggle;
bool running = true;
bool error = false;
HAL_Semaphore_Recursive chamber_sem;

class ChamberCheck {
public:
    void control_chamber() {
        if ((AP_HAL::millis() - _last_check) > 1000) {
            _last_check = AP_HAL::millis();
            if (!is_chamber_healthy()) {
                error = true;
                set_cube_power(false);
            } else {
                error = false;
            }
        }        
        if ((AP_HAL::millis() - _run_start_ms) > RUN_TIME_MS) {
            num_runs++;
            _run_start_ms = AP_HAL::millis();
            if (running) {
                running = false;
                set_cube_power(false);
                start_cool_cmd();
            } else {
                running = true;
                set_cube_power(false);
                start_heat_cmd();
            }
        }
        if (num_runs > (NUM_CYCLES*2)) {
            running = false;
            do_shutdown_cmd();
        }
    }
private:
    uint32_t _last_check = 0;
} chamber_check;

void setup(void)
{
    unused = -1;
    BoardConfig.init();

    hal.console->printf("Starting UAVCAN\n");
    AP::iomcu()->force_safety_off();
    UAVCAN_handler::init();
    hal.rcout->set_output_mode(0xFFFF, AP_HAL::RCOutput::MODE_PWM_BRUSHED);
    hal.rcout->write(2, 2000);
    hal.rcout->write(3, 2000);
    start_heat_cmd();
    //hal.uartB->begin(19200);
    //hal.scheduler->register_io_process(FUNCTOR_BIND(&chamber_check, &ChamberCheck::control_chamber, void));
}

void loop()
{
    chamber_check.control_chamber();
    if ((AP_HAL::millis() - _last_status_print_ms) > 1000) {
        _last_status_print_ms = AP_HAL::millis();
        hal.console->printf("NUM_RUNS: %d CURRENT_RUN: %lu\n", num_runs, AP_HAL::millis() - _run_start_ms);
    }

    if (num_runs > (NUM_CYCLES*2)) {
        if (UAVCAN_handler::failed()) {
            hal.rcout->write(2, 1000);
            hal.rcout->write(3, 2000);
        } else {
            hal.rcout->write(2, 2000);
            hal.rcout->write(3, 1000);
        }
        set_cube_power(false);
    } else {
        if (error) {
            set_cube_power(false);
        } else if (running) {
            set_cube_power(true);
        } else {
            set_cube_power(false);
        }
        if (UAVCAN_handler::failed()) {
            hal.rcout->write(2, 1000); //GREEN
            hal.rcout->write(3, 2000); //RED
        } else {
            if (toggle) {
                hal.rcout->write(2, 2000);
                hal.rcout->write(3, 1500);
                toggle  = false;
            } else {
                hal.rcout->write(2, 1000);
                hal.rcout->write(3, 2000);
                toggle  = true;
            }
        }
    }
    UAVCAN_handler::loop_all();
}

GCS_Dummy _gcs;

extern mavlink_system_t mavlink_system;

const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};

AP_HAL_MAIN();
