/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  DroneCAN ride-along implementation
  Handles receiving uavcan_equipment_ahrs_RawIMU messages from a DroneCAN source
  and passes that data to the simulation model
*/

#include "SIM_DroneCAN.h"

#if HAL_SIM_DRONECAN_ENABLED

#include <stdio.h>
#include <time.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <canard.h>
#include <dronecan_msgs.h>
#include <AP_HAL/utility/replace.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

static const struct {
    const char *name;
    float value;
    bool save;
} sim_defaults[] = {
    { "FRAME_CLASS", 1},
    { "BRD_OPTIONS", 0},
    { "INS_GYR_CAL", 0 },
    { "CAN_P1_DRIVER", 1},
    { "INS_ACC2OFFS_X",    0.001 },
    { "INS_ACC2OFFS_Y",    0.001 },
    { "INS_ACC2OFFS_Z",    0.001 },
    { "INS_ACC2SCAL_X",    1.001 },
    { "INS_ACC2SCAL_Y",    1.001 },
    { "INS_ACC2SCAL_Z",    1.001 },
    { "INS_ACCOFFS_X",     0.001 },
    { "INS_ACCOFFS_Y",     0.001 },
    { "INS_ACCOFFS_Z",     0.001 },
    { "INS_ACCSCAL_X",     1.001 },
    { "INS_ACCSCAL_Y",     1.001 },
    { "INS_ACCSCAL_Z",     1.001 },
    { "COMPASS_OFS_X",     0.001 },
    { "COMPASS_OFS_Y",     0.001 },
    { "COMPASS_OFS_Z",     0.001 },
    { "COMPASS_DEV_ID",    91395}, // DEV ID for DroneCAN IMU from NodeID 10
    { "COMPASS_PRIO1_ID",  91395}, // DEV ID for DroneCAN IMU from NodeID 10
    { "COMPASS_USE2",           0},
    { "COMPASS_USE3",           0},
    { "GPS1_TYPE",              9},
    { "SIM_BARO_DISABLE",       1},
    { "SIM_BAR2_DISABLE",       1},
    { "SIM_BAR3_DISABLE",       1},
    { "SIM_MAG1_DEVID",         0},
    { "SIM_MAG2_DEVID",         0},
    { "SIM_MAG3_DEVID",         0},
    { "SIM_MAG4_DEVID",         0},
    { "SIM_MAG5_DEVID",         0},
    { "SIM_MAG6_DEVID",         0}
};

// Static instance pointer for global access
SIM_DroneCAN* SIM_DroneCAN::_instance;

// Constructor
SIM_DroneCAN::SIM_DroneCAN(const char *frame_str) :
    Aircraft(frame_str)
{
    // Initialize state struct properly instead of using memset
    state = {};
    _instance = this;
    last_rate_print_ms = AP_HAL::millis();
    update_count_for_rate_calc = 0;

    // Set sim_defaults (copied from SIM_JSON)
    for (uint8_t i=0; i<ARRAY_SIZE(sim_defaults); i++) {
        AP_Param::set_default_by_name(sim_defaults[i].name, sim_defaults[i].value);
        if (sim_defaults[i].save) {
            enum ap_var_type ptype;
            AP_Param *p = AP_Param::find(sim_defaults[i].name, &ptype);
            if (!p->configured()) {
                p->save();
            }
        }
    }
    use_time_sync = false;
}


// Static trampoline function for RawIMU messages
void SIM_DroneCAN::handle_raw_imu_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_ahrs_RawIMU& msg)
{
    // Ensure we have an instance to work with
    if (_instance == nullptr) {
        return;
    }

    // Forward to the instance method
    _instance->handle_raw_imu(msg);
}

// Handler for RawIMU messages (instance method)
void SIM_DroneCAN::handle_raw_imu(const uavcan_equipment_ahrs_RawIMU& msg)
{
    // [DEBUG] Rate tracking for raw_imu messages using system clock
    static uint32_t raw_imu_count = 0;
    static uint64_t raw_imu_last_print_ns = 0;

    WITH_SEMAPHORE(raw_imu_sem);

    _initialized = true;
    // Record the receive time
    last_receive_time_ms = AP_HAL::millis();

    // [DEBUG] Calculate and print raw_imu receive rate
    raw_imu_count++;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t now_ns = ts.tv_sec * 1000000000ULL + ts.tv_nsec;
    if (raw_imu_last_print_ns == 0) {
        raw_imu_last_print_ns = now_ns;
    }
    if (now_ns - raw_imu_last_print_ns >= 1000000000ULL) {
        float raw_imu_rate_hz = (float)raw_imu_count / ((now_ns - raw_imu_last_print_ns) / 1000000000.0f);
        printf("[DEBUG] raw_imu receive rate: %.2f Hz\n", raw_imu_rate_hz);
        raw_imu_last_print_ns = now_ns;
        raw_imu_count = 0;
    }


    // Store IMU data from the message into the state struct
    state.timestamp_s = msg.timestamp.usec / 1000000.0;

    // Angular rates (body frame, rad/s)
    state.imu.gyro.x = msg.rate_gyro_latest[0];
    state.imu.gyro.y = msg.rate_gyro_latest[1];
    state.imu.gyro.z = msg.rate_gyro_latest[2];

    // Linear acceleration (body frame, m/s^2)
    state.imu.accel_body.x = msg.accelerometer_latest[0];
    state.imu.accel_body.y = msg.accelerometer_latest[1];
    state.imu.accel_body.z = msg.accelerometer_latest[2];

    raw_imu_event.signal();

    // Log IMU data
    // @LoggerMessage: IMU
    // @Description: Inertial Measurement Unit data
    // @Field: TimeUS: Time since system startup
    // @Field: I: IMU sensor instance number
    // @Field: GyrX: measured rotation rate about X axis (rad/s)
    // @Field: GyrY: measured rotation rate about Y axis (rad/s)
    // @Field: GyrZ: measured rotation rate about Z axis (rad/s)
    // @Field: AccX: acceleration along X axis (m/s^2)
    // @Field: AccY: acceleration along Y axis (m/s^2)
    // @Field: AccZ: acceleration along Z axis (m/s^2)
    // @Field: EG: gyroscope error count
    // @Field: EA: accelerometer error count
    // @Field: T: IMU temperature
    // @Field: GH: gyroscope health
    // @Field: AH: accelerometer health
    // @Field: GHz: gyroscope measurement rate
    // @Field: AHz: accelerometer measurement rate
    AP::logger().WriteStreaming("IMU", "TimeUS,I,GyrX,GyrY,GyrZ,AccX,AccY,AccZ,EG,EA,T,GH,AH,GHz,AHz",
                           "s#EEEooo--O--zz",
                           "F-000000-----00",
                           "QBffffffIIfBBHH",
                           AP_HAL::micros64(),
                           0, // Instance number, assuming 0 for now
                           state.imu.gyro.x,
                           state.imu.gyro.y,
                           state.imu.gyro.z,
                           state.imu.accel_body.x,
                           state.imu.accel_body.y,
                           state.imu.accel_body.z,
                           0, // Gyro error count, assuming 0
                           0, // Accel error count, assuming 0
                           0.0f, // Temperature, assuming 0 for now
                           1, // Gyro health, assuming healthy
                           1, // Accel health, assuming healthy
                           0, // Gyro rate, assuming 0 for now
                           0); // Accel rate, assuming 0 for now
}

/* Update the ride-along system */
// Only consume RawIMU data; actuator outputs are sent
// through AP_DroneCAN and Servo library.
void SIM_DroneCAN::update(const struct sitl_input &input)
{
    if (!_initialized || !AP::can().is_initialized()) {
        return;
    }

    while (!raw_imu_event.wait_blocking()) {}

    WITH_SEMAPHORE(raw_imu_sem);
    // Angular rates in body-frame (rad/s)
    gyro = state.imu.gyro;

    // Accelerations in body-frame (m/s^2)
    accel_body = state.imu.accel_body;

    // Check data timestamp for time handling
    double deltat;
    if (state.timestamp_s < last_timestamp_s) {
        // Physics time has gone backwards, don't reset AP
        printf("Detected physics reset\n");
        deltat = 0;
    } else {
        deltat = state.timestamp_s - last_timestamp_s;
    }
    time_now_us += deltat * 1.0e6;

    if (is_positive(deltat) && deltat < 0.1) {
        // match actual frame rate with desired speedup
        time_advance();
    }
    last_timestamp_s = state.timestamp_s;
    frame_counter++;

    // allow for changes in physics step
    adjust_frame_time(constrain_float(AP::sitl()->loop_rate_hz, rate_hz-1, rate_hz+1));

    update_count_for_rate_calc++;
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_rate_print_ms >= 1000) {
        float current_rate_hz = (float)update_count_for_rate_calc / ((now_ms - last_rate_print_ms) / 1000.0f);
        printf("SIM_DroneCAN update rate: %.2f Hz\n", current_rate_hz);
        last_rate_print_ms = now_ms;
        update_count_for_rate_calc = 0;
    }
}

// Static method to subscribe to messages - called from AP_DroneCAN init
bool SIM_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    // Register callback for uavcan_equipment_ahrs_RawIMU messages
    const auto driver_index = ap_dronecan->get_driver_index();
    return (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_raw_imu_trampoline, driver_index) != nullptr);
}

#endif // HAL_SIM_DRONECAN_ENABLED
