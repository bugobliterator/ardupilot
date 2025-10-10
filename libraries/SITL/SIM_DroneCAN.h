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
*/

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_SIM_DRONECAN_ENABLED
#define HAL_SIM_DRONECAN_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL) && HAL_NUM_CAN_IFACES
#endif

#if HAL_SIM_DRONECAN_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_HAL/Semaphores.h>
#include <canard.h>
#include <dronecan_msgs.h>
#include "SIM_Aircraft.h"
#include <atomic> // Add include for std::atomic

namespace SITL {

class SIM_DroneCAN : public Aircraft {
public:
    SIM_DroneCAN(const char *frame_str);

    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW SIM_DroneCAN(frame_str);
    }

    /* update the ride-along system */
    void update(const struct sitl_input &input) override;

    // Static method to subscribe to messages - called from AP_DroneCAN init
    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);

private:
    // Internal state structure to store received data
    struct {
        uint64_t timestamp_us;
        struct {
            Vector3f gyro;       // rad/s, from RawIMU.rate_gyro_latest
            Vector3f accel_body; // m/s^2, from RawIMU.accelerometer_latest
        } imu;
    } state;

    // Static trampoline for RawIMU messages
    static void handle_raw_imu_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_ahrs_RawIMU& msg);

    // Handle RawIMU messages (instance method)
    void handle_raw_imu(const uavcan_equipment_ahrs_RawIMU& msg);

    // Timestamp of last received data
    uint32_t last_receive_time_ms;

    // Timestamp for frame rate calculation
    uint64_t last_timestamp_us;

    // Frame counter for monitoring
    uint64_t frame_counter;

    // Static pointer to this instance for AP_DroneCAN callbacks
    static SIM_DroneCAN* _instance;

    // Binary semaphore to signal when RawIMU data is received
    HAL_BinarySemaphore raw_imu_event{0};
    HAL_Semaphore raw_imu_sem;

    // Initialization flag
    bool _initialized;

    uint32_t last_rate_print_ms;
    uint32_t update_count_for_rate_calc;
};
} // namespace SITL

#endif // HAL_SIM_DRONECAN_ENABLED