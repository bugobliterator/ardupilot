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
  Send and receive DroneCAN data to allow a second AP instance to ride along
*/

#include "SIM_RideAlong_DroneCAN_Master.h"

#if HAL_SIM_RIDEALONG_DRONECAN_MASTER_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <errno.h>
#include <stdio.h>
#include <SITL/SITL.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Baro/AP_Baro.h> // For barometric calculations
#include <dronecan_msgs.h>   // For DroneCAN message type definitions
#include <AP_DroneCAN/AP_DroneCAN.h> // For AP_DroneCAN class and send methods
#include <AP_Math/AP_Math.h> // For radians()
#include <AP_Compass/AP_Compass.h> // For AP::compass() and rotations

using namespace SITL;

extern const AP_HAL::HAL& hal;

// Static instance pointer for global access
DroneCAN_Master* DroneCAN_Master::_instance;

// Initialize the DroneCAN master
void DroneCAN_Master::init(const int32_t num_slaves)
{
    slave_list = nullptr;
    master_node_id = 1; // Default master node ID
    initialized = false;
    last_gps_update_us = 0;
    last_air_data_update_us = 0;
    last_mag_update_us = 0;

    // Set the instance pointer for use in callback trampolines
    _instance = this;

    printf("DroneCAN Master: Initializing with %d slaves\n", (int)num_slaves);

    // Set sim_defaults (copied from SIM_JSON)
    // for (uint8_t i=0; i<HAL_NUM_CAN_IFACES; i++) {
    //     char param_name[] = "CAN_Dx_UC_OPTION";
    //     param_name[5] = '1' + i;
    //     enum ap_var_type var_type;
    //     AP_Param *vp = AP_Param::find(param_name, &var_type);
    //     int16_t curr_param = ((AP_Int16 *)vp)->get();
    //     ((AP_Int16 *)vp)->set(uint16_t(curr_param) & uint16_t(AP_DroneCAN::Options::DNA_DISABLE));
    // }

    // Initialize slave list based on num_slaves
    slave_info *list = nullptr;
    for (int32_t i = 0; i < num_slaves; i++) {
        slave_info *slave = NEW_NOTHROW slave_info();
        if (slave == nullptr) {
            AP_HAL::panic("Failed to allocate slave info");
            return;
        }

        // Initialize the slave structure
        slave->node_id = 10 + i + 2; // Assign sequential node IDs starting from 10
        slave->frame_rate = 50;
        slave->frame_count = 0;
        memset(slave->servos, 0, sizeof(slave->servos));
        slave->updated = false;
        slave->next = nullptr;

        // Add to linked list
        if (list == nullptr) {
            slave_list = slave;
            list = slave;
        } else {
            list->next = slave;
            list = slave;
        }

        printf("DroneCAN Master: Initialized slave %u with node ID %u\n",
               (unsigned)i + 1, (unsigned)slave->node_id);
    }

    // AP_DroneCAN will be initialized elsewhere in the system
    // Our subscribe_msgs method will be called when AP_DroneCAN initializes

    initialized = true;
    printf("DroneCAN Master: Ready for subscriber registration\n");
}

// Receive servo commands from ride along controllers
void DroneCAN_Master::receive(struct sitl_input &input)
{
    if (!initialized || slave_list == nullptr) {
        return;
    }

    WITH_SEMAPHORE(rcv_sem);
    for (slave_info *slave = slave_list; slave; slave=slave->next) {
        // Create an input frame from this slave's data
        const input_frame frame {
            .instance = uint8_t(slave->node_id-10-1),
            .rate = slave->frame_rate,
            .count = slave->frame_count,
            .pwm = slave->servos,
            .pwm_count = 16
        };
        // Process the input frame (using the common base class method)
        process_input_frame(input, frame);
    }
}

// Send vehicle state to ride along controllers
void DroneCAN_Master::send(const struct sitl_fdm &output, const Vector3d &position)
{
    if (!initialized) {
        return;
    }

    AP_DroneCAN *ap_dronecan = AP_DroneCAN::get_dronecan(0);
    if (ap_dronecan == nullptr) {
        return;
    }

    uint64_t timestamp_us = AP_HAL::micros64();

    // Define update intervals
    static constexpr uint32_t GPS_UPDATE_INTERVAL_US = 100000; // 10Hz (1s / 10 = 0.1s = 100,000 us)
    static constexpr uint32_t AIR_DATA_UPDATE_INTERVAL_US = 40000; // 25Hz (1s / 25 = 0.04s = 40,000 us)
    static constexpr uint32_t MAG_UPDATE_INTERVAL_US = 20000; // 50Hz (1s / 50 = 0.02s = 20,000 us)

    // Conditionally send uavcan.equipment.gnss.Fix2 at 5Hz
    if (timestamp_us - last_gps_update_us >= GPS_UPDATE_INTERVAL_US) {
        // Create and populate uavcan.equipment.gnss.Fix2
        uavcan_equipment_gnss_Fix2 msg_fix2{};
        msg_fix2.timestamp.usec = timestamp_us;
        msg_fix2.gnss_timestamp.usec = timestamp_us; // Simplification for SITL
        msg_fix2.gnss_time_standard = UAVCAN_EQUIPMENT_GNSS_FIX2_GNSS_TIME_STANDARD_GPS;
        msg_fix2.num_leap_seconds = UAVCAN_EQUIPMENT_GNSS_FIX2_NUM_LEAP_SECONDS_UNKNOWN;
        msg_fix2.longitude_deg_1e8 = int64_t(output.longitude * 1e8); // Convert to degrees * 1e8
        msg_fix2.latitude_deg_1e8 = int64_t(output.latitude * 1e8); // Convert to degrees * 1e8
        msg_fix2.height_ellipsoid_mm = static_cast<int32_t>(output.altitude * 1000.0);
        msg_fix2.ned_velocity[0] = output.speedN;
        msg_fix2.ned_velocity[1] = output.speedE;
        msg_fix2.ned_velocity[2] = output.speedD;
        msg_fix2.sats_used = 12; // Default good value for SITL
        msg_fix2.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
        msg_fix2.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK; // Assuming high quality fix for SITL
        msg_fix2.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FIXED;
        msg_fix2.pdop = 1.0f;
        msg_fix2.covariance.len = 0;
        msg_fix2.ecef_position_velocity.len = 0;
        ap_dronecan->send_gnss_fix2(msg_fix2);
        last_gps_update_us = timestamp_us;
    }

    // Conditionally send air data (StaticPressure and StaticTemperature) at 100Hz
    if (timestamp_us - last_air_data_update_us >= AIR_DATA_UPDATE_INTERVAL_US) {
        // Create and populate uavcan.equipment.air_data.StaticPressure
        uavcan_equipment_air_data_StaticPressure msg_pressure{};
        msg_pressure.static_pressure = AP::baro().get_pressure();
        msg_pressure.static_pressure_variance = 0.1f; // Example variance
        ap_dronecan->send_static_pressure(msg_pressure);

        // Create and populate uavcan.equipment.air_data.StaticTemperature
        uavcan_equipment_air_data_StaticTemperature msg_temp{};
        msg_temp.static_temperature = C_TO_KELVIN(AP::baro().get_temperature());
        msg_temp.static_temperature_variance = 0.01f; // Example variance
        ap_dronecan->send_static_temperature(msg_temp);
        last_air_data_update_us = timestamp_us;
    }

    // Conditionally send uavcan.equipment.ahrs.MagneticFieldStrength2 at 50Hz
    if (timestamp_us - last_mag_update_us >= MAG_UPDATE_INTERVAL_US) {
        auto *sitl = AP::sitl();
        if (sitl != nullptr) {
            Vector3f mag_data = sitl->state.bodyMagField; // milli-Gauss

            // Add noise
            mag_data += rand_vec3f() * sitl->mag_noise;

            // Apply SITL-configured offsets, orientation, and scaling for the first compass
            mag_data -= sitl->mag_ofs[0];
            mag_data.rotate_inverse((enum Rotation)sitl->mag_orient[0].get());

            // Apply board orientation
            // This assumes AP_Compass is available and initialized enough to provide board orientation.
            // In SITL, this should generally be the case.
            mag_data.rotate(AP::compass().get_board_orientation());

            mag_data *= sitl->mag_scaling[0];

            uavcan_equipment_ahrs_MagneticFieldStrength2 msg_mag{};
            msg_mag.magnetic_field_ga[0] = mag_data.x / 1000.0f; // Convert mGauss to Gauss
            msg_mag.magnetic_field_ga[1] = mag_data.y / 1000.0f;
            msg_mag.magnetic_field_ga[2] = mag_data.z / 1000.0f;
            msg_mag.sensor_id = 1; // Assign a sensor ID (e.g., 1 for the first magnetometer)
            msg_mag.magnetic_field_covariance.len = 0;
            ap_dronecan->send_magnetic_field_strength2(msg_mag);
            last_mag_update_us = timestamp_us;
        }
    }

    // Create and populate uavcan.equipment.ahrs.RawIMU (frequency unchanged)
    uavcan_equipment_ahrs_RawIMU msg_imu{};
    msg_imu.timestamp.usec = timestamp_us;
    msg_imu.integration_interval = 0; // Not providing integrated samples
    // Assuming output rates are in degrees/second as per original GlobalNavigationSolution usage
    msg_imu.rate_gyro_latest[0] = static_cast<float>(radians(output.rollRate));   // rad/s
    msg_imu.rate_gyro_latest[1] = static_cast<float>(radians(output.pitchRate));  // rad/s
    msg_imu.rate_gyro_latest[2] = static_cast<float>(radians(output.yawRate));    // rad/s
    msg_imu.accelerometer_latest[0] = static_cast<float>(output.xAccel);  // m/s^2
    msg_imu.accelerometer_latest[1] = static_cast<float>(output.yAccel);  // m/s^2
    msg_imu.accelerometer_latest[2] = static_cast<float>(output.zAccel);  // m/s^2
    // Integrals are zero as integration_interval is zero
    msg_imu.rate_gyro_integral[0] = 0;
    msg_imu.rate_gyro_integral[1] = 0;
    msg_imu.rate_gyro_integral[2] = 0;
    msg_imu.accelerometer_integral[0] = 0;
    msg_imu.accelerometer_integral[1] = 0;
    msg_imu.accelerometer_integral[2] = 0;
    msg_imu.covariance.len = 0;
    (void)msg_imu;
    ap_dronecan->send_raw_imu(msg_imu);
}

// Static method to subscribe to messages - called from AP_DroneCAN init
bool DroneCAN_Master::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    // Register callbacks for the message types we're interested in
    const auto driver_index = ap_dronecan->get_driver_index();

    // Register callback for actuator command messages
    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_actuator_command_trampoline, driver_index) == nullptr) {
        return false;
    }

    // Register callback for raw ESC command messages
    if (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_raw_command_trampoline, driver_index) == nullptr) {
        return false;
    }

    printf("DroneCAN Master: Subscribed to messages\n");
    return true;
}

// Static trampoline for actuator command messages
void DroneCAN_Master::handle_actuator_command_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_actuator_ArrayCommand& msg)
{
    // Ensure we have an instance to work with
    if (_instance == nullptr) {
        return;
    }

    // Forward to the instance method
    _instance->handle_actuator_command(transfer, msg);
}

// Static trampoline for raw command messages
void DroneCAN_Master::handle_raw_command_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_esc_RawCommand& msg)
{
    // Ensure we have an instance to work with
    if (_instance == nullptr) {
        return;
    }

    // Forward to the instance method
    _instance->handle_raw_command(transfer, msg);
}

// Handle actuator array command message from slaves
void DroneCAN_Master::handle_actuator_command(const CanardRxTransfer& transfer, const uavcan_equipment_actuator_ArrayCommand& msg)
{
    WITH_SEMAPHORE(rcv_sem);
    uint8_t source_node_id = transfer.source_node_id;

    // Find the slave with this node ID
    slave_info *slave = slave_list;
    while (slave != nullptr) {
        if (slave->node_id == source_node_id) {
            slave->updated = true;
            slave->frame_count++;

            // Process actuator commands
            const uint8_t num_commands = msg.commands.len;

            // Apply the commands to our servos array
            for (uint8_t i=0; i < num_commands && i < 16; i++) {
                const auto& cmd = msg.commands.data[i];

                // Check the command type to interpret the value correctly
                switch (cmd.command_type) {
                    case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_POSITION:
                        // For position commands, map to appropriate servo range
                        if (cmd.actuator_id < 16) {
                            slave->servos[cmd.actuator_id] = cmd.command_value;
                        }
                        break;

                    case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS:
                        // For unitless commands [-1, 1], map to servo range [1000, 2000]
                        if (cmd.actuator_id < 16) {
                            slave->servos[cmd.actuator_id] = 1500.0f + (cmd.command_value * 500.0f);
                        }
                        break;

                    case UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_PWM:
                        // For PWM commands, use value directly (in microseconds)
                        if (cmd.actuator_id < 16) {
                            slave->servos[cmd.actuator_id] = cmd.command_value;
                        }
                        break;

                    default:
                        // Ignore other command types
                        break;
                }
            }

            return; // Found the slave, we're done
        }
        slave = slave->next;
    }
}

// Handle raw ESC commands from slaves
void DroneCAN_Master::handle_raw_command(const CanardRxTransfer& transfer, const uavcan_equipment_esc_RawCommand& msg)
{
    WITH_SEMAPHORE(rcv_sem);
    uint8_t source_node_id = transfer.source_node_id;

    // Find the slave with this node ID
    slave_info *slave = slave_list;
    while (slave != nullptr) {
        if (slave->node_id == source_node_id) {
            slave->updated = true;
            slave->frame_count++;
            // Process raw commands
            const uint8_t num_commands = msg.cmd.len;
            // Apply the commands to our servos array
            for (uint8_t i=0; i < num_commands && i < 16; i++) {
                if (i < 16) {
                    // Convert from raw command to servo PWM range
                    static const int16_t cmd_max = ((1<<13)-1);
                    slave->servos[i] = hal.rcout->scale_esc_to_pwm(float(msg.cmd.data[i]) / cmd_max);
                }
            }

            return; // Found the slave, we're done
        }
        slave = slave->next;
    }
}

#endif  // HAL_SIM_RIDEALONG_DRONECAN_MASTER_ENABLED
