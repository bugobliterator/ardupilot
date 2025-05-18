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

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include "SIM_RideAlong_Master.h"

#if HAL_SIM_RIDEALONG_DRONECAN_MASTER_ENABLED

#include "SITL_Input.h"
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_Math/AP_Math.h>
#include <canard.h>
#include <dronecan_msgs.h>

namespace SITL {

class DroneCAN_Master : public RideAlong_Master {
public:
    DroneCAN_Master() {};

    // setup DroneCAN communications
    void init(const int32_t num_slaves) override;

    // Receive servo commands from ride along controllers
    void receive(struct sitl_input &input) override;

    // send vehicle state to ride along controllers
    void send(const struct sitl_fdm &output, const Vector3d &position) override;

    // Static method to subscribe to messages - called from AP_DroneCAN init
    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);

private:
    struct slave_info {
        uint8_t node_id;
        uint16_t frame_rate;
        uint32_t frame_count;
        uint16_t servos[16];
        bool updated;
        slave_info *next;
    };

    slave_info *slave_list;
    uint8_t master_node_id;
    bool initialized;
    HAL_Semaphore rcv_sem;

    uint64_t last_gps_update_us;
    uint64_t last_air_data_update_us;
    uint64_t last_mag_update_us;

    // Static trampoline functions for callbacks
    static void handle_actuator_command_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_actuator_ArrayCommand& msg);
    static void handle_raw_command_trampoline(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_esc_RawCommand& msg);

    // Callback handlers for DroneCAN messages
    void handle_actuator_command(const CanardRxTransfer& transfer, const uavcan_equipment_actuator_ArrayCommand& msg);
    void handle_raw_command(const CanardRxTransfer& transfer, const uavcan_equipment_esc_RawCommand& msg);

    // DroneCAN driver index used for sending and receiving messages
    uint8_t dronecan_idx;

    // Static pointer to this instance for AP_DroneCAN callbacks
    static DroneCAN_Master* _instance;

    input_frame _last_frame;
};

}

#endif  // HAL_SIM_RIDEALONG_DRONECAN_MASTER_ENABLED
