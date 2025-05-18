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
  Abstract base class for ride-along master functionality
*/

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef HAL_SIM_RIDEALONG_MASTER_ENABLED
#define HAL_SIM_RIDEALONG_MASTER_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

#ifndef HAL_SIM_RIDEALONG_JSON_MASTER_ENABLED
#define HAL_SIM_RIDEALONG_JSON_MASTER_ENABLED HAL_SIM_RIDEALONG_MASTER_ENABLED
#endif

#ifndef HAL_SIM_RIDEALONG_DRONECAN_MASTER_ENABLED
#define HAL_SIM_RIDEALONG_DRONECAN_MASTER_ENABLED HAL_SIM_RIDEALONG_MASTER_ENABLED && HAL_NUM_CAN_IFACES
#endif

#if HAL_SIM_RIDEALONG_MASTER_ENABLED

#include "SITL_Input.h"
#include <AP_Math/AP_Math.h>
#include <SITL/SITL.h>


namespace SITL {

class RideAlong_Master {
public:
    RideAlong_Master() {};
    virtual ~RideAlong_Master() {};

    // setup communications
    virtual void init(const int32_t num_slaves) = 0;

    // Receive control inputs from ride along controllers
    virtual void receive(struct sitl_input &input) = 0;

    // send vehicle state to ride along controllers
    virtual void send(const struct sitl_fdm &output, const Vector3d &position) = 0;

    // Enum for different slave protocol types
    enum class ProtocolType {
        JSON = 0,
        DRONECAN
    };

    // Create a RideAlong master instance based on protocol setting and frame string
    static class RideAlong_Master* create(ProtocolType protocol_type, uint32_t slave_count);

protected:

    struct input_frame {
        uint8_t instance;
        uint16_t rate;
        uint32_t count;
        uint16_t *pwm;
        uint16_t pwm_count;
    };
    void process_input_frame(struct sitl_input &input, const input_frame &frame);
};

}

#endif  // HAL_SIM_RIDEALONG_MASTER_ENABLED