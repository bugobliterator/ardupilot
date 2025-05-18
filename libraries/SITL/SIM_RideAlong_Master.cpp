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

#include "SIM_RideAlong_Master.h"
#include "SIM_RideAlong_DroneCAN_Master.h"
#include "SIM_RideAlong_JSON_Master.h"

#if HAL_SIM_RIDEALONG_MASTER_ENABLED
#include <AP_Logger/AP_Logger.h>

using namespace SITL;

// Create a RideAlong master instance based on protocol setting and frame string
RideAlong_Master* RideAlong_Master::create(RideAlong_Master::ProtocolType protocol_type, uint32_t slave_count)
{
    // Create appropriate ride-along master implementation based on protocol
    if (protocol_type == RideAlong_Master::ProtocolType::DRONECAN) {
        // DroneCAN protocol
        printf("Creating DroneCAN master ride-along with %u slaves\n", slave_count);
        return NEW_NOTHROW DroneCAN_Master();
    } else {
        // JSON protocol (default)
        printf("Creating JSON master ride-along with %u slaves\n", slave_count);
        return NEW_NOTHROW JSON_Master();
    }
}

void RideAlong_Master::process_input_frame(struct sitl_input &input, const input_frame &frame)
{
    const bool use_servos = frame.instance == AP::sitl()->ride_along_master.get();
    if (use_servos) {
        const uint16_t real_count = MIN(ARRAY_SIZE(input.servos), frame.count);
        memcpy(input.servos, frame.pwm, real_count * sizeof(uint16_t));
    }

#if HAL_LOGGING_ENABLED
// @LoggerMessage: SLV1
// @Description: Log data received from JSON simulator 1
// @Field: TimeUS: Time since system startup (us)
// @Field: Instance: Slave instance
// @Field: pwm_count: Number of channels received
// @Field: frame_rate: Slave instance's desired frame rate
// @Field: frame_count: Slave instance's current frame count
// @Field: active: 1 if the servo outputs are being used from this instance
        AP::logger().WriteStreaming("SLV1", "TimeUS,Instance,pwm_count,frame_rate,frame_count,active",
                       "s#----",
                       "F?????",
                       "QBHHIB",
                       AP_HAL::micros64(),
                       frame.instance,
                       frame.pwm_count,
                       frame.rate,
                       frame.count,
                       use_servos);

// @LoggerMessage: SLV2
// @Description: Log data received from JSON simulator 2
// @Field: TimeUS: Time since system startup
// @Field: Instance: Slave instance
// @Field: C1: channel 1 output
// @Field: C2: channel 2 output
// @Field: C3: channel 3 output
// @Field: C4: channel 4 output
// @Field: C5: channel 5 output
// @Field: C6: channel 6 output
// @Field: C7: channel 7 output
// @Field: C8: channel 8 output
// @Field: C9: channel 9 output
// @Field: C10: channel 10 output
// @Field: C11: channel 11 output
// @Field: C12: channel 12 output
// @Field: C13: channel 13 output
// @Field: C14: channel 14 output
        AP::logger().WriteStreaming("SLV2", "TimeUS,Instance,C1,C2,C3,C4,C5,C6,C7,C8,C9,C10,C11,C12,C13,C14",
                       "s#YYYYYYYYYYYYYY",
                       "F?--------------",
                       "QBHHHHHHHHHHHHHH",
                       AP_HAL::micros64(),
                       frame.instance,
                       frame.pwm[0],
                       frame.pwm[1],
                       frame.pwm[2],
                       frame.pwm[3],
                       frame.pwm[4],
                       frame.pwm[5],
                       frame.pwm[6],
                       frame.pwm[7],
                       frame.pwm[8],
                       frame.pwm[9],
                       frame.pwm[10],
                       frame.pwm[11],
                       frame.pwm[12],
                       frame.pwm[13]);
#endif
}
#endif