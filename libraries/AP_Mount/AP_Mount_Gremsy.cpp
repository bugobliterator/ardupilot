#include "AP_Mount_Gremsy.h"
#if HAL_MOUNT_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>


extern const AP_HAL::HAL& hal;

AP_Mount_Gremsy::detected_mount AP_Mount_Gremsy::detected_mounts[AP_MOUNT_MAX_INSTANCES];


AP_Mount_Gremsy::AP_Mount_Gremsy(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance) :
    AP_Mount_Backend(frontend, state, instance)
{}

void AP_Mount_Gremsy::handle_message(mavlink_channel_t chan, const mavlink_message_t &msg)
{
    switch (msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:
            if (msg.compid == MAV_COMP_ID_GIMBAL) {
                if (detected_mounts[0].mount == nullptr) {
                    req_device_info(chan, msg.sysid, msg.compid);
                }
            }
#if AP_MOUNT_MAX_INSTANCES > 1
            else if (msg.compid == MAV_COMP_ID_GIMBAL2) {
                if (detected_mounts[1].mount == nullptr) {
                    req_device_info(chan, msg.sysid, msg.compid);
                }
            }
#endif
            // TODO: add support for more gimbals
            break;
        case MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION:
            mavlink_gimbal_device_information_t info;
            mavlink_msg_gimbal_device_information_decode(&msg, &info);
            if (msg.compid == MAV_COMP_ID_GIMBAL) {
                detected_mounts[0].detected =  true;
            }
#if AP_MOUNT_MAX_INSTANCES > 1
            else if (msg.compid == MAV_COMP_ID_GIMBAL2) {
                detected_mounts[1].detected =  true;
            }
#endif
            break;
    }
}

AP_Mount_Gremsy::~AP_Mount_Gremsy()
{
    for (uint8_t i = 0; i < AP_MOUNT_MAX_INSTANCES; i++) {
        if (detected_mounts[i].mount == this) {
            detected_mounts[i].mount = nullptr;
        }
    }
}

AP_Mount_Backend* AP_Mount_Gremsy::detect(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance)
{
    if (instance > AP_MOUNT_MAX_INSTANCES) {
        return nullptr;
    }
    if (detected_mounts[instance].mount != nullptr) {
        return detected_mounts[instance].mount;
    }
    detected_mounts[instance].mount = new AP_Mount_Gremsy(frontend, state, instance);
    if (detected_mounts[instance].mount == nullptr) {
        return nullptr;
    }
    state._type.set_and_save(AP_Mount::Mount_Type_Gremsy);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Gremsy mount detected");
    return detected_mounts[instance].mount;
}

void AP_Mount_Gremsy::req_device_info(mavlink_channel_t chan, uint8_t dev_sysid, uint8_t dev_compid)
{
    mavlink_msg_command_long_send(
        chan,
        0,
        0,
        MAV_CMD_REQUEST_MESSAGE,
        0, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION, 0, 0, 0, 0, 0, 0);
}

#endif // HAL_MOUNT_ENABLED