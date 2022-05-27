/*
  SToRM32 mount using serial protocol backend class
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_Mount_Backend.h"

#if HAL_MOUNT_ENABLED


class AP_Mount_Gremsy : public AP_Mount_Backend
{

public:
    // Constructor
    AP_Mount_Gremsy(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

    ~AP_Mount_Gremsy() override;

    // init
    void init() override {}

    // update mount position
    void update() override {}

    // has_pan_control
    bool has_pan_control() const override { return true; }

    // set_mode
    void set_mode(enum MAV_MOUNT_MODE mode) override {}

    // send_mount_status
    void send_mount_status(mavlink_channel_t chan) override {}

    static void handle_message(mavlink_channel_t chan, const mavlink_message_t &msg);
    static AP_Mount_Backend* detect(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance);

private:
    uint8_t cmd_retries;
    mavlink_channel_t mavchan;
    // last cmd sent to gimbal
    uint32_t last_cmd_sent;

    uint8_t dev_sysid;
    uint8_t dev_compid;

    mavlink_gimbal_device_information_t device_info;

    void req_manager_info(mavlink_channel_t chan, uint8_t dev_sysid, uint8_t dev_compid);
    static void req_device_info(mavlink_channel_t chan, uint8_t dev_sysid, uint8_t dev_compid);

    static struct detected_mount {
        mavlink_channel_t chan;
        bool detected;
        AP_Mount_Gremsy *mount;
    } detected_mounts[AP_MOUNT_MAX_INSTANCES];
};
#endif // HAL_MOUNT_ENABLED
