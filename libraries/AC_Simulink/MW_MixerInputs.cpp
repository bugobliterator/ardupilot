#include "MW_MixerInputs.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Param/AP_Param.h>

/* Copyright 2024-2025 The MathWorks, Inc. */
void MW_MixerInputs_init() {
    }

void MW_setTorque(float tauRoll, float tauPitch, float tauYaw) {
    // Set the Roll, Pitch and Yaw torque values 
    // When directly writing torque values to the motors, we reset the integrator terms
    // of the main attitude controller to prevent residual PID accumulation from affecting control.
    // This ensures clean torque application without interference from previous control states.
    AP_Motors::get_singleton()->set_roll(tauRoll);
    AC_AttitudeControl::get_singleton()->get_rate_roll_pid().set_integrator(0.0);
    AP_Motors::get_singleton()->set_pitch(tauPitch);
    AC_AttitudeControl::get_singleton()->get_rate_pitch_pid().set_integrator(0.0);
    AP_Motors::get_singleton()->set_yaw(tauYaw);
    AC_AttitudeControl::get_singleton()->get_rate_yaw_pid().set_integrator(0.0);
}

void MW_setThrust(float throttle) {
    AP_Motors::get_singleton()->set_throttle(throttle);
}
void MW_customMixerSet(int8_t motorIndex,
                       const float* rollF,
                       const float* pitchF,
                       const float* yawF,
                       const float* throttleF) {
    // Set roll, pitch, yaw and throttle factors of a motor
    // Set the testing order of the motor same as the
    // sequence in which they were configured
    AP_MotorsMatrix::get_singleton()->add_motor_raw(motorIndex, *rollF, *pitchF, *yawF,
                                                    uint8_t(motorIndex + 1), *throttleF);
}
void MW_customMixerInit(const char* frame, uint8_t numMotors) {
    // Initialize the custom mixer only once if it was not done previously
    if (!AP_MotorsMatrix::get_singleton()->initialised_ok()) {
        AP_MotorsMatrix::get_singleton()->init(numMotors);
        AP_MotorsMatrix::get_singleton()->set_frame_string(frame);
    }
}
