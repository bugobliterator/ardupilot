#include "MW_MixerInputs.h"
#include "AC_Simulink_Normal.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_Param/AP_Param.h>

/* Copyright 2024-2025 The MathWorks, Inc. */
void MW_MixerInputs_init() {
    }

void MW_setTorque(float tauRoll, float tauPitch, float tauYaw) {
    // Stash torques on the AC_Simulink_Normal singleton; AC_CustomControl_Simulink
    // (or its caller) is responsible for forwarding them to AP_Motors and zeroing
    // the standard rate-PID integrators.
    AC_Simulink_Normal* sim = AC_Simulink_Normal::get_singleton();
    if (sim != nullptr) {
        sim->set_torque(tauRoll, tauPitch, tauYaw);
    }
}

void MW_setThrust(float throttle) {
    AC_Simulink_Normal* sim = AC_Simulink_Normal::get_singleton();
    if (sim != nullptr) {
        sim->set_thrust(throttle);
    }
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
