/* Copyright 2024-2025 The MathWorks, Inc. */
#pragma once

#include <AP_Common/AP_Common.h>
#include "AP_Motors/AP_MotorsMatrix.h"
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AP_Motors/AP_Motors_Class.h>

#ifdef __cplusplus

extern "C" { /* sbcheck:ok:extern-c needed because of C++ compatibility for XCP External mode*/

#endif
// Initialize motor handle
void MW_MixerInputs_init();
// Set the roll, pitch and yaw
void MW_setTorque(float tauRoll, float tauPitch, float tauYaw);
// Set throttle
void MW_setThrust(float throttle);
// Assign matrix values i.e. roll factor,pitch factor,yaw factor and throttle factor for each motor
void MW_customMixerSet(int8_t motorIndex,
                       const float* rollF,
                       const float* pitchF,
                       const float* yawF,
                       const float* throttleF);
// Initialize custom mixer matrix
void MW_customMixerInit(const char* frame, uint8_t numMotors);
#ifdef __cplusplus
}
#endif
