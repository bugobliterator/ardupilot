/* Copyright 2024-2025 The MathWorks, Inc. */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_AHRS/AP_AHRS.h>
#ifdef __cplusplus

extern "C" { /* sbcheck:ok:extern-c needed because of C++ compatibility for XCP External mode*/
    
#endif
void MW_EstimatorOutputs_init();
void MW_getCurrentAngVelocity(float* angVel);
void MW_getCurrentAttitudeEuler(float* attitudeEuler);
void MW_getCurrentAttitudeQuat(float* attitudeQuat);
void MW_getCurrentPositionNED(float* positionNED);
void MW_getCurrentVelocityNED(float* velocityNED);
#ifdef __cplusplus
    
}
#endif