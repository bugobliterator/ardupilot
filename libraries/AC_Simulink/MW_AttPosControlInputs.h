/* Copyright 2024-2025 The MathWorks, Inc. */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AC_AttitudeControl/AC_AttitudeControl.h>
#include <AC_AttitudeControl/AC_PosControl.h>
#ifdef __cplusplus

extern "C" { /* sbcheck:ok:extern-c needed because of C++ compatibility for XCP External mode*/
    
#endif
void MW_AttPosControlInputs_init();
void MW_getAngVelocitySetpoint(float* angVelSetpt);
void MW_getAttSetpointEuler(float* attSetptEul);
void MW_getAttSetpointQuat(float* attSetptQuat);
void MW_getPosNEDSetpoint(double* posNEDSetpt);
void MW_getVelNEDSetpoint(float* velNEDSetpt);
#ifdef __cplusplus
    
}
#endif