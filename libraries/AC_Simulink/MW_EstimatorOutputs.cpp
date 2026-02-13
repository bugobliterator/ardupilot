#include "MW_EstimatorOutputs.h"
/* Copyright 2024-2025 The MathWorks, Inc. */

#define UNUSED(x) x = x

void MW_EstimatorOutputs_init(){
	}

void MW_getCurrentAngVelocity(float* angVel){
	Vector3f angVelAP = AP_AHRS::get_singleton()->get_gyro_latest();
	memcpy(&angVel[0],&angVelAP[0],sizeof(Vector3f));
}
	
void MW_getCurrentAttitudeEuler(float* attEuler){
	Quaternion attQuat;
	attQuat.initialise();
	AP_AHRS::get_singleton()->get_quat_body_to_ned(attQuat);
	Vector3f attEulerAP;
	attQuat.to_euler(attEulerAP);
	// Output Euler in ZYX order.  Ardupilot returns in XYZ order.
	attEuler[0] = attEulerAP[2];
	attEuler[1] = attEulerAP[1];
	attEuler[2] = attEulerAP[0];
	}
void MW_getCurrentAttitudeQuat(float* attQuat){
	Quaternion attQuatAP;
	attQuatAP.initialise();
	AP_AHRS::get_singleton()->get_quat_body_to_ned(attQuatAP);
	attQuat[0] = attQuatAP.q1;
	attQuat[1] = attQuatAP.q2;
	attQuat[2] = attQuatAP.q3;
	attQuat[3] = attQuatAP.q4;
}
void MW_getCurrentPositionNED(float* posNED){
	Vector3f posNEDAP;
	bool ret = AP_AHRS::get_singleton()->get_relative_position_NED_origin(posNEDAP);
	UNUSED(ret);
	memcpy(&posNED[0],&posNEDAP[0],sizeof(Vector3f));
}
void MW_getCurrentVelocityNED(float* velNED){
	Vector3f velNEDAP;
	bool ret = AP_AHRS::get_singleton()->get_velocity_NED(velNEDAP);
	UNUSED(ret);
	memcpy(&velNED[0],&velNEDAP[0],sizeof(Vector3f));
}