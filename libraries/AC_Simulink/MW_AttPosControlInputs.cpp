#include "MW_AttPosControlInputs.h"
/* Copyright 2024-2025 The MathWorks, Inc. */

void MW_AttPosControlInputs_init(){
	}

void MW_getAngVelocitySetpoint(float* angVelSetpt){
	Vector3f angVel = AC_AttitudeControl::get_singleton()->get_attitude_target_ang_vel();
	memcpy(&angVelSetpt[0],&angVel[0],sizeof(Vector3f));
}
	
void MW_getAttSetpointEuler(float* attSetptEuler){
	Quaternion attQuat;
	attQuat.initialise();
	attQuat = AC_AttitudeControl::get_singleton()->get_attitude_target_quat();
	Vector3f attEulerAP;
	attQuat.to_euler(attEulerAP);
	// Output Euler in ZYX order.  Ardupilot returns in XYZ order.
	attSetptEuler[0] = attEulerAP[2];
	attSetptEuler[1] = attEulerAP[1];
	attSetptEuler[2] = attEulerAP[0];
}
void MW_getAttSetpointQuat(float* attSetptQuat){
	Quaternion attQuatAP;
	attQuatAP.initialise();
	attQuatAP = AC_AttitudeControl::get_singleton()->get_attitude_target_quat();
	attSetptQuat[0] = attQuatAP.q1;
	attSetptQuat[1] = attQuatAP.q2;
	attSetptQuat[2] = attQuatAP.q3;
	attSetptQuat[3] = attQuatAP.q4;
}
void MW_getPosNEDSetpoint(double* posNEDSetpt){
	Vector3p posNEDAP;
	posNEDAP = AC_PosControl::get_singleton()->get_pos_target_cm();
	//NEU->NED
	posNEDAP[2]=-posNEDAP[2];
	memcpy(&posNEDSetpt[0],&posNEDAP[0],sizeof(Vector3p));
}
void MW_getVelNEDSetpoint(float* velNEDSetpt){
	Vector3f velNEDAP;
	velNEDAP = AC_PosControl::get_singleton()->get_vel_desired_cms();
	//NEU->NED
	velNEDAP[2]= -velNEDAP[2];
	memcpy(&velNEDSetpt[0],&velNEDAP[0],sizeof(Vector3f));
}