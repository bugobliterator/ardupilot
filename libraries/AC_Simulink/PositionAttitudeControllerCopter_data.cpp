//
// File: PositionAttitudeControllerCopter_data.cpp
//
// Code generated for Simulink model 'PositionAttitudeControllerCopter'.
//
// Model version                  : 1.3
// Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
// C/C++ source code generated on : Fri Feb 13 17:23:26 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "PositionAttitudeControllerCopter.h"

// Block parameters (default storage)
P_PositionAttitudeControllerC_T PositionAttitudeControllerCop_P = {
  // Mask Parameter: PIDController_D
  //  Referenced by: '<S41>/Derivative Gain'

  0.0,

  // Mask Parameter: PIDController_I
  //  Referenced by: '<S45>/Integral Gain'

  0.0,

  // Mask Parameter: PIDController_InitialConditionF
  //  Referenced by: '<S43>/Filter'

  0.0,

  // Mask Parameter: PIDController_InitialConditio_g
  //  Referenced by: '<S48>/Integrator'

  0.0,

  // Mask Parameter: PIDController5_LowerSaturationL
  //  Referenced by: '<S319>/Saturation'

  -0.87266462599716477,

  // Mask Parameter: PIDController_LowerSaturationLi
  //  Referenced by:
  //    '<S55>/Saturation'
  //    '<S40>/DeadZone'

  -0.1,

  // Mask Parameter: PIDController_N
  //  Referenced by: '<S51>/Filter Coefficient'

  100.0,

  // Mask Parameter: PIDController5_P
  //  Referenced by: '<S317>/Proportional Gain'

  2.0,

  // Mask Parameter: PIDController_P
  //  Referenced by: '<S53>/Proportional Gain'

  0.2,

  // Mask Parameter: PIDController5_UpperSaturationL
  //  Referenced by: '<S319>/Saturation'

  0.87266462599716477,

  // Mask Parameter: PIDController_UpperSaturationLi
  //  Referenced by:
  //    '<S55>/Saturation'
  //    '<S40>/DeadZone'

  0.1,

  // Mask Parameter: PIDController2_D
  //  Referenced by: '<S149>/Derivative Gain'

  0.0025F,

  // Mask Parameter: PIDController1_D
  //  Referenced by: '<S95>/Derivative Gain'

  0.0025F,

  // Mask Parameter: PID_Altitude_D
  //  Referenced by: '<S523>/Derivative Gain'

  0.01F,

  // Mask Parameter: PID_vz_D
  //  Referenced by: '<S577>/Derivative Gain'

  0.05F,

  // Mask Parameter: PID_Altitude_I
  //  Referenced by: '<S527>/Integral Gain'

  0.01F,

  // Mask Parameter: PID_vz_I
  //  Referenced by: '<S581>/Integral Gain'

  0.1F,

  // Mask Parameter: PIDController1_I
  //  Referenced by: '<S99>/Integral Gain'

  0.0F,

  // Mask Parameter: PIDController2_I
  //  Referenced by: '<S153>/Integral Gain'

  0.0F,

  // Mask Parameter: PIDController2_InitialCondition
  //  Referenced by: '<S151>/Filter'

  0.0F,

  // Mask Parameter: PIDController1_InitialCondition
  //  Referenced by: '<S97>/Filter'

  0.0F,

  // Mask Parameter: PID_Altitude_InitialConditionFo
  //  Referenced by: '<S525>/Filter'

  0.0F,

  // Mask Parameter: PID_vz_InitialConditionForFilte
  //  Referenced by: '<S579>/Filter'

  0.0F,

  // Mask Parameter: PIDController2_InitialConditi_c
  //  Referenced by: '<S156>/Integrator'

  0.0F,

  // Mask Parameter: PIDController1_InitialConditi_l
  //  Referenced by: '<S102>/Integrator'

  0.0F,

  // Mask Parameter: PID_Altitude_InitialCondition_d
  //  Referenced by: '<S530>/Integrator'

  0.0F,

  // Mask Parameter: PID_vz_InitialConditionForInteg
  //  Referenced by: '<S584>/Integrator'

  0.0F,

  // Mask Parameter: PIDController1_LowerSaturationL
  //  Referenced by: '<S431>/Saturation'

  -4.0F,

  // Mask Parameter: PIDController2_LowerSaturationL
  //  Referenced by: '<S483>/Saturation'

  -4.0F,

  // Mask Parameter: PIDController_LowerSaturation_e
  //  Referenced by: '<S379>/Saturation'

  -0.34906584F,

  // Mask Parameter: PIDController2_LowerSaturatio_o
  //  Referenced by:
  //    '<S163>/Saturation'
  //    '<S148>/DeadZone'

  -0.2F,

  // Mask Parameter: PIDController1_LowerSaturatio_l
  //  Referenced by:
  //    '<S109>/Saturation'
  //    '<S94>/DeadZone'

  -0.2F,

  // Mask Parameter: PID_Altitude_LowerSaturationLim
  //  Referenced by:
  //    '<S537>/Saturation'
  //    '<S522>/DeadZone'

  -2.0F,

  // Mask Parameter: PID_vz_LowerSaturationLimit
  //  Referenced by:
  //    '<S591>/Saturation'
  //    '<S576>/DeadZone'

  0.0F,

  // Mask Parameter: PIDController2_N
  //  Referenced by: '<S159>/Filter Coefficient'

  50.0F,

  // Mask Parameter: PIDController1_N
  //  Referenced by: '<S105>/Filter Coefficient'

  50.0F,

  // Mask Parameter: PID_Altitude_N
  //  Referenced by: '<S533>/Filter Coefficient'

  10.0F,

  // Mask Parameter: PID_vz_N
  //  Referenced by: '<S587>/Filter Coefficient'

  10.0F,

  // Mask Parameter: PIDController1_P
  //  Referenced by: '<S429>/Proportional Gain'

  0.6F,

  // Mask Parameter: PIDController2_P
  //  Referenced by: '<S481>/Proportional Gain'

  1.0F,

  // Mask Parameter: PIDController_P_h
  //  Referenced by: '<S377>/Proportional Gain'

  0.5F,

  // Mask Parameter: PIDController4_P
  //  Referenced by: '<S265>/Proportional Gain'

  4.0F,

  // Mask Parameter: PIDController2_P_j
  //  Referenced by: '<S161>/Proportional Gain'

  0.035F,

  // Mask Parameter: PIDController3_P
  //  Referenced by: '<S213>/Proportional Gain'

  4.0F,

  // Mask Parameter: PIDController1_P_a
  //  Referenced by: '<S107>/Proportional Gain'

  0.035F,

  // Mask Parameter: PID_Altitude_P
  //  Referenced by: '<S535>/Proportional Gain'

  1.5F,

  // Mask Parameter: PID_vz_P
  //  Referenced by: '<S589>/Proportional Gain'

  0.5F,

  // Mask Parameter: PIDController1_UpperSaturationL
  //  Referenced by: '<S431>/Saturation'

  4.0F,

  // Mask Parameter: PIDController2_UpperSaturationL
  //  Referenced by: '<S483>/Saturation'

  4.0F,

  // Mask Parameter: PIDController_UpperSaturation_f
  //  Referenced by: '<S379>/Saturation'

  0.34906584F,

  // Mask Parameter: PIDController2_UpperSaturatio_k
  //  Referenced by:
  //    '<S163>/Saturation'
  //    '<S148>/DeadZone'

  0.2F,

  // Mask Parameter: PIDController1_UpperSaturatio_p
  //  Referenced by:
  //    '<S109>/Saturation'
  //    '<S94>/DeadZone'

  0.2F,

  // Mask Parameter: PID_Altitude_UpperSaturationLim
  //  Referenced by:
  //    '<S537>/Saturation'
  //    '<S522>/DeadZone'

  2.0F,

  // Mask Parameter: PID_vz_UpperSaturationLimit
  //  Referenced by:
  //    '<S591>/Saturation'
  //    '<S576>/DeadZone'

  1.0F,

  // Expression: 0
  //  Referenced by: '<S330>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S38>/Constant1'

  0.0,

  // Expression: 40*pi/180
  //  Referenced by: '<S3>/Rate Limiter1'

  0.69813170079773179,

  // Expression: -40*pi/180
  //  Referenced by: '<S3>/Rate Limiter1'

  -0.69813170079773179,

  // Expression: 40*pi/180
  //  Referenced by: '<S3>/Rate Limiter2'

  0.69813170079773179,

  // Expression: -40*pi/180
  //  Referenced by: '<S3>/Rate Limiter2'

  -0.69813170079773179,

  // Expression: 2*pi
  //  Referenced by: '<S5>/Constant'

  6.2831853071795862,

  // Computed Parameter: Integrator_gainval
  //  Referenced by: '<S48>/Integrator'

  0.0025,

  // Computed Parameter: Filter_gainval
  //  Referenced by: '<S43>/Filter'

  0.0025,

  // Expression: 0
  //  Referenced by: '<S38>/Clamping_zero'

  0.0,

  // Computed Parameter: Switch1_Threshold
  //  Referenced by: '<S5>/Switch1'

  0.0F,

  // Computed Parameter: Constant1_Value_e
  //  Referenced by: '<S92>/Constant1'

  0.0F,

  // Computed Parameter: Constant1_Value_b
  //  Referenced by: '<S146>/Constant1'

  0.0F,

  // Computed Parameter: Constant1_Value_p
  //  Referenced by: '<S520>/Constant1'

  0.0F,

  // Computed Parameter: Constant1_Value_h
  //  Referenced by: '<S574>/Constant1'

  0.0F,

  // Computed Parameter: Gain_Gain
  //  Referenced by: '<S331>/Gain'

  -1.0F,

  // Computed Parameter: RateLimiter1_IC
  //  Referenced by: '<S3>/Rate Limiter1'

  0.0F,

  // Computed Parameter: Saturation_UpperSat
  //  Referenced by: '<S3>/Saturation'

  0.34906584F,

  // Computed Parameter: Saturation_LowerSat
  //  Referenced by: '<S3>/Saturation'

  -0.34906584F,

  // Computed Parameter: Integrator_gainval_i
  //  Referenced by: '<S156>/Integrator'

  0.0025F,

  // Computed Parameter: Filter_gainval_p
  //  Referenced by: '<S151>/Filter'

  0.0025F,

  // Computed Parameter: Gain1_Gain
  //  Referenced by: '<S4>/Gain1'

  -1.0F,

  // Computed Parameter: RateLimiter2_IC
  //  Referenced by: '<S3>/Rate Limiter2'

  0.0F,

  // Computed Parameter: Saturation1_UpperSat
  //  Referenced by: '<S3>/Saturation1'

  0.34906584F,

  // Computed Parameter: Saturation1_LowerSat
  //  Referenced by: '<S3>/Saturation1'

  -0.34906584F,

  // Computed Parameter: Integrator_gainval_l
  //  Referenced by: '<S102>/Integrator'

  0.0025F,

  // Computed Parameter: Filter_gainval_d
  //  Referenced by: '<S97>/Filter'

  0.0025F,

  // Computed Parameter: Switch_Threshold
  //  Referenced by: '<S5>/Switch'

  3.14159274F,

  // Computed Parameter: Gain_Gain_o
  //  Referenced by: '<S1>/Gain'

  -1.0F,

  // Computed Parameter: Gain1_Gain_b
  //  Referenced by: '<S330>/Gain1'

  -1.0F,

  // Computed Parameter: Clamping_zero_Value_e
  //  Referenced by: '<S520>/Clamping_zero'

  0.0F,

  // Computed Parameter: Filter_gainval_dy
  //  Referenced by: '<S525>/Filter'

  0.0025F,

  // Computed Parameter: Integrator_gainval_g
  //  Referenced by: '<S530>/Integrator'

  0.0025F,

  // Computed Parameter: Clamping_zero_Value_d
  //  Referenced by: '<S574>/Clamping_zero'

  0.0F,

  // Computed Parameter: Filter_gainval_i
  //  Referenced by: '<S579>/Filter'

  0.0025F,

  // Computed Parameter: Integrator_gainval_e
  //  Referenced by: '<S584>/Integrator'

  0.0025F,

  // Computed Parameter: Clamping_zero_Value_f
  //  Referenced by: '<S92>/Clamping_zero'

  0.0F,

  // Computed Parameter: Clamping_zero_Value_dm
  //  Referenced by: '<S146>/Clamping_zero'

  0.0F,

  // Computed Parameter: Constant_Value_if
  //  Referenced by: '<S38>/Constant'

  1,

  // Computed Parameter: Constant2_Value
  //  Referenced by: '<S38>/Constant2'

  -1,

  // Computed Parameter: Constant3_Value
  //  Referenced by: '<S38>/Constant3'

  1,

  // Computed Parameter: Constant4_Value
  //  Referenced by: '<S38>/Constant4'

  -1,

  // Computed Parameter: Constant_Value_l
  //  Referenced by: '<S92>/Constant'

  1,

  // Computed Parameter: Constant2_Value_l
  //  Referenced by: '<S92>/Constant2'

  -1,

  // Computed Parameter: Constant3_Value_m
  //  Referenced by: '<S92>/Constant3'

  1,

  // Computed Parameter: Constant4_Value_k
  //  Referenced by: '<S92>/Constant4'

  -1,

  // Computed Parameter: Constant_Value_a
  //  Referenced by: '<S146>/Constant'

  1,

  // Computed Parameter: Constant2_Value_b
  //  Referenced by: '<S146>/Constant2'

  -1,

  // Computed Parameter: Constant3_Value_k
  //  Referenced by: '<S146>/Constant3'

  1,

  // Computed Parameter: Constant4_Value_o
  //  Referenced by: '<S146>/Constant4'

  -1,

  // Computed Parameter: Constant_Value_lr
  //  Referenced by: '<S520>/Constant'

  1,

  // Computed Parameter: Constant2_Value_i
  //  Referenced by: '<S520>/Constant2'

  -1,

  // Computed Parameter: Constant3_Value_e
  //  Referenced by: '<S520>/Constant3'

  1,

  // Computed Parameter: Constant4_Value_oh
  //  Referenced by: '<S520>/Constant4'

  -1,

  // Computed Parameter: Constant_Value_h
  //  Referenced by: '<S574>/Constant'

  1,

  // Computed Parameter: Constant2_Value_c
  //  Referenced by: '<S574>/Constant2'

  -1,

  // Computed Parameter: Constant3_Value_c
  //  Referenced by: '<S574>/Constant3'

  1,

  // Computed Parameter: Constant4_Value_a
  //  Referenced by: '<S574>/Constant4'

  -1
};

//
// File trailer for generated code.
//
// [EOF]
//
