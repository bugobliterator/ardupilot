//
// File: PositionAttitudeControllerCopter.h
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
#ifndef PositionAttitudeControllerCopter_h_
#define PositionAttitudeControllerCopter_h_
#include "rtwtypes.h"
#include "MW_AttPosControlInputs.h"
#include "MW_EstimatorOutputs.h"
#include "MW_MixerInputs.h"
#include "PositionAttitudeControllerCopter_types.h"
#include <stddef.h>
#include "MW_target_hardware_resources.h"
#define PositionAttitudeControllerCopter_M (PositionAttitudeControllerCo_M)

// Block signals (default storage)
struct B_PositionAttitudeControllerC_T {
  real_T posNED[3];
  real32_T rtb_CurrentAttitude_m[3];
  real32_T rtb_CurrentPosition_c[3];
  real32_T rtb_CurrentVelocity_k[3];
  real32_T rtb_CurrentAngVelocity_c[3];
  real32_T rtb_AttitudeSetpoint_b[3];
  real_T IntegralGain;                 // '<S45>/Integral Gain'
  real_T FilterCoefficient;            // '<S51>/Filter Coefficient'
};

// Block states (default storage) for system '<Root>'
struct DW_PositionAttitudeController_T {
  ardupilot_internal_block_Read_T obj; // '<Root>/Attitude Setpoint'
  ardupilot_internal_block_Re_i_T obj_o;// '<Root>/Current Ang Velocity'
  ardupilot_internal_block_R_iy_T obj_d;// '<Root>/Current Attitude'
  ardupilot_internal_block__iyr_T obj_n;// '<Root>/Current Position'
  ardupilot_internal_block_iyry_T obj_f;// '<Root>/Current Velocity'
  ardupilot_internal_bloc_iyryn_T obj_d5;// '<Root>/Position Setpoint'
  ardupilot_internal_blo_iyryn3_T obj_nb;// '<Root>/Velocity Setpoint'
  ardupilot_internal_block_Writ_T obj_l;// '<S2>/TTSystemObject'
  real_T Integrator_DSTATE;            // '<S48>/Integrator'
  real_T Filter_DSTATE;                // '<S43>/Filter'
  real32_T Integrator_DSTATE_m;        // '<S156>/Integrator'
  real32_T Filter_DSTATE_d;            // '<S151>/Filter'
  real32_T Integrator_DSTATE_c;        // '<S102>/Integrator'
  real32_T Filter_DSTATE_dx;           // '<S97>/Filter'
  real32_T Filter_DSTATE_j;            // '<S525>/Filter'
  real32_T Integrator_DSTATE_e;        // '<S530>/Integrator'
  real32_T Filter_DSTATE_g;            // '<S579>/Filter'
  real32_T Integrator_DSTATE_l;        // '<S584>/Integrator'
  real32_T PrevY;                      // '<S3>/Rate Limiter1'
  real32_T PrevY_f;                    // '<S3>/Rate Limiter2'
};

// Parameters (default storage)
struct P_PositionAttitudeControllerC_T_ {
  real_T PIDController_D;              // Mask Parameter: PIDController_D
                                          //  Referenced by: '<S41>/Derivative Gain'

  real_T PIDController_I;              // Mask Parameter: PIDController_I
                                          //  Referenced by: '<S45>/Integral Gain'

  real_T PIDController_InitialConditionF;
                              // Mask Parameter: PIDController_InitialConditionF
                                 //  Referenced by: '<S43>/Filter'

  real_T PIDController_InitialConditio_g;
                              // Mask Parameter: PIDController_InitialConditio_g
                                 //  Referenced by: '<S48>/Integrator'

  real_T PIDController5_LowerSaturationL;
                              // Mask Parameter: PIDController5_LowerSaturationL
                                 //  Referenced by: '<S319>/Saturation'

  real_T PIDController_LowerSaturationLi;
                              // Mask Parameter: PIDController_LowerSaturationLi
                                 //  Referenced by:
                                 //    '<S55>/Saturation'
                                 //    '<S40>/DeadZone'

  real_T PIDController_N;              // Mask Parameter: PIDController_N
                                          //  Referenced by: '<S51>/Filter Coefficient'

  real_T PIDController5_P;             // Mask Parameter: PIDController5_P
                                          //  Referenced by: '<S317>/Proportional Gain'

  real_T PIDController_P;              // Mask Parameter: PIDController_P
                                          //  Referenced by: '<S53>/Proportional Gain'

  real_T PIDController5_UpperSaturationL;
                              // Mask Parameter: PIDController5_UpperSaturationL
                                 //  Referenced by: '<S319>/Saturation'

  real_T PIDController_UpperSaturationLi;
                              // Mask Parameter: PIDController_UpperSaturationLi
                                 //  Referenced by:
                                 //    '<S55>/Saturation'
                                 //    '<S40>/DeadZone'

  real32_T PIDController2_D;           // Mask Parameter: PIDController2_D
                                          //  Referenced by: '<S149>/Derivative Gain'

  real32_T PIDController1_D;           // Mask Parameter: PIDController1_D
                                          //  Referenced by: '<S95>/Derivative Gain'

  real32_T PID_Altitude_D;             // Mask Parameter: PID_Altitude_D
                                          //  Referenced by: '<S523>/Derivative Gain'

  real32_T PID_vz_D;                   // Mask Parameter: PID_vz_D
                                          //  Referenced by: '<S577>/Derivative Gain'

  real32_T PID_Altitude_I;             // Mask Parameter: PID_Altitude_I
                                          //  Referenced by: '<S527>/Integral Gain'

  real32_T PID_vz_I;                   // Mask Parameter: PID_vz_I
                                          //  Referenced by: '<S581>/Integral Gain'

  real32_T PIDController1_I;           // Mask Parameter: PIDController1_I
                                          //  Referenced by: '<S99>/Integral Gain'

  real32_T PIDController2_I;           // Mask Parameter: PIDController2_I
                                          //  Referenced by: '<S153>/Integral Gain'

  real32_T PIDController2_InitialCondition;
                              // Mask Parameter: PIDController2_InitialCondition
                                 //  Referenced by: '<S151>/Filter'

  real32_T PIDController1_InitialCondition;
                              // Mask Parameter: PIDController1_InitialCondition
                                 //  Referenced by: '<S97>/Filter'

  real32_T PID_Altitude_InitialConditionFo;
                              // Mask Parameter: PID_Altitude_InitialConditionFo
                                 //  Referenced by: '<S525>/Filter'

  real32_T PID_vz_InitialConditionForFilte;
                              // Mask Parameter: PID_vz_InitialConditionForFilte
                                 //  Referenced by: '<S579>/Filter'

  real32_T PIDController2_InitialConditi_c;
                              // Mask Parameter: PIDController2_InitialConditi_c
                                 //  Referenced by: '<S156>/Integrator'

  real32_T PIDController1_InitialConditi_l;
                              // Mask Parameter: PIDController1_InitialConditi_l
                                 //  Referenced by: '<S102>/Integrator'

  real32_T PID_Altitude_InitialCondition_d;
                              // Mask Parameter: PID_Altitude_InitialCondition_d
                                 //  Referenced by: '<S530>/Integrator'

  real32_T PID_vz_InitialConditionForInteg;
                              // Mask Parameter: PID_vz_InitialConditionForInteg
                                 //  Referenced by: '<S584>/Integrator'

  real32_T PIDController1_LowerSaturationL;
                              // Mask Parameter: PIDController1_LowerSaturationL
                                 //  Referenced by: '<S431>/Saturation'

  real32_T PIDController2_LowerSaturationL;
                              // Mask Parameter: PIDController2_LowerSaturationL
                                 //  Referenced by: '<S483>/Saturation'

  real32_T PIDController_LowerSaturation_e;
                              // Mask Parameter: PIDController_LowerSaturation_e
                                 //  Referenced by: '<S379>/Saturation'

  real32_T PIDController2_LowerSaturatio_o;
                              // Mask Parameter: PIDController2_LowerSaturatio_o
                                 //  Referenced by:
                                 //    '<S163>/Saturation'
                                 //    '<S148>/DeadZone'

  real32_T PIDController1_LowerSaturatio_l;
                              // Mask Parameter: PIDController1_LowerSaturatio_l
                                 //  Referenced by:
                                 //    '<S109>/Saturation'
                                 //    '<S94>/DeadZone'

  real32_T PID_Altitude_LowerSaturationLim;
                              // Mask Parameter: PID_Altitude_LowerSaturationLim
                                 //  Referenced by:
                                 //    '<S537>/Saturation'
                                 //    '<S522>/DeadZone'

  real32_T PID_vz_LowerSaturationLimit;
                                  // Mask Parameter: PID_vz_LowerSaturationLimit
                                     //  Referenced by:
                                     //    '<S591>/Saturation'
                                     //    '<S576>/DeadZone'

  real32_T PIDController2_N;           // Mask Parameter: PIDController2_N
                                          //  Referenced by: '<S159>/Filter Coefficient'

  real32_T PIDController1_N;           // Mask Parameter: PIDController1_N
                                          //  Referenced by: '<S105>/Filter Coefficient'

  real32_T PID_Altitude_N;             // Mask Parameter: PID_Altitude_N
                                          //  Referenced by: '<S533>/Filter Coefficient'

  real32_T PID_vz_N;                   // Mask Parameter: PID_vz_N
                                          //  Referenced by: '<S587>/Filter Coefficient'

  real32_T PIDController1_P;           // Mask Parameter: PIDController1_P
                                          //  Referenced by: '<S429>/Proportional Gain'

  real32_T PIDController2_P;           // Mask Parameter: PIDController2_P
                                          //  Referenced by: '<S481>/Proportional Gain'

  real32_T PIDController_P_h;          // Mask Parameter: PIDController_P_h
                                          //  Referenced by: '<S377>/Proportional Gain'

  real32_T PIDController4_P;           // Mask Parameter: PIDController4_P
                                          //  Referenced by: '<S265>/Proportional Gain'

  real32_T PIDController2_P_j;         // Mask Parameter: PIDController2_P_j
                                          //  Referenced by: '<S161>/Proportional Gain'

  real32_T PIDController3_P;           // Mask Parameter: PIDController3_P
                                          //  Referenced by: '<S213>/Proportional Gain'

  real32_T PIDController1_P_a;         // Mask Parameter: PIDController1_P_a
                                          //  Referenced by: '<S107>/Proportional Gain'

  real32_T PID_Altitude_P;             // Mask Parameter: PID_Altitude_P
                                          //  Referenced by: '<S535>/Proportional Gain'

  real32_T PID_vz_P;                   // Mask Parameter: PID_vz_P
                                          //  Referenced by: '<S589>/Proportional Gain'

  real32_T PIDController1_UpperSaturationL;
                              // Mask Parameter: PIDController1_UpperSaturationL
                                 //  Referenced by: '<S431>/Saturation'

  real32_T PIDController2_UpperSaturationL;
                              // Mask Parameter: PIDController2_UpperSaturationL
                                 //  Referenced by: '<S483>/Saturation'

  real32_T PIDController_UpperSaturation_f;
                              // Mask Parameter: PIDController_UpperSaturation_f
                                 //  Referenced by: '<S379>/Saturation'

  real32_T PIDController2_UpperSaturatio_k;
                              // Mask Parameter: PIDController2_UpperSaturatio_k
                                 //  Referenced by:
                                 //    '<S163>/Saturation'
                                 //    '<S148>/DeadZone'

  real32_T PIDController1_UpperSaturatio_p;
                              // Mask Parameter: PIDController1_UpperSaturatio_p
                                 //  Referenced by:
                                 //    '<S109>/Saturation'
                                 //    '<S94>/DeadZone'

  real32_T PID_Altitude_UpperSaturationLim;
                              // Mask Parameter: PID_Altitude_UpperSaturationLim
                                 //  Referenced by:
                                 //    '<S537>/Saturation'
                                 //    '<S522>/DeadZone'

  real32_T PID_vz_UpperSaturationLimit;
                                  // Mask Parameter: PID_vz_UpperSaturationLimit
                                     //  Referenced by:
                                     //    '<S591>/Saturation'
                                     //    '<S576>/DeadZone'

  real_T Constant_Value;               // Expression: 0
                                          //  Referenced by: '<S330>/Constant'

  real_T Constant1_Value;              // Expression: 0
                                          //  Referenced by: '<S38>/Constant1'

  real_T RateLimiter1_RisingLim;       // Expression: 40*pi/180
                                          //  Referenced by: '<S3>/Rate Limiter1'

  real_T RateLimiter1_FallingLim;      // Expression: -40*pi/180
                                          //  Referenced by: '<S3>/Rate Limiter1'

  real_T RateLimiter2_RisingLim;       // Expression: 40*pi/180
                                          //  Referenced by: '<S3>/Rate Limiter2'

  real_T RateLimiter2_FallingLim;      // Expression: -40*pi/180
                                          //  Referenced by: '<S3>/Rate Limiter2'

  real_T Constant_Value_i;             // Expression: 2*pi
                                          //  Referenced by: '<S5>/Constant'

  real_T Integrator_gainval;           // Computed Parameter: Integrator_gainval
                                          //  Referenced by: '<S48>/Integrator'

  real_T Filter_gainval;               // Computed Parameter: Filter_gainval
                                          //  Referenced by: '<S43>/Filter'

  real_T Clamping_zero_Value;          // Expression: 0
                                          //  Referenced by: '<S38>/Clamping_zero'

  real32_T Switch1_Threshold;          // Computed Parameter: Switch1_Threshold
                                          //  Referenced by: '<S5>/Switch1'

  real32_T Constant1_Value_e;          // Computed Parameter: Constant1_Value_e
                                          //  Referenced by: '<S92>/Constant1'

  real32_T Constant1_Value_b;          // Computed Parameter: Constant1_Value_b
                                          //  Referenced by: '<S146>/Constant1'

  real32_T Constant1_Value_p;          // Computed Parameter: Constant1_Value_p
                                          //  Referenced by: '<S520>/Constant1'

  real32_T Constant1_Value_h;          // Computed Parameter: Constant1_Value_h
                                          //  Referenced by: '<S574>/Constant1'

  real32_T Gain_Gain;                  // Computed Parameter: Gain_Gain
                                          //  Referenced by: '<S331>/Gain'

  real32_T RateLimiter1_IC;            // Computed Parameter: RateLimiter1_IC
                                          //  Referenced by: '<S3>/Rate Limiter1'

  real32_T Saturation_UpperSat;       // Computed Parameter: Saturation_UpperSat
                                         //  Referenced by: '<S3>/Saturation'

  real32_T Saturation_LowerSat;       // Computed Parameter: Saturation_LowerSat
                                         //  Referenced by: '<S3>/Saturation'

  real32_T Integrator_gainval_i;     // Computed Parameter: Integrator_gainval_i
                                        //  Referenced by: '<S156>/Integrator'

  real32_T Filter_gainval_p;           // Computed Parameter: Filter_gainval_p
                                          //  Referenced by: '<S151>/Filter'

  real32_T Gain1_Gain;                 // Computed Parameter: Gain1_Gain
                                          //  Referenced by: '<S4>/Gain1'

  real32_T RateLimiter2_IC;            // Computed Parameter: RateLimiter2_IC
                                          //  Referenced by: '<S3>/Rate Limiter2'

  real32_T Saturation1_UpperSat;     // Computed Parameter: Saturation1_UpperSat
                                        //  Referenced by: '<S3>/Saturation1'

  real32_T Saturation1_LowerSat;     // Computed Parameter: Saturation1_LowerSat
                                        //  Referenced by: '<S3>/Saturation1'

  real32_T Integrator_gainval_l;     // Computed Parameter: Integrator_gainval_l
                                        //  Referenced by: '<S102>/Integrator'

  real32_T Filter_gainval_d;           // Computed Parameter: Filter_gainval_d
                                          //  Referenced by: '<S97>/Filter'

  real32_T Switch_Threshold;           // Computed Parameter: Switch_Threshold
                                          //  Referenced by: '<S5>/Switch'

  real32_T Gain_Gain_o;                // Computed Parameter: Gain_Gain_o
                                          //  Referenced by: '<S1>/Gain'

  real32_T Gain1_Gain_b;               // Computed Parameter: Gain1_Gain_b
                                          //  Referenced by: '<S330>/Gain1'

  real32_T Clamping_zero_Value_e;   // Computed Parameter: Clamping_zero_Value_e
                                       //  Referenced by: '<S520>/Clamping_zero'

  real32_T Filter_gainval_dy;          // Computed Parameter: Filter_gainval_dy
                                          //  Referenced by: '<S525>/Filter'

  real32_T Integrator_gainval_g;     // Computed Parameter: Integrator_gainval_g
                                        //  Referenced by: '<S530>/Integrator'

  real32_T Clamping_zero_Value_d;   // Computed Parameter: Clamping_zero_Value_d
                                       //  Referenced by: '<S574>/Clamping_zero'

  real32_T Filter_gainval_i;           // Computed Parameter: Filter_gainval_i
                                          //  Referenced by: '<S579>/Filter'

  real32_T Integrator_gainval_e;     // Computed Parameter: Integrator_gainval_e
                                        //  Referenced by: '<S584>/Integrator'

  real32_T Clamping_zero_Value_f;   // Computed Parameter: Clamping_zero_Value_f
                                       //  Referenced by: '<S92>/Clamping_zero'

  real32_T Clamping_zero_Value_dm; // Computed Parameter: Clamping_zero_Value_dm
                                      //  Referenced by: '<S146>/Clamping_zero'

  int8_T Constant_Value_if;            // Computed Parameter: Constant_Value_if
                                          //  Referenced by: '<S38>/Constant'

  int8_T Constant2_Value;              // Computed Parameter: Constant2_Value
                                          //  Referenced by: '<S38>/Constant2'

  int8_T Constant3_Value;              // Computed Parameter: Constant3_Value
                                          //  Referenced by: '<S38>/Constant3'

  int8_T Constant4_Value;              // Computed Parameter: Constant4_Value
                                          //  Referenced by: '<S38>/Constant4'

  int8_T Constant_Value_l;             // Computed Parameter: Constant_Value_l
                                          //  Referenced by: '<S92>/Constant'

  int8_T Constant2_Value_l;            // Computed Parameter: Constant2_Value_l
                                          //  Referenced by: '<S92>/Constant2'

  int8_T Constant3_Value_m;            // Computed Parameter: Constant3_Value_m
                                          //  Referenced by: '<S92>/Constant3'

  int8_T Constant4_Value_k;            // Computed Parameter: Constant4_Value_k
                                          //  Referenced by: '<S92>/Constant4'

  int8_T Constant_Value_a;             // Computed Parameter: Constant_Value_a
                                          //  Referenced by: '<S146>/Constant'

  int8_T Constant2_Value_b;            // Computed Parameter: Constant2_Value_b
                                          //  Referenced by: '<S146>/Constant2'

  int8_T Constant3_Value_k;            // Computed Parameter: Constant3_Value_k
                                          //  Referenced by: '<S146>/Constant3'

  int8_T Constant4_Value_o;            // Computed Parameter: Constant4_Value_o
                                          //  Referenced by: '<S146>/Constant4'

  int8_T Constant_Value_lr;            // Computed Parameter: Constant_Value_lr
                                          //  Referenced by: '<S520>/Constant'

  int8_T Constant2_Value_i;            // Computed Parameter: Constant2_Value_i
                                          //  Referenced by: '<S520>/Constant2'

  int8_T Constant3_Value_e;            // Computed Parameter: Constant3_Value_e
                                          //  Referenced by: '<S520>/Constant3'

  int8_T Constant4_Value_oh;           // Computed Parameter: Constant4_Value_oh
                                          //  Referenced by: '<S520>/Constant4'

  int8_T Constant_Value_h;             // Computed Parameter: Constant_Value_h
                                          //  Referenced by: '<S574>/Constant'

  int8_T Constant2_Value_c;            // Computed Parameter: Constant2_Value_c
                                          //  Referenced by: '<S574>/Constant2'

  int8_T Constant3_Value_c;            // Computed Parameter: Constant3_Value_c
                                          //  Referenced by: '<S574>/Constant3'

  int8_T Constant4_Value_a;            // Computed Parameter: Constant4_Value_a
                                          //  Referenced by: '<S574>/Constant4'

};

// Real-time Model Data Structure
struct tag_RTM_PositionAttitudeContr_T {
  const char_T * volatile errorStatus;
  const char_T* getErrorStatus() const;
  void setErrorStatus(const char_T* const volatile aErrorStatus);
};

// Block parameters (default storage)
#ifdef __cplusplus

extern "C"
{

#endif

  extern P_PositionAttitudeControllerC_T PositionAttitudeControllerCop_P;

#ifdef __cplusplus

}

#endif

// Block signals (default storage)
#ifdef __cplusplus

extern "C"
{

#endif

  extern struct B_PositionAttitudeControllerC_T PositionAttitudeControllerCop_B;

#ifdef __cplusplus

}

#endif

// Block states (default storage)
extern struct DW_PositionAttitudeController_T PositionAttitudeControllerCo_DW;

#ifdef __cplusplus

extern "C"
{

#endif

  // Model entry point functions
  extern void PositionAttitudeControllerCopter_initialize(void);
  extern void PositionAttitudeControllerCopter_step(void);
  extern void PositionAttitudeControllerCopter_terminate(void);

#ifdef __cplusplus

}

#endif

// Real-time Model object
#ifdef __cplusplus

extern "C"
{

#endif

  extern RT_MODEL_PositionAttitudeCont_T *const PositionAttitudeControllerCo_M;

#ifdef __cplusplus

}

#endif

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S337>/Data Type Duplicate' : Unused code path elimination
//  Block '<S337>/Data Type Propagation' : Unused code path elimination
//  Block '<S1>/Data Type Conversion' : Eliminate redundant data type conversion
//  Block '<S1>/Data Type Conversion2' : Eliminate redundant data type conversion
//  Block '<S1>/Data Type Conversion3' : Eliminate redundant data type conversion


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'PositionAttitudeControllerCopter'
//  '<S1>'   : 'PositionAttitudeControllerCopter/Position and Attitude Controller'
//  '<S2>'   : 'PositionAttitudeControllerCopter/Write Torque & Thrust'
//  '<S3>'   : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller'
//  '<S4>'   : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller'
//  '<S5>'   : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/Calculate minimum Turn'
//  '<S6>'   : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller'
//  '<S7>'   : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1'
//  '<S8>'   : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2'
//  '<S9>'   : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3'
//  '<S10>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4'
//  '<S11>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5'
//  '<S12>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Anti-windup'
//  '<S13>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/D Gain'
//  '<S14>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/External Derivative'
//  '<S15>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Filter'
//  '<S16>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Filter ICs'
//  '<S17>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/I Gain'
//  '<S18>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Ideal P Gain'
//  '<S19>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Ideal P Gain Fdbk'
//  '<S20>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Integrator'
//  '<S21>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Integrator ICs'
//  '<S22>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/N Copy'
//  '<S23>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/N Gain'
//  '<S24>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/P Copy'
//  '<S25>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Parallel P Gain'
//  '<S26>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Reset Signal'
//  '<S27>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Saturation'
//  '<S28>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Saturation Fdbk'
//  '<S29>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Sum'
//  '<S30>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Sum Fdbk'
//  '<S31>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Tracking Mode'
//  '<S32>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Tracking Mode Sum'
//  '<S33>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Tsamp - Integral'
//  '<S34>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Tsamp - Ngain'
//  '<S35>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/postSat Signal'
//  '<S36>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/preInt Signal'
//  '<S37>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/preSat Signal'
//  '<S38>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Anti-windup/Disc. Clamping Parallel'
//  '<S39>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone'
//  '<S40>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
//  '<S41>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/D Gain/Internal Parameters'
//  '<S42>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/External Derivative/Error'
//  '<S43>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Filter/Disc. Forward Euler Filter'
//  '<S44>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Filter ICs/Internal IC - Filter'
//  '<S45>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/I Gain/Internal Parameters'
//  '<S46>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Ideal P Gain/Passthrough'
//  '<S47>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Ideal P Gain Fdbk/Disabled'
//  '<S48>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Integrator/Discrete'
//  '<S49>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Integrator ICs/Internal IC'
//  '<S50>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/N Copy/Disabled'
//  '<S51>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/N Gain/Internal Parameters'
//  '<S52>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/P Copy/Disabled'
//  '<S53>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Parallel P Gain/Internal Parameters'
//  '<S54>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Reset Signal/Disabled'
//  '<S55>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Saturation/Enabled'
//  '<S56>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Saturation Fdbk/Disabled'
//  '<S57>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Sum/Sum_PID'
//  '<S58>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Sum Fdbk/Disabled'
//  '<S59>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Tracking Mode/Disabled'
//  '<S60>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Tracking Mode Sum/Passthrough'
//  '<S61>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Tsamp - Integral/TsSignalSpecification'
//  '<S62>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/Tsamp - Ngain/Passthrough'
//  '<S63>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/postSat Signal/Forward_Path'
//  '<S64>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/preInt Signal/Internal PreInt'
//  '<S65>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller/preSat Signal/Forward_Path'
//  '<S66>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Anti-windup'
//  '<S67>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/D Gain'
//  '<S68>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/External Derivative'
//  '<S69>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Filter'
//  '<S70>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Filter ICs'
//  '<S71>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/I Gain'
//  '<S72>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Ideal P Gain'
//  '<S73>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Ideal P Gain Fdbk'
//  '<S74>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Integrator'
//  '<S75>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Integrator ICs'
//  '<S76>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/N Copy'
//  '<S77>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/N Gain'
//  '<S78>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/P Copy'
//  '<S79>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Parallel P Gain'
//  '<S80>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Reset Signal'
//  '<S81>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Saturation'
//  '<S82>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Saturation Fdbk'
//  '<S83>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Sum'
//  '<S84>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Sum Fdbk'
//  '<S85>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Tracking Mode'
//  '<S86>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Tracking Mode Sum'
//  '<S87>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Tsamp - Integral'
//  '<S88>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Tsamp - Ngain'
//  '<S89>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/postSat Signal'
//  '<S90>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/preInt Signal'
//  '<S91>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/preSat Signal'
//  '<S92>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Anti-windup/Disc. Clamping Parallel'
//  '<S93>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Anti-windup/Disc. Clamping Parallel/Dead Zone'
//  '<S94>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
//  '<S95>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/D Gain/Internal Parameters'
//  '<S96>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/External Derivative/Error'
//  '<S97>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Filter/Disc. Forward Euler Filter'
//  '<S98>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Filter ICs/Internal IC - Filter'
//  '<S99>'  : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/I Gain/Internal Parameters'
//  '<S100>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Ideal P Gain/Passthrough'
//  '<S101>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Ideal P Gain Fdbk/Disabled'
//  '<S102>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Integrator/Discrete'
//  '<S103>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Integrator ICs/Internal IC'
//  '<S104>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/N Copy/Disabled'
//  '<S105>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/N Gain/Internal Parameters'
//  '<S106>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/P Copy/Disabled'
//  '<S107>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Parallel P Gain/Internal Parameters'
//  '<S108>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Reset Signal/Disabled'
//  '<S109>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Saturation/Enabled'
//  '<S110>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Saturation Fdbk/Disabled'
//  '<S111>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Sum/Sum_PID'
//  '<S112>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Sum Fdbk/Disabled'
//  '<S113>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Tracking Mode/Disabled'
//  '<S114>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Tracking Mode Sum/Passthrough'
//  '<S115>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Tsamp - Integral/TsSignalSpecification'
//  '<S116>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/Tsamp - Ngain/Passthrough'
//  '<S117>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/postSat Signal/Forward_Path'
//  '<S118>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/preInt Signal/Internal PreInt'
//  '<S119>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller1/preSat Signal/Forward_Path'
//  '<S120>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Anti-windup'
//  '<S121>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/D Gain'
//  '<S122>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/External Derivative'
//  '<S123>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Filter'
//  '<S124>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Filter ICs'
//  '<S125>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/I Gain'
//  '<S126>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Ideal P Gain'
//  '<S127>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Ideal P Gain Fdbk'
//  '<S128>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Integrator'
//  '<S129>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Integrator ICs'
//  '<S130>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/N Copy'
//  '<S131>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/N Gain'
//  '<S132>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/P Copy'
//  '<S133>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Parallel P Gain'
//  '<S134>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Reset Signal'
//  '<S135>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Saturation'
//  '<S136>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Saturation Fdbk'
//  '<S137>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Sum'
//  '<S138>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Sum Fdbk'
//  '<S139>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Tracking Mode'
//  '<S140>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Tracking Mode Sum'
//  '<S141>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Tsamp - Integral'
//  '<S142>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Tsamp - Ngain'
//  '<S143>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/postSat Signal'
//  '<S144>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/preInt Signal'
//  '<S145>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/preSat Signal'
//  '<S146>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Anti-windup/Disc. Clamping Parallel'
//  '<S147>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Anti-windup/Disc. Clamping Parallel/Dead Zone'
//  '<S148>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
//  '<S149>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/D Gain/Internal Parameters'
//  '<S150>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/External Derivative/Error'
//  '<S151>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Filter/Disc. Forward Euler Filter'
//  '<S152>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Filter ICs/Internal IC - Filter'
//  '<S153>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/I Gain/Internal Parameters'
//  '<S154>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Ideal P Gain/Passthrough'
//  '<S155>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Ideal P Gain Fdbk/Disabled'
//  '<S156>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Integrator/Discrete'
//  '<S157>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Integrator ICs/Internal IC'
//  '<S158>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/N Copy/Disabled'
//  '<S159>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/N Gain/Internal Parameters'
//  '<S160>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/P Copy/Disabled'
//  '<S161>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Parallel P Gain/Internal Parameters'
//  '<S162>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Reset Signal/Disabled'
//  '<S163>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Saturation/Enabled'
//  '<S164>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Saturation Fdbk/Disabled'
//  '<S165>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Sum/Sum_PID'
//  '<S166>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Sum Fdbk/Disabled'
//  '<S167>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Tracking Mode/Disabled'
//  '<S168>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Tracking Mode Sum/Passthrough'
//  '<S169>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Tsamp - Integral/TsSignalSpecification'
//  '<S170>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/Tsamp - Ngain/Passthrough'
//  '<S171>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/postSat Signal/Forward_Path'
//  '<S172>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/preInt Signal/Internal PreInt'
//  '<S173>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller2/preSat Signal/Forward_Path'
//  '<S174>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Anti-windup'
//  '<S175>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/D Gain'
//  '<S176>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/External Derivative'
//  '<S177>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Filter'
//  '<S178>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Filter ICs'
//  '<S179>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/I Gain'
//  '<S180>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Ideal P Gain'
//  '<S181>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Ideal P Gain Fdbk'
//  '<S182>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Integrator'
//  '<S183>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Integrator ICs'
//  '<S184>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/N Copy'
//  '<S185>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/N Gain'
//  '<S186>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/P Copy'
//  '<S187>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Parallel P Gain'
//  '<S188>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Reset Signal'
//  '<S189>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Saturation'
//  '<S190>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Saturation Fdbk'
//  '<S191>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Sum'
//  '<S192>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Sum Fdbk'
//  '<S193>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Tracking Mode'
//  '<S194>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Tracking Mode Sum'
//  '<S195>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Tsamp - Integral'
//  '<S196>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Tsamp - Ngain'
//  '<S197>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/postSat Signal'
//  '<S198>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/preInt Signal'
//  '<S199>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/preSat Signal'
//  '<S200>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Anti-windup/Disabled'
//  '<S201>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/D Gain/Disabled'
//  '<S202>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/External Derivative/Disabled'
//  '<S203>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Filter/Disabled'
//  '<S204>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Filter ICs/Disabled'
//  '<S205>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/I Gain/Disabled'
//  '<S206>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Ideal P Gain/Passthrough'
//  '<S207>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Ideal P Gain Fdbk/Disabled'
//  '<S208>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Integrator/Disabled'
//  '<S209>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Integrator ICs/Disabled'
//  '<S210>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/N Copy/Disabled wSignal Specification'
//  '<S211>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/N Gain/Disabled'
//  '<S212>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/P Copy/Disabled'
//  '<S213>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Parallel P Gain/Internal Parameters'
//  '<S214>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Reset Signal/Disabled'
//  '<S215>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Saturation/Passthrough'
//  '<S216>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Saturation Fdbk/Disabled'
//  '<S217>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Sum/Passthrough_P'
//  '<S218>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Sum Fdbk/Disabled'
//  '<S219>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Tracking Mode/Disabled'
//  '<S220>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Tracking Mode Sum/Passthrough'
//  '<S221>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Tsamp - Integral/TsSignalSpecification'
//  '<S222>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/Tsamp - Ngain/Passthrough'
//  '<S223>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/postSat Signal/Forward_Path'
//  '<S224>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/preInt Signal/Internal PreInt'
//  '<S225>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller3/preSat Signal/Forward_Path'
//  '<S226>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Anti-windup'
//  '<S227>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/D Gain'
//  '<S228>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/External Derivative'
//  '<S229>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Filter'
//  '<S230>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Filter ICs'
//  '<S231>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/I Gain'
//  '<S232>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Ideal P Gain'
//  '<S233>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Ideal P Gain Fdbk'
//  '<S234>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Integrator'
//  '<S235>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Integrator ICs'
//  '<S236>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/N Copy'
//  '<S237>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/N Gain'
//  '<S238>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/P Copy'
//  '<S239>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Parallel P Gain'
//  '<S240>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Reset Signal'
//  '<S241>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Saturation'
//  '<S242>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Saturation Fdbk'
//  '<S243>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Sum'
//  '<S244>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Sum Fdbk'
//  '<S245>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Tracking Mode'
//  '<S246>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Tracking Mode Sum'
//  '<S247>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Tsamp - Integral'
//  '<S248>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Tsamp - Ngain'
//  '<S249>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/postSat Signal'
//  '<S250>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/preInt Signal'
//  '<S251>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/preSat Signal'
//  '<S252>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Anti-windup/Disabled'
//  '<S253>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/D Gain/Disabled'
//  '<S254>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/External Derivative/Disabled'
//  '<S255>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Filter/Disabled'
//  '<S256>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Filter ICs/Disabled'
//  '<S257>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/I Gain/Disabled'
//  '<S258>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Ideal P Gain/Passthrough'
//  '<S259>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Ideal P Gain Fdbk/Disabled'
//  '<S260>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Integrator/Disabled'
//  '<S261>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Integrator ICs/Disabled'
//  '<S262>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/N Copy/Disabled wSignal Specification'
//  '<S263>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/N Gain/Disabled'
//  '<S264>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/P Copy/Disabled'
//  '<S265>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Parallel P Gain/Internal Parameters'
//  '<S266>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Reset Signal/Disabled'
//  '<S267>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Saturation/Passthrough'
//  '<S268>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Saturation Fdbk/Disabled'
//  '<S269>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Sum/Passthrough_P'
//  '<S270>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Sum Fdbk/Disabled'
//  '<S271>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Tracking Mode/Disabled'
//  '<S272>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Tracking Mode Sum/Passthrough'
//  '<S273>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Tsamp - Integral/TsSignalSpecification'
//  '<S274>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/Tsamp - Ngain/Passthrough'
//  '<S275>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/postSat Signal/Forward_Path'
//  '<S276>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/preInt Signal/Internal PreInt'
//  '<S277>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller4/preSat Signal/Forward_Path'
//  '<S278>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Anti-windup'
//  '<S279>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/D Gain'
//  '<S280>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/External Derivative'
//  '<S281>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Filter'
//  '<S282>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Filter ICs'
//  '<S283>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/I Gain'
//  '<S284>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Ideal P Gain'
//  '<S285>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Ideal P Gain Fdbk'
//  '<S286>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Integrator'
//  '<S287>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Integrator ICs'
//  '<S288>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/N Copy'
//  '<S289>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/N Gain'
//  '<S290>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/P Copy'
//  '<S291>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Parallel P Gain'
//  '<S292>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Reset Signal'
//  '<S293>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Saturation'
//  '<S294>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Saturation Fdbk'
//  '<S295>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Sum'
//  '<S296>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Sum Fdbk'
//  '<S297>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Tracking Mode'
//  '<S298>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Tracking Mode Sum'
//  '<S299>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Tsamp - Integral'
//  '<S300>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Tsamp - Ngain'
//  '<S301>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/postSat Signal'
//  '<S302>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/preInt Signal'
//  '<S303>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/preSat Signal'
//  '<S304>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Anti-windup/Disabled'
//  '<S305>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/D Gain/Disabled'
//  '<S306>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/External Derivative/Disabled'
//  '<S307>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Filter/Disabled'
//  '<S308>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Filter ICs/Disabled'
//  '<S309>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/I Gain/Disabled'
//  '<S310>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Ideal P Gain/Passthrough'
//  '<S311>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Ideal P Gain Fdbk/Disabled'
//  '<S312>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Integrator/Disabled'
//  '<S313>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Integrator ICs/Disabled'
//  '<S314>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/N Copy/Disabled wSignal Specification'
//  '<S315>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/N Gain/Disabled'
//  '<S316>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/P Copy/Disabled'
//  '<S317>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Parallel P Gain/Internal Parameters'
//  '<S318>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Reset Signal/Disabled'
//  '<S319>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Saturation/Enabled'
//  '<S320>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Saturation Fdbk/Disabled'
//  '<S321>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Sum/Passthrough_P'
//  '<S322>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Sum Fdbk/Disabled'
//  '<S323>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Tracking Mode/Disabled'
//  '<S324>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Tracking Mode Sum/Passthrough'
//  '<S325>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Tsamp - Integral/TsSignalSpecification'
//  '<S326>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/Tsamp - Ngain/Passthrough'
//  '<S327>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/postSat Signal/Forward_Path'
//  '<S328>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/preInt Signal/Internal PreInt'
//  '<S329>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Attitude controller/PID Controller5/preSat Signal/Forward_Path'
//  '<S330>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/Altitude Signal conditioning'
//  '<S331>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/Calculate Transformation Matrix'
//  '<S332>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller'
//  '<S333>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1'
//  '<S334>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2'
//  '<S335>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude'
//  '<S336>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz'
//  '<S337>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/Altitude Signal conditioning/Saturation Dynamic'
//  '<S338>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Anti-windup'
//  '<S339>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/D Gain'
//  '<S340>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/External Derivative'
//  '<S341>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Filter'
//  '<S342>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Filter ICs'
//  '<S343>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/I Gain'
//  '<S344>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Ideal P Gain'
//  '<S345>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Ideal P Gain Fdbk'
//  '<S346>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Integrator'
//  '<S347>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Integrator ICs'
//  '<S348>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/N Copy'
//  '<S349>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/N Gain'
//  '<S350>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/P Copy'
//  '<S351>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Parallel P Gain'
//  '<S352>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Reset Signal'
//  '<S353>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Saturation'
//  '<S354>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Saturation Fdbk'
//  '<S355>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Sum'
//  '<S356>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Sum Fdbk'
//  '<S357>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Tracking Mode'
//  '<S358>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Tracking Mode Sum'
//  '<S359>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Tsamp - Integral'
//  '<S360>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Tsamp - Ngain'
//  '<S361>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/postSat Signal'
//  '<S362>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/preInt Signal'
//  '<S363>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/preSat Signal'
//  '<S364>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Anti-windup/Disabled'
//  '<S365>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/D Gain/Disabled'
//  '<S366>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/External Derivative/Disabled'
//  '<S367>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Filter/Disabled'
//  '<S368>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Filter ICs/Disabled'
//  '<S369>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/I Gain/Disabled'
//  '<S370>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Ideal P Gain/Passthrough'
//  '<S371>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Ideal P Gain Fdbk/Disabled'
//  '<S372>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Integrator/Disabled'
//  '<S373>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Integrator ICs/Disabled'
//  '<S374>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/N Copy/Disabled wSignal Specification'
//  '<S375>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/N Gain/Disabled'
//  '<S376>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/P Copy/Disabled'
//  '<S377>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Parallel P Gain/Internal Parameters'
//  '<S378>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Reset Signal/Disabled'
//  '<S379>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Saturation/Enabled'
//  '<S380>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Saturation Fdbk/Disabled'
//  '<S381>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Sum/Passthrough_P'
//  '<S382>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Sum Fdbk/Disabled'
//  '<S383>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Tracking Mode/Disabled'
//  '<S384>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Tracking Mode Sum/Passthrough'
//  '<S385>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Tsamp - Integral/TsSignalSpecification'
//  '<S386>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/Tsamp - Ngain/Passthrough'
//  '<S387>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/postSat Signal/Forward_Path'
//  '<S388>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/preInt Signal/Internal PreInt'
//  '<S389>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller/preSat Signal/Forward_Path'
//  '<S390>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Anti-windup'
//  '<S391>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/D Gain'
//  '<S392>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/External Derivative'
//  '<S393>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Filter'
//  '<S394>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Filter ICs'
//  '<S395>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/I Gain'
//  '<S396>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Ideal P Gain'
//  '<S397>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Ideal P Gain Fdbk'
//  '<S398>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Integrator'
//  '<S399>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Integrator ICs'
//  '<S400>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/N Copy'
//  '<S401>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/N Gain'
//  '<S402>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/P Copy'
//  '<S403>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Parallel P Gain'
//  '<S404>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Reset Signal'
//  '<S405>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Saturation'
//  '<S406>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Saturation Fdbk'
//  '<S407>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Sum'
//  '<S408>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Sum Fdbk'
//  '<S409>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Tracking Mode'
//  '<S410>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Tracking Mode Sum'
//  '<S411>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Tsamp - Integral'
//  '<S412>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Tsamp - Ngain'
//  '<S413>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/postSat Signal'
//  '<S414>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/preInt Signal'
//  '<S415>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/preSat Signal'
//  '<S416>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Anti-windup/Disabled'
//  '<S417>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/D Gain/Disabled'
//  '<S418>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/External Derivative/Disabled'
//  '<S419>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Filter/Disabled'
//  '<S420>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Filter ICs/Disabled'
//  '<S421>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/I Gain/Disabled'
//  '<S422>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Ideal P Gain/Passthrough'
//  '<S423>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Ideal P Gain Fdbk/Disabled'
//  '<S424>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Integrator/Disabled'
//  '<S425>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Integrator ICs/Disabled'
//  '<S426>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/N Copy/Disabled wSignal Specification'
//  '<S427>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/N Gain/Disabled'
//  '<S428>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/P Copy/Disabled'
//  '<S429>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Parallel P Gain/Internal Parameters'
//  '<S430>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Reset Signal/Disabled'
//  '<S431>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Saturation/Enabled'
//  '<S432>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Saturation Fdbk/Disabled'
//  '<S433>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Sum/Passthrough_P'
//  '<S434>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Sum Fdbk/Disabled'
//  '<S435>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Tracking Mode/Disabled'
//  '<S436>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Tracking Mode Sum/Passthrough'
//  '<S437>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Tsamp - Integral/TsSignalSpecification'
//  '<S438>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/Tsamp - Ngain/Passthrough'
//  '<S439>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/postSat Signal/Forward_Path'
//  '<S440>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/preInt Signal/Internal PreInt'
//  '<S441>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller1/preSat Signal/Forward_Path'
//  '<S442>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Anti-windup'
//  '<S443>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/D Gain'
//  '<S444>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/External Derivative'
//  '<S445>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Filter'
//  '<S446>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Filter ICs'
//  '<S447>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/I Gain'
//  '<S448>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Ideal P Gain'
//  '<S449>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Ideal P Gain Fdbk'
//  '<S450>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Integrator'
//  '<S451>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Integrator ICs'
//  '<S452>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/N Copy'
//  '<S453>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/N Gain'
//  '<S454>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/P Copy'
//  '<S455>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Parallel P Gain'
//  '<S456>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Reset Signal'
//  '<S457>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Saturation'
//  '<S458>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Saturation Fdbk'
//  '<S459>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Sum'
//  '<S460>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Sum Fdbk'
//  '<S461>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Tracking Mode'
//  '<S462>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Tracking Mode Sum'
//  '<S463>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Tsamp - Integral'
//  '<S464>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Tsamp - Ngain'
//  '<S465>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/postSat Signal'
//  '<S466>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/preInt Signal'
//  '<S467>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/preSat Signal'
//  '<S468>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Anti-windup/Disabled'
//  '<S469>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/D Gain/Disabled'
//  '<S470>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/External Derivative/Disabled'
//  '<S471>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Filter/Disabled'
//  '<S472>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Filter ICs/Disabled'
//  '<S473>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/I Gain/Disabled'
//  '<S474>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Ideal P Gain/Passthrough'
//  '<S475>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Ideal P Gain Fdbk/Disabled'
//  '<S476>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Integrator/Disabled'
//  '<S477>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Integrator ICs/Disabled'
//  '<S478>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/N Copy/Disabled wSignal Specification'
//  '<S479>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/N Gain/Disabled'
//  '<S480>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/P Copy/Disabled'
//  '<S481>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Parallel P Gain/Internal Parameters'
//  '<S482>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Reset Signal/Disabled'
//  '<S483>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Saturation/Enabled'
//  '<S484>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Saturation Fdbk/Disabled'
//  '<S485>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Sum/Passthrough_P'
//  '<S486>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Sum Fdbk/Disabled'
//  '<S487>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Tracking Mode/Disabled'
//  '<S488>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Tracking Mode Sum/Passthrough'
//  '<S489>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Tsamp - Integral/TsSignalSpecification'
//  '<S490>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/Tsamp - Ngain/Passthrough'
//  '<S491>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/postSat Signal/Forward_Path'
//  '<S492>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/preInt Signal/Internal PreInt'
//  '<S493>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID Controller2/preSat Signal/Forward_Path'
//  '<S494>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Anti-windup'
//  '<S495>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/D Gain'
//  '<S496>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/External Derivative'
//  '<S497>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Filter'
//  '<S498>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Filter ICs'
//  '<S499>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/I Gain'
//  '<S500>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Ideal P Gain'
//  '<S501>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Ideal P Gain Fdbk'
//  '<S502>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Integrator'
//  '<S503>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Integrator ICs'
//  '<S504>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/N Copy'
//  '<S505>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/N Gain'
//  '<S506>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/P Copy'
//  '<S507>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Parallel P Gain'
//  '<S508>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Reset Signal'
//  '<S509>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Saturation'
//  '<S510>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Saturation Fdbk'
//  '<S511>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Sum'
//  '<S512>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Sum Fdbk'
//  '<S513>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Tracking Mode'
//  '<S514>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Tracking Mode Sum'
//  '<S515>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Tsamp - Integral'
//  '<S516>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Tsamp - Ngain'
//  '<S517>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/postSat Signal'
//  '<S518>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/preInt Signal'
//  '<S519>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/preSat Signal'
//  '<S520>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Anti-windup/Disc. Clamping Parallel'
//  '<S521>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Anti-windup/Disc. Clamping Parallel/Dead Zone'
//  '<S522>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
//  '<S523>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/D Gain/Internal Parameters'
//  '<S524>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/External Derivative/Error'
//  '<S525>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Filter/Disc. Forward Euler Filter'
//  '<S526>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Filter ICs/Internal IC - Filter'
//  '<S527>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/I Gain/Internal Parameters'
//  '<S528>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Ideal P Gain/Passthrough'
//  '<S529>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Ideal P Gain Fdbk/Passthrough'
//  '<S530>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Integrator/Discrete'
//  '<S531>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Integrator ICs/Internal IC'
//  '<S532>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/N Copy/Disabled'
//  '<S533>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/N Gain/Internal Parameters'
//  '<S534>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/P Copy/Disabled'
//  '<S535>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Parallel P Gain/Internal Parameters'
//  '<S536>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Reset Signal/Disabled'
//  '<S537>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Saturation/Enabled'
//  '<S538>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Saturation Fdbk/Passthrough'
//  '<S539>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Sum/Sum_PID'
//  '<S540>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Sum Fdbk/Enabled'
//  '<S541>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Tracking Mode/Disabled'
//  '<S542>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Tracking Mode Sum/Passthrough'
//  '<S543>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Tsamp - Integral/TsSignalSpecification'
//  '<S544>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/Tsamp - Ngain/Passthrough'
//  '<S545>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/postSat Signal/Feedback_Path'
//  '<S546>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/preInt Signal/Internal PreInt'
//  '<S547>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_Altitude/preSat Signal/Feedback_Path'
//  '<S548>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Anti-windup'
//  '<S549>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/D Gain'
//  '<S550>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/External Derivative'
//  '<S551>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Filter'
//  '<S552>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Filter ICs'
//  '<S553>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/I Gain'
//  '<S554>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Ideal P Gain'
//  '<S555>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Ideal P Gain Fdbk'
//  '<S556>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Integrator'
//  '<S557>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Integrator ICs'
//  '<S558>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/N Copy'
//  '<S559>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/N Gain'
//  '<S560>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/P Copy'
//  '<S561>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Parallel P Gain'
//  '<S562>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Reset Signal'
//  '<S563>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Saturation'
//  '<S564>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Saturation Fdbk'
//  '<S565>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Sum'
//  '<S566>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Sum Fdbk'
//  '<S567>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Tracking Mode'
//  '<S568>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Tracking Mode Sum'
//  '<S569>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Tsamp - Integral'
//  '<S570>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Tsamp - Ngain'
//  '<S571>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/postSat Signal'
//  '<S572>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/preInt Signal'
//  '<S573>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/preSat Signal'
//  '<S574>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Anti-windup/Disc. Clamping Parallel'
//  '<S575>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Anti-windup/Disc. Clamping Parallel/Dead Zone'
//  '<S576>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
//  '<S577>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/D Gain/Internal Parameters'
//  '<S578>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/External Derivative/Error'
//  '<S579>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Filter/Disc. Forward Euler Filter'
//  '<S580>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Filter ICs/Internal IC - Filter'
//  '<S581>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/I Gain/Internal Parameters'
//  '<S582>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Ideal P Gain/Passthrough'
//  '<S583>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Ideal P Gain Fdbk/Passthrough'
//  '<S584>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Integrator/Discrete'
//  '<S585>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Integrator ICs/Internal IC'
//  '<S586>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/N Copy/Disabled'
//  '<S587>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/N Gain/Internal Parameters'
//  '<S588>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/P Copy/Disabled'
//  '<S589>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Parallel P Gain/Internal Parameters'
//  '<S590>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Reset Signal/Disabled'
//  '<S591>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Saturation/Enabled'
//  '<S592>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Saturation Fdbk/Passthrough'
//  '<S593>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Sum/Sum_PID'
//  '<S594>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Sum Fdbk/Enabled'
//  '<S595>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Tracking Mode/Disabled'
//  '<S596>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Tracking Mode Sum/Passthrough'
//  '<S597>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Tsamp - Integral/TsSignalSpecification'
//  '<S598>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/Tsamp - Ngain/Passthrough'
//  '<S599>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/postSat Signal/Feedback_Path'
//  '<S600>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/preInt Signal/Internal PreInt'
//  '<S601>' : 'PositionAttitudeControllerCopter/Position and Attitude Controller/Position & Altitude controller/PID_vz/preSat Signal/Feedback_Path'

#endif                                 // PositionAttitudeControllerCopter_h_

//
// File trailer for generated code.
//
// [EOF]
//
