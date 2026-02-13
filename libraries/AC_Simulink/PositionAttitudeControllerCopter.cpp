//
// File: PositionAttitudeControllerCopter.cpp
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
#include "rtwtypes.h"
#include <math.h>

const real32_T PositionAttitudeControll_period = 0.0025F;

// Block signals (default storage)
B_PositionAttitudeControllerC_T PositionAttitudeControllerCop_B;

// Block states (default storage)
DW_PositionAttitudeController_T PositionAttitudeControllerCo_DW;

// Real-time model
RT_MODEL_PositionAttitudeCont_T PositionAttitudeControllerCo_M_ =
  RT_MODEL_PositionAttitudeCont_T();
RT_MODEL_PositionAttitudeCont_T *const PositionAttitudeControllerCo_M =
  &PositionAttitudeControllerCo_M_;

// Model step function
void PositionAttitudeControllerCopter_step(void)
{
  real_T rtb_DeadZone;
  real_T u0;
  real32_T rtb_DeadZone_i;
  real32_T rtb_DeadZone_o;
  real32_T rtb_DeadZone_p;
  real32_T rtb_FilterCoefficient_b;
  real32_T rtb_FilterCoefficient_h;
  real32_T rtb_Filter_a;
  real32_T rtb_Filter_nf;
  real32_T rtb_IntegralGain_h;
  real32_T rtb_IntegralGain_ia;
  real32_T rtb_Integrator_p;
  real32_T rtb_Sum5;
  int8_T tmp;
  int8_T tmp_0;

  // MATLABSystem: '<Root>/Current Attitude'
  MW_getCurrentAttitudeEuler
    (&PositionAttitudeControllerCop_B.rtb_CurrentAttitude_m[0]);

  // Trigonometry: '<S331>/SinCos' incorporates:
  //   MATLABSystem: '<Root>/Current Attitude'

  rtb_Filter_nf = static_cast<real32_T>(sin(static_cast<real_T>
    (PositionAttitudeControllerCop_B.rtb_CurrentAttitude_m[0])));
  rtb_Integrator_p = static_cast<real32_T>(cos(static_cast<real_T>
    (PositionAttitudeControllerCop_B.rtb_CurrentAttitude_m[0])));

  // MATLABSystem: '<Root>/Position Setpoint'
  MW_getPosNEDSetpoint(&PositionAttitudeControllerCop_B.posNED[0]);

  // MATLABSystem: '<Root>/Current Position'
  MW_getCurrentPositionNED
    (&PositionAttitudeControllerCop_B.rtb_CurrentPosition_c[0]);

  // MATLABSystem: '<Root>/Current Velocity'
  MW_getCurrentVelocityNED
    (&PositionAttitudeControllerCop_B.rtb_CurrentVelocity_k[0]);

  // MATLABSystem: '<Root>/Velocity Setpoint'
  MW_getVelNEDSetpoint
    (&PositionAttitudeControllerCop_B.rtb_CurrentAngVelocity_c[0]);

  // Saturate: '<S431>/Saturation' incorporates:
  //   Gain: '<S429>/Proportional Gain'
  //   MATLABSystem: '<Root>/Current Position'
  //   MATLABSystem: '<Root>/Position Setpoint'
  //   Sum: '<S4>/Sum1'
  //
  rtb_Sum5 = (static_cast<real32_T>(PositionAttitudeControllerCop_B.posNED[0]) *
              0.01F - PositionAttitudeControllerCop_B.rtb_CurrentPosition_c[0]) *
    PositionAttitudeControllerCop_P.PIDController1_P;

  // Saturate: '<S483>/Saturation' incorporates:
  //   Gain: '<S481>/Proportional Gain'
  //   MATLABSystem: '<Root>/Current Velocity'
  //   MATLABSystem: '<Root>/Velocity Setpoint'
  //   Sum: '<S4>/Sum1'
  //   Sum: '<S4>/Sum4'
  //
  rtb_IntegralGain_h =
    (PositionAttitudeControllerCop_B.rtb_CurrentAngVelocity_c[0] * 0.01F -
     PositionAttitudeControllerCop_B.rtb_CurrentVelocity_k[0]) *
    PositionAttitudeControllerCop_P.PIDController2_P;

  // Saturate: '<S431>/Saturation'
  if (rtb_Sum5 > PositionAttitudeControllerCop_P.PIDController1_UpperSaturationL)
  {
    rtb_Sum5 = PositionAttitudeControllerCop_P.PIDController1_UpperSaturationL;
  } else if (rtb_Sum5 <
             PositionAttitudeControllerCop_P.PIDController1_LowerSaturationL) {
    rtb_Sum5 = PositionAttitudeControllerCop_P.PIDController1_LowerSaturationL;
  }

  // Saturate: '<S483>/Saturation'
  if (rtb_IntegralGain_h >
      PositionAttitudeControllerCop_P.PIDController2_UpperSaturationL) {
    rtb_IntegralGain_h =
      PositionAttitudeControllerCop_P.PIDController2_UpperSaturationL;
  } else if (rtb_IntegralGain_h <
             PositionAttitudeControllerCop_P.PIDController2_LowerSaturationL) {
    rtb_IntegralGain_h =
      PositionAttitudeControllerCop_P.PIDController2_LowerSaturationL;
  }

  // Sum: '<S4>/Sum' incorporates:
  //   Saturate: '<S431>/Saturation'
  //   Saturate: '<S483>/Saturation'

  rtb_DeadZone_p = rtb_Sum5 + rtb_IntegralGain_h;

  // Saturate: '<S431>/Saturation' incorporates:
  //   Gain: '<S429>/Proportional Gain'
  //   MATLABSystem: '<Root>/Current Position'
  //   MATLABSystem: '<Root>/Position Setpoint'
  //   Sum: '<S4>/Sum1'
  //
  rtb_Sum5 = (static_cast<real32_T>(PositionAttitudeControllerCop_B.posNED[1]) *
              0.01F - PositionAttitudeControllerCop_B.rtb_CurrentPosition_c[1]) *
    PositionAttitudeControllerCop_P.PIDController1_P;

  // Saturate: '<S483>/Saturation' incorporates:
  //   Gain: '<S481>/Proportional Gain'
  //   MATLABSystem: '<Root>/Current Velocity'
  //   MATLABSystem: '<Root>/Velocity Setpoint'
  //   Sum: '<S4>/Sum1'
  //   Sum: '<S4>/Sum4'
  //
  rtb_IntegralGain_h =
    (PositionAttitudeControllerCop_B.rtb_CurrentAngVelocity_c[1] * 0.01F -
     PositionAttitudeControllerCop_B.rtb_CurrentVelocity_k[1]) *
    PositionAttitudeControllerCop_P.PIDController2_P;

  // Saturate: '<S431>/Saturation'
  if (rtb_Sum5 > PositionAttitudeControllerCop_P.PIDController1_UpperSaturationL)
  {
    rtb_Sum5 = PositionAttitudeControllerCop_P.PIDController1_UpperSaturationL;
  } else if (rtb_Sum5 <
             PositionAttitudeControllerCop_P.PIDController1_LowerSaturationL) {
    rtb_Sum5 = PositionAttitudeControllerCop_P.PIDController1_LowerSaturationL;
  }

  // Saturate: '<S483>/Saturation'
  if (rtb_IntegralGain_h >
      PositionAttitudeControllerCop_P.PIDController2_UpperSaturationL) {
    rtb_IntegralGain_h =
      PositionAttitudeControllerCop_P.PIDController2_UpperSaturationL;
  } else if (rtb_IntegralGain_h <
             PositionAttitudeControllerCop_P.PIDController2_LowerSaturationL) {
    rtb_IntegralGain_h =
      PositionAttitudeControllerCop_P.PIDController2_LowerSaturationL;
  }

  // Sum: '<S4>/Sum' incorporates:
  //   Saturate: '<S431>/Saturation'
  //   Saturate: '<S483>/Saturation'

  rtb_IntegralGain_h += rtb_Sum5;

  // Saturate: '<S379>/Saturation' incorporates:
  //   Gain: '<S331>/Gain'
  //   Gain: '<S377>/Proportional Gain'
  //   Product: '<S4>/MatrixMultiply'
  //   Reshape: '<S331>/Reshape'
  //   Reshape: '<S331>/Reshape1'

  rtb_Sum5 = (rtb_Integrator_p * rtb_DeadZone_p + rtb_Filter_nf *
              rtb_IntegralGain_h) *
    PositionAttitudeControllerCop_P.PIDController_P_h;
  rtb_Filter_nf = (PositionAttitudeControllerCop_P.Gain_Gain * rtb_Filter_nf *
                   rtb_DeadZone_p + rtb_Integrator_p * rtb_IntegralGain_h) *
    PositionAttitudeControllerCop_P.PIDController_P_h;
  if (rtb_Filter_nf >
      PositionAttitudeControllerCop_P.PIDController_UpperSaturation_f) {
    rtb_Filter_nf =
      PositionAttitudeControllerCop_P.PIDController_UpperSaturation_f;
  } else if (rtb_Filter_nf <
             PositionAttitudeControllerCop_P.PIDController_LowerSaturation_e) {
    rtb_Filter_nf =
      PositionAttitudeControllerCop_P.PIDController_LowerSaturation_e;
  }

  // RateLimiter: '<S3>/Rate Limiter1'
  rtb_FilterCoefficient_b = rtb_Filter_nf -
    PositionAttitudeControllerCo_DW.PrevY;
  rtb_IntegralGain_ia = static_cast<real32_T>
    (PositionAttitudeControllerCop_P.RateLimiter1_RisingLim) *
    PositionAttitudeControll_period;
  if (rtb_FilterCoefficient_b > rtb_IntegralGain_ia) {
    rtb_Filter_nf = rtb_IntegralGain_ia + PositionAttitudeControllerCo_DW.PrevY;
  } else if (rtb_FilterCoefficient_b < static_cast<real32_T>
             (PositionAttitudeControllerCop_P.RateLimiter1_FallingLim) *
             PositionAttitudeControll_period) {
    rtb_Filter_nf = static_cast<real32_T>
      (PositionAttitudeControllerCop_P.RateLimiter1_FallingLim) *
      PositionAttitudeControll_period + PositionAttitudeControllerCo_DW.PrevY;
  }

  PositionAttitudeControllerCo_DW.PrevY = rtb_Filter_nf;

  // End of RateLimiter: '<S3>/Rate Limiter1'

  // MATLABSystem: '<Root>/Current Ang Velocity'
  MW_getCurrentAngVelocity
    (&PositionAttitudeControllerCop_B.rtb_CurrentAngVelocity_c[0]);

  // Saturate: '<S3>/Saturation'
  if (rtb_Filter_nf > PositionAttitudeControllerCop_P.Saturation_UpperSat) {
    rtb_Filter_nf = PositionAttitudeControllerCop_P.Saturation_UpperSat;
  } else if (rtb_Filter_nf < PositionAttitudeControllerCop_P.Saturation_LowerSat)
  {
    rtb_Filter_nf = PositionAttitudeControllerCop_P.Saturation_LowerSat;
  }

  // Sum: '<S3>/Sum1' incorporates:
  //   Gain: '<S265>/Proportional Gain'
  //   MATLABSystem: '<Root>/Current Ang Velocity'
  //   MATLABSystem: '<Root>/Current Attitude'
  //   Saturate: '<S3>/Saturation'
  //   Sum: '<S3>/Sum3'

  rtb_Filter_nf = (rtb_Filter_nf -
                   PositionAttitudeControllerCop_B.rtb_CurrentAttitude_m[2]) *
    PositionAttitudeControllerCop_P.PIDController4_P -
    PositionAttitudeControllerCop_B.rtb_CurrentAngVelocity_c[0];

  // Gain: '<S159>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S151>/Filter'
  //   Gain: '<S149>/Derivative Gain'
  //   Sum: '<S151>/SumD'

  rtb_Integrator_p = (PositionAttitudeControllerCop_P.PIDController2_D *
                      rtb_Filter_nf -
                      PositionAttitudeControllerCo_DW.Filter_DSTATE_d) *
    PositionAttitudeControllerCop_P.PIDController2_N;

  // Sum: '<S165>/Sum' incorporates:
  //   DiscreteIntegrator: '<S156>/Integrator'
  //   Gain: '<S161>/Proportional Gain'

  rtb_DeadZone_p = (PositionAttitudeControllerCop_P.PIDController2_P_j *
                    rtb_Filter_nf +
                    PositionAttitudeControllerCo_DW.Integrator_DSTATE_m) +
    rtb_Integrator_p;

  // Saturate: '<S379>/Saturation'
  if (rtb_Sum5 > PositionAttitudeControllerCop_P.PIDController_UpperSaturation_f)
  {
    rtb_Sum5 = PositionAttitudeControllerCop_P.PIDController_UpperSaturation_f;
  } else if (rtb_Sum5 <
             PositionAttitudeControllerCop_P.PIDController_LowerSaturation_e) {
    rtb_Sum5 = PositionAttitudeControllerCop_P.PIDController_LowerSaturation_e;
  }

  // Gain: '<S4>/Gain1' incorporates:
  //   Saturate: '<S379>/Saturation'

  rtb_IntegralGain_h = PositionAttitudeControllerCop_P.Gain1_Gain * rtb_Sum5;

  // RateLimiter: '<S3>/Rate Limiter2'
  rtb_FilterCoefficient_b = rtb_IntegralGain_h -
    PositionAttitudeControllerCo_DW.PrevY_f;
  rtb_IntegralGain_ia = static_cast<real32_T>
    (PositionAttitudeControllerCop_P.RateLimiter2_RisingLim) *
    PositionAttitudeControll_period;
  if (rtb_FilterCoefficient_b > rtb_IntegralGain_ia) {
    rtb_IntegralGain_h = rtb_IntegralGain_ia +
      PositionAttitudeControllerCo_DW.PrevY_f;
  } else if (rtb_FilterCoefficient_b < static_cast<real32_T>
             (PositionAttitudeControllerCop_P.RateLimiter2_FallingLim) *
             PositionAttitudeControll_period) {
    rtb_IntegralGain_h = static_cast<real32_T>
      (PositionAttitudeControllerCop_P.RateLimiter2_FallingLim) *
      PositionAttitudeControll_period + PositionAttitudeControllerCo_DW.PrevY_f;
  }

  PositionAttitudeControllerCo_DW.PrevY_f = rtb_IntegralGain_h;

  // End of RateLimiter: '<S3>/Rate Limiter2'

  // Saturate: '<S3>/Saturation1'
  if (rtb_IntegralGain_h > PositionAttitudeControllerCop_P.Saturation1_UpperSat)
  {
    rtb_IntegralGain_h = PositionAttitudeControllerCop_P.Saturation1_UpperSat;
  } else if (rtb_IntegralGain_h <
             PositionAttitudeControllerCop_P.Saturation1_LowerSat) {
    rtb_IntegralGain_h = PositionAttitudeControllerCop_P.Saturation1_LowerSat;
  }

  // Sum: '<S3>/Sum' incorporates:
  //   Gain: '<S213>/Proportional Gain'
  //   MATLABSystem: '<Root>/Current Ang Velocity'
  //   MATLABSystem: '<Root>/Current Attitude'
  //   Saturate: '<S3>/Saturation1'
  //   Sum: '<S3>/Sum2'

  rtb_IntegralGain_h = (rtb_IntegralGain_h -
                        PositionAttitudeControllerCop_B.rtb_CurrentAttitude_m[1])
    * PositionAttitudeControllerCop_P.PIDController3_P -
    PositionAttitudeControllerCop_B.rtb_CurrentAngVelocity_c[1];

  // Gain: '<S105>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S97>/Filter'
  //   Gain: '<S95>/Derivative Gain'
  //   Sum: '<S97>/SumD'

  rtb_FilterCoefficient_b = (PositionAttitudeControllerCop_P.PIDController1_D *
    rtb_IntegralGain_h - PositionAttitudeControllerCo_DW.Filter_DSTATE_dx) *
    PositionAttitudeControllerCop_P.PIDController1_N;

  // Sum: '<S111>/Sum' incorporates:
  //   DiscreteIntegrator: '<S102>/Integrator'
  //   Gain: '<S107>/Proportional Gain'

  rtb_DeadZone_o = (PositionAttitudeControllerCop_P.PIDController1_P_a *
                    rtb_IntegralGain_h +
                    PositionAttitudeControllerCo_DW.Integrator_DSTATE_c) +
    rtb_FilterCoefficient_b;

  // MATLABSystem: '<Root>/Attitude Setpoint'
  MW_getAttSetpointEuler
    (&PositionAttitudeControllerCop_B.rtb_AttitudeSetpoint_b[0]);

  // Sum: '<S3>/Sum5' incorporates:
  //   MATLABSystem: '<Root>/Attitude Setpoint'
  //   MATLABSystem: '<Root>/Current Attitude'

  rtb_Sum5 = PositionAttitudeControllerCop_B.rtb_AttitudeSetpoint_b[0] -
    PositionAttitudeControllerCop_B.rtb_CurrentAttitude_m[0];

  // Switch: '<S5>/Switch' incorporates:
  //   Abs: '<S5>/Abs'

  if (static_cast<real32_T>(fabs(static_cast<real_T>(rtb_Sum5))) >
      PositionAttitudeControllerCop_P.Switch_Threshold) {
    // Switch: '<S5>/Switch1' incorporates:
    //   Constant: '<S5>/Constant'
    //   Sum: '<S5>/Add'
    //   Sum: '<S5>/Subtract'

    if (rtb_Sum5 > PositionAttitudeControllerCop_P.Switch1_Threshold) {
      u0 = rtb_Sum5 - PositionAttitudeControllerCop_P.Constant_Value_i;
    } else {
      u0 = PositionAttitudeControllerCop_P.Constant_Value_i + rtb_Sum5;
    }

    // End of Switch: '<S5>/Switch1'
  } else {
    u0 = rtb_Sum5;
  }

  // Gain: '<S317>/Proportional Gain' incorporates:
  //   Switch: '<S5>/Switch'

  u0 *= PositionAttitudeControllerCop_P.PIDController5_P;

  // Saturate: '<S319>/Saturation'
  if (u0 > PositionAttitudeControllerCop_P.PIDController5_UpperSaturationL) {
    u0 = PositionAttitudeControllerCop_P.PIDController5_UpperSaturationL;
  } else if (u0 <
             PositionAttitudeControllerCop_P.PIDController5_LowerSaturationL) {
    u0 = PositionAttitudeControllerCop_P.PIDController5_LowerSaturationL;
  }

  // Sum: '<S3>/Sum4' incorporates:
  //   MATLABSystem: '<Root>/Current Ang Velocity'
  //   Saturate: '<S319>/Saturation'

  PositionAttitudeControllerCop_B.IntegralGain = u0 -
    PositionAttitudeControllerCop_B.rtb_CurrentAngVelocity_c[2];

  // Gain: '<S51>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S43>/Filter'
  //   Gain: '<S41>/Derivative Gain'
  //   Sum: '<S43>/SumD'

  PositionAttitudeControllerCop_B.FilterCoefficient =
    (PositionAttitudeControllerCop_P.PIDController_D *
     PositionAttitudeControllerCop_B.IntegralGain -
     PositionAttitudeControllerCo_DW.Filter_DSTATE) *
    PositionAttitudeControllerCop_P.PIDController_N;

  // Sum: '<S57>/Sum' incorporates:
  //   DiscreteIntegrator: '<S48>/Integrator'
  //   Gain: '<S53>/Proportional Gain'

  rtb_DeadZone = (PositionAttitudeControllerCop_P.PIDController_P *
                  PositionAttitudeControllerCop_B.IntegralGain +
                  PositionAttitudeControllerCo_DW.Integrator_DSTATE) +
    PositionAttitudeControllerCop_B.FilterCoefficient;

  // Gain: '<S330>/Gain1' incorporates:
  //   MATLABSystem: '<Root>/Current Position'

  rtb_Sum5 = PositionAttitudeControllerCop_P.Gain1_Gain_b *
    PositionAttitudeControllerCop_B.rtb_CurrentPosition_c[2];

  // Switch: '<S337>/Switch' incorporates:
  //   Constant: '<S330>/Constant'
  //   RelationalOperator: '<S337>/UpperRelop'
  //   Switch: '<S337>/Switch2'

  if (rtb_Sum5 < PositionAttitudeControllerCop_P.Constant_Value) {
    rtb_Sum5 = static_cast<real32_T>
      (PositionAttitudeControllerCop_P.Constant_Value);
  }

  // Sum: '<S4>/Sum2' incorporates:
  //   Gain: '<S1>/Gain'
  //   MATLABSystem: '<Root>/Position Setpoint'
  //   Switch: '<S337>/Switch'
  //   Switch: '<S337>/Switch2'
  //
  rtb_Filter_a = static_cast<real32_T>(PositionAttitudeControllerCop_B.posNED[2])
    * 0.01F * PositionAttitudeControllerCop_P.Gain_Gain_o - rtb_Sum5;

  // Gain: '<S535>/Proportional Gain'
  rtb_IntegralGain_ia = PositionAttitudeControllerCop_P.PID_Altitude_P *
    rtb_Filter_a;

  // Gain: '<S533>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S525>/Filter'
  //   Gain: '<S523>/Derivative Gain'
  //   Sum: '<S525>/SumD'

  rtb_FilterCoefficient_h = (PositionAttitudeControllerCop_P.PID_Altitude_D *
    rtb_Filter_a - PositionAttitudeControllerCo_DW.Filter_DSTATE_j) *
    PositionAttitudeControllerCop_P.PID_Altitude_N;

  // Sum: '<S540>/Sum Fdbk'
  rtb_Sum5 = (rtb_IntegralGain_ia +
              PositionAttitudeControllerCo_DW.Integrator_DSTATE_e) +
    rtb_FilterCoefficient_h;

  // DeadZone: '<S522>/DeadZone'
  if (rtb_Sum5 > PositionAttitudeControllerCop_P.PID_Altitude_UpperSaturationLim)
  {
    rtb_Sum5 -= PositionAttitudeControllerCop_P.PID_Altitude_UpperSaturationLim;
  } else if (rtb_Sum5 >=
             PositionAttitudeControllerCop_P.PID_Altitude_LowerSaturationLim) {
    rtb_Sum5 = 0.0F;
  } else {
    rtb_Sum5 -= PositionAttitudeControllerCop_P.PID_Altitude_LowerSaturationLim;
  }

  // End of DeadZone: '<S522>/DeadZone'

  // Gain: '<S527>/Integral Gain'
  rtb_Filter_a *= PositionAttitudeControllerCop_P.PID_Altitude_I;

  // Switch: '<S520>/Switch1' incorporates:
  //   Constant: '<S520>/Clamping_zero'
  //   Constant: '<S520>/Constant'
  //   Constant: '<S520>/Constant2'
  //   RelationalOperator: '<S520>/fix for DT propagation issue'

  if (rtb_Sum5 > PositionAttitudeControllerCop_P.Clamping_zero_Value_e) {
    tmp = PositionAttitudeControllerCop_P.Constant_Value_lr;
  } else {
    tmp = PositionAttitudeControllerCop_P.Constant2_Value_i;
  }

  // Switch: '<S520>/Switch2' incorporates:
  //   Constant: '<S520>/Clamping_zero'
  //   Constant: '<S520>/Constant3'
  //   Constant: '<S520>/Constant4'
  //   RelationalOperator: '<S520>/fix for DT propagation issue1'

  if (rtb_Filter_a > PositionAttitudeControllerCop_P.Clamping_zero_Value_e) {
    tmp_0 = PositionAttitudeControllerCop_P.Constant3_Value_e;
  } else {
    tmp_0 = PositionAttitudeControllerCop_P.Constant4_Value_oh;
  }

  // Switch: '<S520>/Switch' incorporates:
  //   Constant: '<S520>/Clamping_zero'
  //   Constant: '<S520>/Constant1'
  //   Logic: '<S520>/AND3'
  //   RelationalOperator: '<S520>/Equal1'
  //   RelationalOperator: '<S520>/Relational Operator'
  //   Switch: '<S520>/Switch1'
  //   Switch: '<S520>/Switch2'

  if ((PositionAttitudeControllerCop_P.Clamping_zero_Value_e != rtb_Sum5) &&
      (tmp == tmp_0)) {
    rtb_Filter_a = PositionAttitudeControllerCop_P.Constant1_Value_p;
  }

  // DiscreteIntegrator: '<S530>/Integrator' incorporates:
  //   Switch: '<S520>/Switch'

  PositionAttitudeControllerCo_DW.Integrator_DSTATE_e +=
    PositionAttitudeControllerCop_P.Integrator_gainval_g * rtb_Filter_a;

  // Sum: '<S539>/Sum'
  rtb_Sum5 = (rtb_IntegralGain_ia +
              PositionAttitudeControllerCo_DW.Integrator_DSTATE_e) +
    rtb_FilterCoefficient_h;

  // Saturate: '<S537>/Saturation'
  if (rtb_Sum5 > PositionAttitudeControllerCop_P.PID_Altitude_UpperSaturationLim)
  {
    rtb_Sum5 = PositionAttitudeControllerCop_P.PID_Altitude_UpperSaturationLim;
  } else if (rtb_Sum5 <
             PositionAttitudeControllerCop_P.PID_Altitude_LowerSaturationLim) {
    rtb_Sum5 = PositionAttitudeControllerCop_P.PID_Altitude_LowerSaturationLim;
  }

  // Sum: '<S4>/Sum3' incorporates:
  //   MATLABSystem: '<Root>/Current Velocity'
  //   Saturate: '<S537>/Saturation'
  //   UnaryMinus: '<S4>/Unary Minus'

  rtb_IntegralGain_ia = rtb_Sum5 -
    (-PositionAttitudeControllerCop_B.rtb_CurrentVelocity_k[2]);

  // Gain: '<S589>/Proportional Gain'
  rtb_Sum5 = PositionAttitudeControllerCop_P.PID_vz_P * rtb_IntegralGain_ia;

  // Gain: '<S587>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S579>/Filter'
  //   Gain: '<S577>/Derivative Gain'
  //   Sum: '<S579>/SumD'

  rtb_Filter_a = (PositionAttitudeControllerCop_P.PID_vz_D * rtb_IntegralGain_ia
                  - PositionAttitudeControllerCo_DW.Filter_DSTATE_g) *
    PositionAttitudeControllerCop_P.PID_vz_N;

  // Sum: '<S594>/Sum Fdbk'
  rtb_DeadZone_i = (rtb_Sum5 +
                    PositionAttitudeControllerCo_DW.Integrator_DSTATE_l) +
    rtb_Filter_a;

  // DeadZone: '<S576>/DeadZone'
  if (rtb_DeadZone_i >
      PositionAttitudeControllerCop_P.PID_vz_UpperSaturationLimit) {
    rtb_DeadZone_i -=
      PositionAttitudeControllerCop_P.PID_vz_UpperSaturationLimit;
  } else if (rtb_DeadZone_i >=
             PositionAttitudeControllerCop_P.PID_vz_LowerSaturationLimit) {
    rtb_DeadZone_i = 0.0F;
  } else {
    rtb_DeadZone_i -=
      PositionAttitudeControllerCop_P.PID_vz_LowerSaturationLimit;
  }

  // End of DeadZone: '<S576>/DeadZone'

  // Gain: '<S581>/Integral Gain'
  rtb_IntegralGain_ia *= PositionAttitudeControllerCop_P.PID_vz_I;

  // Switch: '<S574>/Switch1' incorporates:
  //   Constant: '<S574>/Clamping_zero'
  //   Constant: '<S574>/Constant'
  //   Constant: '<S574>/Constant2'
  //   RelationalOperator: '<S574>/fix for DT propagation issue'

  if (rtb_DeadZone_i > PositionAttitudeControllerCop_P.Clamping_zero_Value_d) {
    tmp = PositionAttitudeControllerCop_P.Constant_Value_h;
  } else {
    tmp = PositionAttitudeControllerCop_P.Constant2_Value_c;
  }

  // Switch: '<S574>/Switch2' incorporates:
  //   Constant: '<S574>/Clamping_zero'
  //   Constant: '<S574>/Constant3'
  //   Constant: '<S574>/Constant4'
  //   RelationalOperator: '<S574>/fix for DT propagation issue1'

  if (rtb_IntegralGain_ia >
      PositionAttitudeControllerCop_P.Clamping_zero_Value_d) {
    tmp_0 = PositionAttitudeControllerCop_P.Constant3_Value_c;
  } else {
    tmp_0 = PositionAttitudeControllerCop_P.Constant4_Value_a;
  }

  // Switch: '<S574>/Switch' incorporates:
  //   Constant: '<S574>/Clamping_zero'
  //   Constant: '<S574>/Constant1'
  //   Logic: '<S574>/AND3'
  //   RelationalOperator: '<S574>/Equal1'
  //   RelationalOperator: '<S574>/Relational Operator'
  //   Switch: '<S574>/Switch1'
  //   Switch: '<S574>/Switch2'

  if ((PositionAttitudeControllerCop_P.Clamping_zero_Value_d != rtb_DeadZone_i) &&
      (tmp == tmp_0)) {
    rtb_IntegralGain_ia = PositionAttitudeControllerCop_P.Constant1_Value_h;
  }

  // DiscreteIntegrator: '<S584>/Integrator' incorporates:
  //   Switch: '<S574>/Switch'

  PositionAttitudeControllerCo_DW.Integrator_DSTATE_l +=
    PositionAttitudeControllerCop_P.Integrator_gainval_e * rtb_IntegralGain_ia;

  // Saturate: '<S163>/Saturation'
  if (rtb_DeadZone_p >
      PositionAttitudeControllerCop_P.PIDController2_UpperSaturatio_k) {
    rtb_IntegralGain_ia =
      PositionAttitudeControllerCop_P.PIDController2_UpperSaturatio_k;
  } else if (rtb_DeadZone_p <
             PositionAttitudeControllerCop_P.PIDController2_LowerSaturatio_o) {
    rtb_IntegralGain_ia =
      PositionAttitudeControllerCop_P.PIDController2_LowerSaturatio_o;
  } else {
    rtb_IntegralGain_ia = rtb_DeadZone_p;
  }

  // Saturate: '<S109>/Saturation'
  if (rtb_DeadZone_o >
      PositionAttitudeControllerCop_P.PIDController1_UpperSaturatio_p) {
    rtb_DeadZone_i =
      PositionAttitudeControllerCop_P.PIDController1_UpperSaturatio_p;
  } else if (rtb_DeadZone_o <
             PositionAttitudeControllerCop_P.PIDController1_LowerSaturatio_l) {
    rtb_DeadZone_i =
      PositionAttitudeControllerCop_P.PIDController1_LowerSaturatio_l;
  } else {
    rtb_DeadZone_i = rtb_DeadZone_o;
  }

  // Saturate: '<S55>/Saturation'
  if (rtb_DeadZone >
      PositionAttitudeControllerCop_P.PIDController_UpperSaturationLi) {
    u0 = PositionAttitudeControllerCop_P.PIDController_UpperSaturationLi;
  } else if (rtb_DeadZone <
             PositionAttitudeControllerCop_P.PIDController_LowerSaturationLi) {
    u0 = PositionAttitudeControllerCop_P.PIDController_LowerSaturationLi;
  } else {
    u0 = rtb_DeadZone;
  }

  // MATLABSystem: '<S2>/TTSystemObject' incorporates:
  //   DataTypeConversion: '<S1>/Data Type Conversion1'
  //   Saturate: '<S109>/Saturation'
  //   Saturate: '<S163>/Saturation'
  //   Saturate: '<S55>/Saturation'

  MW_setTorque(rtb_IntegralGain_ia, rtb_DeadZone_i, static_cast<real32_T>(u0));

  // Sum: '<S593>/Sum'
  rtb_Sum5 = (rtb_Sum5 + PositionAttitudeControllerCo_DW.Integrator_DSTATE_l) +
    rtb_Filter_a;

  // Saturate: '<S591>/Saturation'
  if (rtb_Sum5 > PositionAttitudeControllerCop_P.PID_vz_UpperSaturationLimit) {
    rtb_Sum5 = PositionAttitudeControllerCop_P.PID_vz_UpperSaturationLimit;
  } else if (rtb_Sum5 <
             PositionAttitudeControllerCop_P.PID_vz_LowerSaturationLimit) {
    rtb_Sum5 = PositionAttitudeControllerCop_P.PID_vz_LowerSaturationLimit;
  }

  // MATLABSystem: '<S2>/TTSystemObject' incorporates:
  //   Saturate: '<S591>/Saturation'

  MW_setThrust(rtb_Sum5);

  // DeadZone: '<S40>/DeadZone'
  if (rtb_DeadZone >
      PositionAttitudeControllerCop_P.PIDController_UpperSaturationLi) {
    rtb_DeadZone -=
      PositionAttitudeControllerCop_P.PIDController_UpperSaturationLi;
  } else if (rtb_DeadZone >=
             PositionAttitudeControllerCop_P.PIDController_LowerSaturationLi) {
    rtb_DeadZone = 0.0;
  } else {
    rtb_DeadZone -=
      PositionAttitudeControllerCop_P.PIDController_LowerSaturationLi;
  }

  // End of DeadZone: '<S40>/DeadZone'

  // Gain: '<S45>/Integral Gain'
  PositionAttitudeControllerCop_B.IntegralGain *=
    PositionAttitudeControllerCop_P.PIDController_I;

  // DeadZone: '<S94>/DeadZone'
  if (rtb_DeadZone_o >
      PositionAttitudeControllerCop_P.PIDController1_UpperSaturatio_p) {
    rtb_DeadZone_o -=
      PositionAttitudeControllerCop_P.PIDController1_UpperSaturatio_p;
  } else if (rtb_DeadZone_o >=
             PositionAttitudeControllerCop_P.PIDController1_LowerSaturatio_l) {
    rtb_DeadZone_o = 0.0F;
  } else {
    rtb_DeadZone_o -=
      PositionAttitudeControllerCop_P.PIDController1_LowerSaturatio_l;
  }

  // End of DeadZone: '<S94>/DeadZone'

  // Gain: '<S99>/Integral Gain'
  rtb_IntegralGain_h *= PositionAttitudeControllerCop_P.PIDController1_I;

  // DeadZone: '<S148>/DeadZone'
  if (rtb_DeadZone_p >
      PositionAttitudeControllerCop_P.PIDController2_UpperSaturatio_k) {
    rtb_DeadZone_p -=
      PositionAttitudeControllerCop_P.PIDController2_UpperSaturatio_k;
  } else if (rtb_DeadZone_p >=
             PositionAttitudeControllerCop_P.PIDController2_LowerSaturatio_o) {
    rtb_DeadZone_p = 0.0F;
  } else {
    rtb_DeadZone_p -=
      PositionAttitudeControllerCop_P.PIDController2_LowerSaturatio_o;
  }

  // End of DeadZone: '<S148>/DeadZone'

  // Gain: '<S153>/Integral Gain'
  rtb_Filter_nf *= PositionAttitudeControllerCop_P.PIDController2_I;

  // Switch: '<S146>/Switch1' incorporates:
  //   Constant: '<S146>/Clamping_zero'
  //   Constant: '<S146>/Constant'
  //   Constant: '<S146>/Constant2'
  //   RelationalOperator: '<S146>/fix for DT propagation issue'

  if (rtb_DeadZone_p > PositionAttitudeControllerCop_P.Clamping_zero_Value_dm) {
    tmp = PositionAttitudeControllerCop_P.Constant_Value_a;
  } else {
    tmp = PositionAttitudeControllerCop_P.Constant2_Value_b;
  }

  // Switch: '<S146>/Switch2' incorporates:
  //   Constant: '<S146>/Clamping_zero'
  //   Constant: '<S146>/Constant3'
  //   Constant: '<S146>/Constant4'
  //   RelationalOperator: '<S146>/fix for DT propagation issue1'

  if (rtb_Filter_nf > PositionAttitudeControllerCop_P.Clamping_zero_Value_dm) {
    tmp_0 = PositionAttitudeControllerCop_P.Constant3_Value_k;
  } else {
    tmp_0 = PositionAttitudeControllerCop_P.Constant4_Value_o;
  }

  // Switch: '<S146>/Switch' incorporates:
  //   Constant: '<S146>/Clamping_zero'
  //   Constant: '<S146>/Constant1'
  //   Logic: '<S146>/AND3'
  //   RelationalOperator: '<S146>/Equal1'
  //   RelationalOperator: '<S146>/Relational Operator'
  //   Switch: '<S146>/Switch1'
  //   Switch: '<S146>/Switch2'

  if ((PositionAttitudeControllerCop_P.Clamping_zero_Value_dm != rtb_DeadZone_p)
      && (tmp == tmp_0)) {
    rtb_Filter_nf = PositionAttitudeControllerCop_P.Constant1_Value_b;
  }

  // Update for DiscreteIntegrator: '<S156>/Integrator' incorporates:
  //   Switch: '<S146>/Switch'

  PositionAttitudeControllerCo_DW.Integrator_DSTATE_m +=
    PositionAttitudeControllerCop_P.Integrator_gainval_i * rtb_Filter_nf;

  // Update for DiscreteIntegrator: '<S151>/Filter'
  PositionAttitudeControllerCo_DW.Filter_DSTATE_d +=
    PositionAttitudeControllerCop_P.Filter_gainval_p * rtb_Integrator_p;

  // Switch: '<S92>/Switch1' incorporates:
  //   Constant: '<S92>/Clamping_zero'
  //   Constant: '<S92>/Constant'
  //   Constant: '<S92>/Constant2'
  //   RelationalOperator: '<S92>/fix for DT propagation issue'

  if (rtb_DeadZone_o > PositionAttitudeControllerCop_P.Clamping_zero_Value_f) {
    tmp = PositionAttitudeControllerCop_P.Constant_Value_l;
  } else {
    tmp = PositionAttitudeControllerCop_P.Constant2_Value_l;
  }

  // Switch: '<S92>/Switch2' incorporates:
  //   Constant: '<S92>/Clamping_zero'
  //   Constant: '<S92>/Constant3'
  //   Constant: '<S92>/Constant4'
  //   RelationalOperator: '<S92>/fix for DT propagation issue1'

  if (rtb_IntegralGain_h > PositionAttitudeControllerCop_P.Clamping_zero_Value_f)
  {
    tmp_0 = PositionAttitudeControllerCop_P.Constant3_Value_m;
  } else {
    tmp_0 = PositionAttitudeControllerCop_P.Constant4_Value_k;
  }

  // Switch: '<S92>/Switch' incorporates:
  //   Constant: '<S92>/Clamping_zero'
  //   Constant: '<S92>/Constant1'
  //   Logic: '<S92>/AND3'
  //   RelationalOperator: '<S92>/Equal1'
  //   RelationalOperator: '<S92>/Relational Operator'
  //   Switch: '<S92>/Switch1'
  //   Switch: '<S92>/Switch2'

  if ((PositionAttitudeControllerCop_P.Clamping_zero_Value_f != rtb_DeadZone_o) &&
      (tmp == tmp_0)) {
    rtb_IntegralGain_h = PositionAttitudeControllerCop_P.Constant1_Value_e;
  }

  // Update for DiscreteIntegrator: '<S102>/Integrator' incorporates:
  //   Switch: '<S92>/Switch'

  PositionAttitudeControllerCo_DW.Integrator_DSTATE_c +=
    PositionAttitudeControllerCop_P.Integrator_gainval_l * rtb_IntegralGain_h;

  // Update for DiscreteIntegrator: '<S97>/Filter'
  PositionAttitudeControllerCo_DW.Filter_DSTATE_dx +=
    PositionAttitudeControllerCop_P.Filter_gainval_d * rtb_FilterCoefficient_b;

  // Switch: '<S38>/Switch1' incorporates:
  //   Constant: '<S38>/Clamping_zero'
  //   Constant: '<S38>/Constant'
  //   Constant: '<S38>/Constant2'
  //   RelationalOperator: '<S38>/fix for DT propagation issue'

  if (rtb_DeadZone > PositionAttitudeControllerCop_P.Clamping_zero_Value) {
    tmp = PositionAttitudeControllerCop_P.Constant_Value_if;
  } else {
    tmp = PositionAttitudeControllerCop_P.Constant2_Value;
  }

  // Switch: '<S38>/Switch2' incorporates:
  //   Constant: '<S38>/Clamping_zero'
  //   Constant: '<S38>/Constant3'
  //   Constant: '<S38>/Constant4'
  //   RelationalOperator: '<S38>/fix for DT propagation issue1'

  if (PositionAttitudeControllerCop_B.IntegralGain >
      PositionAttitudeControllerCop_P.Clamping_zero_Value) {
    tmp_0 = PositionAttitudeControllerCop_P.Constant3_Value;
  } else {
    tmp_0 = PositionAttitudeControllerCop_P.Constant4_Value;
  }

  // Switch: '<S38>/Switch' incorporates:
  //   Constant: '<S38>/Clamping_zero'
  //   Constant: '<S38>/Constant1'
  //   Logic: '<S38>/AND3'
  //   RelationalOperator: '<S38>/Equal1'
  //   RelationalOperator: '<S38>/Relational Operator'
  //   Switch: '<S38>/Switch1'
  //   Switch: '<S38>/Switch2'

  if ((PositionAttitudeControllerCop_P.Clamping_zero_Value != rtb_DeadZone) &&
      (tmp == tmp_0)) {
    PositionAttitudeControllerCop_B.IntegralGain =
      PositionAttitudeControllerCop_P.Constant1_Value;
  }

  // Update for DiscreteIntegrator: '<S48>/Integrator' incorporates:
  //   Switch: '<S38>/Switch'

  PositionAttitudeControllerCo_DW.Integrator_DSTATE +=
    PositionAttitudeControllerCop_P.Integrator_gainval *
    PositionAttitudeControllerCop_B.IntegralGain;

  // Update for DiscreteIntegrator: '<S43>/Filter'
  PositionAttitudeControllerCo_DW.Filter_DSTATE +=
    PositionAttitudeControllerCop_P.Filter_gainval *
    PositionAttitudeControllerCop_B.FilterCoefficient;

  // Update for DiscreteIntegrator: '<S525>/Filter'
  PositionAttitudeControllerCo_DW.Filter_DSTATE_j +=
    PositionAttitudeControllerCop_P.Filter_gainval_dy * rtb_FilterCoefficient_h;

  // Update for DiscreteIntegrator: '<S579>/Filter'
  PositionAttitudeControllerCo_DW.Filter_DSTATE_g +=
    PositionAttitudeControllerCop_P.Filter_gainval_i * rtb_Filter_a;
}

// Model initialize function
void PositionAttitudeControllerCopter_initialize(void)
{
  // InitializeConditions for RateLimiter: '<S3>/Rate Limiter1'
  PositionAttitudeControllerCo_DW.PrevY =
    PositionAttitudeControllerCop_P.RateLimiter1_IC;

  // InitializeConditions for DiscreteIntegrator: '<S156>/Integrator'
  PositionAttitudeControllerCo_DW.Integrator_DSTATE_m =
    PositionAttitudeControllerCop_P.PIDController2_InitialConditi_c;

  // InitializeConditions for DiscreteIntegrator: '<S151>/Filter'
  PositionAttitudeControllerCo_DW.Filter_DSTATE_d =
    PositionAttitudeControllerCop_P.PIDController2_InitialCondition;

  // InitializeConditions for RateLimiter: '<S3>/Rate Limiter2'
  PositionAttitudeControllerCo_DW.PrevY_f =
    PositionAttitudeControllerCop_P.RateLimiter2_IC;

  // InitializeConditions for DiscreteIntegrator: '<S102>/Integrator'
  PositionAttitudeControllerCo_DW.Integrator_DSTATE_c =
    PositionAttitudeControllerCop_P.PIDController1_InitialConditi_l;

  // InitializeConditions for DiscreteIntegrator: '<S97>/Filter'
  PositionAttitudeControllerCo_DW.Filter_DSTATE_dx =
    PositionAttitudeControllerCop_P.PIDController1_InitialCondition;

  // InitializeConditions for DiscreteIntegrator: '<S48>/Integrator'
  PositionAttitudeControllerCo_DW.Integrator_DSTATE =
    PositionAttitudeControllerCop_P.PIDController_InitialConditio_g;

  // InitializeConditions for DiscreteIntegrator: '<S43>/Filter'
  PositionAttitudeControllerCo_DW.Filter_DSTATE =
    PositionAttitudeControllerCop_P.PIDController_InitialConditionF;

  // InitializeConditions for DiscreteIntegrator: '<S525>/Filter'
  PositionAttitudeControllerCo_DW.Filter_DSTATE_j =
    PositionAttitudeControllerCop_P.PID_Altitude_InitialConditionFo;

  // InitializeConditions for DiscreteIntegrator: '<S530>/Integrator'
  PositionAttitudeControllerCo_DW.Integrator_DSTATE_e =
    PositionAttitudeControllerCop_P.PID_Altitude_InitialCondition_d;

  // InitializeConditions for DiscreteIntegrator: '<S579>/Filter'
  PositionAttitudeControllerCo_DW.Filter_DSTATE_g =
    PositionAttitudeControllerCop_P.PID_vz_InitialConditionForFilte;

  // InitializeConditions for DiscreteIntegrator: '<S584>/Integrator'
  PositionAttitudeControllerCo_DW.Integrator_DSTATE_l =
    PositionAttitudeControllerCop_P.PID_vz_InitialConditionForInteg;

  // Start for MATLABSystem: '<Root>/Current Attitude'
  PositionAttitudeControllerCo_DW.obj_d.matlabCodegenIsDeleted = false;
  PositionAttitudeControllerCo_DW.obj_d.isInitialized = 1;
  MW_EstimatorOutputs_init();
  PositionAttitudeControllerCo_DW.obj_d.isSetupComplete = true;

  // Start for MATLABSystem: '<Root>/Position Setpoint'
  PositionAttitudeControllerCo_DW.obj_d5.matlabCodegenIsDeleted = false;
  PositionAttitudeControllerCo_DW.obj_d5.isInitialized = 1;
  MW_AttPosControlInputs_init();
  PositionAttitudeControllerCo_DW.obj_d5.isSetupComplete = true;

  // Start for MATLABSystem: '<Root>/Current Position'
  PositionAttitudeControllerCo_DW.obj_n.matlabCodegenIsDeleted = false;
  PositionAttitudeControllerCo_DW.obj_n.isInitialized = 1;
  MW_EstimatorOutputs_init();
  PositionAttitudeControllerCo_DW.obj_n.isSetupComplete = true;

  // Start for MATLABSystem: '<Root>/Current Velocity'
  PositionAttitudeControllerCo_DW.obj_f.matlabCodegenIsDeleted = false;
  PositionAttitudeControllerCo_DW.obj_f.isInitialized = 1;
  MW_EstimatorOutputs_init();
  PositionAttitudeControllerCo_DW.obj_f.isSetupComplete = true;

  // Start for MATLABSystem: '<Root>/Velocity Setpoint'
  PositionAttitudeControllerCo_DW.obj_nb.matlabCodegenIsDeleted = false;
  PositionAttitudeControllerCo_DW.obj_nb.isInitialized = 1;
  MW_AttPosControlInputs_init();
  PositionAttitudeControllerCo_DW.obj_nb.isSetupComplete = true;

  // Start for MATLABSystem: '<Root>/Current Ang Velocity'
  PositionAttitudeControllerCo_DW.obj_o.matlabCodegenIsDeleted = false;
  PositionAttitudeControllerCo_DW.obj_o.isInitialized = 1;
  MW_EstimatorOutputs_init();
  PositionAttitudeControllerCo_DW.obj_o.isSetupComplete = true;

  // Start for MATLABSystem: '<Root>/Attitude Setpoint'
  PositionAttitudeControllerCo_DW.obj.matlabCodegenIsDeleted = false;
  PositionAttitudeControllerCo_DW.obj.isInitialized = 1;
  MW_AttPosControlInputs_init();
  PositionAttitudeControllerCo_DW.obj.isSetupComplete = true;

  // Start for MATLABSystem: '<S2>/TTSystemObject'
  PositionAttitudeControllerCo_DW.obj_l.matlabCodegenIsDeleted = false;
  PositionAttitudeControllerCo_DW.obj_l.isInitialized = 1;
  MW_MixerInputs_init();
  PositionAttitudeControllerCo_DW.obj_l.isSetupComplete = true;
}

// Model terminate function
void PositionAttitudeControllerCopter_terminate(void)
{
  // Terminate for MATLABSystem: '<Root>/Current Attitude'
  if (!PositionAttitudeControllerCo_DW.obj_d.matlabCodegenIsDeleted) {
    PositionAttitudeControllerCo_DW.obj_d.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/Current Attitude'

  // Terminate for MATLABSystem: '<Root>/Position Setpoint'
  if (!PositionAttitudeControllerCo_DW.obj_d5.matlabCodegenIsDeleted) {
    PositionAttitudeControllerCo_DW.obj_d5.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/Position Setpoint'

  // Terminate for MATLABSystem: '<Root>/Current Position'
  if (!PositionAttitudeControllerCo_DW.obj_n.matlabCodegenIsDeleted) {
    PositionAttitudeControllerCo_DW.obj_n.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/Current Position'

  // Terminate for MATLABSystem: '<Root>/Current Velocity'
  if (!PositionAttitudeControllerCo_DW.obj_f.matlabCodegenIsDeleted) {
    PositionAttitudeControllerCo_DW.obj_f.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/Current Velocity'

  // Terminate for MATLABSystem: '<Root>/Velocity Setpoint'
  if (!PositionAttitudeControllerCo_DW.obj_nb.matlabCodegenIsDeleted) {
    PositionAttitudeControllerCo_DW.obj_nb.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/Velocity Setpoint'

  // Terminate for MATLABSystem: '<Root>/Current Ang Velocity'
  if (!PositionAttitudeControllerCo_DW.obj_o.matlabCodegenIsDeleted) {
    PositionAttitudeControllerCo_DW.obj_o.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/Current Ang Velocity'

  // Terminate for MATLABSystem: '<Root>/Attitude Setpoint'
  if (!PositionAttitudeControllerCo_DW.obj.matlabCodegenIsDeleted) {
    PositionAttitudeControllerCo_DW.obj.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<Root>/Attitude Setpoint'

  // Terminate for MATLABSystem: '<S2>/TTSystemObject'
  if (!PositionAttitudeControllerCo_DW.obj_l.matlabCodegenIsDeleted) {
    PositionAttitudeControllerCo_DW.obj_l.matlabCodegenIsDeleted = true;
  }

  // End of Terminate for MATLABSystem: '<S2>/TTSystemObject'
}

const char_T* RT_MODEL_PositionAttitudeCont_T::getErrorStatus() const
{
  return (errorStatus);
}

void RT_MODEL_PositionAttitudeCont_T::setErrorStatus(const char_T* const
  volatile aErrorStatus)
{
  (errorStatus = aErrorStatus);
}

//
// File trailer for generated code.
//
// [EOF]
//
