/*
 * File: gc_pid.c
 *
 * Code generated for Simulink model 'gc_pid'.
 *
 * Model version                  : 1.42
 * Simulink Coder version         : 24.1 (R2024a) 19-Nov-2023
 * C/C++ source code generated on : Sat Oct 11 10:15:31 2025
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "gc_pid.h"
#include "rtwtypes.h"
#include <string.h>

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Model step function */
void gc_pid_step(void)
{
    real32_T rtb_DeadZone;
    real32_T rtb_DeadZone_l;
    real32_T rtb_FilterCoefficient_p;
    real32_T rtb_Filter_fk;
    real32_T rtb_Filter_k;
    real32_T rtb_IntegralGain_f;
    real32_T rtb_Integrator_j;
    real32_T rtb_Sum_p4;
    int8_T tmp;
    int8_T tmp_0;
    boolean_T rtb_RelationalOperator_m;

    /* SwitchCase: '<Root>/Switch Case' incorporates:
     *  Inport: '<Root>/ctrl_mode'
     */
    switch (rtU.ctrl_mode) {
      case 1:
        /* Outputs for IfAction SubSystem: '<Root>/speed ctrl' incorporates:
         *  ActionPort: '<S2>/Action Port'
         */
        /* Sum: '<S2>/Sum' incorporates:
         *  Inport: '<Root>/leftRpm_real'
         *  Inport: '<Root>/leftRpm_tar'
         */
        rtb_Filter_k = rtU.leftRpm_tar - rtU.leftRpm_real;

        /* Gain: '<S202>/Filter Coefficient' incorporates:
         *  DiscreteIntegrator: '<S194>/Filter'
         *  Gain: '<S192>/Derivative Gain'
         *  Sum: '<S194>/SumD'
         */
        rtb_FilterCoefficient_p = (rtP.speed_Kd * rtb_Filter_k -
            rtDW.Filter_DSTATE_j) * rtP.LeftRPMPID_N;

        /* Sum: '<S208>/Sum' incorporates:
         *  DiscreteIntegrator: '<S199>/Integrator'
         *  Gain: '<S204>/Proportional Gain'
         */
        rtb_Integrator_j = (rtP.speed_Kp * rtb_Filter_k +
                            rtDW.Integrator_DSTATE_d) + rtb_FilterCoefficient_p;

        /* DeadZone: '<S191>/DeadZone' */
        if (rtb_Integrator_j > rtP.LeftRPMPID_UpperSaturationLimit) {
            rtb_IntegralGain_f = rtb_Integrator_j -
                rtP.LeftRPMPID_UpperSaturationLimit;
        } else if (rtb_Integrator_j >= rtP.LeftRPMPID_LowerSaturationLimit) {
            rtb_IntegralGain_f = 0.0F;
        } else {
            rtb_IntegralGain_f = rtb_Integrator_j -
                rtP.LeftRPMPID_LowerSaturationLimit;
        }

        /* End of DeadZone: '<S191>/DeadZone' */

        /* RelationalOperator: '<S189>/Relational Operator' incorporates:
         *  Constant: '<S189>/Clamping_zero'
         */
        rtb_RelationalOperator_m = (rtP.Clamping_zero_Value !=
            rtb_IntegralGain_f);

        /* Gain: '<S196>/Integral Gain' */
        rtb_Filter_k *= rtP.speed_Ki;

        /* Switch: '<S189>/Switch1' incorporates:
         *  Constant: '<S189>/Clamping_zero'
         *  Constant: '<S189>/Constant'
         *  Constant: '<S189>/Constant2'
         *  RelationalOperator: '<S189>/fix for DT propagation issue'
         */
        if (rtb_IntegralGain_f > rtP.Clamping_zero_Value) {
            tmp = rtP.Constant_Value;
        } else {
            tmp = rtP.Constant2_Value;
        }

        /* Switch: '<S189>/Switch2' incorporates:
         *  Constant: '<S189>/Clamping_zero'
         *  Constant: '<S189>/Constant3'
         *  Constant: '<S189>/Constant4'
         *  RelationalOperator: '<S189>/fix for DT propagation issue1'
         */
        if (rtb_Filter_k > rtP.Clamping_zero_Value) {
            tmp_0 = rtP.Constant3_Value;
        } else {
            tmp_0 = rtP.Constant4_Value;
        }

        /* Saturate: '<S206>/Saturation' */
        if (rtb_Integrator_j > rtP.LeftRPMPID_UpperSaturationLimit) {
            /* Saturate: '<S206>/Saturation' */
            rtDW.Saturation_b = rtP.LeftRPMPID_UpperSaturationLimit;
        } else if (rtb_Integrator_j < rtP.LeftRPMPID_LowerSaturationLimit) {
            /* Saturate: '<S206>/Saturation' */
            rtDW.Saturation_b = rtP.LeftRPMPID_LowerSaturationLimit;
        } else {
            /* Saturate: '<S206>/Saturation' */
            rtDW.Saturation_b = rtb_Integrator_j;
        }

        /* End of Saturate: '<S206>/Saturation' */

        /* Sum: '<S2>/Sum1' incorporates:
         *  Inport: '<Root>/rightRpm_real'
         *  Inport: '<Root>/rightRpm_tar'
         */
        rtb_IntegralGain_f = rtU.rightRpm_tar - rtU.rightRpm_real;

        /* Gain: '<S254>/Filter Coefficient' incorporates:
         *  DiscreteIntegrator: '<S246>/Filter'
         *  Gain: '<S244>/Derivative Gain'
         *  Sum: '<S246>/SumD'
         */
        rtb_Integrator_j = (rtP.speed_Kd * rtb_IntegralGain_f -
                            rtDW.Filter_DSTATE_f) * rtP.RightRPMPID_N;

        /* Sum: '<S260>/Sum' incorporates:
         *  DiscreteIntegrator: '<S251>/Integrator'
         *  Gain: '<S256>/Proportional Gain'
         */
        rtb_Sum_p4 = (rtP.speed_Kp * rtb_IntegralGain_f +
                      rtDW.Integrator_DSTATE_j) + rtb_Integrator_j;

        /* DeadZone: '<S243>/DeadZone' incorporates:
         *  Saturate: '<S258>/Saturation'
         */
        if (rtb_Sum_p4 > rtP.RightRPMPID_UpperSaturationLimi) {
            rtb_DeadZone_l = rtb_Sum_p4 - rtP.RightRPMPID_UpperSaturationLimi;

            /* Saturate: '<S258>/Saturation' */
            rtDW.Saturation_bm = rtP.RightRPMPID_UpperSaturationLimi;
        } else {
            if (rtb_Sum_p4 >= rtP.RightRPMPID_LowerSaturationLimi) {
                rtb_DeadZone_l = 0.0F;
            } else {
                rtb_DeadZone_l = rtb_Sum_p4 -
                    rtP.RightRPMPID_LowerSaturationLimi;
            }

            if (rtb_Sum_p4 < rtP.RightRPMPID_LowerSaturationLimi) {
                /* Saturate: '<S258>/Saturation' */
                rtDW.Saturation_bm = rtP.RightRPMPID_LowerSaturationLimi;
            } else {
                /* Saturate: '<S258>/Saturation' */
                rtDW.Saturation_bm = rtb_Sum_p4;
            }
        }

        /* End of DeadZone: '<S243>/DeadZone' */

        /* Gain: '<S248>/Integral Gain' */
        rtb_IntegralGain_f *= rtP.speed_Ki;

        /* Switch: '<S189>/Switch' incorporates:
         *  Constant: '<S189>/Constant1'
         *  Logic: '<S189>/AND3'
         *  RelationalOperator: '<S189>/Equal1'
         *  Switch: '<S189>/Switch1'
         *  Switch: '<S189>/Switch2'
         */
        if (rtb_RelationalOperator_m && (tmp == tmp_0)) {
            rtb_Filter_k = rtP.Constant1_Value;
        }

        /* Update for DiscreteIntegrator: '<S199>/Integrator' incorporates:
         *  Switch: '<S189>/Switch'
         */
        rtDW.Integrator_DSTATE_d += rtP.Integrator_gainval * rtb_Filter_k;
        if (rtDW.Integrator_DSTATE_d > rtP.speed_iMax) {
            rtDW.Integrator_DSTATE_d = rtP.speed_iMax;
        } else if (rtDW.Integrator_DSTATE_d < rtP.speed_iMin) {
            rtDW.Integrator_DSTATE_d = rtP.speed_iMin;
        }

        /* End of Update for DiscreteIntegrator: '<S199>/Integrator' */

        /* Update for DiscreteIntegrator: '<S194>/Filter' */
        rtDW.Filter_DSTATE_j += rtP.Filter_gainval * rtb_FilterCoefficient_p;

        /* Switch: '<S241>/Switch1' incorporates:
         *  Constant: '<S241>/Clamping_zero'
         *  Constant: '<S241>/Constant'
         *  Constant: '<S241>/Constant2'
         *  RelationalOperator: '<S241>/fix for DT propagation issue'
         */
        if (rtb_DeadZone_l > rtP.Clamping_zero_Value_f) {
            tmp = rtP.Constant_Value_j;
        } else {
            tmp = rtP.Constant2_Value_j;
        }

        /* Switch: '<S241>/Switch2' incorporates:
         *  Constant: '<S241>/Clamping_zero'
         *  Constant: '<S241>/Constant3'
         *  Constant: '<S241>/Constant4'
         *  RelationalOperator: '<S241>/fix for DT propagation issue1'
         */
        if (rtb_IntegralGain_f > rtP.Clamping_zero_Value_f) {
            tmp_0 = rtP.Constant3_Value_g;
        } else {
            tmp_0 = rtP.Constant4_Value_h;
        }

        /* Switch: '<S241>/Switch' incorporates:
         *  Constant: '<S241>/Clamping_zero'
         *  Constant: '<S241>/Constant1'
         *  Logic: '<S241>/AND3'
         *  RelationalOperator: '<S241>/Equal1'
         *  RelationalOperator: '<S241>/Relational Operator'
         *  Switch: '<S241>/Switch1'
         *  Switch: '<S241>/Switch2'
         */
        if ((rtP.Clamping_zero_Value_f != rtb_DeadZone_l) && (tmp == tmp_0)) {
            rtb_IntegralGain_f = rtP.Constant1_Value_f;
        }

        /* Update for DiscreteIntegrator: '<S251>/Integrator' incorporates:
         *  Switch: '<S241>/Switch'
         */
        rtDW.Integrator_DSTATE_j += rtP.Integrator_gainval_g *
            rtb_IntegralGain_f;
        if (rtDW.Integrator_DSTATE_j > rtP.speed_iMax) {
            rtDW.Integrator_DSTATE_j = rtP.speed_iMax;
        } else if (rtDW.Integrator_DSTATE_j < rtP.speed_iMin) {
            rtDW.Integrator_DSTATE_j = rtP.speed_iMin;
        }

        /* End of Update for DiscreteIntegrator: '<S251>/Integrator' */

        /* Update for DiscreteIntegrator: '<S246>/Filter' */
        rtDW.Filter_DSTATE_f += rtP.Filter_gainval_i * rtb_Integrator_j;

        /* End of Outputs for SubSystem: '<Root>/speed ctrl' */
        break;

      case 2:
        /* Outputs for IfAction SubSystem: '<Root>/angle ctrl' incorporates:
         *  ActionPort: '<S1>/Action Port'
         */
        /* Sum: '<S1>/Sum' incorporates:
         *  Inport: '<Root>/angle_real'
         *  Inport: '<Root>/angle_tar'
         */
        rtb_Filter_k = rtU.angle_tar - rtU.angle_real;

        /* MATLAB Function: '<S1>/ang correction' */
        if (rtb_Filter_k > 180.0F) {
            rtb_Filter_k -= 360.0F;
        } else if (rtb_Filter_k < -180.0F) {
            rtb_Filter_k += 360.0F;
        }

        /* End of MATLAB Function: '<S1>/ang correction' */

        /* Gain: '<S96>/Filter Coefficient' incorporates:
         *  DiscreteIntegrator: '<S88>/Filter'
         *  Gain: '<S86>/Derivative Gain'
         *  Sum: '<S88>/SumD'
         */
        rtb_FilterCoefficient_p = (rtP.angle_A_Kd * rtb_Filter_k -
            rtDW.Filter_DSTATE_d) * rtP.positionPID_N;

        /* Sum: '<S102>/Sum' incorporates:
         *  DiscreteIntegrator: '<S93>/Integrator'
         *  Gain: '<S98>/Proportional Gain'
         */
        rtb_IntegralGain_f = (rtP.angle_A_Kp * rtb_Filter_k +
                              rtDW.Integrator_DSTATE_h) +
            rtb_FilterCoefficient_p;

        /* Sum: '<S1>/Sum1' incorporates:
         *  Inport: '<Root>/leftRpm_real'
         */
        rtb_Integrator_j = rtb_IntegralGain_f - rtU.leftRpm_real;

        /* Sum: '<S1>/Sum2' incorporates:
         *  Inport: '<Root>/rightRpm_real'
         */
        rtb_IntegralGain_f -= rtU.rightRpm_real;

        /* Gain: '<S46>/Filter Coefficient' incorporates:
         *  DiscreteIntegrator: '<S38>/Filter'
         *  Gain: '<S36>/Derivative Gain'
         *  Sum: '<S38>/SumD'
         */
        rtb_Sum_p4 = (rtP.angle_S_Kd * rtb_Integrator_j - rtDW.Filter_DSTATE_k) *
            rtP.leftSpeedPID_N;

        /* Sum: '<S52>/Sum' incorporates:
         *  DiscreteIntegrator: '<S43>/Integrator'
         *  Gain: '<S48>/Proportional Gain'
         */
        rtb_Filter_fk = (rtP.angle_S_Kp * rtb_Integrator_j +
                         rtDW.Integrator_DSTATE_o) + rtb_Sum_p4;

        /* DeadZone: '<S35>/DeadZone' */
        if (rtb_Filter_fk > rtP.leftSpeedPID_UpperSaturationLim) {
            rtb_DeadZone_l = rtb_Filter_fk - rtP.leftSpeedPID_UpperSaturationLim;
        } else if (rtb_Filter_fk >= rtP.leftSpeedPID_LowerSaturationLim) {
            rtb_DeadZone_l = 0.0F;
        } else {
            rtb_DeadZone_l = rtb_Filter_fk - rtP.leftSpeedPID_LowerSaturationLim;
        }

        /* End of DeadZone: '<S35>/DeadZone' */

        /* Gain: '<S40>/Integral Gain' */
        rtb_Integrator_j *= rtP.angle_S_Ki;

        /* Switch: '<S33>/Switch1' incorporates:
         *  Constant: '<S33>/Clamping_zero'
         *  Constant: '<S33>/Constant'
         *  Constant: '<S33>/Constant2'
         *  RelationalOperator: '<S33>/fix for DT propagation issue'
         */
        if (rtb_DeadZone_l > rtP.Clamping_zero_Value_g) {
            tmp = rtP.Constant_Value_d;
        } else {
            tmp = rtP.Constant2_Value_o;
        }

        /* Switch: '<S33>/Switch2' incorporates:
         *  Constant: '<S33>/Clamping_zero'
         *  Constant: '<S33>/Constant3'
         *  Constant: '<S33>/Constant4'
         *  RelationalOperator: '<S33>/fix for DT propagation issue1'
         */
        if (rtb_Integrator_j > rtP.Clamping_zero_Value_g) {
            tmp_0 = rtP.Constant3_Value_gp;
        } else {
            tmp_0 = rtP.Constant4_Value_hw;
        }

        /* Switch: '<S33>/Switch' incorporates:
         *  Constant: '<S33>/Clamping_zero'
         *  Constant: '<S33>/Constant1'
         *  Logic: '<S33>/AND3'
         *  RelationalOperator: '<S33>/Equal1'
         *  RelationalOperator: '<S33>/Relational Operator'
         *  Switch: '<S33>/Switch1'
         *  Switch: '<S33>/Switch2'
         */
        if ((rtP.Clamping_zero_Value_g != rtb_DeadZone_l) && (tmp == tmp_0)) {
            rtb_DeadZone_l = rtP.Constant1_Value_h;
        } else {
            rtb_DeadZone_l = rtb_Integrator_j;
        }

        /* End of Switch: '<S33>/Switch' */

        /* Saturate: '<S50>/Saturation' */
        if (rtb_Filter_fk > rtP.leftSpeedPID_UpperSaturationLim) {
            /* Saturate: '<S50>/Saturation' */
            rtDW.Saturation = rtP.leftSpeedPID_UpperSaturationLim;
        } else if (rtb_Filter_fk < rtP.leftSpeedPID_LowerSaturationLim) {
            /* Saturate: '<S50>/Saturation' */
            rtDW.Saturation = rtP.leftSpeedPID_LowerSaturationLim;
        } else {
            /* Saturate: '<S50>/Saturation' */
            rtDW.Saturation = rtb_Filter_fk;
        }

        /* End of Saturate: '<S50>/Saturation' */

        /* Gain: '<S148>/Filter Coefficient' incorporates:
         *  DiscreteIntegrator: '<S140>/Filter'
         *  Gain: '<S138>/Derivative Gain'
         *  Sum: '<S140>/SumD'
         */
        rtb_Filter_fk = (rtP.angle_S_Kd * rtb_IntegralGain_f -
                         rtDW.Filter_DSTATE_e) * rtP.rightSpeedPID_N;

        /* Sum: '<S154>/Sum' incorporates:
         *  DiscreteIntegrator: '<S145>/Integrator'
         *  Gain: '<S150>/Proportional Gain'
         */
        rtb_Integrator_j = (rtP.angle_S_Kp * rtb_IntegralGain_f +
                            rtDW.Integrator_DSTATE_f) + rtb_Filter_fk;

        /* DeadZone: '<S137>/DeadZone' incorporates:
         *  Saturate: '<S152>/Saturation'
         */
        if (rtb_Integrator_j > rtP.rightSpeedPID_UpperSaturationLi) {
            rtb_DeadZone = rtb_Integrator_j -
                rtP.rightSpeedPID_UpperSaturationLi;

            /* Saturate: '<S152>/Saturation' */
            rtDW.Saturation_i = rtP.rightSpeedPID_UpperSaturationLi;
        } else {
            if (rtb_Integrator_j >= rtP.rightSpeedPID_LowerSaturationLi) {
                rtb_DeadZone = 0.0F;
            } else {
                rtb_DeadZone = rtb_Integrator_j -
                    rtP.rightSpeedPID_LowerSaturationLi;
            }

            if (rtb_Integrator_j < rtP.rightSpeedPID_LowerSaturationLi) {
                /* Saturate: '<S152>/Saturation' */
                rtDW.Saturation_i = rtP.rightSpeedPID_LowerSaturationLi;
            } else {
                /* Saturate: '<S152>/Saturation' */
                rtDW.Saturation_i = rtb_Integrator_j;
            }
        }

        /* End of DeadZone: '<S137>/DeadZone' */

        /* Gain: '<S142>/Integral Gain' */
        rtb_IntegralGain_f *= rtP.angle_S_Ki;

        /* Update for DiscreteIntegrator: '<S93>/Integrator' incorporates:
         *  Gain: '<S90>/Integral Gain'
         */
        rtDW.Integrator_DSTATE_h += rtP.angle_A_Ki * rtb_Filter_k *
            rtP.Integrator_gainval_m;
        if (rtDW.Integrator_DSTATE_h > rtP.angle_A_iMax) {
            rtDW.Integrator_DSTATE_h = rtP.angle_A_iMax;
        } else if (rtDW.Integrator_DSTATE_h < rtP.angle_A_iMin) {
            rtDW.Integrator_DSTATE_h = rtP.angle_A_iMin;
        }

        /* End of Update for DiscreteIntegrator: '<S93>/Integrator' */

        /* Update for DiscreteIntegrator: '<S88>/Filter' */
        rtDW.Filter_DSTATE_d += rtP.Filter_gainval_m * rtb_FilterCoefficient_p;

        /* Update for DiscreteIntegrator: '<S43>/Integrator' */
        rtDW.Integrator_DSTATE_o += rtP.Integrator_gainval_a * rtb_DeadZone_l;
        if (rtDW.Integrator_DSTATE_o > rtP.angle_S_iMax) {
            rtDW.Integrator_DSTATE_o = rtP.angle_S_iMax;
        } else if (rtDW.Integrator_DSTATE_o < rtP.angle_S_iMin) {
            rtDW.Integrator_DSTATE_o = rtP.angle_S_iMin;
        }

        /* End of Update for DiscreteIntegrator: '<S43>/Integrator' */

        /* Update for DiscreteIntegrator: '<S38>/Filter' */
        rtDW.Filter_DSTATE_k += rtP.Filter_gainval_o * rtb_Sum_p4;

        /* Switch: '<S135>/Switch1' incorporates:
         *  Constant: '<S135>/Clamping_zero'
         *  Constant: '<S135>/Constant'
         *  Constant: '<S135>/Constant2'
         *  RelationalOperator: '<S135>/fix for DT propagation issue'
         */
        if (rtb_DeadZone > rtP.Clamping_zero_Value_i) {
            tmp = rtP.Constant_Value_i;
        } else {
            tmp = rtP.Constant2_Value_n;
        }

        /* Switch: '<S135>/Switch2' incorporates:
         *  Constant: '<S135>/Clamping_zero'
         *  Constant: '<S135>/Constant3'
         *  Constant: '<S135>/Constant4'
         *  RelationalOperator: '<S135>/fix for DT propagation issue1'
         */
        if (rtb_IntegralGain_f > rtP.Clamping_zero_Value_i) {
            tmp_0 = rtP.Constant3_Value_c;
        } else {
            tmp_0 = rtP.Constant4_Value_n;
        }

        /* Switch: '<S135>/Switch' incorporates:
         *  Constant: '<S135>/Clamping_zero'
         *  Constant: '<S135>/Constant1'
         *  Logic: '<S135>/AND3'
         *  RelationalOperator: '<S135>/Equal1'
         *  RelationalOperator: '<S135>/Relational Operator'
         *  Switch: '<S135>/Switch1'
         *  Switch: '<S135>/Switch2'
         */
        if ((rtP.Clamping_zero_Value_i != rtb_DeadZone) && (tmp == tmp_0)) {
            rtb_IntegralGain_f = rtP.Constant1_Value_m;
        }

        /* Update for DiscreteIntegrator: '<S145>/Integrator' incorporates:
         *  Switch: '<S135>/Switch'
         */
        rtDW.Integrator_DSTATE_f += rtP.Integrator_gainval_l *
            rtb_IntegralGain_f;
        if (rtDW.Integrator_DSTATE_f > rtP.angle_S_iMax) {
            rtDW.Integrator_DSTATE_f = rtP.angle_S_iMax;
        } else if (rtDW.Integrator_DSTATE_f < rtP.angle_S_iMin) {
            rtDW.Integrator_DSTATE_f = rtP.angle_S_iMin;
        }

        /* End of Update for DiscreteIntegrator: '<S145>/Integrator' */

        /* Update for DiscreteIntegrator: '<S140>/Filter' */
        rtDW.Filter_DSTATE_e += rtP.Filter_gainval_p * rtb_Filter_fk;

        /* End of Outputs for SubSystem: '<Root>/angle ctrl' */
        break;
    }

    /* End of SwitchCase: '<Root>/Switch Case' */

    /* MultiPortSwitch: '<Root>/Index Vector1' incorporates:
     *  Inport: '<Root>/ctrl_mode'
     */
    if (rtU.ctrl_mode == 1) {
        /* Outport: '<Root>/leftRpm_output' */
        rtY.leftRpm_output = rtDW.Saturation_b;

        /* Outport: '<Root>/rightRpm_output' incorporates:
         *  MultiPortSwitch: '<Root>/Index Vector'
         */
        rtY.rightRpm_output = rtDW.Saturation_bm;
    } else {
        /* Outport: '<Root>/leftRpm_output' */
        rtY.leftRpm_output = rtDW.Saturation;

        /* Outport: '<Root>/rightRpm_output' incorporates:
         *  MultiPortSwitch: '<Root>/Index Vector'
         */
        rtY.rightRpm_output = rtDW.Saturation_i;
    }

    /* End of MultiPortSwitch: '<Root>/Index Vector1' */

    /* Sum: '<S3>/Sum' incorporates:
     *  Inport: '<Root>/track_in'
     *  Inport: '<Root>/track_tar'
     */
    rtb_Filter_k = rtU.track_in - rtU.track_tar;

    /* Gain: '<S305>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S297>/Filter'
     *  Gain: '<S295>/Derivative Gain'
     *  Sum: '<S297>/SumD'
     */
    rtb_FilterCoefficient_p = (rtP.track_Kd * rtb_Filter_k - rtDW.Filter_DSTATE)
        * rtP.DiscretePIDController_N;

    /* Outport: '<Root>/track_output' incorporates:
     *  DiscreteIntegrator: '<S302>/Integrator'
     *  Gain: '<S307>/Proportional Gain'
     *  Sum: '<S311>/Sum'
     */
    rtY.track_output = (rtP.track_Kp * rtb_Filter_k + rtDW.Integrator_DSTATE) +
        rtb_FilterCoefficient_p;

    /* Update for DiscreteIntegrator: '<S302>/Integrator' incorporates:
     *  Gain: '<S299>/Integral Gain'
     */
    rtDW.Integrator_DSTATE += rtP.track_Ki * rtb_Filter_k *
        rtP.Integrator_gainval_c;

    /* Update for DiscreteIntegrator: '<S297>/Filter' */
    rtDW.Filter_DSTATE += rtP.Filter_gainval_pw * rtb_FilterCoefficient_p;
}

/* Model initialize function */
void gc_pid_initialize(void)
{
    /* Registration code */

    /* initialize error status */
    rtmSetErrorStatus(rtM, (NULL));

    /* states (dwork) */
    (void) memset((void *)&rtDW, 0,
                  sizeof(DW));

    /* external inputs */
    (void)memset(&rtU, 0, sizeof(ExtU));

    /* external outputs */
    (void)memset(&rtY, 0, sizeof(ExtY));

    /* InitializeConditions for DiscreteIntegrator: '<S302>/Integrator' */
    rtDW.Integrator_DSTATE = rtP.DiscretePIDController_Initial_n;

    /* InitializeConditions for DiscreteIntegrator: '<S297>/Filter' */
    rtDW.Filter_DSTATE = rtP.DiscretePIDController_InitialCo;

    /* SystemInitialize for IfAction SubSystem: '<Root>/speed ctrl' */
    /* InitializeConditions for DiscreteIntegrator: '<S199>/Integrator' */
    rtDW.Integrator_DSTATE_d = rtP.LeftRPMPID_InitialConditionForI;

    /* InitializeConditions for DiscreteIntegrator: '<S194>/Filter' */
    rtDW.Filter_DSTATE_j = rtP.LeftRPMPID_InitialConditionForF;

    /* InitializeConditions for DiscreteIntegrator: '<S251>/Integrator' */
    rtDW.Integrator_DSTATE_j = rtP.RightRPMPID_InitialConditionF_h;

    /* InitializeConditions for DiscreteIntegrator: '<S246>/Filter' */
    rtDW.Filter_DSTATE_f = rtP.RightRPMPID_InitialConditionFor;

    /* SystemInitialize for Saturate: '<S206>/Saturation' incorporates:
     *  Outport: '<S2>/LeftRpm_output'
     */
    rtDW.Saturation_b = rtP.LeftRpm_output_Y0;

    /* SystemInitialize for Saturate: '<S258>/Saturation' incorporates:
     *  Outport: '<S2>/RightRpm_output'
     */
    rtDW.Saturation_bm = rtP.RightRpm_output_Y0;

    /* End of SystemInitialize for SubSystem: '<Root>/speed ctrl' */

    /* SystemInitialize for IfAction SubSystem: '<Root>/angle ctrl' */
    /* InitializeConditions for DiscreteIntegrator: '<S93>/Integrator' */
    rtDW.Integrator_DSTATE_h = rtP.positionPID_InitialConditionF_c;

    /* InitializeConditions for DiscreteIntegrator: '<S88>/Filter' */
    rtDW.Filter_DSTATE_d = rtP.positionPID_InitialConditionFor;

    /* InitializeConditions for DiscreteIntegrator: '<S43>/Integrator' */
    rtDW.Integrator_DSTATE_o = rtP.leftSpeedPID_InitialCondition_l;

    /* InitializeConditions for DiscreteIntegrator: '<S38>/Filter' */
    rtDW.Filter_DSTATE_k = rtP.leftSpeedPID_InitialConditionFo;

    /* InitializeConditions for DiscreteIntegrator: '<S145>/Integrator' */
    rtDW.Integrator_DSTATE_f = rtP.rightSpeedPID_InitialConditio_p;

    /* InitializeConditions for DiscreteIntegrator: '<S140>/Filter' */
    rtDW.Filter_DSTATE_e = rtP.rightSpeedPID_InitialConditionF;

    /* SystemInitialize for Saturate: '<S50>/Saturation' incorporates:
     *  Outport: '<S1>/LeftRpm_output'
     */
    rtDW.Saturation = rtP.LeftRpm_output_Y0_d;

    /* SystemInitialize for Saturate: '<S152>/Saturation' incorporates:
     *  Outport: '<S1>/RightRpm_output'
     */
    rtDW.Saturation_i = rtP.RightRpm_output_Y0_c;

    /* End of SystemInitialize for SubSystem: '<Root>/angle ctrl' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
