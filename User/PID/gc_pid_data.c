/*
 * File: gc_pid_data.c
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

/* Block parameters (default storage) */
P rtP = {
    /* Variable: angle_A_Kd
     * Referenced by: '<S86>/Derivative Gain'
     */
    0.0F,

    /* Variable: angle_A_Ki
     * Referenced by: '<S90>/Integral Gain'
     */
    0.0F,

    /* Variable: angle_A_Kp
     * Referenced by: '<S98>/Proportional Gain'
     */
    0.0F,

    /* Variable: angle_A_iMax
     * Referenced by: '<S93>/Integrator'
     */
    2.14748365E+9F,

    /* Variable: angle_A_iMin
     * Referenced by: '<S93>/Integrator'
     */
    -2.14748365E+9F,

    /* Variable: angle_S_Kd
     * Referenced by:
     *   '<S36>/Derivative Gain'
     *   '<S138>/Derivative Gain'
     */
    0.0F,

    /* Variable: angle_S_Ki
     * Referenced by:
     *   '<S40>/Integral Gain'
     *   '<S142>/Integral Gain'
     */
    0.0F,

    /* Variable: angle_S_Kp
     * Referenced by:
     *   '<S48>/Proportional Gain'
     *   '<S150>/Proportional Gain'
     */
    0.0F,

    /* Variable: angle_S_iMax
     * Referenced by:
     *   '<S43>/Integrator'
     *   '<S145>/Integrator'
     */
    2.14748365E+9F,

    /* Variable: angle_S_iMin
     * Referenced by:
     *   '<S43>/Integrator'
     *   '<S145>/Integrator'
     */
    -2.14748365E+9F,

    /* Variable: speed_Kd
     * Referenced by:
     *   '<S192>/Derivative Gain'
     *   '<S244>/Derivative Gain'
     */
    0.0F,

    /* Variable: speed_Ki
     * Referenced by:
     *   '<S196>/Integral Gain'
     *   '<S248>/Integral Gain'
     */
    0.0F,

    /* Variable: speed_Kp
     * Referenced by:
     *   '<S204>/Proportional Gain'
     *   '<S256>/Proportional Gain'
     */
    0.0F,

    /* Variable: speed_iMax
     * Referenced by:
     *   '<S199>/Integrator'
     *   '<S251>/Integrator'
     */
    2.14748365E+9F,

    /* Variable: speed_iMin
     * Referenced by:
     *   '<S199>/Integrator'
     *   '<S251>/Integrator'
     */
    -2.14748365E+9F,

    /* Variable: track_Kd
     * Referenced by: '<S295>/Derivative Gain'
     */
    0.0F,

    /* Variable: track_Ki
     * Referenced by: '<S299>/Integral Gain'
     */
    0.0F,

    /* Variable: track_Kp
     * Referenced by: '<S307>/Proportional Gain'
     */
    0.0F,

    /* Mask Parameter: LeftRPMPID_InitialConditionForF
     * Referenced by: '<S194>/Filter'
     */
    0.0F,

    /* Mask Parameter: RightRPMPID_InitialConditionFor
     * Referenced by: '<S246>/Filter'
     */
    0.0F,

    /* Mask Parameter: positionPID_InitialConditionFor
     * Referenced by: '<S88>/Filter'
     */
    0.0F,

    /* Mask Parameter: leftSpeedPID_InitialConditionFo
     * Referenced by: '<S38>/Filter'
     */
    0.0F,

    /* Mask Parameter: rightSpeedPID_InitialConditionF
     * Referenced by: '<S140>/Filter'
     */
    0.0F,

    /* Mask Parameter: DiscretePIDController_InitialCo
     * Referenced by: '<S297>/Filter'
     */
    0.0F,

    /* Mask Parameter: LeftRPMPID_InitialConditionForI
     * Referenced by: '<S199>/Integrator'
     */
    0.0F,

    /* Mask Parameter: RightRPMPID_InitialConditionF_h
     * Referenced by: '<S251>/Integrator'
     */
    0.0F,

    /* Mask Parameter: positionPID_InitialConditionF_c
     * Referenced by: '<S93>/Integrator'
     */
    0.0F,

    /* Mask Parameter: leftSpeedPID_InitialCondition_l
     * Referenced by: '<S43>/Integrator'
     */
    0.0F,

    /* Mask Parameter: rightSpeedPID_InitialConditio_p
     * Referenced by: '<S145>/Integrator'
     */
    0.0F,

    /* Mask Parameter: DiscretePIDController_Initial_n
     * Referenced by: '<S302>/Integrator'
     */
    0.0F,

    /* Mask Parameter: LeftRPMPID_LowerSaturationLimit
     * Referenced by:
     *   '<S206>/Saturation'
     *   '<S191>/DeadZone'
     */
    -1000.0F,

    /* Mask Parameter: RightRPMPID_LowerSaturationLimi
     * Referenced by:
     *   '<S258>/Saturation'
     *   '<S243>/DeadZone'
     */
    -1000.0F,

    /* Mask Parameter: leftSpeedPID_LowerSaturationLim
     * Referenced by:
     *   '<S50>/Saturation'
     *   '<S35>/DeadZone'
     */
    -1000.0F,

    /* Mask Parameter: rightSpeedPID_LowerSaturationLi
     * Referenced by:
     *   '<S152>/Saturation'
     *   '<S137>/DeadZone'
     */
    -1000.0F,

    /* Mask Parameter: LeftRPMPID_N
     * Referenced by: '<S202>/Filter Coefficient'
     */
    100.0F,

    /* Mask Parameter: RightRPMPID_N
     * Referenced by: '<S254>/Filter Coefficient'
     */
    100.0F,

    /* Mask Parameter: positionPID_N
     * Referenced by: '<S96>/Filter Coefficient'
     */
    100.0F,

    /* Mask Parameter: leftSpeedPID_N
     * Referenced by: '<S46>/Filter Coefficient'
     */
    100.0F,

    /* Mask Parameter: rightSpeedPID_N
     * Referenced by: '<S148>/Filter Coefficient'
     */
    100.0F,

    /* Mask Parameter: DiscretePIDController_N
     * Referenced by: '<S305>/Filter Coefficient'
     */
    100.0F,

    /* Mask Parameter: LeftRPMPID_UpperSaturationLimit
     * Referenced by:
     *   '<S206>/Saturation'
     *   '<S191>/DeadZone'
     */
    1000.0F,

    /* Mask Parameter: RightRPMPID_UpperSaturationLimi
     * Referenced by:
     *   '<S258>/Saturation'
     *   '<S243>/DeadZone'
     */
    1000.0F,

    /* Mask Parameter: leftSpeedPID_UpperSaturationLim
     * Referenced by:
     *   '<S50>/Saturation'
     *   '<S35>/DeadZone'
     */
    1000.0F,

    /* Mask Parameter: rightSpeedPID_UpperSaturationLi
     * Referenced by:
     *   '<S152>/Saturation'
     *   '<S137>/DeadZone'
     */
    1000.0F,

    /* Computed Parameter: LeftRpm_output_Y0
     * Referenced by: '<S2>/LeftRpm_output'
     */
    0.0F,

    /* Computed Parameter: RightRpm_output_Y0
     * Referenced by: '<S2>/RightRpm_output'
     */
    0.0F,

    /* Computed Parameter: Constant1_Value
     * Referenced by: '<S189>/Constant1'
     */
    0.0F,

    /* Computed Parameter: Constant1_Value_f
     * Referenced by: '<S241>/Constant1'
     */
    0.0F,

    /* Computed Parameter: Clamping_zero_Value
     * Referenced by: '<S189>/Clamping_zero'
     */
    0.0F,

    /* Computed Parameter: Integrator_gainval
     * Referenced by: '<S199>/Integrator'
     */
    0.001F,

    /* Computed Parameter: Filter_gainval
     * Referenced by: '<S194>/Filter'
     */
    0.001F,

    /* Computed Parameter: Clamping_zero_Value_f
     * Referenced by: '<S241>/Clamping_zero'
     */
    0.0F,

    /* Computed Parameter: Integrator_gainval_g
     * Referenced by: '<S251>/Integrator'
     */
    0.001F,

    /* Computed Parameter: Filter_gainval_i
     * Referenced by: '<S246>/Filter'
     */
    0.001F,

    /* Computed Parameter: LeftRpm_output_Y0_d
     * Referenced by: '<S1>/LeftRpm_output'
     */
    0.0F,

    /* Computed Parameter: RightRpm_output_Y0_c
     * Referenced by: '<S1>/RightRpm_output'
     */
    0.0F,

    /* Computed Parameter: Constant1_Value_h
     * Referenced by: '<S33>/Constant1'
     */
    0.0F,

    /* Computed Parameter: Constant1_Value_m
     * Referenced by: '<S135>/Constant1'
     */
    0.0F,

    /* Computed Parameter: Integrator_gainval_m
     * Referenced by: '<S93>/Integrator'
     */
    0.001F,

    /* Computed Parameter: Filter_gainval_m
     * Referenced by: '<S88>/Filter'
     */
    0.001F,

    /* Computed Parameter: Clamping_zero_Value_g
     * Referenced by: '<S33>/Clamping_zero'
     */
    0.0F,

    /* Computed Parameter: Integrator_gainval_a
     * Referenced by: '<S43>/Integrator'
     */
    0.001F,

    /* Computed Parameter: Filter_gainval_o
     * Referenced by: '<S38>/Filter'
     */
    0.001F,

    /* Computed Parameter: Clamping_zero_Value_i
     * Referenced by: '<S135>/Clamping_zero'
     */
    0.0F,

    /* Computed Parameter: Integrator_gainval_l
     * Referenced by: '<S145>/Integrator'
     */
    0.001F,

    /* Computed Parameter: Filter_gainval_p
     * Referenced by: '<S140>/Filter'
     */
    0.001F,

    /* Computed Parameter: Integrator_gainval_c
     * Referenced by: '<S302>/Integrator'
     */
    0.001F,

    /* Computed Parameter: Filter_gainval_pw
     * Referenced by: '<S297>/Filter'
     */
    0.001F,

    /* Computed Parameter: Constant_Value
     * Referenced by: '<S189>/Constant'
     */
    1,

    /* Computed Parameter: Constant2_Value
     * Referenced by: '<S189>/Constant2'
     */
    -1,

    /* Computed Parameter: Constant3_Value
     * Referenced by: '<S189>/Constant3'
     */
    1,

    /* Computed Parameter: Constant4_Value
     * Referenced by: '<S189>/Constant4'
     */
    -1,

    /* Computed Parameter: Constant_Value_j
     * Referenced by: '<S241>/Constant'
     */
    1,

    /* Computed Parameter: Constant2_Value_j
     * Referenced by: '<S241>/Constant2'
     */
    -1,

    /* Computed Parameter: Constant3_Value_g
     * Referenced by: '<S241>/Constant3'
     */
    1,

    /* Computed Parameter: Constant4_Value_h
     * Referenced by: '<S241>/Constant4'
     */
    -1,

    /* Computed Parameter: Constant_Value_d
     * Referenced by: '<S33>/Constant'
     */
    1,

    /* Computed Parameter: Constant2_Value_o
     * Referenced by: '<S33>/Constant2'
     */
    -1,

    /* Computed Parameter: Constant3_Value_gp
     * Referenced by: '<S33>/Constant3'
     */
    1,

    /* Computed Parameter: Constant4_Value_hw
     * Referenced by: '<S33>/Constant4'
     */
    -1,

    /* Computed Parameter: Constant_Value_i
     * Referenced by: '<S135>/Constant'
     */
    1,

    /* Computed Parameter: Constant2_Value_n
     * Referenced by: '<S135>/Constant2'
     */
    -1,

    /* Computed Parameter: Constant3_Value_c
     * Referenced by: '<S135>/Constant3'
     */
    1,

    /* Computed Parameter: Constant4_Value_n
     * Referenced by: '<S135>/Constant4'
     */
    -1
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
