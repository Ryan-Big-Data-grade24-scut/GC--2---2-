/*
 * File: gc_pid.h
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

#ifndef gc_pid_h_
#define gc_pid_h_
#ifndef gc_pid_COMMON_INCLUDES_
#define gc_pid_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* gc_pid_COMMON_INCLUDES_ */

#include <stddef.h>
#include <string.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
    real32_T Saturation;               /* '<S50>/Saturation' */
    real32_T Saturation_i;             /* '<S152>/Saturation' */
    real32_T Saturation_b;             /* '<S206>/Saturation' */
    real32_T Saturation_bm;            /* '<S258>/Saturation' */
    real32_T Integrator_DSTATE;        /* '<S302>/Integrator' */
    real32_T Filter_DSTATE;            /* '<S297>/Filter' */
    real32_T Integrator_DSTATE_h;      /* '<S93>/Integrator' */
    real32_T Filter_DSTATE_d;          /* '<S88>/Filter' */
    real32_T Integrator_DSTATE_o;      /* '<S43>/Integrator' */
    real32_T Filter_DSTATE_k;          /* '<S38>/Filter' */
    real32_T Integrator_DSTATE_f;      /* '<S145>/Integrator' */
    real32_T Filter_DSTATE_e;          /* '<S140>/Filter' */
    real32_T Integrator_DSTATE_d;      /* '<S199>/Integrator' */
    real32_T Filter_DSTATE_j;          /* '<S194>/Filter' */
    real32_T Integrator_DSTATE_j;      /* '<S251>/Integrator' */
    real32_T Filter_DSTATE_f;          /* '<S246>/Filter' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
    int8_T ctrl_mode;                  /* '<Root>/ctrl_mode' */
    real32_T angle_tar;                /* '<Root>/angle_tar' */
    real32_T angle_real;               /* '<Root>/angle_real' */
    real32_T leftRpm_real;             /* '<Root>/leftRpm_real' */
    real32_T rightRpm_real;            /* '<Root>/rightRpm_real' */
    real32_T leftRpm_tar;              /* '<Root>/leftRpm_tar' */
    real32_T rightRpm_tar;             /* '<Root>/rightRpm_tar' */
    real32_T track_in;                 /* '<Root>/track_in' */
    real32_T track_tar;                /* '<Root>/track_tar' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
    real32_T leftRpm_output;           /* '<Root>/leftRpm_output' */
    real32_T rightRpm_output;          /* '<Root>/rightRpm_output' */
    real32_T track_output;             /* '<Root>/track_output' */
} ExtY;

/* Parameters (default storage) */
struct P_ {
    real32_T angle_A_Kd;               /* Variable: angle_A_Kd
                                        * Referenced by: '<S86>/Derivative Gain'
                                        */
    real32_T angle_A_Ki;               /* Variable: angle_A_Ki
                                        * Referenced by: '<S90>/Integral Gain'
                                        */
    real32_T angle_A_Kp;               /* Variable: angle_A_Kp
                                        * Referenced by: '<S98>/Proportional Gain'
                                        */
    real32_T angle_A_iMax;             /* Variable: angle_A_iMax
                                        * Referenced by: '<S93>/Integrator'
                                        */
    real32_T angle_A_iMin;             /* Variable: angle_A_iMin
                                        * Referenced by: '<S93>/Integrator'
                                        */
    real32_T angle_S_Kd;               /* Variable: angle_S_Kd
                                        * Referenced by:
                                        *   '<S36>/Derivative Gain'
                                        *   '<S138>/Derivative Gain'
                                        */
    real32_T angle_S_Ki;               /* Variable: angle_S_Ki
                                        * Referenced by:
                                        *   '<S40>/Integral Gain'
                                        *   '<S142>/Integral Gain'
                                        */
    real32_T angle_S_Kp;               /* Variable: angle_S_Kp
                                        * Referenced by:
                                        *   '<S48>/Proportional Gain'
                                        *   '<S150>/Proportional Gain'
                                        */
    real32_T angle_S_iMax;             /* Variable: angle_S_iMax
                                        * Referenced by:
                                        *   '<S43>/Integrator'
                                        *   '<S145>/Integrator'
                                        */
    real32_T angle_S_iMin;             /* Variable: angle_S_iMin
                                        * Referenced by:
                                        *   '<S43>/Integrator'
                                        *   '<S145>/Integrator'
                                        */
    real32_T speed_Kd;                 /* Variable: speed_Kd
                                        * Referenced by:
                                        *   '<S192>/Derivative Gain'
                                        *   '<S244>/Derivative Gain'
                                        */
    real32_T speed_Ki;                 /* Variable: speed_Ki
                                        * Referenced by:
                                        *   '<S196>/Integral Gain'
                                        *   '<S248>/Integral Gain'
                                        */
    real32_T speed_Kp;                 /* Variable: speed_Kp
                                        * Referenced by:
                                        *   '<S204>/Proportional Gain'
                                        *   '<S256>/Proportional Gain'
                                        */
    real32_T speed_iMax;               /* Variable: speed_iMax
                                        * Referenced by:
                                        *   '<S199>/Integrator'
                                        *   '<S251>/Integrator'
                                        */
    real32_T speed_iMin;               /* Variable: speed_iMin
                                        * Referenced by:
                                        *   '<S199>/Integrator'
                                        *   '<S251>/Integrator'
                                        */
    real32_T track_Kd;                 /* Variable: track_Kd
                                        * Referenced by: '<S295>/Derivative Gain'
                                        */
    real32_T track_Ki;                 /* Variable: track_Ki
                                        * Referenced by: '<S299>/Integral Gain'
                                        */
    real32_T track_Kp;                 /* Variable: track_Kp
                                        * Referenced by: '<S307>/Proportional Gain'
                                        */
    real32_T LeftRPMPID_InitialConditionForF;
                              /* Mask Parameter: LeftRPMPID_InitialConditionForF
                               * Referenced by: '<S194>/Filter'
                               */
    real32_T RightRPMPID_InitialConditionFor;
                              /* Mask Parameter: RightRPMPID_InitialConditionFor
                               * Referenced by: '<S246>/Filter'
                               */
    real32_T positionPID_InitialConditionFor;
                              /* Mask Parameter: positionPID_InitialConditionFor
                               * Referenced by: '<S88>/Filter'
                               */
    real32_T leftSpeedPID_InitialConditionFo;
                              /* Mask Parameter: leftSpeedPID_InitialConditionFo
                               * Referenced by: '<S38>/Filter'
                               */
    real32_T rightSpeedPID_InitialConditionF;
                              /* Mask Parameter: rightSpeedPID_InitialConditionF
                               * Referenced by: '<S140>/Filter'
                               */
    real32_T DiscretePIDController_InitialCo;
                              /* Mask Parameter: DiscretePIDController_InitialCo
                               * Referenced by: '<S297>/Filter'
                               */
    real32_T LeftRPMPID_InitialConditionForI;
                              /* Mask Parameter: LeftRPMPID_InitialConditionForI
                               * Referenced by: '<S199>/Integrator'
                               */
    real32_T RightRPMPID_InitialConditionF_h;
                              /* Mask Parameter: RightRPMPID_InitialConditionF_h
                               * Referenced by: '<S251>/Integrator'
                               */
    real32_T positionPID_InitialConditionF_c;
                              /* Mask Parameter: positionPID_InitialConditionF_c
                               * Referenced by: '<S93>/Integrator'
                               */
    real32_T leftSpeedPID_InitialCondition_l;
                              /* Mask Parameter: leftSpeedPID_InitialCondition_l
                               * Referenced by: '<S43>/Integrator'
                               */
    real32_T rightSpeedPID_InitialConditio_p;
                              /* Mask Parameter: rightSpeedPID_InitialConditio_p
                               * Referenced by: '<S145>/Integrator'
                               */
    real32_T DiscretePIDController_Initial_n;
                              /* Mask Parameter: DiscretePIDController_Initial_n
                               * Referenced by: '<S302>/Integrator'
                               */
    real32_T LeftRPMPID_LowerSaturationLimit;
                              /* Mask Parameter: LeftRPMPID_LowerSaturationLimit
                               * Referenced by:
                               *   '<S206>/Saturation'
                               *   '<S191>/DeadZone'
                               */
    real32_T RightRPMPID_LowerSaturationLimi;
                              /* Mask Parameter: RightRPMPID_LowerSaturationLimi
                               * Referenced by:
                               *   '<S258>/Saturation'
                               *   '<S243>/DeadZone'
                               */
    real32_T leftSpeedPID_LowerSaturationLim;
                              /* Mask Parameter: leftSpeedPID_LowerSaturationLim
                               * Referenced by:
                               *   '<S50>/Saturation'
                               *   '<S35>/DeadZone'
                               */
    real32_T rightSpeedPID_LowerSaturationLi;
                              /* Mask Parameter: rightSpeedPID_LowerSaturationLi
                               * Referenced by:
                               *   '<S152>/Saturation'
                               *   '<S137>/DeadZone'
                               */
    real32_T LeftRPMPID_N;             /* Mask Parameter: LeftRPMPID_N
                                        * Referenced by: '<S202>/Filter Coefficient'
                                        */
    real32_T RightRPMPID_N;            /* Mask Parameter: RightRPMPID_N
                                        * Referenced by: '<S254>/Filter Coefficient'
                                        */
    real32_T positionPID_N;            /* Mask Parameter: positionPID_N
                                        * Referenced by: '<S96>/Filter Coefficient'
                                        */
    real32_T leftSpeedPID_N;           /* Mask Parameter: leftSpeedPID_N
                                        * Referenced by: '<S46>/Filter Coefficient'
                                        */
    real32_T rightSpeedPID_N;          /* Mask Parameter: rightSpeedPID_N
                                        * Referenced by: '<S148>/Filter Coefficient'
                                        */
    real32_T DiscretePIDController_N; /* Mask Parameter: DiscretePIDController_N
                                       * Referenced by: '<S305>/Filter Coefficient'
                                       */
    real32_T LeftRPMPID_UpperSaturationLimit;
                              /* Mask Parameter: LeftRPMPID_UpperSaturationLimit
                               * Referenced by:
                               *   '<S206>/Saturation'
                               *   '<S191>/DeadZone'
                               */
    real32_T RightRPMPID_UpperSaturationLimi;
                              /* Mask Parameter: RightRPMPID_UpperSaturationLimi
                               * Referenced by:
                               *   '<S258>/Saturation'
                               *   '<S243>/DeadZone'
                               */
    real32_T leftSpeedPID_UpperSaturationLim;
                              /* Mask Parameter: leftSpeedPID_UpperSaturationLim
                               * Referenced by:
                               *   '<S50>/Saturation'
                               *   '<S35>/DeadZone'
                               */
    real32_T rightSpeedPID_UpperSaturationLi;
                              /* Mask Parameter: rightSpeedPID_UpperSaturationLi
                               * Referenced by:
                               *   '<S152>/Saturation'
                               *   '<S137>/DeadZone'
                               */
    real32_T LeftRpm_output_Y0;        /* Computed Parameter: LeftRpm_output_Y0
                                        * Referenced by: '<S2>/LeftRpm_output'
                                        */
    real32_T RightRpm_output_Y0;       /* Computed Parameter: RightRpm_output_Y0
                                        * Referenced by: '<S2>/RightRpm_output'
                                        */
    real32_T Constant1_Value;          /* Computed Parameter: Constant1_Value
                                        * Referenced by: '<S189>/Constant1'
                                        */
    real32_T Constant1_Value_f;        /* Computed Parameter: Constant1_Value_f
                                        * Referenced by: '<S241>/Constant1'
                                        */
    real32_T Clamping_zero_Value;     /* Computed Parameter: Clamping_zero_Value
                                       * Referenced by: '<S189>/Clamping_zero'
                                       */
    real32_T Integrator_gainval;       /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S199>/Integrator'
                                        */
    real32_T Filter_gainval;           /* Computed Parameter: Filter_gainval
                                        * Referenced by: '<S194>/Filter'
                                        */
    real32_T Clamping_zero_Value_f; /* Computed Parameter: Clamping_zero_Value_f
                                     * Referenced by: '<S241>/Clamping_zero'
                                     */
    real32_T Integrator_gainval_g;   /* Computed Parameter: Integrator_gainval_g
                                      * Referenced by: '<S251>/Integrator'
                                      */
    real32_T Filter_gainval_i;         /* Computed Parameter: Filter_gainval_i
                                        * Referenced by: '<S246>/Filter'
                                        */
    real32_T LeftRpm_output_Y0_d;     /* Computed Parameter: LeftRpm_output_Y0_d
                                       * Referenced by: '<S1>/LeftRpm_output'
                                       */
    real32_T RightRpm_output_Y0_c;   /* Computed Parameter: RightRpm_output_Y0_c
                                      * Referenced by: '<S1>/RightRpm_output'
                                      */
    real32_T Constant1_Value_h;        /* Computed Parameter: Constant1_Value_h
                                        * Referenced by: '<S33>/Constant1'
                                        */
    real32_T Constant1_Value_m;        /* Computed Parameter: Constant1_Value_m
                                        * Referenced by: '<S135>/Constant1'
                                        */
    real32_T Integrator_gainval_m;   /* Computed Parameter: Integrator_gainval_m
                                      * Referenced by: '<S93>/Integrator'
                                      */
    real32_T Filter_gainval_m;         /* Computed Parameter: Filter_gainval_m
                                        * Referenced by: '<S88>/Filter'
                                        */
    real32_T Clamping_zero_Value_g; /* Computed Parameter: Clamping_zero_Value_g
                                     * Referenced by: '<S33>/Clamping_zero'
                                     */
    real32_T Integrator_gainval_a;   /* Computed Parameter: Integrator_gainval_a
                                      * Referenced by: '<S43>/Integrator'
                                      */
    real32_T Filter_gainval_o;         /* Computed Parameter: Filter_gainval_o
                                        * Referenced by: '<S38>/Filter'
                                        */
    real32_T Clamping_zero_Value_i; /* Computed Parameter: Clamping_zero_Value_i
                                     * Referenced by: '<S135>/Clamping_zero'
                                     */
    real32_T Integrator_gainval_l;   /* Computed Parameter: Integrator_gainval_l
                                      * Referenced by: '<S145>/Integrator'
                                      */
    real32_T Filter_gainval_p;         /* Computed Parameter: Filter_gainval_p
                                        * Referenced by: '<S140>/Filter'
                                        */
    real32_T Integrator_gainval_c;   /* Computed Parameter: Integrator_gainval_c
                                      * Referenced by: '<S302>/Integrator'
                                      */
    real32_T Filter_gainval_pw;        /* Computed Parameter: Filter_gainval_pw
                                        * Referenced by: '<S297>/Filter'
                                        */
    int8_T Constant_Value;             /* Computed Parameter: Constant_Value
                                        * Referenced by: '<S189>/Constant'
                                        */
    int8_T Constant2_Value;            /* Computed Parameter: Constant2_Value
                                        * Referenced by: '<S189>/Constant2'
                                        */
    int8_T Constant3_Value;            /* Computed Parameter: Constant3_Value
                                        * Referenced by: '<S189>/Constant3'
                                        */
    int8_T Constant4_Value;            /* Computed Parameter: Constant4_Value
                                        * Referenced by: '<S189>/Constant4'
                                        */
    int8_T Constant_Value_j;           /* Computed Parameter: Constant_Value_j
                                        * Referenced by: '<S241>/Constant'
                                        */
    int8_T Constant2_Value_j;          /* Computed Parameter: Constant2_Value_j
                                        * Referenced by: '<S241>/Constant2'
                                        */
    int8_T Constant3_Value_g;          /* Computed Parameter: Constant3_Value_g
                                        * Referenced by: '<S241>/Constant3'
                                        */
    int8_T Constant4_Value_h;          /* Computed Parameter: Constant4_Value_h
                                        * Referenced by: '<S241>/Constant4'
                                        */
    int8_T Constant_Value_d;           /* Computed Parameter: Constant_Value_d
                                        * Referenced by: '<S33>/Constant'
                                        */
    int8_T Constant2_Value_o;          /* Computed Parameter: Constant2_Value_o
                                        * Referenced by: '<S33>/Constant2'
                                        */
    int8_T Constant3_Value_gp;         /* Computed Parameter: Constant3_Value_gp
                                        * Referenced by: '<S33>/Constant3'
                                        */
    int8_T Constant4_Value_hw;         /* Computed Parameter: Constant4_Value_hw
                                        * Referenced by: '<S33>/Constant4'
                                        */
    int8_T Constant_Value_i;           /* Computed Parameter: Constant_Value_i
                                        * Referenced by: '<S135>/Constant'
                                        */
    int8_T Constant2_Value_n;          /* Computed Parameter: Constant2_Value_n
                                        * Referenced by: '<S135>/Constant2'
                                        */
    int8_T Constant3_Value_c;          /* Computed Parameter: Constant3_Value_c
                                        * Referenced by: '<S135>/Constant3'
                                        */
    int8_T Constant4_Value_n;          /* Computed Parameter: Constant4_Value_n
                                        * Referenced by: '<S135>/Constant4'
                                        */
};

/* Parameters (default storage) */
typedef struct P_ P;

/* Real-time Model Data Structure */
struct tag_RTM {
    const char_T * volatile errorStatus;
};

/* Block parameters (default storage) */
extern P rtP;

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Model entry point functions */
extern void gc_pid_initialize(void);
extern void gc_pid_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'gc_pid'
 * '<S1>'   : 'gc_pid/angle ctrl'
 * '<S2>'   : 'gc_pid/speed ctrl'
 * '<S3>'   : 'gc_pid/track ctrl'
 * '<S4>'   : 'gc_pid/angle ctrl/ang correction'
 * '<S5>'   : 'gc_pid/angle ctrl/leftSpeed PID'
 * '<S6>'   : 'gc_pid/angle ctrl/position PID'
 * '<S7>'   : 'gc_pid/angle ctrl/rightSpeed PID'
 * '<S8>'   : 'gc_pid/angle ctrl/leftSpeed PID/Anti-windup'
 * '<S9>'   : 'gc_pid/angle ctrl/leftSpeed PID/D Gain'
 * '<S10>'  : 'gc_pid/angle ctrl/leftSpeed PID/External Derivative'
 * '<S11>'  : 'gc_pid/angle ctrl/leftSpeed PID/Filter'
 * '<S12>'  : 'gc_pid/angle ctrl/leftSpeed PID/Filter ICs'
 * '<S13>'  : 'gc_pid/angle ctrl/leftSpeed PID/I Gain'
 * '<S14>'  : 'gc_pid/angle ctrl/leftSpeed PID/Ideal P Gain'
 * '<S15>'  : 'gc_pid/angle ctrl/leftSpeed PID/Ideal P Gain Fdbk'
 * '<S16>'  : 'gc_pid/angle ctrl/leftSpeed PID/Integrator'
 * '<S17>'  : 'gc_pid/angle ctrl/leftSpeed PID/Integrator ICs'
 * '<S18>'  : 'gc_pid/angle ctrl/leftSpeed PID/N Copy'
 * '<S19>'  : 'gc_pid/angle ctrl/leftSpeed PID/N Gain'
 * '<S20>'  : 'gc_pid/angle ctrl/leftSpeed PID/P Copy'
 * '<S21>'  : 'gc_pid/angle ctrl/leftSpeed PID/Parallel P Gain'
 * '<S22>'  : 'gc_pid/angle ctrl/leftSpeed PID/Reset Signal'
 * '<S23>'  : 'gc_pid/angle ctrl/leftSpeed PID/Saturation'
 * '<S24>'  : 'gc_pid/angle ctrl/leftSpeed PID/Saturation Fdbk'
 * '<S25>'  : 'gc_pid/angle ctrl/leftSpeed PID/Sum'
 * '<S26>'  : 'gc_pid/angle ctrl/leftSpeed PID/Sum Fdbk'
 * '<S27>'  : 'gc_pid/angle ctrl/leftSpeed PID/Tracking Mode'
 * '<S28>'  : 'gc_pid/angle ctrl/leftSpeed PID/Tracking Mode Sum'
 * '<S29>'  : 'gc_pid/angle ctrl/leftSpeed PID/Tsamp - Integral'
 * '<S30>'  : 'gc_pid/angle ctrl/leftSpeed PID/Tsamp - Ngain'
 * '<S31>'  : 'gc_pid/angle ctrl/leftSpeed PID/postSat Signal'
 * '<S32>'  : 'gc_pid/angle ctrl/leftSpeed PID/preSat Signal'
 * '<S33>'  : 'gc_pid/angle ctrl/leftSpeed PID/Anti-windup/Disc. Clamping Parallel'
 * '<S34>'  : 'gc_pid/angle ctrl/leftSpeed PID/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S35>'  : 'gc_pid/angle ctrl/leftSpeed PID/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S36>'  : 'gc_pid/angle ctrl/leftSpeed PID/D Gain/Internal Parameters'
 * '<S37>'  : 'gc_pid/angle ctrl/leftSpeed PID/External Derivative/Error'
 * '<S38>'  : 'gc_pid/angle ctrl/leftSpeed PID/Filter/Disc. Forward Euler Filter'
 * '<S39>'  : 'gc_pid/angle ctrl/leftSpeed PID/Filter ICs/Internal IC - Filter'
 * '<S40>'  : 'gc_pid/angle ctrl/leftSpeed PID/I Gain/Internal Parameters'
 * '<S41>'  : 'gc_pid/angle ctrl/leftSpeed PID/Ideal P Gain/Passthrough'
 * '<S42>'  : 'gc_pid/angle ctrl/leftSpeed PID/Ideal P Gain Fdbk/Disabled'
 * '<S43>'  : 'gc_pid/angle ctrl/leftSpeed PID/Integrator/Discrete'
 * '<S44>'  : 'gc_pid/angle ctrl/leftSpeed PID/Integrator ICs/Internal IC'
 * '<S45>'  : 'gc_pid/angle ctrl/leftSpeed PID/N Copy/Disabled'
 * '<S46>'  : 'gc_pid/angle ctrl/leftSpeed PID/N Gain/Internal Parameters'
 * '<S47>'  : 'gc_pid/angle ctrl/leftSpeed PID/P Copy/Disabled'
 * '<S48>'  : 'gc_pid/angle ctrl/leftSpeed PID/Parallel P Gain/Internal Parameters'
 * '<S49>'  : 'gc_pid/angle ctrl/leftSpeed PID/Reset Signal/Disabled'
 * '<S50>'  : 'gc_pid/angle ctrl/leftSpeed PID/Saturation/Enabled'
 * '<S51>'  : 'gc_pid/angle ctrl/leftSpeed PID/Saturation Fdbk/Disabled'
 * '<S52>'  : 'gc_pid/angle ctrl/leftSpeed PID/Sum/Sum_PID'
 * '<S53>'  : 'gc_pid/angle ctrl/leftSpeed PID/Sum Fdbk/Disabled'
 * '<S54>'  : 'gc_pid/angle ctrl/leftSpeed PID/Tracking Mode/Disabled'
 * '<S55>'  : 'gc_pid/angle ctrl/leftSpeed PID/Tracking Mode Sum/Passthrough'
 * '<S56>'  : 'gc_pid/angle ctrl/leftSpeed PID/Tsamp - Integral/TsSignalSpecification'
 * '<S57>'  : 'gc_pid/angle ctrl/leftSpeed PID/Tsamp - Ngain/Passthrough'
 * '<S58>'  : 'gc_pid/angle ctrl/leftSpeed PID/postSat Signal/Forward_Path'
 * '<S59>'  : 'gc_pid/angle ctrl/leftSpeed PID/preSat Signal/Forward_Path'
 * '<S60>'  : 'gc_pid/angle ctrl/position PID/Anti-windup'
 * '<S61>'  : 'gc_pid/angle ctrl/position PID/D Gain'
 * '<S62>'  : 'gc_pid/angle ctrl/position PID/External Derivative'
 * '<S63>'  : 'gc_pid/angle ctrl/position PID/Filter'
 * '<S64>'  : 'gc_pid/angle ctrl/position PID/Filter ICs'
 * '<S65>'  : 'gc_pid/angle ctrl/position PID/I Gain'
 * '<S66>'  : 'gc_pid/angle ctrl/position PID/Ideal P Gain'
 * '<S67>'  : 'gc_pid/angle ctrl/position PID/Ideal P Gain Fdbk'
 * '<S68>'  : 'gc_pid/angle ctrl/position PID/Integrator'
 * '<S69>'  : 'gc_pid/angle ctrl/position PID/Integrator ICs'
 * '<S70>'  : 'gc_pid/angle ctrl/position PID/N Copy'
 * '<S71>'  : 'gc_pid/angle ctrl/position PID/N Gain'
 * '<S72>'  : 'gc_pid/angle ctrl/position PID/P Copy'
 * '<S73>'  : 'gc_pid/angle ctrl/position PID/Parallel P Gain'
 * '<S74>'  : 'gc_pid/angle ctrl/position PID/Reset Signal'
 * '<S75>'  : 'gc_pid/angle ctrl/position PID/Saturation'
 * '<S76>'  : 'gc_pid/angle ctrl/position PID/Saturation Fdbk'
 * '<S77>'  : 'gc_pid/angle ctrl/position PID/Sum'
 * '<S78>'  : 'gc_pid/angle ctrl/position PID/Sum Fdbk'
 * '<S79>'  : 'gc_pid/angle ctrl/position PID/Tracking Mode'
 * '<S80>'  : 'gc_pid/angle ctrl/position PID/Tracking Mode Sum'
 * '<S81>'  : 'gc_pid/angle ctrl/position PID/Tsamp - Integral'
 * '<S82>'  : 'gc_pid/angle ctrl/position PID/Tsamp - Ngain'
 * '<S83>'  : 'gc_pid/angle ctrl/position PID/postSat Signal'
 * '<S84>'  : 'gc_pid/angle ctrl/position PID/preSat Signal'
 * '<S85>'  : 'gc_pid/angle ctrl/position PID/Anti-windup/Passthrough'
 * '<S86>'  : 'gc_pid/angle ctrl/position PID/D Gain/Internal Parameters'
 * '<S87>'  : 'gc_pid/angle ctrl/position PID/External Derivative/Error'
 * '<S88>'  : 'gc_pid/angle ctrl/position PID/Filter/Disc. Forward Euler Filter'
 * '<S89>'  : 'gc_pid/angle ctrl/position PID/Filter ICs/Internal IC - Filter'
 * '<S90>'  : 'gc_pid/angle ctrl/position PID/I Gain/Internal Parameters'
 * '<S91>'  : 'gc_pid/angle ctrl/position PID/Ideal P Gain/Passthrough'
 * '<S92>'  : 'gc_pid/angle ctrl/position PID/Ideal P Gain Fdbk/Disabled'
 * '<S93>'  : 'gc_pid/angle ctrl/position PID/Integrator/Discrete'
 * '<S94>'  : 'gc_pid/angle ctrl/position PID/Integrator ICs/Internal IC'
 * '<S95>'  : 'gc_pid/angle ctrl/position PID/N Copy/Disabled'
 * '<S96>'  : 'gc_pid/angle ctrl/position PID/N Gain/Internal Parameters'
 * '<S97>'  : 'gc_pid/angle ctrl/position PID/P Copy/Disabled'
 * '<S98>'  : 'gc_pid/angle ctrl/position PID/Parallel P Gain/Internal Parameters'
 * '<S99>'  : 'gc_pid/angle ctrl/position PID/Reset Signal/Disabled'
 * '<S100>' : 'gc_pid/angle ctrl/position PID/Saturation/Passthrough'
 * '<S101>' : 'gc_pid/angle ctrl/position PID/Saturation Fdbk/Disabled'
 * '<S102>' : 'gc_pid/angle ctrl/position PID/Sum/Sum_PID'
 * '<S103>' : 'gc_pid/angle ctrl/position PID/Sum Fdbk/Disabled'
 * '<S104>' : 'gc_pid/angle ctrl/position PID/Tracking Mode/Disabled'
 * '<S105>' : 'gc_pid/angle ctrl/position PID/Tracking Mode Sum/Passthrough'
 * '<S106>' : 'gc_pid/angle ctrl/position PID/Tsamp - Integral/TsSignalSpecification'
 * '<S107>' : 'gc_pid/angle ctrl/position PID/Tsamp - Ngain/Passthrough'
 * '<S108>' : 'gc_pid/angle ctrl/position PID/postSat Signal/Forward_Path'
 * '<S109>' : 'gc_pid/angle ctrl/position PID/preSat Signal/Forward_Path'
 * '<S110>' : 'gc_pid/angle ctrl/rightSpeed PID/Anti-windup'
 * '<S111>' : 'gc_pid/angle ctrl/rightSpeed PID/D Gain'
 * '<S112>' : 'gc_pid/angle ctrl/rightSpeed PID/External Derivative'
 * '<S113>' : 'gc_pid/angle ctrl/rightSpeed PID/Filter'
 * '<S114>' : 'gc_pid/angle ctrl/rightSpeed PID/Filter ICs'
 * '<S115>' : 'gc_pid/angle ctrl/rightSpeed PID/I Gain'
 * '<S116>' : 'gc_pid/angle ctrl/rightSpeed PID/Ideal P Gain'
 * '<S117>' : 'gc_pid/angle ctrl/rightSpeed PID/Ideal P Gain Fdbk'
 * '<S118>' : 'gc_pid/angle ctrl/rightSpeed PID/Integrator'
 * '<S119>' : 'gc_pid/angle ctrl/rightSpeed PID/Integrator ICs'
 * '<S120>' : 'gc_pid/angle ctrl/rightSpeed PID/N Copy'
 * '<S121>' : 'gc_pid/angle ctrl/rightSpeed PID/N Gain'
 * '<S122>' : 'gc_pid/angle ctrl/rightSpeed PID/P Copy'
 * '<S123>' : 'gc_pid/angle ctrl/rightSpeed PID/Parallel P Gain'
 * '<S124>' : 'gc_pid/angle ctrl/rightSpeed PID/Reset Signal'
 * '<S125>' : 'gc_pid/angle ctrl/rightSpeed PID/Saturation'
 * '<S126>' : 'gc_pid/angle ctrl/rightSpeed PID/Saturation Fdbk'
 * '<S127>' : 'gc_pid/angle ctrl/rightSpeed PID/Sum'
 * '<S128>' : 'gc_pid/angle ctrl/rightSpeed PID/Sum Fdbk'
 * '<S129>' : 'gc_pid/angle ctrl/rightSpeed PID/Tracking Mode'
 * '<S130>' : 'gc_pid/angle ctrl/rightSpeed PID/Tracking Mode Sum'
 * '<S131>' : 'gc_pid/angle ctrl/rightSpeed PID/Tsamp - Integral'
 * '<S132>' : 'gc_pid/angle ctrl/rightSpeed PID/Tsamp - Ngain'
 * '<S133>' : 'gc_pid/angle ctrl/rightSpeed PID/postSat Signal'
 * '<S134>' : 'gc_pid/angle ctrl/rightSpeed PID/preSat Signal'
 * '<S135>' : 'gc_pid/angle ctrl/rightSpeed PID/Anti-windup/Disc. Clamping Parallel'
 * '<S136>' : 'gc_pid/angle ctrl/rightSpeed PID/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S137>' : 'gc_pid/angle ctrl/rightSpeed PID/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S138>' : 'gc_pid/angle ctrl/rightSpeed PID/D Gain/Internal Parameters'
 * '<S139>' : 'gc_pid/angle ctrl/rightSpeed PID/External Derivative/Error'
 * '<S140>' : 'gc_pid/angle ctrl/rightSpeed PID/Filter/Disc. Forward Euler Filter'
 * '<S141>' : 'gc_pid/angle ctrl/rightSpeed PID/Filter ICs/Internal IC - Filter'
 * '<S142>' : 'gc_pid/angle ctrl/rightSpeed PID/I Gain/Internal Parameters'
 * '<S143>' : 'gc_pid/angle ctrl/rightSpeed PID/Ideal P Gain/Passthrough'
 * '<S144>' : 'gc_pid/angle ctrl/rightSpeed PID/Ideal P Gain Fdbk/Disabled'
 * '<S145>' : 'gc_pid/angle ctrl/rightSpeed PID/Integrator/Discrete'
 * '<S146>' : 'gc_pid/angle ctrl/rightSpeed PID/Integrator ICs/Internal IC'
 * '<S147>' : 'gc_pid/angle ctrl/rightSpeed PID/N Copy/Disabled'
 * '<S148>' : 'gc_pid/angle ctrl/rightSpeed PID/N Gain/Internal Parameters'
 * '<S149>' : 'gc_pid/angle ctrl/rightSpeed PID/P Copy/Disabled'
 * '<S150>' : 'gc_pid/angle ctrl/rightSpeed PID/Parallel P Gain/Internal Parameters'
 * '<S151>' : 'gc_pid/angle ctrl/rightSpeed PID/Reset Signal/Disabled'
 * '<S152>' : 'gc_pid/angle ctrl/rightSpeed PID/Saturation/Enabled'
 * '<S153>' : 'gc_pid/angle ctrl/rightSpeed PID/Saturation Fdbk/Disabled'
 * '<S154>' : 'gc_pid/angle ctrl/rightSpeed PID/Sum/Sum_PID'
 * '<S155>' : 'gc_pid/angle ctrl/rightSpeed PID/Sum Fdbk/Disabled'
 * '<S156>' : 'gc_pid/angle ctrl/rightSpeed PID/Tracking Mode/Disabled'
 * '<S157>' : 'gc_pid/angle ctrl/rightSpeed PID/Tracking Mode Sum/Passthrough'
 * '<S158>' : 'gc_pid/angle ctrl/rightSpeed PID/Tsamp - Integral/TsSignalSpecification'
 * '<S159>' : 'gc_pid/angle ctrl/rightSpeed PID/Tsamp - Ngain/Passthrough'
 * '<S160>' : 'gc_pid/angle ctrl/rightSpeed PID/postSat Signal/Forward_Path'
 * '<S161>' : 'gc_pid/angle ctrl/rightSpeed PID/preSat Signal/Forward_Path'
 * '<S162>' : 'gc_pid/speed ctrl/Left RPM PID'
 * '<S163>' : 'gc_pid/speed ctrl/Right RPM PID'
 * '<S164>' : 'gc_pid/speed ctrl/Left RPM PID/Anti-windup'
 * '<S165>' : 'gc_pid/speed ctrl/Left RPM PID/D Gain'
 * '<S166>' : 'gc_pid/speed ctrl/Left RPM PID/External Derivative'
 * '<S167>' : 'gc_pid/speed ctrl/Left RPM PID/Filter'
 * '<S168>' : 'gc_pid/speed ctrl/Left RPM PID/Filter ICs'
 * '<S169>' : 'gc_pid/speed ctrl/Left RPM PID/I Gain'
 * '<S170>' : 'gc_pid/speed ctrl/Left RPM PID/Ideal P Gain'
 * '<S171>' : 'gc_pid/speed ctrl/Left RPM PID/Ideal P Gain Fdbk'
 * '<S172>' : 'gc_pid/speed ctrl/Left RPM PID/Integrator'
 * '<S173>' : 'gc_pid/speed ctrl/Left RPM PID/Integrator ICs'
 * '<S174>' : 'gc_pid/speed ctrl/Left RPM PID/N Copy'
 * '<S175>' : 'gc_pid/speed ctrl/Left RPM PID/N Gain'
 * '<S176>' : 'gc_pid/speed ctrl/Left RPM PID/P Copy'
 * '<S177>' : 'gc_pid/speed ctrl/Left RPM PID/Parallel P Gain'
 * '<S178>' : 'gc_pid/speed ctrl/Left RPM PID/Reset Signal'
 * '<S179>' : 'gc_pid/speed ctrl/Left RPM PID/Saturation'
 * '<S180>' : 'gc_pid/speed ctrl/Left RPM PID/Saturation Fdbk'
 * '<S181>' : 'gc_pid/speed ctrl/Left RPM PID/Sum'
 * '<S182>' : 'gc_pid/speed ctrl/Left RPM PID/Sum Fdbk'
 * '<S183>' : 'gc_pid/speed ctrl/Left RPM PID/Tracking Mode'
 * '<S184>' : 'gc_pid/speed ctrl/Left RPM PID/Tracking Mode Sum'
 * '<S185>' : 'gc_pid/speed ctrl/Left RPM PID/Tsamp - Integral'
 * '<S186>' : 'gc_pid/speed ctrl/Left RPM PID/Tsamp - Ngain'
 * '<S187>' : 'gc_pid/speed ctrl/Left RPM PID/postSat Signal'
 * '<S188>' : 'gc_pid/speed ctrl/Left RPM PID/preSat Signal'
 * '<S189>' : 'gc_pid/speed ctrl/Left RPM PID/Anti-windup/Disc. Clamping Parallel'
 * '<S190>' : 'gc_pid/speed ctrl/Left RPM PID/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S191>' : 'gc_pid/speed ctrl/Left RPM PID/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S192>' : 'gc_pid/speed ctrl/Left RPM PID/D Gain/Internal Parameters'
 * '<S193>' : 'gc_pid/speed ctrl/Left RPM PID/External Derivative/Error'
 * '<S194>' : 'gc_pid/speed ctrl/Left RPM PID/Filter/Disc. Forward Euler Filter'
 * '<S195>' : 'gc_pid/speed ctrl/Left RPM PID/Filter ICs/Internal IC - Filter'
 * '<S196>' : 'gc_pid/speed ctrl/Left RPM PID/I Gain/Internal Parameters'
 * '<S197>' : 'gc_pid/speed ctrl/Left RPM PID/Ideal P Gain/Passthrough'
 * '<S198>' : 'gc_pid/speed ctrl/Left RPM PID/Ideal P Gain Fdbk/Disabled'
 * '<S199>' : 'gc_pid/speed ctrl/Left RPM PID/Integrator/Discrete'
 * '<S200>' : 'gc_pid/speed ctrl/Left RPM PID/Integrator ICs/Internal IC'
 * '<S201>' : 'gc_pid/speed ctrl/Left RPM PID/N Copy/Disabled'
 * '<S202>' : 'gc_pid/speed ctrl/Left RPM PID/N Gain/Internal Parameters'
 * '<S203>' : 'gc_pid/speed ctrl/Left RPM PID/P Copy/Disabled'
 * '<S204>' : 'gc_pid/speed ctrl/Left RPM PID/Parallel P Gain/Internal Parameters'
 * '<S205>' : 'gc_pid/speed ctrl/Left RPM PID/Reset Signal/Disabled'
 * '<S206>' : 'gc_pid/speed ctrl/Left RPM PID/Saturation/Enabled'
 * '<S207>' : 'gc_pid/speed ctrl/Left RPM PID/Saturation Fdbk/Disabled'
 * '<S208>' : 'gc_pid/speed ctrl/Left RPM PID/Sum/Sum_PID'
 * '<S209>' : 'gc_pid/speed ctrl/Left RPM PID/Sum Fdbk/Disabled'
 * '<S210>' : 'gc_pid/speed ctrl/Left RPM PID/Tracking Mode/Disabled'
 * '<S211>' : 'gc_pid/speed ctrl/Left RPM PID/Tracking Mode Sum/Passthrough'
 * '<S212>' : 'gc_pid/speed ctrl/Left RPM PID/Tsamp - Integral/TsSignalSpecification'
 * '<S213>' : 'gc_pid/speed ctrl/Left RPM PID/Tsamp - Ngain/Passthrough'
 * '<S214>' : 'gc_pid/speed ctrl/Left RPM PID/postSat Signal/Forward_Path'
 * '<S215>' : 'gc_pid/speed ctrl/Left RPM PID/preSat Signal/Forward_Path'
 * '<S216>' : 'gc_pid/speed ctrl/Right RPM PID/Anti-windup'
 * '<S217>' : 'gc_pid/speed ctrl/Right RPM PID/D Gain'
 * '<S218>' : 'gc_pid/speed ctrl/Right RPM PID/External Derivative'
 * '<S219>' : 'gc_pid/speed ctrl/Right RPM PID/Filter'
 * '<S220>' : 'gc_pid/speed ctrl/Right RPM PID/Filter ICs'
 * '<S221>' : 'gc_pid/speed ctrl/Right RPM PID/I Gain'
 * '<S222>' : 'gc_pid/speed ctrl/Right RPM PID/Ideal P Gain'
 * '<S223>' : 'gc_pid/speed ctrl/Right RPM PID/Ideal P Gain Fdbk'
 * '<S224>' : 'gc_pid/speed ctrl/Right RPM PID/Integrator'
 * '<S225>' : 'gc_pid/speed ctrl/Right RPM PID/Integrator ICs'
 * '<S226>' : 'gc_pid/speed ctrl/Right RPM PID/N Copy'
 * '<S227>' : 'gc_pid/speed ctrl/Right RPM PID/N Gain'
 * '<S228>' : 'gc_pid/speed ctrl/Right RPM PID/P Copy'
 * '<S229>' : 'gc_pid/speed ctrl/Right RPM PID/Parallel P Gain'
 * '<S230>' : 'gc_pid/speed ctrl/Right RPM PID/Reset Signal'
 * '<S231>' : 'gc_pid/speed ctrl/Right RPM PID/Saturation'
 * '<S232>' : 'gc_pid/speed ctrl/Right RPM PID/Saturation Fdbk'
 * '<S233>' : 'gc_pid/speed ctrl/Right RPM PID/Sum'
 * '<S234>' : 'gc_pid/speed ctrl/Right RPM PID/Sum Fdbk'
 * '<S235>' : 'gc_pid/speed ctrl/Right RPM PID/Tracking Mode'
 * '<S236>' : 'gc_pid/speed ctrl/Right RPM PID/Tracking Mode Sum'
 * '<S237>' : 'gc_pid/speed ctrl/Right RPM PID/Tsamp - Integral'
 * '<S238>' : 'gc_pid/speed ctrl/Right RPM PID/Tsamp - Ngain'
 * '<S239>' : 'gc_pid/speed ctrl/Right RPM PID/postSat Signal'
 * '<S240>' : 'gc_pid/speed ctrl/Right RPM PID/preSat Signal'
 * '<S241>' : 'gc_pid/speed ctrl/Right RPM PID/Anti-windup/Disc. Clamping Parallel'
 * '<S242>' : 'gc_pid/speed ctrl/Right RPM PID/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S243>' : 'gc_pid/speed ctrl/Right RPM PID/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S244>' : 'gc_pid/speed ctrl/Right RPM PID/D Gain/Internal Parameters'
 * '<S245>' : 'gc_pid/speed ctrl/Right RPM PID/External Derivative/Error'
 * '<S246>' : 'gc_pid/speed ctrl/Right RPM PID/Filter/Disc. Forward Euler Filter'
 * '<S247>' : 'gc_pid/speed ctrl/Right RPM PID/Filter ICs/Internal IC - Filter'
 * '<S248>' : 'gc_pid/speed ctrl/Right RPM PID/I Gain/Internal Parameters'
 * '<S249>' : 'gc_pid/speed ctrl/Right RPM PID/Ideal P Gain/Passthrough'
 * '<S250>' : 'gc_pid/speed ctrl/Right RPM PID/Ideal P Gain Fdbk/Disabled'
 * '<S251>' : 'gc_pid/speed ctrl/Right RPM PID/Integrator/Discrete'
 * '<S252>' : 'gc_pid/speed ctrl/Right RPM PID/Integrator ICs/Internal IC'
 * '<S253>' : 'gc_pid/speed ctrl/Right RPM PID/N Copy/Disabled'
 * '<S254>' : 'gc_pid/speed ctrl/Right RPM PID/N Gain/Internal Parameters'
 * '<S255>' : 'gc_pid/speed ctrl/Right RPM PID/P Copy/Disabled'
 * '<S256>' : 'gc_pid/speed ctrl/Right RPM PID/Parallel P Gain/Internal Parameters'
 * '<S257>' : 'gc_pid/speed ctrl/Right RPM PID/Reset Signal/Disabled'
 * '<S258>' : 'gc_pid/speed ctrl/Right RPM PID/Saturation/Enabled'
 * '<S259>' : 'gc_pid/speed ctrl/Right RPM PID/Saturation Fdbk/Disabled'
 * '<S260>' : 'gc_pid/speed ctrl/Right RPM PID/Sum/Sum_PID'
 * '<S261>' : 'gc_pid/speed ctrl/Right RPM PID/Sum Fdbk/Disabled'
 * '<S262>' : 'gc_pid/speed ctrl/Right RPM PID/Tracking Mode/Disabled'
 * '<S263>' : 'gc_pid/speed ctrl/Right RPM PID/Tracking Mode Sum/Passthrough'
 * '<S264>' : 'gc_pid/speed ctrl/Right RPM PID/Tsamp - Integral/TsSignalSpecification'
 * '<S265>' : 'gc_pid/speed ctrl/Right RPM PID/Tsamp - Ngain/Passthrough'
 * '<S266>' : 'gc_pid/speed ctrl/Right RPM PID/postSat Signal/Forward_Path'
 * '<S267>' : 'gc_pid/speed ctrl/Right RPM PID/preSat Signal/Forward_Path'
 * '<S268>' : 'gc_pid/track ctrl/Discrete PID Controller'
 * '<S269>' : 'gc_pid/track ctrl/Discrete PID Controller/Anti-windup'
 * '<S270>' : 'gc_pid/track ctrl/Discrete PID Controller/D Gain'
 * '<S271>' : 'gc_pid/track ctrl/Discrete PID Controller/External Derivative'
 * '<S272>' : 'gc_pid/track ctrl/Discrete PID Controller/Filter'
 * '<S273>' : 'gc_pid/track ctrl/Discrete PID Controller/Filter ICs'
 * '<S274>' : 'gc_pid/track ctrl/Discrete PID Controller/I Gain'
 * '<S275>' : 'gc_pid/track ctrl/Discrete PID Controller/Ideal P Gain'
 * '<S276>' : 'gc_pid/track ctrl/Discrete PID Controller/Ideal P Gain Fdbk'
 * '<S277>' : 'gc_pid/track ctrl/Discrete PID Controller/Integrator'
 * '<S278>' : 'gc_pid/track ctrl/Discrete PID Controller/Integrator ICs'
 * '<S279>' : 'gc_pid/track ctrl/Discrete PID Controller/N Copy'
 * '<S280>' : 'gc_pid/track ctrl/Discrete PID Controller/N Gain'
 * '<S281>' : 'gc_pid/track ctrl/Discrete PID Controller/P Copy'
 * '<S282>' : 'gc_pid/track ctrl/Discrete PID Controller/Parallel P Gain'
 * '<S283>' : 'gc_pid/track ctrl/Discrete PID Controller/Reset Signal'
 * '<S284>' : 'gc_pid/track ctrl/Discrete PID Controller/Saturation'
 * '<S285>' : 'gc_pid/track ctrl/Discrete PID Controller/Saturation Fdbk'
 * '<S286>' : 'gc_pid/track ctrl/Discrete PID Controller/Sum'
 * '<S287>' : 'gc_pid/track ctrl/Discrete PID Controller/Sum Fdbk'
 * '<S288>' : 'gc_pid/track ctrl/Discrete PID Controller/Tracking Mode'
 * '<S289>' : 'gc_pid/track ctrl/Discrete PID Controller/Tracking Mode Sum'
 * '<S290>' : 'gc_pid/track ctrl/Discrete PID Controller/Tsamp - Integral'
 * '<S291>' : 'gc_pid/track ctrl/Discrete PID Controller/Tsamp - Ngain'
 * '<S292>' : 'gc_pid/track ctrl/Discrete PID Controller/postSat Signal'
 * '<S293>' : 'gc_pid/track ctrl/Discrete PID Controller/preSat Signal'
 * '<S294>' : 'gc_pid/track ctrl/Discrete PID Controller/Anti-windup/Passthrough'
 * '<S295>' : 'gc_pid/track ctrl/Discrete PID Controller/D Gain/Internal Parameters'
 * '<S296>' : 'gc_pid/track ctrl/Discrete PID Controller/External Derivative/Error'
 * '<S297>' : 'gc_pid/track ctrl/Discrete PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S298>' : 'gc_pid/track ctrl/Discrete PID Controller/Filter ICs/Internal IC - Filter'
 * '<S299>' : 'gc_pid/track ctrl/Discrete PID Controller/I Gain/Internal Parameters'
 * '<S300>' : 'gc_pid/track ctrl/Discrete PID Controller/Ideal P Gain/Passthrough'
 * '<S301>' : 'gc_pid/track ctrl/Discrete PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S302>' : 'gc_pid/track ctrl/Discrete PID Controller/Integrator/Discrete'
 * '<S303>' : 'gc_pid/track ctrl/Discrete PID Controller/Integrator ICs/Internal IC'
 * '<S304>' : 'gc_pid/track ctrl/Discrete PID Controller/N Copy/Disabled'
 * '<S305>' : 'gc_pid/track ctrl/Discrete PID Controller/N Gain/Internal Parameters'
 * '<S306>' : 'gc_pid/track ctrl/Discrete PID Controller/P Copy/Disabled'
 * '<S307>' : 'gc_pid/track ctrl/Discrete PID Controller/Parallel P Gain/Internal Parameters'
 * '<S308>' : 'gc_pid/track ctrl/Discrete PID Controller/Reset Signal/Disabled'
 * '<S309>' : 'gc_pid/track ctrl/Discrete PID Controller/Saturation/Passthrough'
 * '<S310>' : 'gc_pid/track ctrl/Discrete PID Controller/Saturation Fdbk/Disabled'
 * '<S311>' : 'gc_pid/track ctrl/Discrete PID Controller/Sum/Sum_PID'
 * '<S312>' : 'gc_pid/track ctrl/Discrete PID Controller/Sum Fdbk/Disabled'
 * '<S313>' : 'gc_pid/track ctrl/Discrete PID Controller/Tracking Mode/Disabled'
 * '<S314>' : 'gc_pid/track ctrl/Discrete PID Controller/Tracking Mode Sum/Passthrough'
 * '<S315>' : 'gc_pid/track ctrl/Discrete PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S316>' : 'gc_pid/track ctrl/Discrete PID Controller/Tsamp - Ngain/Passthrough'
 * '<S317>' : 'gc_pid/track ctrl/Discrete PID Controller/postSat Signal/Forward_Path'
 * '<S318>' : 'gc_pid/track ctrl/Discrete PID Controller/preSat Signal/Forward_Path'
 */
#endif                                 /* gc_pid_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
