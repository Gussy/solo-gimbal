/*
 * motor_drive_state_machine.h
 *
 *  Created on: Jan 7, 2015
 *      Author: abamberger
 */

#ifndef MOTOR_DRIVE_STATE_MACHINE_H_
#define MOTOR_DRIVE_STATE_MACHINE_H_

#include "PM_Sensorless.h"
#include "control/running_average_filter.h"
#include "control/average_power_filter.h"
#include "load_axis_parms_state_machine.h"

#define PRE_INIT_TIME_MS 2000
#define FAULT_REVIVE_TIME_MS 1000

typedef enum {
    STATE_PRE_INIT,
    STATE_INIT,
    STATE_LOAD_OWN_INIT_PARAMS,
    STATE_REQUEST_AXIS_INIT_PARAMS,
    STATE_WAIT_FOR_OTHER_AXES_INIT_PARAMS_LOADED,
    STATE_COMMAND_AZ_INIT,
    STATE_WAIT_FOR_AZ_INIT,
    STATE_COMMAND_ROLL_INIT,
    STATE_WAIT_FOR_ROLL_INIT,
    STATE_CALIBRATING_CURRENT_MEASUREMENTS,
    STATE_TAKE_COMMUTATION_CALIBRATION_DATA,
    STATE_HOMING,
    STATE_WAIT_FOR_AXES_HOME,
    STATE_INITIALIZE_POSITION_LOOPS,
    STATE_RUNNING,
    STATE_DISABLED,
    STATE_SANDSTORM,
    STATE_FAULT
} MotorDriveState;

typedef struct {
    MotorDriveState motor_drive_state;
    PARK park_xform_parms;
    CLARKE clarke_xform_parms;
    IPARK ipark_xform_parms;
    PID_GRANDO_CONTROLLER pid_id;
    PID_GRANDO_CONTROLLER pid_iq;
    SVGENDQ svgen_parms;
    PWMGEN pwm_gen_parms;
    RAMPGEN rg1;
    _iq cal_offset_A;
    _iq cal_offset_B;
    _iq cal_filt_gain;
    _iq iq_ref;
    Uint32 current_cal_timer;
    Uint32 pre_init_timer;
    Uint32 fault_revive_counter;
} MotorDriveParms;

void MotorDriveStateMachine(AxisParms* axis_parms, ControlBoardParms* cb_parms, MotorDriveParms* md_parms, EncoderParms* encoder_parms, ParamSet* param_set, RunningAvgFilterParms* pos_loop_stage_1, RunningAvgFilterParms* pos_loop_stage_2, AveragePowerFilterParms* pf_parms, LoadAxisParmsStateInfo* load_ap_state_info);

#endif /* MOTOR_DRIVE_STATE_MACHINE_H_ */
