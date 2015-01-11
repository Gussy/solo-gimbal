/*
 * motor_drive_state_machine.c
 *
 *  Created on: Jan 7, 2015
 *      Author: abamberger
 */

#include "motor_drive_state_machine.h"
#include "PM_Sensorless.h"
#include "PM_Sensorless-Settings.h"
#include "cand.h"
#include "cb.h"
#include "device_init.h"
#include "commutation_calibration_state_machine.h"
#include "homing_calibration_state_machine.h"
#include "load_axis_parms_state_machine.h"
#include "PID.h"
#include "PeripheralHeaderIncludes.h"

#include <string.h>

CommutationCalibrationParms cc_parms = {
    COMMUTATION_CALIBRATION_STATE_INIT,     // Commutation calibration state
    0,                                      // Current iteration
    0,                                      // Current electrical cycle
    0,                                      // Current electrical sub-cycle
    0,                                      // Current direction
    0,                                      // Settling timer
    RMPCNTL_DEFAULTS,                       // Ramp control parameters
    {0}                                     // Calibration data
};

HomingCalibrationParms hc_parms = {
    HOMING_STATE_INIT,          // Current homing state
    HOMING_STATE_INIT,          // Next homing state
    AXIS_STOP_1,                // Current stop being searched for
    HOMING_DIRECTION_FORWARD,   // Current homing direction
    0,                          // Current find stop iteration
    0,                          // Current find stop stall count
    0,                          // Mechanical stop 1 encoder location
    0,                          // Mechanical stop 2 encoder location
    0,                          // Settling timer
    0.0,                        // Last mechanical theta,
    0.0000001,                  // EPSILON
    RMPCNTL_DEFAULTS            // Ramp control parameters
};

LoadAxisParmsStateInfo load_ap_state_info = {
    LOAD_AXIS_PARMS_STATE_LOAD_RATE_P,      // Load axis parms state
    EL,                                     // Current load axis
    FALSE,                                  // Axis parms load complete
};

void MotorDriveStateMachine(AxisParms* axis_parms, ControlBoardParms* cb_parms, MotorDriveParms* md_parms, EncoderParms* encoder_parms, ParamSet* param_set, RunningAvgFilterParms* pos_loop_stage_1, RunningAvgFilterParms* pos_loop_stage_2, AveragePowerFilterParms* pf_parms)
{
    switch (md_parms->motor_drive_state) {
        case STATE_PRE_INIT:
            // Set park transformation angle to 0
            md_parms->park_xform_parms.Angle = 0;

            // Drive id and iq to 0 (not driving the motor)
            md_parms->pid_id.term.Ref = 0;
            md_parms->pid_iq.term.Ref = 0;

            md_parms->pre_init_timer++;
            if (md_parms->pre_init_timer > (((Uint32)ISR_FREQUENCY) * ((Uint32)PRE_INIT_TIME_MS))) {
                md_parms->motor_drive_state = STATE_INIT;
            }
            break;

        case STATE_INIT:
            md_parms->current_cal_timer = 0;

            // Clear axis home flags
            memset(cb_parms->axes_homed, 0x00, AXIS_CNT * sizeof(Uint8));

            // Set park transformation angle to 0
            md_parms->park_xform_parms.Angle = 0;

            // Drive id and iq to 0 (not driving the motor)
            md_parms->pid_id.term.Ref = 0;
            md_parms->pid_iq.term.Ref = 0;

            // Enable periodic transmission of the BIT CAN message
            axis_parms->BIT_heartbeat_enable = TRUE;

            // If we're the EL board, transmit an enable message to the other boards
            if (GetBoardHWID() == EL) {
#ifndef ENABLE_AXIS_CALIBRATION_PROCEDURE
                cand_tx_command(CAND_ID_ALL_AXES, CAND_CMD_ENABLE);
#endif
            }

            //TODO: Temporarily sequencing the initialization of the different axes
            /*
            if (board_hw_id == EL) {
                // We're the control board, so need to go through the init sequence
                md_parms->motor_drive_state = STATE_COMMAND_AZ_INIT;
            } else {
                // We're not the control board, so just need to init ourselves
                md_parms->motor_drive_state = STATE_CALIBRATING_CURRENT_MEASUREMENTS;
            }
            */

            md_parms->rg1.Freq = 10;

            // Set our own blink state to init
            axis_parms->blink_state = BLINK_INIT;

            if (GetBoardHWID() == AZ) {
                // If we're the AZ board, we first have to load our own parameters from flash, then transmit parameters
                // to the other axes.
                md_parms->motor_drive_state = STATE_LOAD_OWN_INIT_PARAMS;
            } else {
                // If we're EL or ROLL, we have to receive our parameters from the AZ board
                md_parms->motor_drive_state = STATE_WAIT_FOR_OWN_AXIS_INIT_PARAMS;
            }
            break;

        case STATE_LOAD_OWN_INIT_PARAMS:
            // This state is only run on the AZ board to load commutation calibration parameters and torque loop PID gains
            // The rate loop PID gains are only needed on the EL board, so these are loaded over CAN
            // TODO: Load our own commutation calibration parameters and torque loop PID gains.  Just testing with rate loop PID gains for now

            // After we've loaded our own init parameters, make a note of it, so we can continue later when we're waiting for
            // all axes to have received their init parameters
            axis_parms->other_axis_init_params_recvd[AZ] = TRUE;

            // Fake that we got a heartbeat from ourself, because if we're here CAN is initialized and ready to go
            axis_parms->other_axis_hb_recvd[AZ] = TRUE;

            // Once we're done loading our own parameters, we need to wait to hear a heartbeat from the other axes,
            // and we can then start transmitting parameters to them
            md_parms->motor_drive_state = STATE_WAIT_FOR_OTHER_AXIS_HEARTBEATS;
            break;

        case STATE_WAIT_FOR_OTHER_AXIS_HEARTBEATS:
            // Once we've received one BIT heartbeat from each axis, we're ok to start transmitting parameters
            if (axis_parms->other_axis_hb_recvd[EL] && axis_parms->other_axis_hb_recvd[AZ] && axis_parms->other_axis_hb_recvd[ROLL]) {
                md_parms->motor_drive_state = STATE_TRANSMIT_INIT_PARAMS;
            }
            break;

        case STATE_TRANSMIT_INIT_PARAMS:
            // Run the load init parms state machine to sequence through loading the axis parms
            LoadAxisParmsStateMachine(&load_ap_state_info);

            // If we've completed transmitting the params to the other axes,
            // we need to wait to receive confirmation from the axes that they've
            // received all of their parameters before we continue
            if (load_ap_state_info.axis_parms_load_complete) {
                md_parms->motor_drive_state = STATE_WAIT_FOR_OTHER_AXES_INIT_PARAMS_LOADED;
            }
            break;

        case STATE_WAIT_FOR_OWN_AXIS_INIT_PARAMS:
        {
            // Set park transformation angle to 0
            md_parms->park_xform_parms.Angle = 0;

            // Drive id and iq to 0 (not driving the motor)
            md_parms->pid_id.term.Ref = 0;
            md_parms->pid_iq.term.Ref = 0;

            // Check the received params to see if we got any of the parameters we're waiting for
            IntOrFloat float_converter;
            // Rate loop EL PID parameters
            if (*(param_set[CAND_PID_RATE_EL_P].sema)) {
                float_converter.uint32_val = param_set[CAND_PID_RATE_EL_P].param;
                *(param_set[CAND_PID_RATE_EL_P].sema) = FALSE;
                rate_pid_loop_float[EL].gainP = float_converter.float_val;
                axis_parms->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_EL_P_RECVD;
            }
            if (*(param_set[CAND_PID_RATE_EL_I].sema)) {
                float_converter.uint32_val = param_set[CAND_PID_RATE_EL_I].param;
                *(param_set[CAND_PID_RATE_EL_I].sema) = FALSE;
                rate_pid_loop_float[EL].gainI = float_converter.float_val;
                axis_parms->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_EL_I_RECVD;
            }
            if (*(param_set[CAND_PID_RATE_EL_D].sema)) {
                float_converter.uint32_val = param_set[CAND_PID_RATE_EL_D].param;
                *(param_set[CAND_PID_RATE_EL_D].sema) = FALSE;
                rate_pid_loop_float[EL].gainD = float_converter.float_val;
                axis_parms->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_EL_D_RECVD;
            }
            if (*(param_set[CAND_PID_RATE_EL_WINDUP].sema)) {
                float_converter.uint32_val = param_set[CAND_PID_RATE_EL_WINDUP].param;
                *(param_set[CAND_PID_RATE_EL_WINDUP].sema) = FALSE;
                rate_pid_loop_float[EL].integralMax = float_converter.float_val;
                rate_pid_loop_float[EL].integralMin = -float_converter.float_val;
                axis_parms->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_EL_WINDUP_RECVD;
            }
            // Rate loop AZ PID Parameters
            if (*(param_set[CAND_PID_RATE_AZ_P].sema)) {
                float_converter.uint32_val = param_set[CAND_PID_RATE_AZ_P].param;
                *(param_set[CAND_PID_RATE_AZ_P].sema) = FALSE;
                rate_pid_loop_float[AZ].gainP = float_converter.float_val;
                axis_parms->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_AZ_P_RECVD;
            }
            if (*(param_set[CAND_PID_RATE_AZ_I].sema)) {
                float_converter.uint32_val = param_set[CAND_PID_RATE_AZ_I].param;
                *(param_set[CAND_PID_RATE_AZ_I].sema) = FALSE;
                rate_pid_loop_float[AZ].gainI = float_converter.float_val;
                axis_parms->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_AZ_I_RECVD;
            }
            if (*(param_set[CAND_PID_RATE_AZ_D].sema)) {
                float_converter.uint32_val = param_set[CAND_PID_RATE_AZ_D].param;
                *(param_set[CAND_PID_RATE_AZ_D].sema) = FALSE;
                rate_pid_loop_float[AZ].gainD = float_converter.float_val;
                axis_parms->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_AZ_D_RECVD;
            }
            if (*(param_set[CAND_PID_RATE_AZ_WINDUP].sema)) {
                float_converter.uint32_val = param_set[CAND_PID_RATE_AZ_WINDUP].param;
                *(param_set[CAND_PID_RATE_AZ_WINDUP].sema) = FALSE;
                rate_pid_loop_float[AZ].integralMax = float_converter.float_val;
                rate_pid_loop_float[AZ].integralMin = -float_converter.float_val;
                axis_parms->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_AZ_WINDUP_RECVD;
            }
            // Rate loop ROLL PID Parameters
            if (*(param_set[CAND_PID_RATE_RL_P].sema)) {
                float_converter.uint32_val = param_set[CAND_PID_RATE_RL_P].param;
                *(param_set[CAND_PID_RATE_RL_P].sema) = FALSE;
                rate_pid_loop_float[ROLL].gainP = float_converter.float_val;
                axis_parms->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_RL_P_RECVD;
            }
            if (*(param_set[CAND_PID_RATE_RL_I].sema)) {
                float_converter.uint32_val = param_set[CAND_PID_RATE_RL_I].param;
                *(param_set[CAND_PID_RATE_RL_I].sema) = FALSE;
                rate_pid_loop_float[ROLL].gainI = float_converter.float_val;
                axis_parms->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_RL_I_RECVD;
            }
            if (*(param_set[CAND_PID_RATE_RL_D].sema)) {
                float_converter.uint32_val = param_set[CAND_PID_RATE_RL_D].param;
                *(param_set[CAND_PID_RATE_RL_D].sema) = FALSE;
                rate_pid_loop_float[ROLL].gainD = float_converter.float_val;
                axis_parms->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_RL_D_RECVD;
            }
            if (*(param_set[CAND_PID_RATE_RL_WINDUP].sema)) {
                float_converter.uint32_val = param_set[CAND_PID_RATE_RL_WINDUP].param;
                *(param_set[CAND_PID_RATE_RL_WINDUP].sema) = FALSE;
                rate_pid_loop_float[ROLL].integralMax = float_converter.float_val;
                rate_pid_loop_float[ROLL].integralMin = -float_converter.float_val;
                axis_parms->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_RL_WINDUP_RECVD;
            }
            // Commutation calibration parameters
            if (*(param_set[CAND_PID_COMMUTATION_CALIBRATION_SLOPE].sema)) {
                float_converter.uint32_val = param_set[CAND_PID_COMMUTATION_CALIBRATION_SLOPE].param;
                *(param_set[CAND_PID_COMMUTATION_CALIBRATION_SLOPE].sema) = FALSE;
                AxisCalibrationSlopes[GIMBAL_TARGET][GetBoardHWID()] = float_converter.float_val;
                axis_parms->init_param_recvd_flags_2 |= INIT_PARAM_COMMUTATION_CALIBRATION_SLOPE_RECVD;
            }
            if (*(param_set[CAND_PID_COMMUTATION_CALIBRATION_INTERCEPT].sema)) {
                float_converter.uint32_val = param_set[CAND_PID_COMMUTATION_CALIBRATION_INTERCEPT].param;
                *(param_set[CAND_PID_COMMUTATION_CALIBRATION_INTERCEPT].sema) = FALSE;
                AxisCalibrationIntercepts[GIMBAL_TARGET][GetBoardHWID()] = float_converter.float_val;
                axis_parms->init_param_recvd_flags_2 |= INIT_PARAM_COMMUTATION_CALIBRATION_INTERCEPT_RECVD;
            }
            if (*(param_set[CAND_PID_COMMUTATION_CALIBRATION_HOME_OFFSET].sema)) {
                AxisHomePositions[GIMBAL_TARGET][GetBoardHWID()] = param_set[CAND_PID_COMMUTATION_CALIBRATION_HOME_OFFSET].param;
                *(param_set[CAND_PID_COMMUTATION_CALIBRATION_HOME_OFFSET].sema) = FALSE;
                axis_parms->init_param_recvd_flags_2 |= INIT_PARAM_COMMUTATION_CALIBRATION_HOME_OFFSET_RECVD;
            }
            // Torque loop PID parameters
            if (*(param_set[CAND_PID_TORQUE_KP].sema)) {
                float_converter.uint32_val = param_set[CAND_PID_TORQUE_KP].param;
                *(param_set[CAND_PID_TORQUE_KP].sema) = FALSE;
                md_parms->pid_id.param.Kp = float_converter.float_val;
                md_parms->pid_iq.param.Kp = float_converter.float_val;
                axis_parms->init_param_recvd_flags_2 |= INIT_PARAM_TORQUE_PID_KP_RECVD;
            }
            if (*(param_set[CAND_PID_TORQUE_KI].sema)) {
                float_converter.uint32_val = param_set[CAND_PID_TORQUE_KI].param;
                *(param_set[CAND_PID_TORQUE_KI].sema) = FALSE;
                md_parms->pid_id.param.Ki = float_converter.float_val;
                md_parms->pid_iq.param.Ki = float_converter.float_val;
                axis_parms->init_param_recvd_flags_2 |= INIT_PARAM_TORQUE_PID_KI_RECVD;
            }
            if (*(param_set[CAND_PID_TORQUE_KD].sema)) {
                float_converter.uint32_val = param_set[CAND_PID_TORQUE_KD].param;
                *(param_set[CAND_PID_TORQUE_KD].sema) = FALSE;
                md_parms->pid_id.param.Kd = float_converter.float_val;
                md_parms->pid_iq.param.Kd = float_converter.float_val;
                axis_parms->init_param_recvd_flags_2 |= INIT_PARAM_TORQUE_PID_KD_RECVD;
            }

            // Wait for all of the init parameters we need to be received
            if (GetBoardHWID() == EL) {
                // If we're the EL axis, we receive commutation calibration parameters, torque loop PID gains, and rate loop PID gains
                // TODO: Check for receipt of all parameters here, only testing with rate loop PID gains for now
                if (axis_parms->init_param_recvd_flags_1 == ALL_INIT_PARAMS_RECVD_1) {
                    axis_parms->all_init_params_recvd = TRUE;
                    // We've received all of our parameters, so we can continue our init sequence
                    md_parms->motor_drive_state = STATE_CALIBRATING_CURRENT_MEASUREMENTS;
                }
            } else {
                // If we're the ROLL axis, we receive only commutation calibration parameters and torque loop PID gains
                // The AZ axis loads its parameters itself, and never runs this state, so we don't need to worry about it here
                // TODO: Check for receipt of parameters here, only testing with rate loop PID gains for now
                axis_parms->all_init_params_recvd = TRUE;
                // We've received all of our parameters, so we can continue our init sequence
                md_parms->motor_drive_state = STATE_CALIBRATING_CURRENT_MEASUREMENTS;
            }
        }
        break;

        case STATE_WAIT_FOR_OTHER_AXES_INIT_PARAMS_LOADED:
            // Wait for all of the axes to have received their init parameters.  These flags are updated in response to a bit set in the periodic BIT
            // messages that all axes send
            if (axis_parms->other_axis_init_params_recvd[EL] && axis_parms->other_axis_init_params_recvd[AZ] && axis_parms->other_axis_init_params_recvd[ROLL]) {
                // If all axes have received their init parameters, we can continue our init sequence
                md_parms->motor_drive_state = STATE_CALIBRATING_CURRENT_MEASUREMENTS;
            }
            break;

        case STATE_COMMAND_AZ_INIT:
        case STATE_WAIT_FOR_AZ_INIT:
        case STATE_COMMAND_ROLL_INIT:
        case STATE_WAIT_FOR_ROLL_INIT:
            // Set park transformation angle to 0
            md_parms->park_xform_parms.Angle = 0;

            // Drive id and iq to 0 (not driving the motor)
            md_parms->pid_id.term.Ref = 0;
            md_parms->pid_iq.term.Ref = 0;

            switch (md_parms->motor_drive_state) {
            case STATE_COMMAND_AZ_INIT:
                cand_tx_command(CAND_ID_AZ, CAND_CMD_ENABLE);
                md_parms->motor_drive_state = STATE_WAIT_FOR_AZ_INIT;
                break;

            case STATE_WAIT_FOR_AZ_INIT:
                if (cb_parms->axes_homed[AZ] == TRUE) {
                    md_parms->motor_drive_state = STATE_COMMAND_ROLL_INIT;
                }
                break;

            case STATE_COMMAND_ROLL_INIT:
                cand_tx_command(CAND_ID_ROLL, CAND_CMD_ENABLE);
                md_parms->motor_drive_state = STATE_WAIT_FOR_ROLL_INIT;
                break;

            case STATE_WAIT_FOR_ROLL_INIT:
                if (cb_parms->axes_homed[ROLL] == TRUE) {
                    md_parms->motor_drive_state = STATE_CALIBRATING_CURRENT_MEASUREMENTS;
                }
                break;
            }
            break;

        case STATE_CALIBRATING_CURRENT_MEASUREMENTS:
            //  LPF to average the calibration offsets.  Run this for a few seconds at boot to calibrate the phase current measurements
            md_parms->cal_offset_A = _IQ15mpy(md_parms->cal_filt_gain, _IQtoIQ15(md_parms->clarke_xform_parms.As)) + md_parms->cal_offset_A;
            md_parms->cal_offset_B = _IQ15mpy(md_parms->cal_filt_gain, _IQtoIQ15(md_parms->clarke_xform_parms.Bs)) + md_parms->cal_offset_B;

            // Set park transformation angle to 0
            md_parms->park_xform_parms.Angle = 0;

            // Drive id and iq to 0 (not driving the motor)
            md_parms->pid_id.term.Ref = 0;
            md_parms->pid_iq.term.Ref = 0;

            // Time out the calibration time.  After it has expired, transition to the next state
            md_parms->current_cal_timer++;
            if (md_parms->current_cal_timer > (((Uint32)ISR_FREQUENCY) * ((Uint32)CURRENT_CALIBRATION_TIME_MS))) {
#ifdef ENABLE_AXIS_CALIBRATION_PROCEDURE
                md_parms->motor_drive_state = STATE_TAKE_COMMUTATION_CALIBRATION_DATA;
#else
                md_parms->motor_drive_state = STATE_HOMING;
#endif
            }
            break;

        case STATE_TAKE_COMMUTATION_CALIBRATION_DATA:
            CommutationCalibrationStateMachine(md_parms, encoder_parms, &cc_parms);
            break;

        case STATE_HOMING:
            HomingCalibrationStateMachine(md_parms, encoder_parms, cb_parms, axis_parms, &hc_parms);
            break;

        case STATE_WAIT_FOR_AXES_HOME:
            // Set park transformation angle to 0
            md_parms->park_xform_parms.Angle = 0;

            // Drive id and iq to 0
            md_parms->pid_id.term.Ref = 0;
            md_parms->pid_iq.term.Ref = 0;

            if ((cb_parms->axes_homed[AZ] == TRUE) && (cb_parms->axes_homed[EL] == TRUE) && (cb_parms->axes_homed[ROLL] == TRUE)) {
                //cb_parms->enabled = TRUE;
                //md_parms->motor_drive_state = STATE_RUNNING;
                //axis_parms->blink_state = BLINK_READY;
                md_parms->motor_drive_state = STATE_INITIALIZE_POSITION_LOOPS;
            }
            break;

        case STATE_INITIALIZE_POSITION_LOOPS:
            if ((cb_parms->encoder_value_received[AZ] == TRUE) &&
                    (cb_parms->encoder_value_received[EL] == TRUE) &&
                    (cb_parms->encoder_value_received[ROLL == TRUE])) {
                // Now that we've received all of the encoder values, we can go ahead and initialize the position loop filters
                initialize_running_average_filter(pos_loop_stage_1,
                        CorrectEncoderError(cb_parms->encoder_readings[AZ]),
                        CorrectEncoderError(cb_parms->encoder_readings[EL]),
                        CorrectEncoderError(cb_parms->encoder_readings[ROLL]));

                initialize_running_average_filter(pos_loop_stage_2,
                        pos_loop_stage_1->az_avg,
                        pos_loop_stage_1->el_avg,
                        pos_loop_stage_1->rl_avg);

                // Now we're ready to move to the running state
                cb_parms->enabled = TRUE;
                md_parms->motor_drive_state = STATE_RUNNING;
                axis_parms->blink_state = BLINK_READY;
            } else {
                // Send a zero torque command to the other axes to generate an encoder response
                // (we update our own encoder value in a different place)
                MDBSendTorques(0, 0);
            }
            break;

        case STATE_RUNNING:
            // Set park transformation angle to currently measured rotor electrical angle
            md_parms->park_xform_parms.Angle = encoder_parms->elec_theta;

            // Drive id to 0, iq to requested torque from rate loop (to commutate motor at requested torque)
            md_parms->pid_id.term.Ref = 0;
            md_parms->pid_iq.term.Ref = md_parms->iq_ref;

            // If new current command from CAN bus get it.
            if (*param_set[CAND_PID_TORQUE].sema) {
                CBSendEncoder(encoder_parms->virtual_counts);
                md_parms->iq_ref = ((int16) param_set[CAND_PID_TORQUE].param) / 32767.0;
                *param_set[CAND_PID_TORQUE].sema = FALSE;
            }
            break;

        case STATE_DISABLED:
            // Set park transformation angle to 0
            md_parms->park_xform_parms.Angle = 0;

            // Request 0 current on both id and iq
            md_parms->pid_id.term.Ref = 0;
            md_parms->pid_iq.term.Ref = 0;
            break;

        case STATE_FAULT:
            // Set park transformation angle to 0
            md_parms->park_xform_parms.Angle = 0;

            // Request 0 current on both id and iq
            md_parms->pid_id.term.Ref = 0;
            md_parms->pid_iq.term.Ref = 0;

#ifdef AUTO_REVIVE_FROM_FAULT
            if (md_parms->fault_revive_counter++ > (((Uint32)ISR_FREQUENCY) * ((Uint32)FAULT_REVIVE_TIME_MS))) {
                md_parms->fault_revive_counter = 0;
                reset_average_power_filter(pf_parms);
                md_parms->motor_drive_state = STATE_RUNNING;
            }
#endif
            break;
        }
}
