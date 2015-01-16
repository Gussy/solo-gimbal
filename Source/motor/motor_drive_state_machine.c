/*
 * motor_drive_state_machine.c
 *
 *  Created on: Jan 7, 2015
 *      Author: abamberger
 */

#include "motor/motor_drive_state_machine.h"
#include "PM_Sensorless.h"
#include "PM_Sensorless-Settings.h"
#include "can/cand.h"
#include "can/cb.h"
#include "hardware/device_init.h"
#include "motor/commutation_calibration_state_machine.h"
#include "motor/homing_calibration_state_machine.h"
#include "control/PID.h"
#include "parameters/flash_params.h"
#include "PeripheralHeaderIncludes.h"

#include <string.h>

CommutationCalibrationParms cc_parms = {
    COMMUTATION_CALIBRATION_STATE_INIT,     // Commutation calibration state
    0,                                      // Current iteration
    0,                                      // Current electrical cycle
    0,                                      // Current electrical sub-cycle
    0,                                      // Current direction
    0,                                      // Settling timer
    0,
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

void MotorDriveStateMachine(AxisParms* axis_parms,
        ControlBoardParms* cb_parms,
        MotorDriveParms* md_parms,
        EncoderParms* encoder_parms,
        ParamSet* param_set,
        RunningAvgFilterParms* pos_loop_stage_1,
        RunningAvgFilterParms* pos_loop_stage_2,
        AveragePowerFilterParms* pf_parms,
        LoadAxisParmsStateInfo* load_ap_state_info)
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
//#ifndef ENABLE_AXIS_CALIBRATION_PROCEDURE
                cand_tx_command(CAND_ID_ALL_AXES, CAND_CMD_ENABLE);
//#endif
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
#ifndef ENABLE_AXIS_CALIBRATION_PROCEDURE
            if (GetBoardHWID() == AZ) {
                // If we're the AZ board, we first have to load our own parameters from flash, then transmit parameters
                // to the other axes.
                md_parms->motor_drive_state = STATE_LOAD_OWN_INIT_PARAMS;
            } else {
                // If we're EL or ROLL, we have to request our parameters from the AZ board
                md_parms->motor_drive_state = STATE_REQUEST_AXIS_INIT_PARAMS;
            }
#else
            md_parms->motor_drive_state = STATE_CALIBRATING_CURRENT_MEASUREMENTS;
#endif
            break;

        case STATE_LOAD_OWN_INIT_PARAMS:
            // This state is only run on the AZ board to load commutation calibration parameters and torque loop PID gains
            // The rate loop PID gains are only needed on the EL board, so these are loaded over CAN
            md_parms->pid_id.param.Kp = flash_params.torque_pid_kp[AZ];
            md_parms->pid_id.param.Ki = flash_params.torque_pid_ki[AZ];
            md_parms->pid_id.param.Kd = flash_params.torque_pid_kd[AZ];

            md_parms->pid_iq.param.Kp = flash_params.torque_pid_kp[AZ];
            md_parms->pid_iq.param.Ki = flash_params.torque_pid_ki[AZ];
            md_parms->pid_iq.param.Kd = flash_params.torque_pid_kd[AZ];

            AxisCalibrationSlopes[GIMBAL_TARGET][AZ] = flash_params.AxisCalibrationSlopes[AZ];
            AxisCalibrationIntercepts[GIMBAL_TARGET][AZ] = flash_params.AxisCalibrationIntercepts[AZ];
            AxisHomePositions[GIMBAL_TARGET][AZ] = flash_params.AxisHomePositions[AZ];

            // After we've loaded our own init parameters, make a note of it, so we can continue later when we're waiting for
            // all axes to have received their init parameters
            axis_parms->other_axis_init_params_recvd[AZ] = TRUE;

            // Once we're done loading our own parameters, we need to wait for the other axes to request and receive all of
            // their parameters
#ifdef AZ_TEST
            md_parms->motor_drive_state = STATE_CALIBRATING_CURRENT_MEASUREMENTS;
#else
            md_parms->motor_drive_state = STATE_WAIT_FOR_OTHER_AXES_INIT_PARAMS_LOADED;
#endif
            break;

        case STATE_REQUEST_AXIS_INIT_PARAMS:
            // Run the load init parms state machine to sequence through requesting the axis parms
            LoadAxisParmsStateMachine(load_ap_state_info);

            // If we've completed requesting and receiving the params from AZ,
            // we can continue with our init sequence
            if (load_ap_state_info->axis_parms_load_complete) {
                axis_parms->all_init_params_recvd = TRUE;
                md_parms->motor_drive_state = STATE_CALIBRATING_CURRENT_MEASUREMENTS;
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
#ifndef AZ_TEST

        	if (((GetBoardHWID() == AZ)&&((cb_parms->axes_homed[ROLL]))&&((cb_parms->axes_homed[EL])))||
        		((GetBoardHWID() == ROLL)&&((cb_parms->axes_homed[EL])))||
        		((GetBoardHWID() == EL)))
#endif
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
#if 0
            def ENABLE_AXIS_CALIBRATION_PROCEDURE
            {
            	static short sent = 0;
            	if (cb_parms->axes_homed[ROLL]) {
            		if (sent & 0x02 == 0) {
            			cand_tx_command(CAND_ID_AZ, CAND_CMD_ENABLE);
            			sent |= 0x02;
            		}
            	} else {
            		if (sent & 0x01 == 0) {
            			cand_tx_command(CAND_ID_ROLL, CAND_CMD_ENABLE);
            			sent |= 0x01;
            		}
            	}
            }
#endif

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
