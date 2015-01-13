/*
 * homing_calibration_state_machine.c
 *
 *  Created on: Jan 7, 2015
 *      Author: abamberger
 */

#include "motor/homing_calibration_state_machine.h"
#include "hardware/HWSpecific.h"
#include "hardware/device_init.h"

_iq IdRefLockHoming = _IQ(0.18); // 0.5A if 2.75A max scale is correct

void HomingCalibrationStateMachine(MotorDriveParms* md_parms, EncoderParms* encoder_parms, ControlBoardParms* cb_parms, AxisParms* axis_parms, HomingCalibrationParms* hc_parms)
{
    switch (hc_parms->homing_state) {
        case HOMING_STATE_INIT:
        {
            // Set up the ramp control macro for locking to the first electrical 0
            hc_parms->ramp_cntl.TargetValue = IdRefLockHoming;
            hc_parms->ramp_cntl.RampDelayMax = HOMING_ID_RAMP_SPEED;
            hc_parms->ramp_cntl.SetpointValue = 0.0;
            hc_parms->ramp_cntl.EqualFlag = 0;

            //TODO: For now, hardcoding the pot calibration parameters per-axis
            // The automatic calibration routine is too unstable right now
            encoder_parms->calibration_slope = AxisCalibrationSlopes[GIMBAL_TARGET][GetBoardHWID()];
            encoder_parms->calibration_intercept = AxisCalibrationIntercepts[GIMBAL_TARGET][GetBoardHWID()];
            // Also hardcode the axis home position
            encoder_parms->virtual_counts_offset = -AxisHomePositions[GIMBAL_TARGET][GetBoardHWID()];

            //TODO: Short circuit the rest of homing, since we're hardcoding these values for now
            cb_parms->axes_homed[GetBoardHWID()] = TRUE;
            if (GetBoardHWID() == EL) {
                // If we're the EL board, we need to wait for the other axes to indicate that they've finished homing before
                // we enable the rate loops.  Otherwise, we can just start running our own torque loop
                md_parms->motor_drive_state = STATE_WAIT_FOR_AXES_HOME;
            } else {
                md_parms->motor_drive_state = STATE_RUNNING;
                axis_parms->blink_state = BLINK_READY;
            }

            //homing_state = HOMING_STATE_TAKE_CALIBRATION_POINT_1;
        }
        break;

        case HOMING_STATE_TAKE_CALIBRATION_POINT_1:
        {
            // Lock rotor to electrical 0, ramp up id current
            md_parms->park_xform_parms.Angle = 0.0;

            RC_MACRO(hc_parms->ramp_cntl)
            md_parms->pid_id.term.Ref = hc_parms->ramp_cntl.SetpointValue;
            md_parms->pid_iq.term.Ref = 0.0;

            if (hc_parms->ramp_cntl.EqualFlag > 0) {

                // Allow the axis some time to settle at its new position before taking a data point
                hc_parms->settling_timer++;
                if (hc_parms->settling_timer > (((Uint32)ISR_FREQUENCY) * ((Uint32)POT_CALIBRATION_SETTLING_TIME_MS))) {
                    hc_parms->settling_timer = 0;

                    hc_parms->ramp_cntl.SetpointValue = IdRefLockHoming;
                    hc_parms->ramp_cntl.TargetValue = 2 * IdRefLockHoming;
                    hc_parms->ramp_cntl.RampDelayMax = 1;
                    hc_parms->ramp_cntl.EqualFlag = 0;

                    hc_parms->homing_state = HOMING_STATE_RAMP_CURRENT_UP_1;
                }
            }
        }
        break;

        case HOMING_STATE_RAMP_CURRENT_UP_1:
        {
            RC_MACRO(hc_parms->ramp_cntl)
            md_parms->pid_id.term.Ref = hc_parms->ramp_cntl.SetpointValue;
            md_parms->pid_iq.term.Ref = 0.0;

            if (hc_parms->ramp_cntl.EqualFlag > 0) {
                encoder_parms->calibration_mech_y0 = encoder_parms->mech_theta;

                hc_parms->ramp_cntl.SetpointValue = 2 * IdRefLockHoming;
                hc_parms->ramp_cntl.TargetValue = IdRefLockHoming;
                hc_parms->ramp_cntl.RampDelayMax = 1;
                hc_parms->ramp_cntl.EqualFlag = 0;

                hc_parms->homing_state = HOMING_STATE_RAMP_CURRENT_DOWN_1;
            }
        }
        break;

        case HOMING_STATE_RAMP_CURRENT_DOWN_1:
        {
            RC_MACRO(hc_parms->ramp_cntl)
            md_parms->pid_id.term.Ref = hc_parms->ramp_cntl.SetpointValue;
            md_parms->pid_iq.term.Ref = 0.0;

            if (hc_parms->ramp_cntl.EqualFlag > 0) {
                // Set up the ramp control to ramp the angle to half of an electrical cycle
                hc_parms->ramp_cntl.TargetValue = 0.5;
                hc_parms->ramp_cntl.SetpointValue = 0.0;
                hc_parms->ramp_cntl.RampDelayMax = HOMING_ANGLE_RAMP_SPEED;
                hc_parms->ramp_cntl.EqualFlag = 0;

                hc_parms->homing_state = HOMING_STATE_MOVE_ID;
                hc_parms->next_homing_state = HOMING_STATE_TAKE_CALIBRATION_POINT_2;
            }
        }
        break;

        case HOMING_STATE_TAKE_CALIBRATION_POINT_2:
        {
            md_parms->pid_id.term.Ref = IdRefLockHoming;
            md_parms->pid_iq.term.Ref = 0.0;

            // Allow the axis some time to settle at its new position before taking a data point
            hc_parms->settling_timer++;
            if (hc_parms->settling_timer > (((Uint32)ISR_FREQUENCY) * ((Uint32)POT_CALIBRATION_SETTLING_TIME_MS))) {
                hc_parms->settling_timer = 0;

                hc_parms->ramp_cntl.SetpointValue = IdRefLockHoming;
                hc_parms->ramp_cntl.TargetValue = 2 * IdRefLockHoming;
                hc_parms->ramp_cntl.RampDelayMax = 1;
                hc_parms->ramp_cntl.EqualFlag = 0;

                hc_parms->homing_state = HOMING_STATE_RAMP_CURRENT_UP_2;
            }
        }
        break;

        case HOMING_STATE_RAMP_CURRENT_UP_2:
        {
            RC_MACRO(hc_parms->ramp_cntl);
            md_parms->pid_id.term.Ref = hc_parms->ramp_cntl.SetpointValue;
            md_parms->pid_iq.term.Ref = 0.0;

            if (hc_parms->ramp_cntl.EqualFlag > 0) {
                encoder_parms->calibration_mech_y1 = encoder_parms->mech_theta;

                hc_parms->ramp_cntl.SetpointValue = 2 * IdRefLockHoming;
                hc_parms->ramp_cntl.TargetValue = IdRefLockHoming;
                hc_parms->ramp_cntl.RampDelayMax = 1;
                hc_parms->ramp_cntl.EqualFlag = 0;

                hc_parms->homing_state = HOMING_STATE_RAMP_CURRENT_DOWN_2;
            }
        }
        break;

        case HOMING_STATE_RAMP_CURRENT_DOWN_2:
        {
            RC_MACRO(hc_parms->ramp_cntl)
            md_parms->pid_id.term.Ref = hc_parms->ramp_cntl.SetpointValue;
            md_parms->pid_iq.term.Ref = 0.0;

            if (hc_parms->ramp_cntl.EqualFlag > 0) {
                // Make sure we've moved the axis roughly the expected amount between the two data points.  If not, it probably means we ran into
                // a hard stop, so we should try the other direction.  If we've already tried both directions, we've got a homing error, so fail out
                float expected_travel = (1.0f / (POLES / 2)) / 2.0f;
                float actual_travel = fabs(encoder_parms->calibration_mech_y1 - encoder_parms->calibration_mech_y0);
                float allowable_deviation = 0.05;
                if ((actual_travel > (expected_travel * (1.0f + allowable_deviation))) || (actual_travel < (expected_travel * (1.0f - allowable_deviation)))) {
                    if (hc_parms->direction == HOMING_DIRECTION_FORWARD) {
                        hc_parms->direction = HOMING_DIRECTION_BACKWARD;
                        hc_parms->ramp_cntl.TargetValue = 1.0;
                        hc_parms->ramp_cntl.SetpointValue = 0.5;
                        hc_parms->ramp_cntl.RampDelayMax = HOMING_ANGLE_RAMP_SPEED;
                        hc_parms->ramp_cntl.EqualFlag = 0;
                        hc_parms->homing_state = HOMING_STATE_MOVE_ID;
                        hc_parms->next_homing_state = HOMING_STATE_TAKE_CALIBRATION_POINT_1;
                    } else {
                        // We're unable to calibrate the pot correctly, so transmit an error and go to the fault state
                        AxisFault(CAND_FAULT_CALIBRATING_POT);
                        md_parms->motor_drive_state = STATE_FAULT;
                    }
                } else {
                    // If we've moved close enough to what we expect, compute the final calibration line
                    if (encoder_parms->calibration_mech_y1 > encoder_parms->calibration_mech_y0) {
                        encoder_parms->calibration_slope = (encoder_parms->calibration_mech_y1 - encoder_parms->calibration_mech_y0) / 0.5;
                        encoder_parms->calibration_intercept = encoder_parms->calibration_mech_y0 - (encoder_parms->calibration_slope * (ROUND(encoder_parms->calibration_mech_y0 * (POLES / 2)) + 1));
                    } else {
                        encoder_parms->calibration_slope = (encoder_parms->calibration_mech_y0 - encoder_parms->calibration_mech_y1) / 0.5;
                        encoder_parms->calibration_intercept = encoder_parms->calibration_mech_y1 - (encoder_parms->calibration_slope * (ROUND(encoder_parms->calibration_mech_y1 * (POLES / 2)) + 1));
                    }

                    // Set up ramp control for finding stops
                    hc_parms->ramp_cntl.SetpointValue = 0.0;
                    hc_parms->ramp_cntl.TargetValue = IdRefLockHoming;
                    hc_parms->ramp_cntl.RampDelayMax = HOMING_ID_RAMP_SPEED;
                    hc_parms->ramp_cntl.EqualFlag = 0;
                    hc_parms->homing_state = HOMING_STATE_FIND_STOP_INIT;
                }
            }
        }
        break;

        case HOMING_STATE_MOVE_ID:
        {
            RC_MACRO(hc_parms->ramp_cntl)
            if (hc_parms->direction == HOMING_DIRECTION_FORWARD) {
                md_parms->park_xform_parms.Angle = hc_parms->ramp_cntl.SetpointValue;
            } else {
                md_parms->park_xform_parms.Angle = 1.0 - hc_parms->ramp_cntl.SetpointValue;
            }

            md_parms->pid_id.term.Ref = IdRefLockHoming;
            md_parms->pid_iq.term.Ref = 0.0;

            if (hc_parms->ramp_cntl.EqualFlag > 0) {
                hc_parms->homing_state = hc_parms->next_homing_state;
            }
        }
        break;

        case HOMING_STATE_FIND_STOP_INIT:
        {
            // Lock rotor to electrical 0, ramp up id current
            md_parms->park_xform_parms.Angle = 0.0;

            RC_MACRO(hc_parms->ramp_cntl)
            md_parms->pid_id.term.Ref = hc_parms->ramp_cntl.SetpointValue;
            md_parms->pid_iq.term.Ref = 0.0;

            if (hc_parms->ramp_cntl.EqualFlag > 0) {
                hc_parms->find_stop_step = 0;
                hc_parms->last_mech_theta = encoder_parms->mech_theta;
                if (hc_parms->finding_stop == AXIS_STOP_1) {
                    hc_parms->direction = HOMING_DIRECTION_FORWARD;
                } else {
                    hc_parms->direction = HOMING_DIRECTION_BACKWARD;
                }

                hc_parms->ramp_cntl.SetpointValue = 0.0;
                hc_parms->ramp_cntl.TargetValue = MIN(hc_parms->ramp_cntl.SetpointValue + HOMING_STOP_SEARCH_STEP_SIZE, 1.0);
                hc_parms->ramp_cntl.RampDelayMax = HOMING_ANGLE_RAMP_SPEED;
                hc_parms->ramp_cntl.EqualFlag = 0;
                hc_parms->homing_state = HOMING_STATE_MOVE_ID;
                hc_parms->next_homing_state = HOMING_STATE_FIND_STOP;
            }
        }
        break;

        case HOMING_STATE_FIND_STOP:
        {
            md_parms->pid_id.term.Ref = IdRefLockHoming;
            md_parms->pid_iq.term.Ref = 0.0;

            if ((fabs(encoder_parms->mech_theta - hc_parms->last_mech_theta) < hc_parms->EPSILON) && (hc_parms->stalled_count++ > HOMING_NUM_STALLED_COUNTS)) {
                // We're up against a stop.  Remember it, and move onto the next homing step
                hc_parms->stalled_count = 0;

                if (hc_parms->finding_stop == AXIS_STOP_1) {
                    hc_parms->mech_stop_1 = encoder_parms->virtual_counts;

                    // Set up ramp control to return axis to electrical 0
                    hc_parms->ramp_cntl.SetpointValue = 0.0;
                    hc_parms->ramp_cntl.TargetValue = IdRefLockHoming;
                    hc_parms->ramp_cntl.RampDelayMax = HOMING_ID_RAMP_SPEED;
                    hc_parms->ramp_cntl.EqualFlag = 0;

                    hc_parms->finding_stop = AXIS_STOP_2;
                    hc_parms->homing_state = HOMING_STATE_FIND_STOP_INIT;
                } else {
                    hc_parms->mech_stop_2 = encoder_parms->virtual_counts;

                    // We're done homing, so indicate as such
                    cb_parms->axes_homed[GetBoardHWID()] = TRUE;

                    // Compute the encoder counts offset, so that a 0 encoder value corresponds to the middle of this axis' mechanical range
                    encoder_parms->virtual_counts_offset = MIN(hc_parms->mech_stop_1, hc_parms->mech_stop_2) + (abs(hc_parms->mech_stop_1 - hc_parms->mech_stop_2) / 2);

                    if (GetBoardHWID() == EL) { // EL axis is control board
                        md_parms->motor_drive_state = STATE_WAIT_FOR_AXES_HOME;
                    } else {
                        md_parms->motor_drive_state = STATE_RUNNING;
                        axis_parms->blink_state = BLINK_READY;
                    }
                }
            } else {
                hc_parms->last_mech_theta = encoder_parms->mech_theta;
                hc_parms->find_stop_step++;
                if (hc_parms->find_stop_step > HOMING_MAX_STOP_SEARCH_STEPS) {
                    // Don't try to find stops indefinitely.  If we've exceeded the maximum number of search steps, error out
                    AxisFault(CAND_FAULT_FIND_STOP_TIMEOUT);
                    md_parms->motor_drive_state = STATE_FAULT;
                } else {
                    if (hc_parms->ramp_cntl.TargetValue == 1.0) {
                        hc_parms->ramp_cntl.SetpointValue = 0.0;
                        hc_parms->ramp_cntl.TargetValue = 0.0;
                    } else {
                        hc_parms->ramp_cntl.SetpointValue = hc_parms->ramp_cntl.TargetValue;
                    }
                    hc_parms->ramp_cntl.TargetValue = MIN(hc_parms->ramp_cntl.TargetValue + HOMING_STOP_SEARCH_STEP_SIZE, 1.0);
                    hc_parms->ramp_cntl.RampDelayMax = HOMING_ANGLE_RAMP_SPEED;
                    hc_parms->ramp_cntl.EqualFlag = 0;
                    hc_parms->homing_state = HOMING_STATE_MOVE_ID;
                    hc_parms->next_homing_state = HOMING_STATE_FIND_STOP;
                }
            }
        }
        break;
    }
}
