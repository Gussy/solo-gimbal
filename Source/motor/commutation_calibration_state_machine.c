/*
 * commutation_calibration_state_machine.c
 *
 *  Created on: Jan 7, 2015
 *      Author: abamberger
 */

#include "commutation_calibration_state_machine.h"

_iq IdRefLockCommutationCalibration = _IQ(0.18); // 0.5A if 2.75A max scale is correct

void CommutationCalibrationStateMachine(MotorDriveParms* md_parms, EncoderParms* encoder_parms, CommutationCalibrationParms* cc_parms)
{
    switch (cc_parms->calibration_state) {
        case COMMUTATION_CALIBRATION_STATE_INIT:
            // Set up the ramp control macro for locking to the first electrical 0
            md_parms->park_xform_parms.Angle = 0;
            cc_parms->ramp_cntl.TargetValue = IdRefLockCommutationCalibration;
            cc_parms->ramp_cntl.RampDelayMax = COMMUTATION_CALIBRATION_RAMP_SPEED;
            cc_parms->calibration_state = COMMUTATION_CALIBRATION_STATE_RAMP_ID;
            break;


        case COMMUTATION_CALIBRATION_STATE_RAMP_ID:
            RC_MACRO(cc_parms->ramp_cntl)
            md_parms->pid_id.term.Ref = cc_parms->ramp_cntl.SetpointValue;
            md_parms->pid_iq.term.Ref = 0;

            if (cc_parms->ramp_cntl.EqualFlag > 0) {
                cc_parms->calibration_state = COMMUTATION_CALIBRATION_STATE_WAIT_FOR_SETTLE;
            }
            break;

        case COMMUTATION_CALIBRATION_STATE_WAIT_FOR_SETTLE:
            if (cc_parms->settling_timer++ > (((Uint32)ISR_FREQUENCY) * ((Uint32)COMMUTATION_CALIBRATION_SETTLING_TIME_MS))) {
                cc_parms->settling_timer = 0;

                // Take this point of data
                Uint16 dataIndex = (cc_parms->current_iteration * COMMUTATION_CALIBRATION_NUM_ELECTRICAL_CYCLES * COMMUTATION_CALIBRATION_ELECTRICAL_CYCLE_SUBDIVISIONS * 2) +
                        (cc_parms->current_elec_cycle * COMMUTATION_CALIBRATION_ELECTRICAL_CYCLE_SUBDIVISIONS) +
                        (cc_parms->current_elec_sub_cycle) +
                        (COMMUTATION_CALIBRATION_NUM_ELECTRICAL_CYCLES * COMMUTATION_CALIBRATION_ELECTRICAL_CYCLE_SUBDIVISIONS * cc_parms->current_dir);
                cc_parms->calibration_data[dataIndex] = encoder_parms->mech_theta;

                // Update the cycle numbers, based on the current direction
                // Move to the next electrical subcycle
                cc_parms->current_elec_sub_cycle++;
                if (cc_parms->current_elec_sub_cycle >= COMMUTATION_CALIBRATION_ELECTRICAL_CYCLE_SUBDIVISIONS) {
                    // If we've exceeded the number of electrical subcycles in this electrical cycle, reset the subcycle and move to the next cycle
                    cc_parms->current_elec_sub_cycle = 0;
                    cc_parms->current_elec_cycle++;

                    if (cc_parms->current_elec_cycle >= COMMUTATION_CALIBRATION_NUM_ELECTRICAL_CYCLES) {
                        // If we've exceeded the number of electrical cycles in this iteration, reset the cycle and reverse the direction
                        cc_parms->current_elec_cycle = 0;

                        // If we were going forward, reverse and go backward.  If we were going backward, reverse and go forward, and increment the iteration
                        if (cc_parms->current_dir == 0) {
                            cc_parms->current_dir = 1;
                        } else {
                            cc_parms->current_dir = 0;
                            cc_parms->current_iteration++;

                            if (cc_parms->current_iteration >= COMMUTATION_CALIBRATION_NUM_ITERATIONS) {
                                // If we've exceeded the number of test iterations, we're done, so change state to disabled
                                md_parms->motor_drive_state = STATE_DISABLED;
                            }
                        }
                    }
                }

                // Set up the move to the next position
                // Wrap target angle back to 0 if we get to 1
                if (cc_parms->ramp_cntl.TargetValue == 1.0) {
                    cc_parms->ramp_cntl.TargetValue = 0.0;
                }
                // Set our starting position as our last target position
                cc_parms->ramp_cntl.SetpointValue = cc_parms->ramp_cntl.TargetValue;

                // Set up new target angle
                if (cc_parms->current_dir == 0) {
                    cc_parms->ramp_cntl.TargetValue = (1.0f / COMMUTATION_CALIBRATION_ELECTRICAL_CYCLE_SUBDIVISIONS) * cc_parms->current_elec_sub_cycle;
                    if ((cc_parms->ramp_cntl.TargetValue == 0.0) && (cc_parms->current_elec_cycle != 0)) {
                        // Need to correct for this so we don't go backward by accident when we're trying to move forward
                        // But don't do the correction when we're at the start of a new iteration, or we'll accidentally skip an entire cycle ahead
                        cc_parms->ramp_cntl.TargetValue = 1.0;
                    }
                } else {
                    cc_parms->ramp_cntl.TargetValue = (1.0f / COMMUTATION_CALIBRATION_ELECTRICAL_CYCLE_SUBDIVISIONS) * (COMMUTATION_CALIBRATION_ELECTRICAL_CYCLE_SUBDIVISIONS - (cc_parms->current_elec_sub_cycle + 1));
                    if (cc_parms->ramp_cntl.SetpointValue == 0.0) {
                        // Need to correct for this so we don't go forward by accident when we're trying to move backward
                        cc_parms->ramp_cntl.SetpointValue = 1.0;
                    }
                }
                cc_parms->ramp_cntl.EqualFlag = 0;

                cc_parms->calibration_state = COMMUTATION_CALIBRATION_STATE_MOVE_ID;

            }
            break;

        case COMMUTATION_CALIBRATION_STATE_MOVE_ID:
            RC_MACRO(cc_parms->ramp_cntl)
            md_parms->pid_id.term.Ref = IdRefLockCommutationCalibration;
            md_parms->pid_iq.term.Ref = 0;

            md_parms->park_xform_parms.Angle = cc_parms->ramp_cntl.SetpointValue;

            if (cc_parms->ramp_cntl.EqualFlag > 0) {
                // We've hit our next data point, so move to settle state
                cc_parms->calibration_state = COMMUTATION_CALIBRATION_STATE_WAIT_FOR_SETTLE;
            }
            break;
    }
}
