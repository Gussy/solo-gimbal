/*
 * commutation_calibration_state_machine.c
 *
 *  Created on: Jan 7, 2015
 *      Author: abamberger
 */

#include "motor/commutation_calibration_state_machine.h"
#include "can/cand.h"
#include "hardware/device_init.h"
#include "parameters/flash_params.h"

static void calc_slope_intercept(CommutationCalibrationParms* cc_parms, int start, int end, float *slope, float *intercept)
{
	float average_slope = 0;
	float average_intercept = 0;
	float temp;
	int i;
	for (i = start; i < end; i++) {
		average_slope += (cc_parms->calibration_data[i] - cc_parms->calibration_data[i-1])/(1.0f/COMMUTATION_CALIBRATION_ELECTRICAL_CYCLE_SUBDIVISIONS);
	}
	average_slope /= (end - start);
	for (i = start; i < end; i++) {
		temp = (cc_parms->calibration_data[i] - (((float)(i-cc_parms->ezero_step))/COMMUTATION_CALIBRATION_ELECTRICAL_CYCLE_SUBDIVISIONS)*average_slope);
		average_intercept += temp;
	}
	average_intercept /= (end - start);
	*slope = average_slope;
	*intercept = average_intercept;

}


_iq IdRefLockCommutationCalibration = _IQ(0.18); // 0.5A if 2.75A max scale is correct

void CommutationCalibrationStateMachine(MotorDriveParms* md_parms, EncoderParms* encoder_parms, CommutationCalibrationParms* cc_parms)
{
	static float last_position;
	static float new_position = 0;
	static Uint16 hardstop = 0;
    switch (cc_parms->calibration_state) {
        case COMMUTATION_CALIBRATION_STATE_INIT:
		    encoder_parms->calibration_slope = AxisCalibrationSlopes[GIMBAL_TARGET][GetBoardHWID()];
		    encoder_parms->calibration_intercept = AxisCalibrationIntercepts[GIMBAL_TARGET][GetBoardHWID()];
        	// don't calibrate if we got slope set already
        	if (encoder_parms->calibration_slope != 0) {
    		    md_parms->motor_drive_state = STATE_HOMING;
    		    break;
        	}
            // Set up the ramp control macro for locking to the first electrical 0
            md_parms->park_xform_parms.Angle = 0;//cc_parms->ramp_cntl.TargetValue =
            cc_parms->ramp_cntl.SetpointValue = 0;
            cc_parms->ramp_cntl.TargetValue = IdRefLockCommutationCalibration;
            cc_parms->ramp_cntl.RampDelayMax = COMMUTATION_CALIBRATION_RAMP_SPEED;
            cc_parms->calibration_state = COMMUTATION_CALIBRATION_STATE_RAMP_ID;
            Uint16 dataIndex;
            for (dataIndex = 0; dataIndex < COMMUTATION_ARRAY_SIZE; dataIndex++) cc_parms->calibration_data[dataIndex] = 0;
            break;


        case COMMUTATION_CALIBRATION_STATE_RAMP_ID:
            RC_MACRO(cc_parms->ramp_cntl)
            md_parms->pid_id.term.Ref = cc_parms->ramp_cntl.SetpointValue;
            md_parms->pid_iq.term.Ref = 0;
            md_parms->park_xform_parms.Angle = 0;//cc_parms->ramp_cntl.TargetValue =

            if (cc_parms->ramp_cntl.EqualFlag > 0) {
                cc_parms->calibration_state = COMMUTATION_CALIBRATION_STATE_MOVE_TO_HARDSTOP;
                last_position = encoder_parms->mech_theta + 0.003;
                cc_parms->settling_timer = 0;
                cc_parms->ramp_cntl.SetpointValue = cc_parms->ramp_cntl.TargetValue = 0;
            }
            break;

        case COMMUTATION_CALIBRATION_STATE_MOVE_TO_HARDSTOP:
            RC_MACRO(cc_parms->ramp_cntl)
            md_parms->pid_id.term.Ref = IdRefLockCommutationCalibration;
            md_parms->pid_iq.term.Ref = 0;

            md_parms->park_xform_parms.Angle = cc_parms->ramp_cntl.SetpointValue;

            if (cc_parms->ramp_cntl.EqualFlag == 0) {
            	break;
            }
            if (cc_parms->settling_timer++ > (((Uint32)ISR_FREQUENCY) * ((Uint32)COMMUTATION_CALIBRATION_HARDSTOP_SETTLING_TIME_MS))) {
                cc_parms->settling_timer = 0;
                if (fabs(new_position - encoder_parms->mech_theta) > 0.0005) {
                	new_position = encoder_parms->mech_theta;
                	break;
                }
                float new_mech_theta = encoder_parms->mech_theta;
                if (((last_position - new_mech_theta) < 0.002)) {
                	hardstop++;
                	if (hardstop > 1) {
						cc_parms->calibration_state = COMMUTATION_CALIBRATION_STATE_MOVE_UP_FROM_HARDSTOP;
						last_position = new_mech_theta;
						cc_parms->settling_timer = 0;
		                cc_parms->ramp_cntl.TargetValue += 4*(1.0f/COMMUTATION_CALIBRATION_ELECTRICAL_CYCLE_SUBDIVISIONS);
		                if (cc_parms->ramp_cntl.TargetValue > 1.0) cc_parms->ramp_cntl.TargetValue = 1.0;
		                cc_parms->current_iteration = 0;
						hardstop = 0;
	                    break;
                	}
                } else {
                	hardstop = 0;
                }
                last_position = new_mech_theta;
                if (cc_parms->ramp_cntl.TargetValue == 0) cc_parms->ramp_cntl.TargetValue = 1.0;
                cc_parms->ramp_cntl.SetpointValue = cc_parms->ramp_cntl.TargetValue;
                cc_parms->ramp_cntl.TargetValue -= (1.0f/COMMUTATION_CALIBRATION_ELECTRICAL_CYCLE_SUBDIVISIONS);
                cc_parms->ramp_cntl.EqualFlag = 0;
            }
        	break;
        case COMMUTATION_CALIBRATION_STATE_MOVE_UP_FROM_HARDSTOP:
            RC_MACRO(cc_parms->ramp_cntl)
            md_parms->pid_id.term.Ref = IdRefLockCommutationCalibration;
            md_parms->pid_iq.term.Ref = 0;

            md_parms->park_xform_parms.Angle = cc_parms->ramp_cntl.SetpointValue;

            if (cc_parms->ramp_cntl.EqualFlag == 0) {
            	break;
            }
            if (cc_parms->settling_timer++ > (((Uint32)ISR_FREQUENCY) * ((Uint32)COMMUTATION_CALIBRATION_SETTLING_TIME_MS))) {
                cc_parms->settling_timer = 0;
                if (fabs(new_position - encoder_parms->mech_theta) > 0.0005) {
                	new_position = encoder_parms->mech_theta;
                	break;
                }
                float new_mech_theta = encoder_parms->mech_theta;
                cc_parms->calibration_data[cc_parms->current_iteration++] = encoder_parms->mech_theta;
                if (((cc_parms->current_iteration > 2)&&((last_position - new_mech_theta) > -0.002))||(cc_parms->current_iteration >= COMMUTATION_ARRAY_SIZE)) {
                	hardstop++;
                	if ((hardstop > 1)||(cc_parms->current_iteration >= COMMUTATION_ARRAY_SIZE)) {
                		if (cc_parms->current_iteration > 16) {
                			calc_slope_intercept(cc_parms,2,cc_parms->current_iteration-3,&AxisCalibrationSlopes[GIMBAL_TARGET][GetBoardHWID()],&AxisCalibrationIntercepts[GIMBAL_TARGET][GetBoardHWID()]);
    						cc_parms->calibration_state = COMMUTATION_CALIBRATION_STATE_TEST;
                		} else {
                			md_parms->motor_drive_state = STATE_FAULT;
                		}
		                cc_parms->current_iteration = 0;
						last_position = encoder_parms->mech_theta;
						cc_parms->settling_timer = 0;
						hardstop = 0;
	                    break;
                	}
                } else {
                	hardstop = 0;
                }
                last_position = new_mech_theta;
                if (cc_parms->ramp_cntl.TargetValue == 1.0) {
                	// this is zero
                	cc_parms->ramp_cntl.TargetValue = 0.0;
                	cc_parms->ezero_step = cc_parms->current_iteration%COMMUTATION_CALIBRATION_ELECTRICAL_CYCLE_SUBDIVISIONS;
                }
                cc_parms->ramp_cntl.SetpointValue = cc_parms->ramp_cntl.TargetValue;
                cc_parms->ramp_cntl.TargetValue += (1.0f/COMMUTATION_CALIBRATION_ELECTRICAL_CYCLE_SUBDIVISIONS);
                cc_parms->ramp_cntl.EqualFlag = 0;
            }
        	break;
#if 0
        case COMMUTATION_CALIBRATION_STATE_WAIT_FOR_SETTLE:
            if (cc_parms->settling_timer++ > (((Uint32)ISR_FREQUENCY) * ((Uint32)COMMUTATION_CALIBRATION_SETTLING_TIME_MS))) {
                cc_parms->settling_timer = 0;
                if (fabs(new_position - encoder_parms->mech_theta) > 0.0005) {
                	new_position = encoder_parms->mech_theta;
                	break;
                }

                // Take this point of data
                Uint16 dataIndex = (cc_parms->current_iteration * COMMUTATION_CALIBRATION_NUM_ELECTRICAL_CYCLES * COMMUTATION_CALIBRATION_ELECTRICAL_CYCLE_SUBDIVISIONS * 2) +
                        (cc_parms->current_elec_cycle * COMMUTATION_CALIBRATION_ELECTRICAL_CYCLE_SUBDIVISIONS) +
                        (cc_parms->current_elec_sub_cycle) +
                        (COMMUTATION_CALIBRATION_NUM_ELECTRICAL_CYCLES * COMMUTATION_CALIBRATION_ELECTRICAL_CYCLE_SUBDIVISIONS * cc_parms->current_dir);
                float new_mech_theta = encoder_parms->mech_theta;
                cc_parms->calibration_data[dataIndex] = encoder_parms->mech_theta;
                if (((cc_parms->current_dir == 0)&&((last_position - new_mech_theta) > -0.002)&&(dataIndex != 0))||
                	((cc_parms->current_dir == 1)&&((last_position - new_mech_theta) <  0.002)&&(dataIndex != 0))) {
                	hardstop++;
                	cc_parms->calibration_data[dataIndex] = 0;
                	if (dataIndex > 16) {
                		calc_slope_intercept(cc_parms,2,dataIndex-2,&AxisCalibrationSlopes[GIMBAL_TARGET][GetBoardHWID()],&AxisCalibrationIntercepts[GIMBAL_TARGET][GetBoardHWID()]);
                		cc_parms->calibration_state = COMMUTATION_CALIBRATION_STATE_TEST;
                		break;
                	}
#if 0
                	if (dataIndex < 16) {
                		// backup and start over
                		cc_parms->ramp_cntl.TargetValue = 0;
                		cc_parms->ramp_cntl.SetpointValue = 1;
                        cc_parms->ramp_cntl.EqualFlag = 0;
                        cc_parms->calibration_state = COMMUTATION_CALIBRATION_STATE_MOVE_ID;
                        cc_parms->current_iteration = 0;
                        cc_parms->current_elec_cycle = 0;
                        cc_parms->current_elec_sub_cycle = 0;
                        break;
                	}
#endif
                }
                last_position = new_mech_theta;

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
                                //md_parms->motor_drive_state = STATE_DISABLED;
                                int i;
                                int count = 0;
                                for (i = 0; i < 32; i++) {
                                	if (cc_parms->calibration_data[i] == 0) {
                                		if ((i-count) > 12) {
                                			calc_slope_intercept(cc_parms,count+2,i-2,&AxisCalibrationSlopes[GIMBAL_TARGET][GetBoardHWID()],&AxisCalibrationIntercepts[GIMBAL_TARGET][GetBoardHWID()]);
                                			cc_parms->calibration_state = COMMUTATION_CALIBRATION_STATE_TEST;
                                		    return;
                                		}
                                		count = i;
                                	}
                                }
                                count = 32;
                                for (i = 32; i < 64; i++) {
                                	if (cc_parms->calibration_data[i] == 0) {
                                		if ((i-count) > 12) {
                                			calc_slope_intercept(cc_parms,count+2,i-2,&AxisCalibrationSlopes[GIMBAL_TARGET][GetBoardHWID()],&AxisCalibrationIntercepts[GIMBAL_TARGET][GetBoardHWID()]);
                                			cc_parms->calibration_state = COMMUTATION_CALIBRATION_STATE_TEST;
                                		    return;
                                		}
                                		count = i;
                                	}
                                }
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
#endif
        case COMMUTATION_CALIBRATION_STATE_TEST:
        	// go back three steps and verify the position
            // Wrap target angle back to 0 if we get to 1
            if (cc_parms->ramp_cntl.TargetValue == 1.0) {
                cc_parms->ramp_cntl.TargetValue = 0.0;
            }
            // Set our starting position as our last target position
            cc_parms->ramp_cntl.SetpointValue = cc_parms->ramp_cntl.TargetValue;
            cc_parms->calibration_state = COMMUTATION_CALIBRATION_STATE_COMPLETE;
        	break;
        case COMMUTATION_CALIBRATION_STATE_TEST_MOVE:
            RC_MACRO(cc_parms->ramp_cntl)
            md_parms->pid_id.term.Ref = IdRefLockCommutationCalibration;
            md_parms->pid_iq.term.Ref = 0;

            md_parms->park_xform_parms.Angle = cc_parms->ramp_cntl.SetpointValue;

            if (cc_parms->ramp_cntl.EqualFlag > 0) {
                // We've hit our next data point, so move to settle state
                cc_parms->calibration_state = COMMUTATION_CALIBRATION_STATE_TEST_CHECK_POS;
            }
        	break;
        case COMMUTATION_CALIBRATION_STATE_TEST_CHECK_POS:
        	break;
        case COMMUTATION_CALIBRATION_STATE_COMPLETE:
		    encoder_parms->calibration_slope = AxisCalibrationSlopes[GIMBAL_TARGET][GetBoardHWID()];
		    encoder_parms->calibration_intercept = AxisCalibrationIntercepts[GIMBAL_TARGET][GetBoardHWID()];
		    md_parms->motor_drive_state = STATE_HOMING;
		    if (GetBoardHWID() != AZ) {
                IntOrFloat float_converter;
                float_converter.float_val = encoder_parms->calibration_slope;
		    	cand_tx_response(CAND_ID_AZ,CAND_PID_COMMUTATION_CALIBRATION_SLOPE,float_converter.uint32_val);
                float_converter.float_val = encoder_parms->calibration_intercept;
		    	cand_tx_response(CAND_ID_AZ,CAND_PID_COMMUTATION_CALIBRATION_INTERCEPT,float_converter.uint32_val);
		    } else {
        		flash_params.AxisCalibrationSlopes[AZ] = encoder_parms->calibration_slope;
        		flash_params.AxisCalibrationIntercepts[AZ] = encoder_parms->calibration_intercept;
        		write_flash();
		    }
		    break;

    }
}
