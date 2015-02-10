/*
 * rate_loops.c
 *
 *  Created on: Jan 7, 2015
 *      Author: abamberger
 */

#include "control/rate_loops.h"
#include "can/cb.h"
#include "can/cand.h"
#include "control/PID.h"
#include "hardware/gyro.h"
#include "control/gyro_kinematics_correction.h"
#include "PM_Sensorless-Settings.h"

static void SendEncoderTelemetry(int16 az_encoder, int16 el_encoder, int16 rl_encoder);
static void SendGyroTelemetry(int32 az_gyro, int32 el_gyro, int32 rl_gyro);
static void SendAccelTelemetry(int32 az_accel, int32 el_accel, int32 rl_accel);

Uint32 gyro_read_1_start = 0;
Uint32 gyro_read_1_end = 0;
Uint32 gyro_read_1_elapsed_time = 0;
Uint32 gyro_read_2_start = 0;
Uint32 gyro_read_2_end = 0;
Uint32 gyro_read_2_elapsed_time = 0;
Uint32 gyro_read_total_elapsed_time = 0;
Uint16 telemetry_decimation_count = 0;

void RunRateLoops(ControlBoardParms* cb_parms, ParamSet* param_set, RunningAvgFilterParms* pos_loop_stage_1, RunningAvgFilterParms* pos_loop_stage_2, BalanceProcedureParms* bal_proc_parms)
{
    static int16 raw_gyro_readings[AXIS_CNT] = {0, 0, 0};
    static int16 raw_accel_readings[AXIS_CNT] = {0, 0, 0};
    static Uint32 gyro_data_pass_1 = 0;
    static Uint32 gyro_data_pass_2 = 0;

#ifdef TEST_MAX_TORQUE
    static int16 DigbyCount;
#endif

    switch (cb_parms->rate_loop_pass) {
    case READ_GYRO_PASS:
        ReadGyro(&(raw_gyro_readings[GyroAxisMap[X_AXIS]]), &(raw_gyro_readings[GyroAxisMap[Y_AXIS]]), &(raw_gyro_readings[GyroAxisMap[Z_AXIS]]));
        cb_parms->rate_loop_pass = READ_ACCEL_PASS;
        break;

    case READ_ACCEL_PASS:
        ReadAccel(&(raw_accel_readings[GyroAxisMap[X_AXIS]]), &(raw_accel_readings[GyroAxisMap[Y_AXIS]]), &(raw_accel_readings[GyroAxisMap[Z_AXIS]]));
        cb_parms->rate_loop_pass = KINEMATICS_PASS;
        break;

    case KINEMATICS_PASS:
        // Unpack the gyro data into the correct axes, and apply the gyro offsets
        raw_gyro_readings[GyroAxisMap[X_AXIS]] -= cb_parms->gyro_offsets[X_AXIS];
        raw_gyro_readings[GyroAxisMap[Y_AXIS]] -= cb_parms->gyro_offsets[Y_AXIS];
        raw_gyro_readings[GyroAxisMap[Z_AXIS]] -= cb_parms->gyro_offsets[Z_AXIS];

        // If the system anaylzer is enabled, output the new value here
#ifdef ENABLE_RATE_LOOP_TUNING
#ifdef USE_SYS_ANALYZER
        SystemAnalyzerSendReceive(*SysAnalyzerDataPtr);
#endif
#endif

        // Do gyro sign correction
        cb_parms->gyro_readings[AZ] = (raw_gyro_readings[AZ] * GyroSignMap[AZ]);
        cb_parms->gyro_readings[EL] = (raw_gyro_readings[EL] * GyroSignMap[EL]);
        cb_parms->gyro_readings[ROLL] = (raw_gyro_readings[ROLL] * GyroSignMap[ROLL]);

        // Do the 10-cycle integration of the raw gyro readings for the 100Hz gyro telemetry
        cb_parms->integrated_raw_gyro_readings[AZ] += cb_parms->gyro_readings[AZ];
        cb_parms->integrated_raw_gyro_readings[EL] += cb_parms->gyro_readings[EL];
        cb_parms->integrated_raw_gyro_readings[ROLL] += cb_parms->gyro_readings[ROLL];

        // Do the 10-cycle integration of the raw accelerometer readings for the 100Hz accelerometer telemetry
        cb_parms->integrated_raw_accel_readings[AZ] += raw_accel_readings[AZ];
        cb_parms->integrated_raw_accel_readings[EL] += raw_accel_readings[EL];
        cb_parms->integrated_raw_accel_readings[ROLL] += raw_accel_readings[ROLL];

        // Do gyro kinematics correction
        do_gyro_correction(&(cb_parms->gyro_readings[0]), &(cb_parms->encoder_readings[0]), &(cb_parms->corrected_gyro_readings[0]));
        //TODO: Temp for testing with a single axis
        //cb_parms->corrected_gyro_readings[AZ] = cb_parms->gyro_readings[AZ];
        //cb_parms->corrected_gyro_readings[EL] = cb_parms->gyro_readings[EL];
        //cb_parms->corrected_gyro_readings[ROLL] = cb_parms->gyro_readings[ROLL];

        // Run the 1st stage of the running average filter
        running_avg_filter_iteration(pos_loop_stage_1,
                        CorrectEncoderError(cb_parms->encoder_readings[AZ]),
                        CorrectEncoderError(cb_parms->encoder_readings[EL]),
                        CorrectEncoderError(cb_parms->encoder_readings[ROLL]));

        // See if we need to run the 2nd stage of the running average filter
        if (cb_parms->pos_loop_2nd_stage_decimation_count >= RUNNING_AVERAGE_DECIMATION_LIMIT) {
            cb_parms->pos_loop_2nd_stage_decimation_count = 0;
            running_avg_filter_iteration(pos_loop_stage_2,
                    pos_loop_stage_1->az_avg,
                    pos_loop_stage_1->el_avg,
                    pos_loop_stage_1->rl_avg);
        }

        // Position errors with running average
        cb_parms->filtered_position_errors[AZ] = CorrectEncoderError(cb_parms->angle_targets[AZ] - pos_loop_stage_2->az_avg);
        cb_parms->filtered_position_errors[EL] = CorrectEncoderError(cb_parms->angle_targets[EL] - pos_loop_stage_2->el_avg);
        cb_parms->filtered_position_errors[ROLL] = CorrectEncoderError(cb_parms->angle_targets[ROLL] - pos_loop_stage_2->rl_avg);
        // Position errors with no running average
        cb_parms->unfiltered_position_errors[AZ] = CorrectEncoderError(cb_parms->angle_targets[AZ] - cb_parms->encoder_readings[AZ]);
        cb_parms->unfiltered_position_errors[EL] = CorrectEncoderError(cb_parms->angle_targets[EL] - cb_parms->encoder_readings[EL]);
        cb_parms->unfiltered_position_errors[ROLL] = CorrectEncoderError(cb_parms->angle_targets[ROLL] - cb_parms->encoder_readings[ROLL]);

        //SendDebug1ToAz(cb_parms->encoder_readings[AZ], cb_parms->encoder_readings[EL], cb_parms->encoder_readings[ROLL]);

        // Set up the next rate loop pass to be the az error computation pass
        cb_parms->rate_loop_pass = ERROR_AZ_PASS;
        break;

        case ERROR_AZ_PASS:
#if defined(ENABLE_BALANCE_PROCEDURE)
            if (bal_proc_parms->balance_axis == AZ) {
                cb_parms->axis_errors[AZ] = CorrectEncoderError(cb_parms->angle_targets[AZ] - cb_parms->encoder_readings[AZ]);
                cb_parms->axis_errors[EL] = 0;
                cb_parms->axis_errors[ROLL] = 0;
            }
#elif defined(ENABLE_RATE_LOOP_TUNING)
            cb_parms->axis_errors[AZ] = cb_parms->tuning_rate_inject[AZ] - cb_parms->corrected_gyro_readings[AZ];
#else
#if (POSITION_LOOP_TYPE == 0)
            cb_parms->axis_errors[AZ] = UpdatePID_Float(PID_DATA_POSITION_LOOP, AZ, cb_parms->unfiltered_position_errors[AZ] - cb_parms->corrected_gyro_readings[AZ]);
#elif (POSITION_LOOP_TYPE == 1)
            if (cb_parms->unfiltered_position_errors[AZ] > (position_loop_deadband_counts + cb_parms->position_deadband_hysteresis_positive[AZ])) {
                // We're outside the deadband in the positive direction, so both rate and position errors are taken into account
                cb_parms->axis_errors[AZ] = UpdatePID_Float(PID_DATA_POSITION_LOOP, AZ, cb_parms->filtered_position_errors[AZ]) - cb_parms->corrected_gyro_readings[AZ];

                // Keep track of which side of the deadband we are for hysteresis
                cb_parms->out_of_position_deadband_positive[AZ] = TRUE;
                cb_parms->out_of_position_deadband_negative[AZ] = FALSE;

                // Clear all hysteresis (since we're out of the deadband)
                cb_parms->position_deadband_hysteresis_positive[AZ] = 0;
                cb_parms->position_deadband_hysteresis_negative[AZ] = 0;
            } else if (cb_parms->unfiltered_position_errors[AZ] < (-position_loop_deadband_counts + cb_parms->position_deadband_hysteresis_negative[AZ])) {
                // We're outside the deadband in the negative direction, so both rate and position errors are taken into account
                cb_parms->axis_errors[AZ] = UpdatePID_Float(PID_DATA_POSITION_LOOP, AZ, cb_parms->filtered_position_errors[AZ]) - cb_parms->corrected_gyro_readings[AZ];

                // Keep track of which side of the deadband we are for hysteresis
                cb_parms->out_of_position_deadband_positive[AZ] = FALSE;
                cb_parms->out_of_position_deadband_negative[AZ] = TRUE;

                // Clear all hysteresis (since we're out of the deadband)
                cb_parms->position_deadband_hysteresis_positive[AZ] = 0;
                cb_parms->position_deadband_hysteresis_negative[AZ] = 0;
            } else {
                // We're in the deadband

                // If we've just entered the deadband for this axis, clear the position loop PID history (so it starts from scratch the next time we go out of the deadband)
                if (cb_parms->out_of_position_deadband_positive[AZ] || cb_parms->out_of_position_deadband_negative[AZ]) {
                    ClearPIDHistory_Float(PID_DATA_POSITION_LOOP, AZ);

                    // Also flush the running average pipeline to the target position value
                    flush_running_avg_pipeline(&(pos_loop_filter_parms_stage_2), AZ, cb_parms->angle_targets[AZ]);

                    // Depending on which side of the deadband we just entered from, update the hysteresis offsets
                    if (cb_parms->out_of_position_deadband_positive[AZ]) {
                        cb_parms->position_deadband_hysteresis_positive[AZ] = position_loop_deadband_hysteresis;
                        cb_parms->out_of_position_deadband_positive[AZ] = FALSE;
                    } else {
                        cb_parms->position_deadband_hysteresis_negative[AZ] = -position_loop_deadband_hysteresis;
                        cb_parms->out_of_position_deadband_negative[AZ] = FALSE;
                    }
                }

                // We're in the deadband, so only rate errors are taken into account
                cb_parms->axis_errors[AZ] = -cb_parms->corrected_gyro_readings[AZ];
            }
#elif (POSITION_LOOP_TYPE == 2)
            cb_parms->axis_errors[AZ] = UpdatePID_Float(PID_DATA_POSITION_LOOP, AZ, cb_parms->unfiltered_position_errors[AZ]) - cb_parms->corrected_gyro_readings[AZ];
#else
            cb_parms->axis_errors[AZ] = cb_parms->rate_cmd_inject[AZ] - cb_parms->corrected_gyro_readings[AZ];
#endif
#endif
            // Set up the next rate loop pass to be the el error computation pass
            cb_parms->rate_loop_pass = ERROR_EL_PASS;
            break;

        case ERROR_EL_PASS:
#if defined(ENABLE_BALANCE_PROCEDURE)
            if (bal_proc_parms->balance_axis == EL) {
                cb_parms->axis_errors[EL] = CorrectEncoderError(cb_parms->angle_targets[EL] - cb_parms->encoder_readings[EL]);
                cb_parms->axis_errors[AZ] = 0;
                cb_parms->axis_errors[ROLL] = 0;
            }
#elif defined(ENABLE_RATE_LOOP_TUNING)
            cb_parms->axis_errors[EL] = cb_parms->tuning_rate_inject[EL] - cb_parms->corrected_gyro_readings[EL];
#else
#if (POSITION_LOOP_TYPE == 0)
            cb_parms->axis_errors[EL] = UpdatePID_Float(PID_DATA_POSITION_LOOP, EL, cb_parms->unfiltered_position_errors[EL] - cb_parms->corrected_gyro_readings[EL]);
#elif (POSITION_LOOP_TYPE == 1)
            if (cb_parms->unfiltered_position_errors[EL] > (position_loop_deadband_counts + cb_parms->position_deadband_hysteresis_positive[EL])) {
                // We're outside the deadband in the positive direction, so both rate and position errors are taken into account
                cb_parms->axis_errors[EL] = UpdatePID_Float(PID_DATA_POSITION_LOOP, EL, cb_parms->filtered_position_errors[EL]) - cb_parms->corrected_gyro_readings[EL];

                // Keep track of which side of the deadband we are for hysteresis
                cb_parms->out_of_position_deadband_positive[EL] = TRUE;
                cb_parms->out_of_position_deadband_negative[EL] = FALSE;

                // Clear all hysteresis (since we're out of the deadband)
                cb_parms->position_deadband_hysteresis_positive[EL] = 0;
                cb_parms->position_deadband_hysteresis_negative[EL] = 0;
            } else if (cb_parms->unfiltered_position_errors[EL] < (-position_loop_deadband_counts + cb_parms->position_deadband_hysteresis_negative[EL])) {
                // We're outside the deadband in the negative direction, so both rate and position errors are taken into account
                cb_parms->axis_errors[EL] = UpdatePID_Float(PID_DATA_POSITION_LOOP, EL, cb_parms->filtered_position_errors[EL]) - cb_parms->corrected_gyro_readings[EL];

                // Keep track of which side of the deadband we are for hysteresis
                cb_parms->out_of_position_deadband_positive[EL] = FALSE;
                cb_parms->out_of_position_deadband_negative[EL] = TRUE;

                // Clear all hysteresis (since we're out of the deadband)
                cb_parms->position_deadband_hysteresis_positive[EL] = 0;
                cb_parms->position_deadband_hysteresis_negative[EL] = 0;
            } else {
                // We're in the deadband

                // If we've just entered the deadband for this axis, clear the position loop PID history (so it starts from scratch the next time we go out of the deadband)
                if (cb_parms->out_of_position_deadband_positive[EL] || cb_parms->out_of_position_deadband_negative[EL]) {
                    ClearPIDHistory_Float(PID_DATA_POSITION_LOOP, EL);

                    // Also flush the running average pipeline to the target position value
                    flush_running_avg_pipeline(&(pos_loop_filter_parms_stage_2), EL, cb_parms->angle_targets[EL]);

                    // Depending on which side of the deadband we just entered from, update the hysteresis offsets
                    if (cb_parms->out_of_position_deadband_positive[EL]) {
                        cb_parms->position_deadband_hysteresis_positive[EL] = position_loop_deadband_hysteresis;
                        cb_parms->out_of_position_deadband_positive[EL] = FALSE;
                    } else {
                        cb_parms->position_deadband_hysteresis_negative[EL] = -position_loop_deadband_hysteresis;
                        cb_parms->out_of_position_deadband_negative[EL] = FALSE;
                    }
                }

                // We're in the deadband, so only rate errors are taken into account
                cb_parms->axis_errors[EL] = -cb_parms->corrected_gyro_readings[EL];
            }
#elif (POSITION_LOOP_TYPE == 2)
            cb_parms->axis_errors[EL] = UpdatePID_Float(PID_DATA_POSITION_LOOP, EL, cb_parms->unfiltered_position_errors[EL]) - cb_parms->corrected_gyro_readings[EL];
#else
            cb_parms->axis_errors[EL] = cb_parms->rate_cmd_inject[EL] - cb_parms->corrected_gyro_readings[EL];
#endif
#endif
            // Set up the next rate loop pass to be the roll error computation pass
            cb_parms->rate_loop_pass = ERROR_ROLL_PASS;
            break;

        case ERROR_ROLL_PASS:
#if defined(ENABLE_BALANCE_PROCEDURE)
            if (bal_proc_parms->balance_axis == ROLL) {
                cb_parms->axis_errors[ROLL] = CorrectEncoderError(cb_parms->angle_targets[ROLL] - cb_parms->encoder_readings[ROLL]);
                cb_parms->axis_errors[AZ] = 0;
                cb_parms->axis_errors[EL] = 0;
            }
#elif defined(ENABLE_RATE_LOOP_TUNING)
            cb_parms->axis_errors[ROLL] = cb_parms->tuning_rate_inject[ROLL] - cb_parms->corrected_gyro_readings[ROLL];
#else
#if (POSITION_LOOP_TYPE == 0)
            cb_parms->axis_errors[ROLL] = UpdatePID_Float(PID_DATA_POSITION_LOOP, ROLL, cb_parms->unfiltered_position_errors[ROLL] - cb_parms->corrected_gyro_readings[ROLL]);
#elif (POSITION_LOOP_TYPE == 1)
            if (cb_parms->unfiltered_position_errors[ROLL] > (position_loop_deadband_counts + cb_parms->position_deadband_hysteresis_positive[ROLL])) {
                // We're outside the deadband in the positive direction, so both rate and position errors are taken into account
                cb_parms->axis_errors[ROLL] = UpdatePID_Float(PID_DATA_POSITION_LOOP, ROLL, cb_parms->filtered_position_errors[ROLL]) - cb_parms->corrected_gyro_readings[ROLL];

                // Keep track of which side of the deadband we are for hysteresis
                cb_parms->out_of_position_deadband_positive[ROLL] = TRUE;
                cb_parms->out_of_position_deadband_negative[ROLL] = FALSE;

                // Clear all hysteresis (since we're out of the deadband)
                cb_parms->position_deadband_hysteresis_positive[ROLL] = 0;
                cb_parms->position_deadband_hysteresis_negative[ROLL] = 0;
            } else if (cb_parms->unfiltered_position_errors[ROLL] < (-position_loop_deadband_counts + cb_parms->position_deadband_hysteresis_negative[ROLL])) {
                // We're outside the deadband in the negative direction, so both rate and position errors are taken into account
                cb_parms->axis_errors[ROLL] = UpdatePID_Float(PID_DATA_POSITION_LOOP, ROLL, cb_parms->filtered_position_errors[ROLL]) - cb_parms->corrected_gyro_readings[ROLL];

                // Keep track of which side of the deadband we are for hysteresis
                cb_parms->out_of_position_deadband_positive[ROLL] = FALSE;
                cb_parms->out_of_position_deadband_negative[ROLL] = TRUE;

                // Clear all hysteresis (since we're out of the deadband)
                cb_parms->position_deadband_hysteresis_positive[ROLL] = 0;
                cb_parms->position_deadband_hysteresis_negative[ROLL] = 0;
            } else {
                // We're in the deadband

                // If we've just entered the deadband for this axis, clear the position loop PID history (so it starts from scratch the next time we go out of the deadband)
                if (cb_parms->out_of_position_deadband_positive[ROLL] || cb_parms->out_of_position_deadband_negative[ROLL]) {
                    ClearPIDHistory_Float(PID_DATA_POSITION_LOOP, ROLL);

                    // Also flush the running average pipeline to the target position value
                    flush_running_avg_pipeline(&(pos_loop_filter_parms_stage_2), ROLL, cb_parms->angle_targets[ROLL]);

                    // Depending on which side of the deadband we just entered from, update the hysteresis offsets
                    if (cb_parms->out_of_position_deadband_positive[ROLL]) {
                        cb_parms->position_deadband_hysteresis_positive[ROLL] = position_loop_deadband_hysteresis;
                        cb_parms->out_of_position_deadband_positive[ROLL] = FALSE;
                    } else {
                        cb_parms->position_deadband_hysteresis_negative[ROLL] = -position_loop_deadband_hysteresis;
                        cb_parms->out_of_position_deadband_negative[ROLL] = FALSE;
                    }
                }

                // We're in the deadband, so only rate errors are taken into account
                cb_parms->axis_errors[ROLL] = -cb_parms->corrected_gyro_readings[ROLL];
            }
#elif (POSITION_LOOP_TYPE == 2)
            cb_parms->axis_errors[ROLL] = UpdatePID_Float(PID_DATA_POSITION_LOOP, ROLL, cb_parms->unfiltered_position_errors[ROLL]) - cb_parms->corrected_gyro_readings[ROLL];
#else
            cb_parms->axis_errors[ROLL] = cb_parms->rate_cmd_inject[ROLL] - cb_parms->corrected_gyro_readings[ROLL];
#endif
#endif
            // Set up the next rate loop pass to be the torque command output pass
            cb_parms->rate_loop_pass = TORQUE_OUT_PASS;
            break;

        case TORQUE_OUT_PASS:
            // Run PID rate loops
            // Fixed point PID code
            //cb_parms->motor_torques[AZ] = UpdatePID(AZ, cb_parms->axis_errors[AZ]) * TorqueSignMap[AZ];
            //cb_parms->motor_torques[EL] = UpdatePID(EL, cb_parms->axis_errors[EL]) * TorqueSignMap[EL];
            //cb_parms->motor_torques[ROLL] = UpdatePID(ROLL, cb_parms->axis_errors[ROLL]) * TorqueSignMap[ROLL];
            // Floating point PID code
            // Run the rate loop PID unless we're in balance procedure mode, then run the position loop PID
#ifndef ENABLE_BALANCE_PROCEDURE

            // Compute the new motor torque commands
            cb_parms->motor_torques[AZ] = UpdatePID_Float(PID_DATA_RATE_LOOP, AZ, cb_parms->axis_errors[AZ]) * TorqueSignMap[AZ];
            cb_parms->motor_torques[EL] = UpdatePID_Float(PID_DATA_RATE_LOOP, EL, cb_parms->axis_errors[EL]) * TorqueSignMap[EL];
            cb_parms->motor_torques[ROLL] = UpdatePID_Float(PID_DATA_RATE_LOOP, ROLL, cb_parms->axis_errors[ROLL]) * TorqueSignMap[ROLL];

            //SendDebug1ToAz(el_pos_error, cb_parms->unfiltered_position_errors[EL], (int16_t)(pos_pid_loop_float[EL].integralCumulative + 0.5));
            //SendDebug1ToAz(cb_parms->unfiltered_position_errors[AZ], cb_parms->unfiltered_position_errors[EL], cb_parms->unfiltered_position_errors[ROLL]);

            // For AZ, we need to inject extra torque if we're getting close to either of the stops (because we have limited mechanical travel on AZ, and we
            // don't want to rail up against the stop).  To do this, outside of a deadband in the middle of travel, we linearly increase an extra torque injection
            // up to max torque as we get closer and closer to the stop.
            float extra_torque = 0;
            if ((cb_parms->encoder_readings[AZ] > 5000) && (cb_parms->encoder_readings[AZ] < ((int16)AZ_KEEP_OFF_STOP_START_COUNT_NEGATIVE))) {
                // We're starting to get close to the stop on the negative side, so we need to start adding gain to the position error
                extra_torque = (((AZ_KEEP_OFF_STOP_START_COUNT_NEGATIVE - ((float)cb_parms->encoder_readings[AZ])) / AZ_KEEP_OFF_STOP_SPAN_NEGATIVE) * AZ_KEEP_OFF_STOP_MAX_TORQUE) * TorqueSignMap[AZ];
            } else if ((cb_parms->encoder_readings[AZ] < 5000) && (cb_parms->encoder_readings[AZ] > ((int16)AZ_KEEP_OFF_STOP_START_COUNT_POSITIVE))) {
                // We're starting to get close to the stop on the positive side, so we need to start adding gain to the position error
                extra_torque = (((AZ_KEEP_OFF_STOP_START_COUNT_POSITIVE - ((float)cb_parms->encoder_readings[AZ])) / AZ_KEEP_OFF_STOP_SPAN_POSITIVE) * AZ_KEEP_OFF_STOP_MAX_TORQUE) * TorqueSignMap[AZ];
            }

            int32 az_torque = cb_parms->motor_torques[AZ] + (int32)extra_torque;
            if (az_torque > INT16_MAX) {
                az_torque = INT16_MAX;
            } else if (az_torque < INT16_MIN) {
                az_torque = INT16_MIN;
            }

            cb_parms->motor_torques[AZ] = az_torque;

#else
            cb_parms->motor_torques[AZ] = UpdatePID_Float(PID_DATA_POSITION_LOOP, AZ, cb_parms->axis_errors[AZ]) * TorqueSignMap[AZ];
            cb_parms->motor_torques[EL] = UpdatePID_Float(PID_DATA_POSITION_LOOP, EL, cb_parms->axis_errors[EL]) * TorqueSignMap[EL];
            cb_parms->motor_torques[ROLL] = UpdatePID_Float(PID_DATA_POSITION_LOOP, ROLL, cb_parms->axis_errors[ROLL]) * TorqueSignMap[ROLL];
#endif
#ifdef TEST_MAX_TORQUE
            // Torque command with DIGBY STEPS
            DigbyCount++;
            if (DigbyCount < 10000) {
                // do nothing
            } else if (DigbyCount < 15000) {
                //Toggle max torque
                if ((DigbyCount >> 7) & 0x0001) {
                    // Max positive for 1/8s
                    //cb_parms->motor_torques[EL] = OUTPUT_LIMIT_UPPER_FLOAT;
                    cb_parms->motor_torques[EL] = 5000;
                } else {
                    // Max negative for 1/8s
                    //cb_parms->motor_torques[EL] = OUTPUT_LIMIT_LOWER_FLOAT;
                    cb_parms->motor_torques[EL] = -5000;
                }
            } else {
                DigbyCount = 0;
            }

#endif

#ifndef ENABLE_CURRENT_TOGGLE
            // Send out motor torques
            MDBSendTorques(cb_parms->motor_torques[AZ], cb_parms->motor_torques[ROLL]);
            //TODO: Start with just testing AZ
            //MDBSendTorques(0, 0);

            // Also update our own torque (fake like we got a value over CAN)
            param_set[CAND_PID_TORQUE].param = cb_parms->motor_torques[EL];
            //TODO: Zero out EL torque for testing
            //param_set[CAND_PID_TORQUE].param = 0;
            *param_set[CAND_PID_TORQUE].sema = TRUE;
#endif

            // Send encoder, gyro, and accelerometer telemetry at a decimated rate of 100Hz
            if (++telemetry_decimation_count >= TELEMETRY_DECIMATION_LIMIT) {
                SendEncoderTelemetry(cb_parms->encoder_readings[AZ], cb_parms->encoder_readings[EL], cb_parms->encoder_readings[ROLL]);
                SendGyroTelemetry(cb_parms->integrated_raw_gyro_readings[AZ], cb_parms->integrated_raw_gyro_readings[EL], cb_parms->integrated_raw_gyro_readings[ROLL]);
                SendAccelTelemetry(cb_parms->integrated_raw_accel_readings[AZ], cb_parms->integrated_raw_accel_readings[EL], cb_parms->integrated_raw_accel_readings[ROLL]);

                // Zero out the gyro integrators for the next cycle
                cb_parms->integrated_raw_gyro_readings[AZ] = 0;
                cb_parms->integrated_raw_gyro_readings[EL] = 0;
                cb_parms->integrated_raw_gyro_readings[ROLL] = 0;

                // Zero out the accel integrators for the next cycle
                cb_parms->integrated_raw_accel_readings[AZ] = 0;
                cb_parms->integrated_raw_accel_readings[EL] = 0;
                cb_parms->integrated_raw_accel_readings[ROLL] = 0;

                telemetry_decimation_count = 0;
            }

            // We've completed one full rate loop iteration, so on the next pass go back to the beginning
            cb_parms->rate_loop_pass = READ_GYRO_PASS;
            break;
    }
}

static void SendEncoderTelemetry(int16 az_encoder, int16 el_encoder, int16 rl_encoder)
{
    Uint8 encoder_readings[6];
    encoder_readings[0] = (az_encoder >> 8) & 0x00FF;
    encoder_readings[1] = (az_encoder & 0x00FF);
    encoder_readings[2] = (el_encoder >> 8) & 0x00FF;
    encoder_readings[3] = (el_encoder & 0x00FF);
    encoder_readings[4] = (rl_encoder >> 8) & 0x00FF;
    encoder_readings[5] = (rl_encoder & 0x00FF);

    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_ENCODER_TELEMETRY, encoder_readings, 6);
}

static void SendGyroTelemetry(int32 az_gyro, int32 el_gyro, int32 rl_gyro)
{
    Uint8 gyro_az_readings[4];
    Uint8 gyro_el_readings[4];
    Uint8 gyro_rl_readings[4];

    gyro_az_readings[0] = (az_gyro >> 24) & 0x000000FF;
    gyro_az_readings[1] = (az_gyro >> 16) & 0x000000FF;
    gyro_az_readings[2] = (az_gyro >> 8) & 0x000000FF;
    gyro_az_readings[3] = (az_gyro & 0x000000FF);

    gyro_el_readings[0] = (el_gyro >> 24) & 0x000000FF;
    gyro_el_readings[1] = (el_gyro >> 16) & 0x000000FF;
    gyro_el_readings[2] = (el_gyro >> 8) & 0x000000FF;
    gyro_el_readings[3] = (el_gyro & 0x000000FF);

    gyro_rl_readings[0] = (rl_gyro >> 24) & 0x000000FF;
    gyro_rl_readings[1] = (rl_gyro >> 16) & 0x000000FF;
    gyro_rl_readings[2] = (rl_gyro >> 8) & 0x000000FF;
    gyro_rl_readings[3] = (rl_gyro & 0x000000FF);

    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_GYRO_AZ_TELEMETRY, gyro_az_readings, 4);
    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_GYRO_EL_TELEMETRY, gyro_el_readings, 4);
    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_GYRO_RL_TELEMETRY, gyro_rl_readings, 4);
}

static void SendAccelTelemetry(int32 az_accel, int32 el_accel, int32 rl_accel)
{
    Uint8 accel_az_readings[4];
    Uint8 accel_el_readings[4];
    Uint8 accel_rl_readings[4];

    accel_az_readings[0] = (az_accel >> 24) & 0x000000FF;
    accel_az_readings[1] = (az_accel >> 16) & 0x000000FF;
    accel_az_readings[2] = (az_accel >> 8) & 0x000000FF;
    accel_az_readings[3] = (az_accel & 0x000000FF);

    accel_el_readings[0] = (el_accel >> 24) & 0x000000FF;
    accel_el_readings[1] = (el_accel >> 16) & 0x000000FF;
    accel_el_readings[2] = (el_accel >> 8) & 0x000000FF;
    accel_el_readings[3] = (el_accel & 0x000000FF);

    accel_rl_readings[0] = (rl_accel >> 24) & 0x000000FF;
    accel_rl_readings[1] = (rl_accel >> 16) & 0x000000FF;
    accel_rl_readings[2] = (rl_accel >> 8) & 0x000000FF;
    accel_rl_readings[3] = (rl_accel & 0x000000FF);

    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_ACCEL_AZ_TELEMETRY, accel_az_readings, 4);
    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_ACCEL_EL_TELEMETRY, accel_el_readings, 4);
    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_ACCEL_RL_TELEMETRY, accel_rl_readings, 4);
}
