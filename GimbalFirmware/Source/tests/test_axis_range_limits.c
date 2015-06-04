/*
 * test_axis_range_limits.c
 *
 *  Created on: Mar 2, 2015
 *      Author: abamberger
 */

#include "tests/test_axis_range_limits.h"
#include "PM_Sensorless-Settings.h"
#include "PM_Sensorless.h"
#include "MAVLink/ardupilotmega/mavlink.h"
#include "can/cb.h"

#include <math.h>

TorqueDataMonitor torque_data = {
    0,      // Last torque command
    0,      // Max motor torque command
    0,      // Max motor torque command location
    0,      // Motor torque average accumulator
    0       // Motor torque accumulator number of samples
};

// local prototypes
static void send_error_and_reset_parms(TestAxisRangeLimitsParms* test_parms, ControlBoardParms* cb_parms, AxisRangeLimitsTestStatus error);
static void update_torque_data(TorqueDataMonitor* torque_data, ControlBoardParms* cb_parms, GimbalAxis test_axis);
static void clear_torque_data(TorqueDataMonitor* torque_data);

//
// Test the full range of movement on all 3 axes
//
int RunTestAxisRangeLimitsIteration(TestAxisRangeLimitsParms* test_parms, ControlBoardParms* cb_parms)
{
#define ENCODER_COUNTS_PER_DEGREE   27
#define ENCODER_OFF_STOP            (4 * ENCODER_COUNTS_PER_DEGREE)

    TestResult                 result_id_encoder_range;
    TestResult                 result_id_encoder_asymmetry;
    AxisRangeLimitsTestSection next_section;
    static int16  last_encoder_position;

    update_torque_data(&torque_data, cb_parms, test_parms->test_axis);

    switch (test_parms->test_state) {
        case RANGE_LIMITS_STATE_INIT:
        {
            // This test may be run multiple times, so need to initialize all test variables each time
            test_parms->test_axis = EL;
            test_parms->status_output_decimation_count = 0;
            test_parms->settle_counter = 0;
            test_parms->current_section = AXIS_RANGE_TEST_SECTION_EL_CHECK_NEG;
            clear_torque_data(&torque_data);

            // Switch to position mode for this test
            cb_parms->control_loop_type = POSITION_MODE;

            cb_parms->angle_targets[test_parms->test_axis] = 0;
            test_parms->current_axis_position = 0.0;

            CANSendFactoryTestProgress(FACTORY_TEST_AXIS_RANGE_LIMITS, test_parms->current_section, 0, AXIS_RANGE_TEST_STATUS_IN_PROGRESS);

            test_parms->test_state = RANGE_LIMITS_STATE_SETTLE_AT_HOME_1;
        }
        break;

        case RANGE_LIMITS_STATE_SETTLE_AT_HOME_1:
            if (test_parms->settle_counter++ >= ((Uint32)ISR_FREQUENCY * (Uint32)AXIS_SETTLE_TIME_MS)) {
                test_parms->settle_counter = 0;
                test_parms->last_encoder_reading = cb_parms->encoder_readings[test_parms->test_axis];
                last_encoder_position = test_parms->last_encoder_reading + 300; // 300 just guarantees hard stop won't be detected right away

                // Move more quickly while finding stops to give axes extra momentum
                switch (test_parms->test_axis) {
                    case EL :
                        test_parms->position_step = EL_AXIS_POS_STEP_SIZE_FIND_STOPS;
                        break;
                    case ROLL :
                        test_parms->position_step = RL_AXIS_POS_STEP_SIZE_FIND_STOPS;
                        break;
                    case AZ :
                    default :
                        test_parms->position_step = AZ_AXIS_POS_STEP_SIZE_FIND_STOPS;
                        break;
                }
                test_parms->next_position_pause_point = -PAUSE_POINT_INCR;
                test_parms->test_state = RANGE_LIMITS_STATE_MOVE_TO_NEGATIVE_LIMIT;
            }
            break;

        case RANGE_LIMITS_STATE_MOVE_TO_NEGATIVE_LIMIT:
            if (test_parms->current_axis_position <= test_parms->next_position_pause_point) {
                // wait here for a bit
                if (test_parms->settle_counter++ >= ((Uint32)ISR_FREQUENCY * (Uint32)HARDSTOP_SETTLE_TIME_MS)) {
                    test_parms->settle_counter = 0;
                    if (abs(test_parms->last_encoder_reading - cb_parms->encoder_readings[test_parms->test_axis]) > MAX_STOPPED_ENCODER_CHANGE_ALLOWED) {
                        // still moving so keep waiting
                        test_parms->last_encoder_reading = cb_parms->encoder_readings[test_parms->test_axis];
                    } else {
                        // stopped moving at this position
                        // if haven't moved much since the last position, we're at a hard stop
                        int16  stop_threshold = HARDSTOP_INT_ENCODER_DETECTION_THRESHOLD;

                        if (test_parms->test_axis == AZ) {
                            stop_threshold = HARDSTOP_INT_ENCODER_DETECTION_THRESHOLD_YAW;
                        }

                        if (abs(last_encoder_position - test_parms->last_encoder_reading) < stop_threshold) {
                            // we've reached the hard stop, save the encoder value and move to the next state
                            test_parms->encoder_hard_stop_neg[test_parms->test_axis] = test_parms->last_encoder_reading;
                            test_parms->status_output_decimation_count = 0;
                            test_parms->settle_counter = 0;
                            test_parms->test_state = RANGE_LIMITS_STATE_CHECK_NEGATIVE_LIMIT;
                            break;
                        }
                        // hard stop not detected yet - move to next pause point
                        last_encoder_position = cb_parms->encoder_readings[test_parms->test_axis];
                        test_parms->next_position_pause_point -= PAUSE_POINT_INCR;
                    }
                }
            } else {
                // not paused so take a step
                test_parms->current_axis_position -= test_parms->position_step;
                cb_parms->angle_targets[test_parms->test_axis] = (int16)test_parms->current_axis_position;
            }


            if (test_parms->status_output_decimation_count++ >= 1000) {
                test_parms->status_output_decimation_count = 0;
                Uint8 progress = (Uint8)((test_parms->current_axis_position / test_parms->axis_range_min[test_parms->test_axis]) * 100.0);
                if (progress >= 100) {
                    progress = 99;
                }
                CANSendFactoryTestProgress(FACTORY_TEST_AXIS_RANGE_LIMITS, test_parms->current_section, progress, AXIS_RANGE_TEST_STATUS_IN_PROGRESS);
            }

            test_parms->last_encoder_reading = cb_parms->encoder_readings[test_parms->test_axis];
            break;

        case RANGE_LIMITS_STATE_CHECK_NEGATIVE_LIMIT:
            CANSendFactoryTestProgress(FACTORY_TEST_AXIS_RANGE_LIMITS, test_parms->current_section, 100, AXIS_RANGE_TEST_STATUS_SUCCEEDED);
            switch (test_parms->test_axis) {
                case AZ:
                    test_parms->current_section = AXIS_RANGE_TEST_SECTION_AZ_CHECK_POS;
                    break;

                case EL:
                    test_parms->current_section = AXIS_RANGE_TEST_SECTION_EL_CHECK_POS;
                    break;

                case ROLL:
                    test_parms->current_section = AXIS_RANGE_TEST_SECTION_RL_CHECK_POS;
                    break;
            }
            test_parms->test_state = RANGE_LIMITS_STATE_MOVE_TO_HOME_1;
            break;

        case RANGE_LIMITS_STATE_MOVE_TO_HOME_1:
            test_parms->current_axis_position += test_parms->position_step;
            cb_parms->angle_targets[test_parms->test_axis] = (int16)test_parms->current_axis_position;

            if (test_parms->status_output_decimation_count++ >= 1000) {
                test_parms->status_output_decimation_count = 0;
                Uint8 progress = (Uint8)((1.0 - (test_parms->current_axis_position / test_parms->axis_range_min[test_parms->test_axis])) * 50.0);
                if (progress > 50) {
                    progress = 50;
                }
                CANSendFactoryTestProgress(FACTORY_TEST_AXIS_RANGE_LIMITS, test_parms->current_section, progress, AXIS_RANGE_TEST_STATUS_IN_PROGRESS);
            }

            if (test_parms->current_axis_position >= 0.0) {
                test_parms->status_output_decimation_count = 0;
                test_parms->settle_counter = 0;
                test_parms->last_encoder_reading = cb_parms->encoder_readings[test_parms->test_axis];
                last_encoder_position = test_parms->last_encoder_reading + 300; // 300 just guarantees hard stop won't be detected right away
                test_parms->next_position_pause_point = PAUSE_POINT_INCR;
                test_parms->test_state = RANGE_LIMITS_STATE_MOVE_TO_POSITIVE_LIMIT;
            }
            break;

        case RANGE_LIMITS_STATE_MOVE_TO_POSITIVE_LIMIT:
            if (test_parms->current_axis_position >= test_parms->next_position_pause_point) {
                // wait here for a bit
                if (test_parms->settle_counter++ >= ((Uint32)ISR_FREQUENCY * (Uint32)HARDSTOP_SETTLE_TIME_MS)) {
                    test_parms->settle_counter = 0;
                    if (abs(test_parms->last_encoder_reading - cb_parms->encoder_readings[test_parms->test_axis]) > MAX_STOPPED_ENCODER_CHANGE_ALLOWED) {
                        // still moving so keep waiting
                        test_parms->last_encoder_reading = cb_parms->encoder_readings[test_parms->test_axis];
                    } else {
                        // stopped moving at this position
                        // if haven't moved much since the last position, we're at a hard stop
                        int16  stop_threshold = HARDSTOP_INT_ENCODER_DETECTION_THRESHOLD;

                        if (test_parms->test_axis == AZ) {
                            stop_threshold = HARDSTOP_INT_ENCODER_DETECTION_THRESHOLD_YAW;
                        }

                        if (abs(last_encoder_position - test_parms->last_encoder_reading) < stop_threshold) {
                            // we've reached the hard stop, save the encoder value and move to the next state
                            test_parms->encoder_hard_stop_plus[test_parms->test_axis] = test_parms->last_encoder_reading;
                            test_parms->status_output_decimation_count = 0;
                            test_parms->settle_counter = 0;
                            test_parms->test_state = RANGE_LIMITS_STATE_CHECK_POSITIVE_LIMIT;
                            break;
                        }
                        // hard stop not detected yet - move to next pause point
                        last_encoder_position = cb_parms->encoder_readings[test_parms->test_axis];
                        test_parms->next_position_pause_point += PAUSE_POINT_INCR;
                    }
                }
            } else {
                // not paused so take a step
                test_parms->current_axis_position += test_parms->position_step;
                cb_parms->angle_targets[test_parms->test_axis] = (int16)test_parms->current_axis_position;
            }

            if (test_parms->status_output_decimation_count++ >= 1000) {
                test_parms->status_output_decimation_count = 0;
                Uint8 progress = (Uint8)(50.0 + ((test_parms->current_axis_position / test_parms->axis_range_max[test_parms->test_axis]) * 50.0));
                if (progress >= 100) {
                    progress = 99;
                }
                CANSendFactoryTestProgress(FACTORY_TEST_AXIS_RANGE_LIMITS, test_parms->current_section, progress, AXIS_RANGE_TEST_STATUS_IN_PROGRESS);
            }

            test_parms->last_encoder_reading = cb_parms->encoder_readings[test_parms->test_axis];
            break;

        case RANGE_LIMITS_STATE_CHECK_POSITIVE_LIMIT:
            CANSendFactoryTestProgress(FACTORY_TEST_AXIS_RANGE_LIMITS, test_parms->current_section, 100, AXIS_RANGE_TEST_STATUS_SUCCEEDED);
            test_parms->test_state = RANGE_LIMITS_STATE_MOVE_OFF_POS_STOP;
            break;

        case RANGE_LIMITS_STATE_MOVE_OFF_POS_STOP:
            if (cb_parms->encoder_readings[test_parms->test_axis] > (test_parms->encoder_hard_stop_plus[test_parms->test_axis] - ENCODER_OFF_STOP)) {
                test_parms->current_axis_position -= test_parms->position_step;
                cb_parms->angle_targets[test_parms->test_axis] = (int16)test_parms->current_axis_position;
            }
            else {
                switch (test_parms->test_axis) {
                    case AZ:
                        next_section = AXIS_RANGE_TEST_SECTION_AZ_RETURN_HOME;
                        result_id_encoder_range = TEST_RESULT_ENCODER_RANGE_AZ;
                        result_id_encoder_asymmetry = TEST_RESULT_ENCODER_ASYMMETRY_AZ;
                        break;

                    case EL:
                        next_section = AXIS_RANGE_TEST_SECTION_EL_RETURN_HOME;
                        result_id_encoder_range = TEST_RESULT_ENCODER_RANGE_EL;
                        result_id_encoder_asymmetry = TEST_RESULT_ENCODER_ASYMMETRY_EL;
                        break;

                    case ROLL:
                        next_section = AXIS_RANGE_TEST_SECTION_RL_RETURN_HOME;
                        result_id_encoder_range = TEST_RESULT_ENCODER_RANGE_RL;
                        result_id_encoder_asymmetry = TEST_RESULT_ENCODER_ASYMMETRY_RL;
                        break;
                }
                // total encoder range
                CANSendTestResult(result_id_encoder_range,
                                  ((float)test_parms->encoder_hard_stop_plus[test_parms->test_axis] - (float)test_parms->encoder_hard_stop_neg[test_parms->test_axis]));
                // how asymmetric the pos & neg axes are (zero is perfectly balanced between positive and negative axes)
                CANSendTestResult(result_id_encoder_asymmetry,
                                  ((float)test_parms->encoder_hard_stop_plus[test_parms->test_axis] + (float)test_parms->encoder_hard_stop_neg[test_parms->test_axis]));

                // test_parms->current_section = next_section;     TODO?
                //CANSendFactoryTestProgress(FACTORY_TEST_AXIS_RANGE_LIMITS, test_parms->current_section, 0, AXIS_RANGE_TEST_STATUS_SUCCEEDED);

                // Move more slowly while measuring torque
                test_parms->position_step = AXIS_POS_STEP_SIZE_MEASURE_TORQUE;

                //ResetMaxTorqueCmd(cb_parms, test_parms->test_axis);
                clear_torque_data(&torque_data);
                test_parms->test_state = RANGE_LIMITS_STATE_CHECK_NEGATIVE_TORQUE_POS_TO_HOME;
            }
            break;

        case RANGE_LIMITS_STATE_CHECK_NEGATIVE_TORQUE_POS_TO_HOME:
            test_parms->current_axis_position -= test_parms->position_step;
            cb_parms->angle_targets[test_parms->test_axis] = (int16)test_parms->current_axis_position;
            // TODO report progress
            if (test_parms->current_axis_position <= 0) {
                test_parms->status_output_decimation_count = 0;   // TODO for when progress is reported
                test_parms->test_state = RANGE_LIMITS_STATE_CHECK_NEGATIVE_TORQUE_HOME_TO_NEG;
            }
            break;

        case RANGE_LIMITS_STATE_CHECK_NEGATIVE_TORQUE_HOME_TO_NEG:
            if (cb_parms->encoder_readings[test_parms->test_axis] > (test_parms->encoder_hard_stop_neg[test_parms->test_axis] + ENCODER_OFF_STOP)) {
                test_parms->current_axis_position -= test_parms->position_step;
                cb_parms->angle_targets[test_parms->test_axis] = (int16)test_parms->current_axis_position;
            }
            else {
                //test_parms->motor_torque_max_neg[test_parms->test_axis] = -GetMaxTorqueCmd(cb_parms, test_parms->test_axis);
                test_parms->motor_torque_max_neg[test_parms->test_axis] = -torque_data.max_motor_torque_cmd;
                test_parms->motor_torque_max_neg_loc[test_parms->test_axis] = torque_data.max_motor_torque_cmd_loc;
                test_parms->motor_torque_avg_neg[test_parms->test_axis] = -((float)torque_data.motor_torque_accum / (float)torque_data.motor_torque_accum_nsamps);

                switch (test_parms->test_axis) {
                    case AZ:
                        CANSendTestResult(TEST_RESULT_NEG_MAX_TORQUE_AZ, (float)test_parms->motor_torque_max_neg[test_parms->test_axis]);
                        CANSendTestResult(TEST_RESULT_NEG_MAX_TORQUE_LOC_AZ, (float)test_parms->motor_torque_max_neg_loc[test_parms->test_axis]);
                        CANSendTestResult(TEST_RESULT_NEG_AVG_TORQUE_AZ, test_parms->motor_torque_avg_neg[test_parms->test_axis]);
                        break;

                    case EL:
                        CANSendTestResult(TEST_RESULT_NEG_MAX_TORQUE_EL, (float)test_parms->motor_torque_max_neg[test_parms->test_axis]);
                        CANSendTestResult(TEST_RESULT_NEG_MAX_TORQUE_LOC_EL, (float)test_parms->motor_torque_max_neg_loc[test_parms->test_axis]);
                        CANSendTestResult(TEST_RESULT_NEG_AVG_TORQUE_EL, test_parms->motor_torque_avg_neg[test_parms->test_axis]);
                        break;

                    case ROLL:
                        CANSendTestResult(TEST_RESULT_NEG_MAX_TORQUE_RL, (float)test_parms->motor_torque_max_neg[test_parms->test_axis]);
                        CANSendTestResult(TEST_RESULT_NEG_MAX_TORQUE_LOC_RL, (float)test_parms->motor_torque_max_neg_loc[test_parms->test_axis]);
                        CANSendTestResult(TEST_RESULT_NEG_AVG_TORQUE_RL, test_parms->motor_torque_avg_neg[test_parms->test_axis]);
                        break;
                }
                //ResetMaxTorqueCmd(cb_parms, test_parms->test_axis);
                clear_torque_data(&torque_data);
                test_parms->test_state = RANGE_LIMITS_STATE_CHECK_POSITIVE_TORQUE_NEG_TO_HOME;
            }
            break;

        case RANGE_LIMITS_STATE_CHECK_POSITIVE_TORQUE_NEG_TO_HOME:
            test_parms->current_axis_position += test_parms->position_step;
            cb_parms->angle_targets[test_parms->test_axis] = (int16)test_parms->current_axis_position;
            // TODO report progress
            if (test_parms->current_axis_position >= 0) {
                test_parms->status_output_decimation_count = 0;   // TODO for when progress is reported
                test_parms->test_state = RANGE_LIMITS_STATE_CHECK_POSITIVE_TORQUE_HOME_TO_POS;
            }
            break;

        case RANGE_LIMITS_STATE_CHECK_POSITIVE_TORQUE_HOME_TO_POS:
            if (cb_parms->encoder_readings[test_parms->test_axis] < (test_parms->encoder_hard_stop_plus[test_parms->test_axis] - ENCODER_OFF_STOP)) {
                test_parms->current_axis_position += test_parms->position_step;
                cb_parms->angle_targets[test_parms->test_axis] = (int16)test_parms->current_axis_position;
            }
            else {
                //test_parms->motor_torque_max_pos[test_parms->test_axis] = GetMaxTorqueCmd(cb_parms, test_parms->test_axis);
                test_parms->motor_torque_max_pos[test_parms->test_axis] = torque_data.max_motor_torque_cmd;
                test_parms->motor_torque_max_pos_loc[test_parms->test_axis] = torque_data.max_motor_torque_cmd_loc;
                test_parms->motor_torque_avg_pos[test_parms->test_axis] = (float)torque_data.motor_torque_accum / (float)torque_data.motor_torque_accum_nsamps;

                switch (test_parms->test_axis) {
                    case AZ:
                        CANSendTestResult(TEST_RESULT_POS_MAX_TORQUE_AZ, (float)test_parms->motor_torque_max_pos[test_parms->test_axis]);
                        CANSendTestResult(TEST_RESULT_POS_MAX_TORQUE_LOC_AZ, (float)test_parms->motor_torque_max_pos_loc[test_parms->test_axis]);
                        CANSendTestResult(TEST_RESULT_POS_AVG_TORQUE_AZ, test_parms->motor_torque_avg_pos[test_parms->test_axis]);
                        next_section = AXIS_RANGE_TEST_SECTION_AZ_RETURN_HOME;
                        break;

                    case EL:
                        CANSendTestResult(TEST_RESULT_POS_MAX_TORQUE_EL, (float)test_parms->motor_torque_max_pos[test_parms->test_axis]);
                        CANSendTestResult(TEST_RESULT_POS_MAX_TORQUE_LOC_EL, (float)test_parms->motor_torque_max_pos_loc[test_parms->test_axis]);
                        CANSendTestResult(TEST_RESULT_POS_AVG_TORQUE_EL, test_parms->motor_torque_avg_pos[test_parms->test_axis]);
                        next_section = AXIS_RANGE_TEST_SECTION_EL_RETURN_HOME;
                        break;

                    case ROLL:
                        CANSendTestResult(TEST_RESULT_POS_MAX_TORQUE_RL, (float)test_parms->motor_torque_max_pos[test_parms->test_axis]);
                        CANSendTestResult(TEST_RESULT_POS_MAX_TORQUE_LOC_RL, (float)test_parms->motor_torque_max_pos_loc[test_parms->test_axis]);
                        CANSendTestResult(TEST_RESULT_POS_AVG_TORQUE_RL, test_parms->motor_torque_avg_pos[test_parms->test_axis]);
                        next_section = AXIS_RANGE_TEST_SECTION_RL_RETURN_HOME;
                        break;
                }

                test_parms->test_state = RANGE_LIMITS_STATE_MOVE_TO_HOME_2;

                test_parms->current_section = next_section;
                CANSendFactoryTestProgress(FACTORY_TEST_AXIS_RANGE_LIMITS, test_parms->current_section, 0, AXIS_RANGE_TEST_STATUS_IN_PROGRESS);
            }
            break;

        case RANGE_LIMITS_STATE_MOVE_TO_HOME_2:
            test_parms->current_axis_position -= test_parms->position_step;

            cb_parms->angle_targets[test_parms->test_axis] = (int16)test_parms->current_axis_position;

            if (test_parms->status_output_decimation_count++ >= 1000) {
                test_parms->status_output_decimation_count = 0;
                Uint8 progress = (Uint8)((1.0 - (test_parms->current_axis_position / test_parms->axis_range_max[test_parms->test_axis])) * 100.0);
                if (progress >= 100) {
                    progress = 99;
                }
                CANSendFactoryTestProgress(FACTORY_TEST_AXIS_RANGE_LIMITS, test_parms->current_section, progress, AXIS_RANGE_TEST_STATUS_IN_PROGRESS);
            }

            if (test_parms->current_axis_position <= 0) {
                test_parms->status_output_decimation_count = 0;
                test_parms->test_state = RANGE_LIMITS_STATE_SETTLE_AT_HOME_2;
            }
            break;

        case RANGE_LIMITS_STATE_SETTLE_AT_HOME_2:
            if (test_parms->settle_counter++ >= ((Uint32)ISR_FREQUENCY * (Uint32)AXIS_SETTLE_TIME_MS)) {
                test_parms->settle_counter = 0;
                test_parms->test_state = RANGE_LIMITS_STATE_CHECK_HOME_POSITION;
            }
            break;

        case RANGE_LIMITS_STATE_CHECK_HOME_POSITION:
            {
                int16 error = abs(cb_parms->encoder_readings[test_parms->test_axis]);

                if (error > ALLOWED_POSITION_ERROR_COUNTS) {
                    send_error_and_reset_parms(test_parms, cb_parms, AXIS_RANGE_TEST_STATUS_FAILED_HOME);
                    return -1;
                }
                else {
                    CANSendFactoryTestProgress(FACTORY_TEST_AXIS_RANGE_LIMITS, test_parms->current_section, 100, AXIS_RANGE_TEST_STATUS_SUCCEEDED);

                    switch (test_parms->test_axis) {
                        case EL:
                            test_parms->test_axis = ROLL;
                            cb_parms->angle_targets[test_parms->test_axis] = 0;
                            test_parms->test_state = RANGE_LIMITS_STATE_SETTLE_AT_HOME_1;
                            test_parms->current_section = AXIS_RANGE_TEST_SECTION_RL_CHECK_NEG;
                            break;

                        case ROLL:
                            test_parms->test_axis = AZ;
                            cb_parms->angle_targets[test_parms->test_axis] = 0;
                            test_parms->test_state = RANGE_LIMITS_STATE_SETTLE_AT_HOME_1;
                            test_parms->current_section = AXIS_RANGE_TEST_SECTION_AZ_CHECK_NEG;
                            break;

                        case AZ:
                            cb_parms->angle_targets[AZ] = 0;
                            cb_parms->angle_targets[EL] = 0;
                            cb_parms->angle_targets[ROLL] = 0;
                            cb_parms->control_loop_type = RATE_MODE;
                            test_parms->test_state = RANGE_LIMITS_STATE_INIT;
                            return 1;
                            break;
                    }
                }
            }
            break;
    }

    return 0;
}

static void send_error_and_reset_parms(TestAxisRangeLimitsParms* test_parms, ControlBoardParms* cb_parms, AxisRangeLimitsTestStatus error)
{
    CANSendFactoryTestProgress(FACTORY_TEST_AXIS_RANGE_LIMITS, test_parms->current_section, 0, error);

    // Reset test parameters and other settings so the test can be re-run again
    cb_parms->angle_targets[AZ] = 0;
    cb_parms->angle_targets[EL] = 0;
    cb_parms->angle_targets[ROLL] = 0;
    cb_parms->control_loop_type = RATE_MODE;
    test_parms->test_state = RANGE_LIMITS_STATE_INIT;
}

static void update_torque_data(TorqueDataMonitor* torque_data, ControlBoardParms* cb_parms, GimbalAxis test_axis)
{
    // First, check if the new torque command is the same as the last torque command.  No need to add new data if it's the same as the last cycle
    // This will be helpful because this will be called at the torque loop rate (10kHz), but the torque command is only updated at the rate loop rate (1kHz)
    if (cb_parms->motor_torques[test_axis] != torque_data->last_torque_cmd) {
        if (abs(cb_parms->motor_torques[test_axis]) > torque_data->max_motor_torque_cmd) {
            torque_data->max_motor_torque_cmd = abs(cb_parms->motor_torques[test_axis]);
            torque_data->max_motor_torque_cmd_loc = cb_parms->encoder_readings[test_axis];
        }
        torque_data->motor_torque_accum += abs(cb_parms->motor_torques[test_axis]);
        torque_data->motor_torque_accum_nsamps++;

        torque_data->last_torque_cmd = cb_parms->motor_torques[test_axis];
    }
}

static void clear_torque_data(TorqueDataMonitor* torque_data)
{
    torque_data->last_torque_cmd = 0;
    torque_data->max_motor_torque_cmd = 0;
    torque_data->max_motor_torque_cmd_loc = 0;
    torque_data->motor_torque_accum = 0;
    torque_data->motor_torque_accum_nsamps = 0;
}

