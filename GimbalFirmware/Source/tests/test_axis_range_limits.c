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

// local prototypes
static void send_error_and_reset_parms(TestAxisRangeLimitsParms* test_parms, ControlBoardParms* cb_parms, AxisRangeLimitsTestStatus error);


//
// Test the full range of movement on all 3 axes
//
int RunTestAxisRangeLimitsIteration(TestAxisRangeLimitsParms* test_parms, ControlBoardParms* cb_parms)
{
#define ENCODER_COUNTS_PER_DEGREE   27
#define ENCODER_OFF_STOP            (4 * ENCODER_COUNTS_PER_DEGREE)

    TestResult                 result_id_encoder_range;
    TestResult                 result_id_encoder_asymmetry;
    TestResult                 result_id_torque;
    AxisRangeLimitsTestSection next_section;
    static int16  last_encoder_position;

    switch (test_parms->test_state) {
        case RANGE_LIMITS_STATE_INIT:
        {
            // This test may be run multiple times, so need to initialize all test variables each time
            test_parms->test_axis = EL;
            test_parms->status_output_decimation_count = 0;
            test_parms->settle_counter = 0;
            test_parms->current_section = AXIS_RANGE_TEST_SECTION_EL_CHECK_NEG;

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
                switch (test_parms->test_axis) {
                    case EL :
                        test_parms->position_step = EL_AXIS_POS_STEP_SIZE;
                        break;
                    case ROLL :
                        test_parms->position_step = RL_AXIS_POS_STEP_SIZE;
                        break;
                    case AZ :
                    default :
                        test_parms->position_step = AZ_AXIS_POS_STEP_SIZE;
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

                        //TODO: For debug
                        /*
                        Uint8 params[6];
                        params[0] = (last_encoder_position >> 8) & 0x00FF;
                        params[1] = (last_encoder_position & 0x00FF);
                        params[2] = (test_parms->last_encoder_reading >> 8) & 0x00FF;
                        params[3] = (test_parms->last_encoder_reading & 0x00FF);
                        params[4] = (cb_parms->angle_targets[test_parms->test_axis] >> 8) & 0x00FF;
                        params[5] = (cb_parms->angle_targets[test_parms->test_axis] & 0x00FF);
                        cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_ARBITRARY_DEBUG, params, 6);
                        */

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

                        //TODO: For debug
                        /*
                        Uint8 params[6];
                        params[0] = (last_encoder_position >> 8) & 0x00FF;
                        params[1] = (last_encoder_position & 0x00FF);
                        params[2] = (test_parms->last_encoder_reading >> 8) & 0x00FF;
                        params[3] = (test_parms->last_encoder_reading & 0x00FF);
                        params[4] = (cb_parms->angle_targets[test_parms->test_axis] >> 8) & 0x00FF;
                        params[5] = (cb_parms->angle_targets[test_parms->test_axis] & 0x00FF);
                        cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_ARBITRARY_DEBUG, params, 6);
                        */

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
                ResetMaxTorqueCmd(cb_parms, test_parms->test_axis);
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
                test_parms->motor_torque_max_neg[test_parms->test_axis] = -GetMaxTorqueCmd(cb_parms, test_parms->test_axis);
                switch (test_parms->test_axis) {
                    case AZ:
                        result_id_torque = TEST_RESULT_NEG_MAX_TORQUE_AZ;
                        break;

                    case EL:
                        result_id_torque = TEST_RESULT_NEG_MAX_TORQUE_EL;
                        break;

                    case ROLL:
                        result_id_torque = TEST_RESULT_NEG_MAX_TORQUE_RL;
                        break;
                }
                CANSendTestResult(result_id_torque, (float)test_parms->motor_torque_max_neg[test_parms->test_axis]);
                ResetMaxTorqueCmd(cb_parms, test_parms->test_axis);
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
                switch (test_parms->test_axis) {
                    case AZ:
                        next_section = AXIS_RANGE_TEST_SECTION_AZ_RETURN_HOME;
                        result_id_torque = TEST_RESULT_POS_MAX_TORQUE_AZ;
                        break;

                    case EL:
                        next_section = AXIS_RANGE_TEST_SECTION_EL_RETURN_HOME;
                        result_id_torque = TEST_RESULT_POS_MAX_TORQUE_EL;
                        break;

                    case ROLL:
                        next_section = AXIS_RANGE_TEST_SECTION_RL_RETURN_HOME;
                        result_id_torque = TEST_RESULT_POS_MAX_TORQUE_RL;
                        break;
                }
                test_parms->motor_torque_max_pos[test_parms->test_axis] = GetMaxTorqueCmd(cb_parms, test_parms->test_axis);
                CANSendTestResult(result_id_torque, (float)test_parms->motor_torque_max_pos[test_parms->test_axis]);

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

