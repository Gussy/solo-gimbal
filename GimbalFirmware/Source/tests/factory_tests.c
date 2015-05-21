/*
 * factory_tests.c
 *
 *  Created on: Mar 2, 2015
 *      Author: abamberger
 */

#include "tests/factory_tests.h"
#include "motor/motor_drive_state_machine.h"
#include "PM_Sensorless.h"
#include "hardware/led.h"
#include "can/cand.h"
#include "can/cb.h"

static int RunTestGyroHealth(TestGyroHealthParms* test_parms, ControlBoardParms* cb_parms);


int RunFactoryTestsIteration(FactoryTestsParms* test_parms, MotorDriveParms* md_parms, ControlBoardParms* cb_parms, AxisParms* axis_parms, EncoderParms* encoder_parms)
{
    int  tests_complete = 0;

    switch (test_parms->test_type) {
        case TEST_AXIS_RANGE_LIMITS:
            if (RunTestAxisRangeLimitsIteration(test_parms->axis_range_limits_parms, cb_parms) != 0) {
                // A non-zero return means the test is finished (whether for an error or because it completed successfully)
                // For now, exit out of factory tests mode (may want to sequence tests in the future)
                tests_complete = 1;
            }
            break;

        case TEST_GYRO_HEALTH:
            if (RunTestGyroHealth(test_parms->gyro_health_limits_parms, cb_parms) != 0) {
                tests_complete = 1;
            }
            break;
    }
    return(tests_complete);
}


//#define DEBUG
static int RunTestGyroHealth(TestGyroHealthParms* test_parms, ControlBoardParms* cb_parms)
{
    int      test_complete = 0;
    LED_RGBA led_green = {0, 0xff, 0, 0xff};
    LED_RGBA led_red = {0xff, 0, 0, 0xff};
    #ifdef DEBUG
    static int16  max_gyro_short_term[AXIS_CNT] = {0, 0, 0};
    static int16  min_gyro_short_term[AXIS_CNT] = {0, 0, 0};
    static int16  max_gyro_long_term[AXIS_CNT]  = {0, 0, 0};
    static int16  min_gyro_long_term[AXIS_CNT]  = {0, 0, 0};
    static Uint32 debug_timer = 0;
    static int16  cycle = 0;
    //Uint8 debug_data[3];
    #endif

    switch (test_parms->test_state) {
        case GYRO_HEALTH_STATE_INIT:
            cb_parms->control_loop_type = POSITION_MODE;
            cb_parms->angle_targets[EL] = 0;
            cb_parms->angle_targets[AZ] = 0;
            cb_parms->angle_targets[ROLL] = 0;

            test_parms->test_state = GYRO_HEALTH_STATE_SETTLE_AT_HOME;
            test_parms->settle_counter = 0;
            break;

        case GYRO_HEALTH_STATE_SETTLE_AT_HOME:
            if (test_parms->settle_counter++ > 100000) { // 10 sec
                test_parms->gyro_status = GYROS_NEVER_FAILED;
                led_set_mode(LED_MODE_BLINK_FOREVER, led_green, 0);
                test_parms->test_state = GYRO_HEALTH_STATE_MONITORING;
            }
            break;

        case GYRO_HEALTH_STATE_MONITORING:
            if ((cb_parms->corrected_gyro_readings[EL] > test_parms->gyro_limit_min[EL]) &&
                (cb_parms->corrected_gyro_readings[EL] < test_parms->gyro_limit_max[EL]) &&
                (cb_parms->corrected_gyro_readings[AZ] > test_parms->gyro_limit_min[AZ]) &&
                (cb_parms->corrected_gyro_readings[AZ] < test_parms->gyro_limit_max[AZ]) &&
                (cb_parms->corrected_gyro_readings[ROLL] > test_parms->gyro_limit_min[ROLL]) &&
                (cb_parms->corrected_gyro_readings[ROLL] < test_parms->gyro_limit_max[ROLL])) {
                // gyros currently within limits
                if (test_parms->gyro_status == GYRO_CURRENTLY_FAILED) {
                    test_parms->gyro_status = GYRO_FAILED_PREVIOUSLY;
                    led_set_alternate_color(led_green);
                    led_set_mode(LED_MODE_BLINK_ALTERNATE_COLOR, led_red, 0);
                }
            }
            else {
                if (test_parms->gyro_status != GYRO_CURRENTLY_FAILED) {
                    test_parms->gyro_status = GYRO_CURRENTLY_FAILED;
                    led_set_mode(LED_MODE_SOLID, led_red, 0);
                }
            }
            #ifdef DEBUG
            if (cb_parms->corrected_gyro_readings[EL] < min_gyro_short_term[EL]) {
                min_gyro_short_term[EL] = cb_parms->corrected_gyro_readings[EL];
            }
            if (cb_parms->corrected_gyro_readings[EL] > max_gyro_short_term[EL]) {
                max_gyro_short_term[EL] = cb_parms->corrected_gyro_readings[EL];
            }
            if (cb_parms->corrected_gyro_readings[AZ] < min_gyro_short_term[AZ]) {
                min_gyro_short_term[AZ] = cb_parms->corrected_gyro_readings[AZ];
            }
            if (cb_parms->corrected_gyro_readings[AZ] > max_gyro_short_term[AZ]) {
                max_gyro_short_term[AZ] = cb_parms->corrected_gyro_readings[AZ];
            }
            if (cb_parms->corrected_gyro_readings[ROLL] < min_gyro_short_term[ROLL]) {
                min_gyro_short_term[ROLL] = cb_parms->corrected_gyro_readings[ROLL];
            }
            if (cb_parms->corrected_gyro_readings[ROLL] > max_gyro_short_term[ROLL]) {
                 max_gyro_short_term[ROLL] = cb_parms->corrected_gyro_readings[ROLL];
            }
            if (cb_parms->corrected_gyro_readings[EL] < min_gyro_long_term[EL]) {
                min_gyro_long_term[EL] = cb_parms->corrected_gyro_readings[EL];
            }
            if (cb_parms->corrected_gyro_readings[EL] > max_gyro_long_term[EL]) {
                max_gyro_long_term[EL] = cb_parms->corrected_gyro_readings[EL];
            }
            if (cb_parms->corrected_gyro_readings[AZ] < min_gyro_long_term[AZ]) {
                min_gyro_long_term[AZ] = cb_parms->corrected_gyro_readings[AZ];
            }
            if (cb_parms->corrected_gyro_readings[AZ] > max_gyro_long_term[AZ]) {
                max_gyro_long_term[AZ] = cb_parms->corrected_gyro_readings[AZ];
            }
            if (cb_parms->corrected_gyro_readings[ROLL] < min_gyro_long_term[ROLL]) {
                min_gyro_long_term[ROLL] = cb_parms->corrected_gyro_readings[ROLL];
            }
            if (cb_parms->corrected_gyro_readings[ROLL] > max_gyro_long_term[ROLL]) {
                 max_gyro_long_term[ROLL] = cb_parms->corrected_gyro_readings[ROLL];
            }
            if (debug_timer++ >= 10000) {
                debug_timer = 0;
                switch (cycle) {
                    case 0:
                        CANSendTestResult(TEST_RESULT_NEG_RANGE_EL, (float)min_gyro_short_term[EL]);
                        min_gyro_short_term[EL] = 0;
                        cycle++;
                        break;
                    case 1:
                        CANSendTestResult(TEST_RESULT_NEG_MAX_TORQUE_EL, (float)min_gyro_long_term[EL]);
                        cycle++;
                        break;
                    case 2:
                        CANSendTestResult(TEST_RESULT_POS_RANGE_EL, (float)max_gyro_short_term[EL]);
                        max_gyro_short_term[EL] = 0;
                        cycle++;
                        break;
                    case 3:
                        CANSendTestResult(TEST_RESULT_POS_MAX_TORQUE_EL, (float)max_gyro_long_term[EL]);
                        cycle++;
                        break;
                    case 4:
                        CANSendTestResult(TEST_RESULT_NEG_RANGE_RL, (float)min_gyro_short_term[ROLL]);
                        min_gyro_short_term[ROLL] = 0;
                        cycle++;
                        break;
                    case 5:
                        CANSendTestResult(TEST_RESULT_NEG_MAX_TORQUE_RL, (float)min_gyro_long_term[ROLL]);
                        cycle++;
                        break;
                    case 6:
                        CANSendTestResult(TEST_RESULT_POS_RANGE_RL, (float)max_gyro_short_term[ROLL]);
                        max_gyro_short_term[ROLL] = 0;
                        cycle++;
                        break;
                    case 7:
                        CANSendTestResult(TEST_RESULT_POS_MAX_TORQUE_RL, (float)max_gyro_long_term[ROLL]);
                        cycle++;
                        break;
                    case 8:
                        CANSendTestResult(TEST_RESULT_NEG_RANGE_AZ, (float)min_gyro_short_term[AZ]);
                        min_gyro_short_term[AZ] = 0;
                        cycle++;
                        break;
                    case 9:
                        CANSendTestResult(TEST_RESULT_NEG_MAX_TORQUE_AZ, (float)min_gyro_long_term[AZ]);
                        cycle++;
                        break;
                    case 10:
                        CANSendTestResult(TEST_RESULT_POS_RANGE_AZ, (float)max_gyro_short_term[AZ]);
                        max_gyro_short_term[AZ] = 0;
                        cycle++;
                        break;
                    case 11:
                        CANSendTestResult(TEST_RESULT_POS_MAX_TORQUE_AZ, (float)max_gyro_long_term[AZ]);
                        cycle = 0;
                        break;
                    default:
                        cycle = 0;
                        break;
                }
            }
            #endif    // DEBUG
            break;

        case GYRO_HEALTH_STATE_TEST_TERMINATED:
            // TODO don't have a way to get here
            cb_parms->control_loop_type = RATE_MODE;

            test_parms->test_state = GYRO_HEALTH_STATE_INIT;

            led_set_mode(LED_MODE_OFF, led_green, 0);
            test_complete = 1;
            break;
    }
    return(test_complete);
}


