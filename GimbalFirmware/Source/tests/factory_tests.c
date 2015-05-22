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


int ProcessFactoryTestArgumentsSuccessful(FactoryTestsParms* test_parms, uint8_t test_type, uint8_t test_argument)
{
    int  success = 0;

    test_parms->test_type = (TestType)test_type;
    switch (test_parms->test_type) {
        case TEST_AXIS_RANGE_LIMITS:
            // no arguments at this time
            success = 1;
            break;

        case TEST_GYRO_HEALTH:
            switch ((TestArgument)test_argument) {
                case TEST_ARG_START:
                    // note: if test is already running, this will restart it
                    test_parms->gyro_health_parms->test_state = GYRO_HEALTH_STATE_INIT;
                    success = 1;
                    break;

                case TEST_ARG_STOP:
                    test_parms->gyro_health_parms->test_state = GYRO_HEALTH_STATE_TEST_TERMINATED;
                    success = 1;
                    break;
            }
            break;
    }
    return(success);
}


void RunFactoryTestsMotorLoop(FactoryTestsParms* test_parms, MotorDriveParms* md_parms, ControlBoardParms* cb_parms, AxisParms* axis_parms, EncoderParms* encoder_parms)
{
    switch (test_parms->test_type) {
        case TEST_AXIS_RANGE_LIMITS:
            if (RunTestAxisRangeLimitsIteration(test_parms->axis_range_limits_parms, cb_parms) != 0) {
                // A non-zero return means the test is finished (whether for an error or because it completed successfully)
                // For now, exit out of factory tests mode (may want to sequence tests in the future)
                cb_parms->running_tests = FALSE;
                CANSendFactoryTestsComplete();
            }
            break;
    }
}


void RunFactoryTestsRateLoop(FactoryTestsParms* test_parms, MotorDriveParms* md_parms, ControlBoardParms* cb_parms, AxisParms* axis_parms, EncoderParms* encoder_parms)
{
    switch (test_parms->test_type) {
        case TEST_GYRO_HEALTH:
            if (RunTestGyroHealth(test_parms->gyro_health_parms, cb_parms) != 0) {
                cb_parms->running_tests = FALSE;
                CANSendFactoryTestsComplete();
            }
            break;
    }
}


//#define DEBUG
static int RunTestGyroHealth(TestGyroHealthParms* test_parms, ControlBoardParms* cb_parms)
{
    int      test_complete = 0;
    LED_RGBA led_green = {0, 0xff, 0, 0xff};
    LED_RGBA led_red = {0xff, 0, 0, 0xff};
    #ifdef DEBUG
    static int16  max_gyro[AXIS_CNT] = {0, 0, 0};
    static int16  min_gyro[AXIS_CNT] = {0, 0, 0};
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
            if (test_parms->settle_counter++ > 5000) { // 5 sec
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
            if (cb_parms->corrected_gyro_readings[EL] < min_gyro[EL]) {
                min_gyro[EL] = cb_parms->corrected_gyro_readings[EL];
            }
            if (cb_parms->corrected_gyro_readings[EL] > max_gyro[EL]) {
                max_gyro[EL] = cb_parms->corrected_gyro_readings[EL];
            }
            if (cb_parms->corrected_gyro_readings[AZ] < min_gyro[AZ]) {
                min_gyro[AZ] = cb_parms->corrected_gyro_readings[AZ];
            }
            if (cb_parms->corrected_gyro_readings[AZ] > max_gyro[AZ]) {
                max_gyro[AZ] = cb_parms->corrected_gyro_readings[AZ];
            }
            if (cb_parms->corrected_gyro_readings[ROLL] < min_gyro[ROLL]) {
                min_gyro[ROLL] = cb_parms->corrected_gyro_readings[ROLL];
            }
            if (cb_parms->corrected_gyro_readings[ROLL] > max_gyro[ROLL]) {
                 max_gyro[ROLL] = cb_parms->corrected_gyro_readings[ROLL];
            }
            #endif    // DEBUG
            break;

        case GYRO_HEALTH_STATE_TEST_TERMINATED:
            cb_parms->control_loop_type = RATE_MODE;

            test_parms->test_state = GYRO_HEALTH_STATE_INIT;

            led_set_mode(LED_MODE_OFF, led_green, 0);
            test_complete = 1;
            break;
    }
    return(test_complete);
}


