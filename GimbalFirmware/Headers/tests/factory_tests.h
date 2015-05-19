/*
 * factory_tests.h
 *
 *  Created on: Mar 2, 2015
 *      Author: abamberger
 */

#ifndef FACTORY_TESTS_H_
#define FACTORY_TESTS_H_

#include "tests/test_axis_range_limits.h"
#include "motor/motor_drive_state_machine.h"
#include "PM_Sensorless.h"

typedef enum {
    TEST_AXIS_RANGE_LIMITS
} TestType;

typedef enum {
    TEST_RESULT_NEG_RANGE_AZ = 0,
    TEST_RESULT_POS_RANGE_AZ,
    TEST_RESULT_NEG_MAX_TORQUE_AZ,
    TEST_RESULT_POS_MAX_TORQUE_AZ,
    TEST_RESULT_NEG_RANGE_EL,
    TEST_RESULT_POS_RANGE_EL,
    TEST_RESULT_NEG_MAX_TORQUE_EL,
    TEST_RESULT_POS_MAX_TORQUE_EL,
    TEST_RESULT_NEG_RANGE_RL,
    TEST_RESULT_POS_RANGE_RL,
    TEST_RESULT_NEG_MAX_TORQUE_RL,
    TEST_RESULT_POS_MAX_TORQUE_RL
} TestResult;

typedef struct {
    TestType test_type;
    TestAxisRangeLimitsParms* axis_range_limits_parms;
} FactoryTestsParms;

int RunFactoryTestsIteration(FactoryTestsParms* test_parms, MotorDriveParms* md_parms, ControlBoardParms* cb_parms, AxisParms* axis_parms, EncoderParms* encoder_parms);

#endif /* FACTORY_TESTS_H_ */
