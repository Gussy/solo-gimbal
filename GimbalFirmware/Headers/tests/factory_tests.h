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

typedef struct {
    TestType test_type;
    TestAxisRangeLimitsParms* axis_range_limits_parms;
} FactoryTestsParms;

int RunFactoryTestsIteration(FactoryTestsParms* test_parms, MotorDriveParms* md_parms, ControlBoardParms* cb_parms, AxisParms* axis_parms, EncoderParms* encoder_parms);

#endif /* FACTORY_TESTS_H_ */
