/*
 * factory_tests.c
 *
 *  Created on: Mar 2, 2015
 *      Author: abamberger
 */

#include "tests/factory_tests.h"
#include "motor/motor_drive_state_machine.h"
#include "PM_Sensorless.h"

int RunFactoryTestsIteration(FactoryTestsParms* test_parms, MotorDriveParms* md_parms, ControlBoardParms* cb_parms, AxisParms* axis_parms, EncoderParms* encoder_parms)
{
    switch (test_parms->test_type) {
        case TEST_AXIS_RANGE_LIMITS:
            if (RunTestAxisRangeLimitsIteration(test_parms->axis_range_limits_parms, cb_parms) != 0) {
                // A non-zero return means the test is finished (whether for an error or because it completed successfully)
                // For now, exit out of factory tests mode (may want to sequence tests in the future)
                return 1;
            } else {
                return 0;
            }
        default:
        	return 0;
    }
}
