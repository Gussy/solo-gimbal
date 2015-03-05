/*
 * test_axis_range_limits.h
 *
 *  Created on: Mar 2, 2015
 *      Author: abamberger
 */

#ifndef TEST_AXIS_RANGE_LIMITS_H_
#define TEST_AXIS_RANGE_LIMITS_H_

#include "hardware/HWSpecific.h"
#include "PM_Sensorless.h"
#include "PeripheralHeaderIncludes.h"

#define AXIS_MOVE_TIME_MS 5000
#define AXIS_SETTLE_TIME_MS 7500
#define ALLOWED_POSITION_ERROR_COUNTS 135.0

typedef enum {
    RANGE_LIMITS_STATE_INIT,
    RANGE_LIMITS_STATE_SETTLE_AT_HOME_1,
    RANGE_LIMITS_STATE_MOVE_TO_NEGATIVE_LIMIT,
    RANGE_LIMITS_STATE_SETTLE_AT_NEGATIVE_LIMIT,
    RANGE_LIMITS_STATE_CHECK_NEGATIVE_LIMIT,
    RANGE_LIMITS_STATE_MOVE_TO_HOME_1,
    RANGE_LIMITS_STATE_MOVE_TO_POSITIVE_LIMIT,
    RANGE_LIMITS_STATE_SETTLE_AT_POSITIVE_LIMIT,
    RANGE_LIMITS_STATE_CHECK_POSITIVE_LIMIT,
    RANGE_LIMITS_STATE_MOVE_TO_HOME_2,
    RANGE_LIMITS_STATE_SETTLE_AT_HOME_2,
    RANGE_LIMITS_STATE_CHECK_HOME_POSITION
} TestAxisRangeLimitsState;

typedef enum {
    AXIS_RANGE_TEST_STATUS_IN_PROGRESS = 0,
    AXIS_RANGE_TEST_STATUS_SUCCEEDED,
    AXIS_RANGE_TEST_STATUS_FAILED_NEGATIVE,
    AXIS_RANGE_TEST_STATUS_FAILED_POSITIVE,
    AXIS_RANGE_TEST_STATUS_FAILED_HOME
} AxisRangeLimitsTestStatus;

typedef enum {
    AXIS_RANGE_TEST_SECTION_EL_CHECK_NEG,
    AXIS_RANGE_TEST_SECTION_EL_CHECK_POS,
    AXIS_RANGE_TEST_SECTION_EL_RETURN_HOME,
    AXIS_RANGE_TEST_SECTION_RL_CHECK_NEG,
    AXIS_RANGE_TEST_SECTION_RL_CHECK_POS,
    AXIS_RANGE_TEST_SECTION_RL_RETURN_HOME,
    AXIS_RANGE_TEST_SECTION_AZ_CHECK_NEG,
    AXIS_RANGE_TEST_SECTION_AZ_CHECK_POS,
    AXIS_RANGE_TEST_SECTION_AZ_RETURN_HOME
} AxisRangeLimitsTestSection;

typedef struct {
    GimbalAxis test_axis;
    TestAxisRangeLimitsState test_state;
    AxisRangeLimitsTestSection current_section;
    float axis_range_min[AXIS_CNT];
    float axis_range_max[AXIS_CNT];
    float current_axis_position;
    float position_step;
    Uint32 status_output_decimation_count;
    Uint32 settle_counter;
} TestAxisRangeLimitsParms;

int RunTestAxisRangeLimitsIteration(TestAxisRangeLimitsParms* test_parms, ControlBoardParms* cb_parms);

#endif /* TEST_AXIS_RANGE_LIMITS_H_ */
