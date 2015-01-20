/*
 * rate_loops.h
 *
 *  Created on: Jan 7, 2015
 *      Author: abamberger
 */

#ifndef RATE_LOOPS_H_
#define RATE_LOOPS_H_

#include "PeripheralHeaderIncludes.h"
#include "PM_Sensorless.h"
#include "running_average_filter.h"



#define AZ_KEEP_OFF_STOP_START_COUNT_NEGATIVE ((float)ENCODER_COUNTS_PER_REV - DEGREES_TO_COUNTS(5.0)) // -5 degrees
#define AZ_KEEP_OFF_STOP_END_COUNT_NEGATIVE ((float)ENCODER_COUNTS_PER_REV - DEGREES_TO_COUNTS(20.0)) // -20 degrees
#define AZ_KEEP_OFF_STOP_START_COUNT_POSITIVE DEGREES_TO_COUNTS(10.0) // 10 degrees
#define AZ_KEEP_OFF_STOP_END_COUNT_POSITIVE DEGREES_TO_COUNTS(25.0) // 25 degrees
#define AZ_KEEP_OFF_STOP_SPAN_NEGATIVE (AZ_KEEP_OFF_STOP_START_COUNT_NEGATIVE - AZ_KEEP_OFF_STOP_END_COUNT_NEGATIVE)
#define AZ_KEEP_OFF_STOP_SPAN_POSITIVE (AZ_KEEP_OFF_STOP_END_COUNT_POSITIVE - AZ_KEEP_OFF_STOP_START_COUNT_POSITIVE)
#define AZ_KEEP_OFF_STOP_MAX_TORQUE 32767.0

void RunRateLoops(ControlBoardParms* cb_parms, ParamSet* param_set, RunningAvgFilterParms* pos_loop_stage_1, RunningAvgFilterParms* pos_loop_stage_2, BalanceProcedureParms* bal_proc_parms);

#endif /* RATE_LOOPS_H_ */
