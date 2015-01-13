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

void RunRateLoops(ControlBoardParms* cb_parms, ParamSet* param_set, RunningAvgFilterParms* pos_loop_stage_1, RunningAvgFilterParms* pos_loop_stage_2, BalanceProcedureParms* bal_proc_parms);

#endif /* RATE_LOOPS_H_ */
