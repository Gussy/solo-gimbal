#ifndef RATE_LOOPS_H_
#define RATE_LOOPS_H_

#include "PeripheralHeaderIncludes.h"
#include "PM_Sensorless.h"
#include "running_average_filter.h"

// The rate loops run at 1kHz, and we want to output telemetry at 100Hz, so we decimate by 10
#define TELEMETRY_DECIMATION_LIMIT 10

void RunRateLoops(ControlBoardParms* cb_parms, ParamSet* param_set);

#endif /* RATE_LOOPS_H_ */
