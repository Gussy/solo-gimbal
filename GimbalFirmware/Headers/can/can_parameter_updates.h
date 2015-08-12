#ifndef CAN_PARAMETER_UPDATES_H_
#define CAN_PARAMETER_UPDATES_H_

#include "PM_Sensorless.h"

void init_param_set(ParamSet *param_set);
void ProcessParamUpdates(ParamSet* param_set, ControlBoardParms* cb_parms, DebugData* debug_data);

#endif /* CAN_PARAMETER_UPDATES_H_ */
