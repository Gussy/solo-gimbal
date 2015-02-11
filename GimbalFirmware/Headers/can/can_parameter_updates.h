/*
 * can_parameter_updates.h
 *
 *  Created on: Feb 11, 2015
 *      Author: abamberger
 */

#ifndef CAN_PARAMETER_UPDATES_H_
#define CAN_PARAMETER_UPDATES_H_

#include "PM_Sensorless.h"

void ProcessParamUpdates(ParamSet* param_set, ControlBoardParms* cb_parms, DebugData* debug_data, BalanceProcedureParms* balance_proc_parms);

#endif /* CAN_PARAMETER_UPDATES_H_ */
