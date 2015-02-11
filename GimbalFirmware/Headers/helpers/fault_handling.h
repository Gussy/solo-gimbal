/*
 * fault_handling.h
 *
 *  Created on: Feb 11, 2015
 *      Author: abamberger
 */

#ifndef FAULT_HANDLING_H_
#define FAULT_HANDLING_H_

#include "PM_Sensorless.h"
#include "motor/motor_drive_state_machine.h"

typedef enum {
    FAULT_TYPE_INFO,
    FAULT_TYPE_RECOVERABLE,
    FAULT_TYPE_UNRECOVERABLE
} FaultType;

void AxisFault(CAND_FaultCode fault_code, FaultType fault_type, ControlBoardParms* cb_parms, MotorDriveParms* md_parms, AxisParms* axis_parms);

#endif /* FAULT_HANDLING_H_ */
