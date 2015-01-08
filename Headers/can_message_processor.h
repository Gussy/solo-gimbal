/*
 * can_message_processor.h
 *
 *  Created on: Jan 7, 2015
 *      Author: abamberger
 */

#ifndef CAN_MESSAGE_PROCESSOR_H_
#define CAN_MESSAGE_PROCESSOR_H_

#include "PM_Sensorless.h"
#include "motor_drive_state_machine.h"

void Process_CAN_Messages(AxisParms* axis_parms, MotorDriveParms* md_parms, ControlBoardParms* cb_parms, ParamSet* param_set);

#endif /* CAN_MESSAGE_PROCESSOR_H_ */
