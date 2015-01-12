/*
 * init_axis_parms_state_machine.h
 *
 *  Created on: Jan 9, 2015
 *      Author: abamberger
 */

#ifndef INIT_AXIS_PARMS_STATE_MACHINE_H_
#define INIT_AXIS_PARMS_STATE_MACHINE_H_

#include "PeripheralHeaderIncludes.h"
#include "HWSpecific.h"

typedef union {
    Uint32 uint32_val;
    float float_val;
} IntOrFloat;

typedef enum {
    LOAD_AXIS_PARMS_STATE_LOAD_TORQUE_KP,
    LOAD_AXIS_PARMS_STATE_LOAD_TORQUE_KI,
    LOAD_AXIS_PARMS_STATE_LOAD_TORQUE_KD,
    LOAD_AXIS_PARMS_STATE_LOAD_COMMUTATION_CALIBRATION_SLOPE,
    LOAD_AXIS_PARMS_STATE_LOAD_COMMUTATION_CALIBRATION_INTERCEPT,
    LOAD_AXIS_PARMS_STATE_LOAD_COMMUTATION_CALIBRATION_HOME_OFFSET,
    LOAD_AXIS_PARMS_STATE_LOAD_RATE_P,
    LOAD_AXIS_PARMS_STATE_LOAD_RATE_I,
    LOAD_AXIS_PARMS_STATE_LOAD_RATE_D,
    LOAD_AXIS_PARMS_STATE_LOAD_RATE_WINDUP
} InitAxisParmsState;

typedef struct {
    InitAxisParmsState load_axis_parms_state;
    GimbalAxis current_load_axis;
    Uint16 axis_parms_load_complete;
} LoadAxisParmsStateInfo;

void LoadAxisParmsStateMachine(LoadAxisParmsStateInfo* init_parms_state_info);

#endif /* INIT_AXIS_PARMS_STATE_MACHINE_H_ */
