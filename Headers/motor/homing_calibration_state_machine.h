/*
 * homing_calibration_state_machine.h
 *
 *  Created on: Jan 7, 2015
 *      Author: abamberger
 */

#ifndef HOMING_CALIBRATION_STATE_MACHINE_H_
#define HOMING_CALIBRATION_STATE_MACHINE_H_

#include "PM_Sensorless.h"
#include "motor_drive_state_machine.h"

#define HOMING_ID_RAMP_SPEED 5
#define HOMING_ANGLE_RAMP_SPEED 2
#define HOMING_MAX_STOP_SEARCH_STEPS 500
#define HOMING_STOP_SEARCH_STEP_SIZE 0.01
#define HOMING_NUM_STALLED_COUNTS 10
#define POT_CALIBRATION_SETTLING_TIME_MS 1000

typedef enum {
    HOMING_STATE_INIT,
    HOMING_STATE_TAKE_CALIBRATION_POINT_1,
    HOMING_STATE_TAKE_CALIBRATION_POINT_2,
    HOMING_STATE_RAMP_CURRENT_UP_1,
    HOMING_STATE_RAMP_CURRENT_DOWN_1,
    HOMING_STATE_RAMP_CURRENT_UP_2,
    HOMING_STATE_RAMP_CURRENT_DOWN_2,
    HOMING_STATE_MOVE_ID,
    HOMING_STATE_FIND_STOP_INIT,
    HOMING_STATE_FIND_STOP
} HomingState;

typedef enum {
    AXIS_STOP_1,
    AXIS_STOP_2
} AxisStop;

typedef enum {
    HOMING_DIRECTION_FORWARD,
    HOMING_DIRECTION_BACKWARD
} HomingDirection;

typedef struct {
    HomingState homing_state;
    HomingState next_homing_state;
    AxisStop finding_stop;
    HomingDirection direction;
    Uint16 find_stop_step;
    Uint16 stalled_count;
    Uint16 mech_stop_1;
    Uint16 mech_stop_2;
    Uint32 settling_timer;
    float last_mech_theta;
    float EPSILON;
    RMPCNTL ramp_cntl;
} HomingCalibrationParms;

void HomingCalibrationStateMachine(MotorDriveParms* md_parms, EncoderParms* encoder_parms, ControlBoardParms* cb_parms, AxisParms* axis_parms, HomingCalibrationParms* hc_parms);

#endif /* HOMING_CALIBRATION_STATE_MACHINE_H_ */
