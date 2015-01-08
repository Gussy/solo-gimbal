/*
 * commutation_calibration_state_machine.h
 *
 *  Created on: Jan 7, 2015
 *      Author: abamberger
 */

#include "PM_Sensorless.h"
#include "motor_drive_state_machine.h"

#ifndef COMMUTATION_CALIBRATION_STATE_MACHINE_H_
#define COMMUTATION_CALIBRATION_STATE_MACHINE_H_

#define COMMUTATION_CALIBRATION_NUM_ELECTRICAL_CYCLES 4
#define COMMUTATION_CALIBRATION_NUM_ITERATIONS 1
#define COMMUTATION_CALIBRATION_ELECTRICAL_CYCLE_SUBDIVISIONS 16
#define COMMUTATION_CALIBRATION_RAMP_SPEED 5
#define COMMUTATION_CALIBRATION_SETTLING_TIME_MS 500

typedef enum {
    COMMUTATION_CALIBRATION_STATE_INIT,
    COMMUTATION_CALIBRATION_STATE_COMMUTATE,
    COMMUTATION_CALIBRATION_STATE_LOCK_ROTOR,
    COMMUTATION_CALIBRATION_STATE_RAMP_ID,
    COMMUTATION_CALIBRATION_STATE_WAIT_FOR_SETTLE,
    COMMUTATION_CALIBRATION_STATE_MOVE_ID
} CommutationCalibrationState;

typedef struct {
    CommutationCalibrationState calibration_state;
    Uint16 current_iteration;
    Uint16 current_elec_cycle;
    Uint16 current_elec_sub_cycle;
    Uint16 current_dir;
    Uint32 settling_timer;
    RMPCNTL ramp_cntl;
    float calibration_data[COMMUTATION_CALIBRATION_NUM_ITERATIONS * COMMUTATION_CALIBRATION_NUM_ELECTRICAL_CYCLES * COMMUTATION_CALIBRATION_ELECTRICAL_CYCLE_SUBDIVISIONS * 2];
} CommutationCalibrationParms;

void CommutationCalibrationStateMachine(MotorDriveParms* md_parms, EncoderParms* encoder_parms, CommutationCalibrationParms* cc_parms);

#endif /* COMMUTATION_CALIBRATION_STATE_MACHINE_H_ */
