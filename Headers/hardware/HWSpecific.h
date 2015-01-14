//Parameters for Board2 and first test Knuckle
//#define HWSpecificCalibratedAngle -133
//#define HWSpecificCal_offset_A 0.49315
//#define HWSpecificCal_offset_B 0.49470
/*
 * HWSpecific.h
 *
 *  Created on: Mar 18, 2014
 *      Author: rmorrill
 */

#ifndef HWSPECIFIC_H_
#define HWSPECIFIC_H_

//#include "cand_BitFields.h"

#define HW_REV 1 // 1 is old hardware, 2 is new hardware

enum gimbal_targets {
	GIMBAL_G1 = 0,  // Gimbal SN 102
	GIMBAL_G2,      // Gimbal SN 103
	GIMBAL_G3,      // SATB
	GIMBAL_G4,      // Rev. 2 SN P001
	GIMBAL_G5,      // Rev. 2 SN P002
	GIMBAL_CNT
};

typedef enum {
    EL = 0,
    AZ = 1,
    ROLL = 2,
    AXIS_CNT
} GimbalAxis;

typedef enum {
    X_AXIS = 0,
    Y_AXIS,
    Z_AXIS
} GyroAxis;

#define ENCODER_COUNTS_PER_REV 10000

#define GIMBAL_TARGET GIMBAL_G5

// Map gyro axes to gimbal axes
static const GimbalAxis GyroAxisMap[AXIS_CNT] = {
        AZ,
        EL,
        ROLL
};

#if (HW_REV == 1)
static const int GyroSignMap[AXIS_CNT] = {
        -1, // EL
        -1, // AZ
        -1  // ROLL
};

static const int TorqueSignMap[AXIS_CNT] = {
        -1, // EL
        -1, // AZ
        -1  // ROLL
};

static const int EncoderSignMap[AXIS_CNT] = {
        -1, // EL
        -1, // AZ
        -1  // ROLL
};
#elif (HW_REV == 2)
static const int GyroSignMap[AXIS_CNT] = {
        1, // EL
        1, // AZ
        -1  // ROLL
};

static const int TorqueSignMap[AXIS_CNT] = {
        1, // EL
        -1, // AZ
        -1  // ROLL
};

static const int EncoderSignMap[AXIS_CNT] = {
        1, // EL
        -1, // AZ
        -1  // ROLL
};
#endif

extern int AxisHomePositions[GIMBAL_CNT][AXIS_CNT];
extern float AxisCalibrationSlopes[GIMBAL_CNT][AXIS_CNT];
extern float AxisCalibrationIntercepts[GIMBAL_CNT][AXIS_CNT];
extern float AxisTorqueLoopKp[AXIS_CNT];
extern float AxisTorqueLoopKi[AXIS_CNT];
extern float AxisTorqueLoopKd[AXIS_CNT];

#endif /* HWSPECIFIC_H_ */
