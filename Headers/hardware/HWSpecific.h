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

#define HW_REV 2 // 1 is old hardware, 2 is new hardware

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
#define COUNTS_PER_DEGREE ((float)ENCODER_COUNTS_PER_REV / 360.0)

#define DEGREES_TO_COUNTS(x) (x * COUNTS_PER_DEGREE)

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

extern float AxisHomePositions[AXIS_CNT];
extern float AxisCalibrationSlopes[AXIS_CNT];
extern float AxisCalibrationIntercepts[AXIS_CNT];
extern float AxisTorqueLoopKp[AXIS_CNT];
extern float AxisTorqueLoopKi[AXIS_CNT];
extern float AxisTorqueLoopKd[AXIS_CNT];

#endif /* HWSPECIFIC_H_ */
