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

enum gimbal_targets {
	GIMBAL_G1 = 0,  // Gimbal SN 102
	GIMBAL_G2,      // Gimbal SN 103
	GIMBAL_G3,      // SATB
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

#define GIMBAL_TARGET GIMBAL_G2

// Map gyro axes to gimbal axes
static const GimbalAxis GyroAxisMap[AXIS_CNT] = {
        AZ,
        EL,
        ROLL
};

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

static const int AxisHomePositions[GIMBAL_CNT][AXIS_CNT] = {
    // Gimbal G1
    {
        5372,   // EL
        5595,   // AZ
        4806    // ROLL
    },
    // Gimbal G2
    {
        5135,   // EL
        4696,   // AZ
        4319,   // ROLL
    },
    // Gimbal G3
    {
        0,      // EL
        0,      // AZ
        0       // ROLL
    }
};

static float AxisCalibrationSlopes[GIMBAL_CNT][AXIS_CNT] = {
    // Gimbal G1
    {
        0.1245,     // EL
        0.1231,     // AZ
        0.1324      // ROLL
    },
    // Gimbal G2
    {
        0.127,      // EL
        0.1267,     // AZ
        0.1274,     // ROLL
    },
    // Gimbal G3
    {
        0.0,        // EL
        0.1352,     // AZ
        0.0,        // ROLL
    }
};

static float AxisCalibrationIntercepts[GIMBAL_CNT][AXIS_CNT] = {
    // Gimbal G1
    {
        0.3408,  // EL
        0.3659,  // AZ
        0.3635   // ROLL
    },
    // Gimbal G2
    {
        0.3801,     // EL
        0.4128,     // AZ
        0.43,       // ROLL
    },
    // Gimbal G3
    {
        0.0,        // EL
        0.107,      // AZ
        0.0,        // ROLL
    }
};

static float AxisTorqueLoopKp[AXIS_CNT] = {
	1.25,	// EL
	0.8,	// AZ
	0.8		// ROLL
};

static float AxisTorqueLoopKi[AXIS_CNT] = {
	0.75,	// EL
	0.75,	// AZ
	0.75	// ROLL
};

static float AxisTorqueLoopKd[AXIS_CNT] = {
	1.0,	// EL
	0.0,	// AZ
	0.0		// ROLL
};

#endif /* HWSPECIFIC_H_ */
