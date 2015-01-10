/*
 * HWSpecific.c
 *
 *  Created on: Jan 10, 2015
 *      Author: abamberger
 */

#include "HWSpecific.h"

int AxisHomePositions[GIMBAL_CNT][AXIS_CNT] = {
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

float AxisCalibrationSlopes[GIMBAL_CNT][AXIS_CNT] = {
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

float AxisCalibrationIntercepts[GIMBAL_CNT][AXIS_CNT] = {
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

float AxisTorqueLoopKp[AXIS_CNT] = {
    1.25,   // EL
    0.8,    // AZ
    0.8     // ROLL
};

float AxisTorqueLoopKi[AXIS_CNT] = {
    0.75,   // EL
    0.75,   // AZ
    0.75    // ROLL
};

float AxisTorqueLoopKd[AXIS_CNT] = {
    1.0,    // EL
    0.0,    // AZ
    0.0     // ROLL
};
