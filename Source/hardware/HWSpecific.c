/*
 * HWSpecific.c
 *
 *  Created on: Jan 10, 2015
 *      Author: abamberger
 */

#include "hardware/HWSpecific.h"

int AxisHomePositions[GIMBAL_CNT][AXIS_CNT] = {
    // Gimbal G1
    {
        5372,   // EL
        5595,   // AZ
        4806    // ROLL
    },
    /*
    // Gimbal G2
    {
        5135,   // EL
        4696,   // AZ
        4319,   // ROLL
    },
    */
    // TODO: Temp for testing
    { 0, 0, 0},
    // Gimbal G3
    {
        0,      // EL
        0,      // AZ
        0       // ROLL
    },
    // Gimbal G4
    {
        4950,      // EL
        0,      // AZ
        4995       // ROLL
    },
    // Gimbal G5
    {
        5120,      // EL
        4898,      // AZ
        4944       // ROLL
    }
};

float AxisCalibrationSlopes[GIMBAL_CNT][AXIS_CNT] = {
    // Gimbal G1
    {
        0.1245,     // EL
        0.1231,     // AZ
        0.1324      // ROLL
    },
    /*
    // Gimbal G2
    {
        0.127,      // EL
        0.1267,     // AZ
        0.1274,     // ROLL
    },
    */
    //TODO: Temp for testing
    {0.0, 0.0, 0.0},
    // Gimbal G3
    {
        0.0,        // EL
        0.1352,     // AZ
        0.0,        // ROLL
    },
    // Gimbal G4
    {
        0.1279,        // EL
        0.0,     // AZ
        0.139         // ROLL
    },
    // Gimbal G5
    {
        0.126,        // EL
        0.1247,       // AZ
        0.1245        // ROLL
    }
};

float AxisCalibrationIntercepts[GIMBAL_CNT][AXIS_CNT] = {
    // Gimbal G1
    {
        0.3408,  // EL
        0.3659,  // AZ
        0.3635   // ROLL
    },
    /*
    // Gimbal G2
    {
        0.3801,     // EL
        0.4128,     // AZ
        0.43,       // ROLL
    },
    */
    //TODO: For testing
    {0.0, 0.0, 0.0},
    // Gimbal G3
    {
        0.0,        // EL
        0.107,      // AZ
        0.0,        // ROLL
    },
    // Gimbal G4
    {
        0.4288,        // EL
        0.0,      // AZ
        0.3122        // ROLL
    },
    // Gimbal G5
    {
        0.4536,      // EL
        0.3718,      // AZ
        0.4079       // ROLL
    }
};

float AxisTorqueLoopKp[AXIS_CNT] = {
    //TODO: For testing
    0.0, 0.0, 0.0
   /*
    1.25,   // EL
    0.8,    // AZ
    0.8     // ROLL
    */
};

float AxisTorqueLoopKi[AXIS_CNT] = {
    //TODO: For testing
    0.0, 0.0, 0.0
    /*
    0.75,   // EL
    0.75,   // AZ
    0.75    // ROLL
    */
};

float AxisTorqueLoopKd[AXIS_CNT] = {
    //TODO: For testing
    0.0, 0.0, 0.0
    /*
    1.0,    // EL
    0.0,    // AZ
    0.0     // ROLL
    */
};
