/*
 * PID.c
 *
 *  Created on: Sep 12, 2014
 *      Author: abamberger
 */

#include "control/PID.h"

#include <stdlib.h>
PIDData_Float rate_pid_loop_float[AXIS_CNT] = {
    // These get loaded over CAN at boot, so they are initialized to zero
    // (Except overall gain and d-term alpha, which are hardcoded)
    { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.1, 0.0 },
    { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.1, 0.0 },
    { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.1, 0.0 }
};

float UpdatePID_Float(GimbalAxis axis, float error)
{
    float pTerm, dTerm, iTerm, result;
    float deltaError;
    float output;

    PIDData_Float* PIDInfo;

    // Look up the proper tuning parameters based on the requested PID data type
    PIDInfo = &rate_pid_loop_float[axis];

    // Calcuate proportional gain, gain * current error
    pTerm = PIDInfo->gainP * error;

    // Calculate integral gain, gain * accumulation of historical error
    PIDInfo->integralCumulative += error;

    // Limit accumulated integral error to windup limits
    if (PIDInfo->integralCumulative > PIDInfo->integralMax) {
        PIDInfo->integralCumulative = PIDInfo->integralMax;
    } else if (PIDInfo->integralCumulative < PIDInfo->integralMin) {
        PIDInfo->integralCumulative = PIDInfo->integralMin;
    }

    iTerm = PIDInfo->gainI * PIDInfo->integralCumulative;

    // Calculate derivative gain, gain * difference in error
    if(PIDInfo->gainD) {
        deltaError = error - PIDInfo->errorPrevious;

        deltaError = (deltaError * PIDInfo->dTermAlpha) + ((1.0 - PIDInfo->dTermAlpha) * PIDInfo->errorPrevious);

        dTerm = deltaError * PIDInfo->gainD;
    } else  {
        dTerm = 0;
    }
    PIDInfo->errorPrevious = deltaError;

    // Calculate result, sum of three individual gain terms
    result = (pTerm + iTerm + dTerm);

    // Limit output
    if (result > OUTPUT_LIMIT_UPPER_FLOAT) {
        output = OUTPUT_LIMIT_UPPER_FLOAT;
    } else if (result < OUTPUT_LIMIT_LOWER_FLOAT) {
        output = OUTPUT_LIMIT_LOWER_FLOAT;
    } else {
        output = result;
    }

    return(output);
}
