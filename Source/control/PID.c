/*
 * PID.c
 *
 *  Created on: Sep 12, 2014
 *      Author: abamberger
 */

#include "control/PID.h"

#include <stdlib.h>

PIDData rate_pid_loop[AXIS_CNT] = {
    // Proportional loop gain
    // Integral loop gain
    // Derivative loop gain
    // Maximum Positive Integral Windup
    // Maximum Negative Integral Windup
    // Total loop gain
    // Current cumulative integral error
    // Previous delta (derivative) error
    // Proportional divisor (number of shifts to the right)
    // Integral divisor (number of shifts to the right)
    // Derivative divisor (number of shifts to the right)
    // Overall Divisor
    // Historical error values (for D term calculation)
    // Starting read position (in error value array) for D calculation
    // Starting write position (in error value array) for D calculation
    { 125, 175,  0x0000, 1000, -1000, 1, 0, 0, 9, 8, 0, 0, {0}, 1, 0},
    { 125, 175,  0x0000, 2000, -2000, 1, 0, 0, 7, 8, 0, 0, {0}, 1, 0},
    { 175, 130,  0x0000, 2000, -2000, 1, 0, 0, 6, 5, 0, 0, {0}, 1, 0}
};

PIDData_Float rate_pid_loop_float[AXIS_CNT] = {
    // Old not-so-great tuning
    /*
    { 1.75, 0.25, 0.0, 2000.0, -2000.0, 1.0, 0.0, 0.1, 0.0 },
    { 5.0, 0.5, 1.0, 2000.0, -2000.0, 1.0, 0.0, 0.1, 0.0 },
    { 10.0, 0.25, 0.0, 2000.0, -2000.0, 1.0, 0.0, 0.1, 0.0 }
    */
    // New, sportier tuning
    /*
    { 2.5, 0.25, 0.0, 32767.0, -32768.0, 1.0, 0.0, 0.1, 0.0 },
    { 2.0, 0.5, 1.0, 32767.0, -32768.0, 1.0, 0.0, 0.1, 0.0 },
    { 5.0, 0.5, 0.0, 32767.0, -32768.0, 1.0, 0.0, 0.1, 0.0 }
    */
    // Zero tuning for testing param init over CAN
    { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.1, 0.0 },
    { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.1, 0.0 },
    { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.1, 0.0 }
};

PIDData_Float pos_pid_loop_float[AXIS_CNT] = {
    { 5.0, 0.0,  0.0, 2000.0, -2000.0, 1.0, 0.0, 0.0, 0.0 },
    { 5.0, 0.0, 0.0, 2000.0, -2000.0, 1.0, 0.0, 0.0, 0.0 },
    { 5.0, 0.0, 0.0, 2000.0, -2000.0, 1.0, 0.0, 0.0, 0.0 }
};


/**********************************************************************/
int16 UpdatePID(GimbalAxis axis, int16 error)
/**********************************************************************/
{
    int32 pTerm, dTerm, iTerm, result;
    int16 deltaError;
    int16 output;

    PIDData PIDInfo = rate_pid_loop[axis];

    // Calcuate proportional gain, gain * current error
    pTerm = (int32)(((int32)PIDInfo.gainP * error)) >> PIDInfo.divP;

    // Calculate integral gain, gain * accumulation of historical error
    PIDInfo.integralCumulative += error;

    // Limit accumulated integral error to windup limits
    if (PIDInfo.integralCumulative > PIDInfo.integralMax) {
        PIDInfo.integralCumulative = PIDInfo.integralMax;
    } else if (PIDInfo.integralCumulative < PIDInfo.integralMin) {
        PIDInfo.integralCumulative = PIDInfo.integralMin;
    }

    iTerm = ((int32)((int32)PIDInfo.gainI * PIDInfo.integralCumulative)) >> PIDInfo.divI;

    // Calculate derivative gain, gain * difference in error
    if(PIDInfo.gainD) {
        deltaError = error - PIDInfo.previousErrors[PIDInfo.previousErrorReadIndex++];
        PIDInfo.previousErrors[PIDInfo.previousErrorWriteIndex++] = error;

        // Wrap the indexes appropriately
        if (PIDInfo.previousErrorReadIndex >= PREVIOUS_ERROR_BUFFER_SIZE) {
            PIDInfo.previousErrorReadIndex = 0;
        }

        if (PIDInfo.previousErrorWriteIndex >= PREVIOUS_ERROR_BUFFER_SIZE) {
            PIDInfo.previousErrorWriteIndex = 0;
        }

        //deltaError = error - PIDInfo->errorPrevious;
        dTerm = ((int32)((int32)deltaError * PIDInfo.gainD)) >> PIDInfo.divD;
    } else  {
        dTerm = 0;
    }
    PIDInfo.errorPrevious = error;

    // Calculate result, sum of three individual gain terms
    result = (pTerm + iTerm + dTerm);
    result >>= PIDInfo.divOverall;

    // Limit output
    if (result > OUTPUT_LIMIT_UPPER) {
        output = OUTPUT_LIMIT_UPPER;
    } else if (result < OUTPUT_LIMIT_LOWER) {
        output = OUTPUT_LIMIT_LOWER;
    } else {
        output = (int16) result;
    }

    return(output);
}

float UpdatePID_Float(PIDDataType data_type, GimbalAxis axis, float error)
{
    float pTerm, dTerm, iTerm, result;
    float deltaError;
    float output;

    PIDData_Float* PIDInfo;

    // Look up the proper tuning parameters based on the requested PID data type
    switch (data_type) {
    case PID_DATA_RATE_LOOP:
        PIDInfo = &rate_pid_loop_float[axis];
        break;

    case PID_DATA_POSITION_LOOP:
        PIDInfo = &pos_pid_loop_float[axis];
        break;

    default:
        // If the data type requested isn't valid, just return 0 without running the PID loop
        return 0.0;
    }

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

void ClearPIDHistory_Float(PIDDataType data_type, GimbalAxis axis)
{
    PIDData_Float* PIDInfo = NULL;

    // Look up the proper tuning parameters based on the requested PID data type
    switch (data_type) {
    case PID_DATA_RATE_LOOP:
        PIDInfo = &rate_pid_loop_float[axis];
        break;

    case PID_DATA_POSITION_LOOP:
        PIDInfo = &pos_pid_loop_float[axis];
        break;

    default:
        // If the data type requested isn't valid, just return without doing anything
        return;
    }

    // Clear the accumulator and previous error
    PIDInfo->integralCumulative = 0.0;
    PIDInfo->errorPrevious = 0.0;
}
