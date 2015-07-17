#include "control/PID.h"

#include <stdlib.h>
PIDData_Float rate_pid_loop_float[AXIS_CNT] = {
    // These get loaded over CAN at boot, so they are initialized to zero
    // (Except overall gain and d-term alpha, which are hardcoded)
    { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.1, 0.0 },
    { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.1, 0.0 },
    { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.1, 0.0 }
};

float UpdatePID_Float(GimbalAxis axis, float setpoint, float process_var, float p_detune_ratio, float i_detune_ratio, float d_detune_ratio)
{
    float pTerm, dTerm, iTerm, result;
    float deltaPv;
    float output;
    float error = setpoint - process_var;

    PIDData_Float* PIDInfo;

    // Look up the proper tuning parameters based on the requested PID data type
    PIDInfo = &rate_pid_loop_float[axis];

    // Calcuate proportional gain, gain * current error
    pTerm = (PIDInfo->gainP * p_detune_ratio) * error;

    // Calculate integral gain, gain * accumulation of historical error
    PIDInfo->integralCumulative += error;

    // Limit accumulated integral error to windup limits
    if (PIDInfo->integralCumulative > PIDInfo->integralMax) {
        PIDInfo->integralCumulative = PIDInfo->integralMax;
    } else if (PIDInfo->integralCumulative < PIDInfo->integralMin) {
        PIDInfo->integralCumulative = PIDInfo->integralMin;
    }

    iTerm = (PIDInfo->gainI * i_detune_ratio) * PIDInfo->integralCumulative;

    // Calculate derivative gain, gain * difference in error
    if(PIDInfo->gainD) {
        deltaPv = process_var - PIDInfo->processVarPrevious;

        deltaPv = (deltaPv * PIDInfo->dTermAlpha) + ((1.0 - PIDInfo->dTermAlpha) * PIDInfo->processVarPrevious);

        dTerm = -deltaPv * (PIDInfo->gainD * d_detune_ratio);
    } else  {
        dTerm = 0;
    }
    PIDInfo->processVarPrevious = process_var;

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
