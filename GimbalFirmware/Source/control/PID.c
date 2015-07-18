#include "control/PID.h"

#include <stdlib.h>
PIDData_Float rate_pid_loop_float[AXIS_CNT] = {
    // These get loaded over CAN at boot, so they are initialized to zero
    // (Except overall gain and d-term alpha, which are hardcoded)
    { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.24, 0.0, 0.0 },
    { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.24, 0.0, 0.0 },
    { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.24, 0.0, 0.0 }
};

float UpdatePID_Float(GimbalAxis axis, float setpoint, float process_var, float output_limit)
{
    float pTerm, dTerm;
    float deltaPv;
    float output;
    float error = setpoint - process_var;

    PIDData_Float* PIDInfo;

    // Look up the proper tuning parameters based on the requested PID data type
    PIDInfo = &rate_pid_loop_float[axis];

    // Calcuate proportional gain, gain * current error
    pTerm = PIDInfo->gainP * error;

    // Calculate derivative gain, gain * difference in error
    if(PIDInfo->gainD) {
        deltaPv = process_var - PIDInfo->processVarPrevious;
        PIDInfo->deltaPvFilt += (deltaPv-PIDInfo->deltaPvFilt) * PIDInfo->dTermAlpha;

        dTerm = -deltaPv * PIDInfo->gainD;
    } else  {
        dTerm = 0;
    }
    PIDInfo->processVarPrevious = process_var;

    output = (pTerm + PIDInfo->integralCumulative + dTerm);

    if((output <= output_limit && output >= -output_limit) || (output > output_limit && error < 0) || (output < -output_limit && error > 0)) {
        float i_increment = error * PIDInfo->gainI;
        PIDInfo->integralCumulative += i_increment;
        output += i_increment;
    }

    // Limit output
    if (output > output_limit) {
        output = output_limit;
    } else if (output < -output_limit) {
        output = -output_limit;
    }

    return(output);
}
