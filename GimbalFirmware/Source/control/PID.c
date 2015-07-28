#include "control/PID.h"

#include <stdlib.h>

PIDData_Float rate_pid_loop_float[AXIS_CNT] = {
    // These get loaded over CAN at boot and are hard-coded with the default PID values
    // The hardcoded PID gains can be overriden by the parameteres stored in flash, by
    //  setting the GMB_CUST_GAINS parameter from 0.0 to 1.0

    // Elevation
    {
            .gainP = 1.850000,
            .gainI = 0.200000,
            .gainD = 0.000000,
            .integralMax = 32768.0,
            .integralMin = -32768.0,
            .gainTotal = 1.0,
            .integralCumulative = 0.0,
            .dTermAlpha = 0.24,
            .processVarPrevious = 0.0,
            .deltaPvFilt = 0.0
    },

    // Azimuth
    {
            .gainP = 2.700000,
            .gainI = 0.500000,
            .gainD = 0.000000,
            .integralMax = 32768.0,
            .integralMin = -32768.0,
            .gainTotal = 1.0,
            .integralCumulative = 0.0,
            .dTermAlpha = 0.24,
            .processVarPrevious = 0.0,
            .deltaPvFilt = 0.0
    },

    // Roll
    {
            .gainP = 7.000000,
            .gainI = 0.500000,
            .gainD = 0.000000,
            .integralMax = 32768.0,
            .integralMin = -32768.0,
            .gainTotal = 1.0,
            .integralCumulative = 0.0,
            .dTermAlpha = 0.24,
            .processVarPrevious = 0.0,
            .deltaPvFilt = 0.0
    }
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
