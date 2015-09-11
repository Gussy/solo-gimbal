#include "control/PID.h"
#include <stdlib.h>

// These are loaded over CAN at boot
PIDData_Float rate_pid_loop_float[AXIS_CNT] = {
    // These get loaded over CAN at boot and are hard-coded with the default PID values
    // The hardcoded PID gains and dTermAlpha can be overriden by the parameteres stored
    //  in flash, by setting the GMB_CUST_GAINS parameter from 0.0 to 1.0

    // Elevation
    {
        .gainP = 0.0,
        .gainI = 0.0,
        .gainD = 0.0,
        .integralCumulative = 0.0,
        .dTermAlpha = 0.0,
        .processVarPrevious = 0.0,
        .deltaPvFilt = 0.0
    },

    // Azimuth
    {
        .gainP = 0.0,
        .gainI = 0.0,
        .gainD = 0.0,
        .integralCumulative = 0.0,
        .dTermAlpha = 0.0,
        .processVarPrevious = 0.0,
        .deltaPvFilt = 0.0
    },

    // Roll
    {
        .gainP = 0.0,
        .gainI = 0.0,
        .gainD = 0.0,
        .integralCumulative = 0.0,
        .dTermAlpha = 0.0,
        .processVarPrevious = 0.0,
        .deltaPvFilt = 0.0
    }
};

float UpdatePID_Float(GimbalAxis axis, float setpoint, float process_var, float output_limit, float gain, float ff)
{
    /*
     * This function implements a modified PID controller (D term on process
     * variable only) with support for integrator windup protection and injection
     * of a feedforward term which is added to the output.
     *
     * The gain term in the function parameters scales the P, I, and D gains,
     * providing a way to implement gain scheduling.
     */
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
    if(PIDInfo->gainD != 0.0f) {
        deltaPv = process_var - PIDInfo->processVarPrevious;
        PIDInfo->deltaPvFilt += (deltaPv-PIDInfo->deltaPvFilt) * PIDInfo->dTermAlpha;

        dTerm = -PIDInfo->deltaPvFilt * PIDInfo->gainD;
    } else  {
        dTerm = 0;
    }
    PIDInfo->processVarPrevious = process_var;

    output = (pTerm + dTerm) * gain + PIDInfo->integralCumulative + ff;

    if((output <= output_limit && output >= -output_limit) || (output > output_limit && error < 0) || (output < -output_limit && error > 0)) {
        float i_increment = error * PIDInfo->gainI * gain;
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
