#ifndef PID_H_
#define PID_H_

#include "f2806x_int8.h"
#include "PeripheralHeaderIncludes.h"
#include "hardware/HWSpecific.h"

typedef struct {
    float   gainP;
    float   gainI;
    float   gainD;
    float   integralMax;
    float   integralMin;
    float   gainTotal;
    float   integralCumulative;
    float	dTermAlpha;
    float   processVarPrevious;
    float   deltaPvFilt;
} PIDData_Float;

extern PIDData_Float rate_pid_loop_float[AXIS_CNT];

float UpdatePID_Float(GimbalAxis axis, float setpoint, float process_var, float output_limit, float gain, float ff);


#endif /* PID_H_ */
