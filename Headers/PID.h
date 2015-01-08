/*
 * PID.h
 *
 *  Created on: Sep 12, 2014
 *      Author: abamberger
 */

#ifndef PID_H_
#define PID_H_

#include "PeripheralHeaderIncludes.h"
#include "HWSpecific.h"

#define OUTPUT_LIMIT             32767
// Used to limit result
#define OUTPUT_LIMIT_UPPER      (OUTPUT_LIMIT)
// Used to limit result
#define OUTPUT_LIMIT_LOWER      (-OUTPUT_LIMIT)

#define OUTPUT_LIMIT_FLOAT          32767.0
#define OUTPUT_LIMIT_UPPER_FLOAT    (OUTPUT_LIMIT_FLOAT)
#define OUTPUT_LIMIT_LOWER_FLOAT    (-OUTPUT_LIMIT_FLOAT)

#define PREVIOUS_ERROR_BUFFER_SIZE 3

typedef enum {
    PID_DATA_RATE_LOOP,
    PID_DATA_POSITION_LOOP
} PIDDataType;

typedef struct _PIDData
{
    Uint16  gainP;                  // Proportional loop gain
    Uint16  gainI;                  // Integral loop gain
    Uint16  gainD;                  // Derivative loop gain
    int16   integralMax;            // Maximum Positive Integral Windup
    int16   integralMin;            // Maximum Negative Integral Windup
    int16   gainTotal;              // Total loop gain
    int16   integralCumulative;     // Current cumulative integral error
    int16   errorPrevious;          // Previous delta (derivative) error
    Uint8   divP;                   // Proportional divisor (number of shifts to the right)
    Uint8   divI;                   // Integral divisor (number of shifts to the right)
    Uint8   divD;                   // Derivative divisor (number of shifts to the right)
    Uint8   divOverall;             // Overall Divisor
    int16   previousErrors[PREVIOUS_ERROR_BUFFER_SIZE];
    Uint8   previousErrorReadIndex;
    Uint8   previousErrorWriteIndex;
} PIDData;

typedef struct {
    float   gainP;
    float   gainI;
    float   gainD;
    float   integralMax;
    float   integralMin;
    float   gainTotal;
    float   integralCumulative;
    float	dTermAlpha;
    float   errorPrevious;
} PIDData_Float;

int16 UpdatePID(GimbalAxis axis, int16 sError);

float UpdatePID_Float(PIDDataType data_type, GimbalAxis axis, float error);

void ClearPIDHistory_Float(PIDDataType data_type, GimbalAxis axis);

#endif /* PID_H_ */
