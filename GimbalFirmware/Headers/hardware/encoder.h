#ifndef ENCODER_H_
#define ENCODER_H_

#include "PM_Sensorless.h"

#define ANALOG_POT_MECH_DIVIDER 4096.0 // Resolution of 10-bit ADC

#define ENCODER_COUNTS_PER_REV 10000
#define COUNTS_PER_DEGREE ((float)ENCODER_COUNTS_PER_REV / 360.0)
#define DEGREES_TO_COUNTS(x) (x * COUNTS_PER_DEGREE)


void UpdateEncoderReadings(EncoderParms* encoder_parms, ControlBoardParms* cb_parms);

#endif /* ENCODER_H_ */
