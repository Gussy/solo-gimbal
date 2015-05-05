#ifndef ENCODER_H_
#define ENCODER_H_

#include "PM_Sensorless.h"

#define ANALOG_POT_MECH_DIVIDER 4096.0 // Resolution of 10-bit ADC

void UpdateEncoderReadings(EncoderParms* encoder_parms, ControlBoardParms* cb_parms);

#endif /* ENCODER_H_ */
