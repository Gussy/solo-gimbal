#ifndef LED_RED_H_
#define LED_RED_H_

#define STATUS_LED_ON()				{GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;}
#define STATUS_LED_OFF()			{GpioDataRegs.GPASET.bit.GPIO7 = 1;}
#define STATUS_LED_TOGGLE()			{GpioDataRegs.GPATOGGLE.bit.GPIO7 = 1;}

#endif /* LED_RED_H_ */
