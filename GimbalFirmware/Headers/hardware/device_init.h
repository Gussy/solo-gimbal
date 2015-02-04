/*
 * device_init.h
 *
 *  Created on: Dec 9, 2014
 *      Author: abamberger
 */

#ifndef DEVICE_INIT_H_
#define DEVICE_INIT_H_

#include "f2806x_int8.h"
#include "PeripheralHeaderIncludes.h"

void DeviceInit(void);
void InitInterrupts();
Uint8 GetBoardHWID();
void ISR_ILLEGAL(void);

#endif /* DEVICE_INIT_H_ */
