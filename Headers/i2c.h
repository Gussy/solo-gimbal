/*
 * i2c.h
 *
 *  Created on: Jan 8, 2015
 *      Author: abamberger
 */

#ifndef I2C_H_
#define I2C_H_

#include "PeripheralHeaderIncludes.h"

#define I2C_BUFFER_SIZE 128

void init_i2c();
Uint16 i2c_get_sdir();
Uint16 i2c_get_aas();
Uint16 i2c_get_bb();
interrupt void i2c_fifo_isr(void);
void i2c_send_data(Uint8* data, int length);

#endif /* I2C_H_ */
