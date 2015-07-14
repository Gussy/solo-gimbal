#ifndef _GOPRO_I2C_H
#define _GOPRO_I2C_H

#include <stdint.h>
#include <stdbool.h>

#include "PeripheralHeaderIncludes.h"

void gopro_i2c_init();
bool gopro_i2c_in_progress();
void gopro_i2c_on_timeout();

void gopro_i2c_send(const Uint8 *buf, Uint8 len);

#endif // _GOPRO_I2C_H
