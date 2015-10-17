#ifndef _GOPRO_I2C_H
#define _GOPRO_I2C_H

#include <stdint.h>
#include <stdbool.h>

#include "f2806x_int8.h"
#include "PeripheralHeaderIncludes.h"

void gopro_i2c_init();
bool gopro_i2c_in_progress();
void gopro_i2c_on_timeout();

void gopro_i2c_send(const uint8_t *buf, uint8_t len);

#endif // _GOPRO_I2C_H
