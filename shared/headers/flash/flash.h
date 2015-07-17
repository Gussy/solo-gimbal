#ifndef FLASH_H_
#define FLASH_H_

#include "parameters/flash_params.h"

#define OTP_START_ADDR  0x3D7800
#define OTP_END_ADDR    0x3D7BFF

#define START_ADDR      0x3D8000
#define FLASH_END_ADDR  0x3F7FFF

int erase_our_flash(void);
int erase_param_flash(void);
int write_flash(void);
int init_flash(void);

/**
 * Compute the 16-bit checksum of the flash parameters struct, return the checksum
 */
Uint16 compute_flash_params_checksum();

#endif /* FLASH_H_ */
