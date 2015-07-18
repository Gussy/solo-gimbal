#ifndef FLASH_H_
#define FLASH_H_

#include "parameters/flash_params.h"

int erase_our_flash(void);
int erase_param_flash(void);
int write_flash(void);
int init_flash(void);

/**
 * Compute the 16-bit checksum of the flash parameters struct, return the checksum
 */
Uint16 compute_flash_params_checksum();

#endif /* FLASH_H_ */
