#ifndef FLASH_H_
#define FLASH_H_

#include "parameters/flash_params.h"

/**
 * write what is in the current flash_params to flash, return a negative on failure
 */
int write_flash(void);


/**
 * initialize the flash, return positive on success, negative on failure
 *
 * Either way, the flash_param_struct needs to be initialized, if negative, to the default
 */
int init_flash(void);

#endif /* FLASH_H_ */
