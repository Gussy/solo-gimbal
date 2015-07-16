#ifndef FLASH_H_
#define FLASH_H_

#include "parameters/flash_params.h"

int erase_our_flash(void);
int erase_param_flash(void);
int write_flash(void);
int init_flash(void);

#endif /* FLASH_H_ */
