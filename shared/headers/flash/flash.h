#ifndef FLASH_H_
#define FLASH_H_

#include "F2806x_Device.h" // Include for Uint16 typedef

int erase_our_flash(void);
int erase_param_flash(void);
int write_flash(void);
int init_flash(void);
Uint16 compute_flash_params_checksum(void);

#endif /* FLASH_H_ */
