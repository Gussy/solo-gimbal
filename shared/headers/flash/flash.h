#ifndef FLASH_H_
#define FLASH_H_

#include "F2806x_Device.h" // Include for Uint16 typedef

int erase_our_flash(void);
int write_flash(void);
int init_flash(void);

#endif /* FLASH_H_ */
