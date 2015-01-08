/*
 * flash_params.h
 *
 *  Created on: Jan 5, 2015
 *      Author: ksmith
 */

#ifndef FLASH_PARAMS_H_
#define FLASH_PARAMS_H_


#include "HWSpecific.h"

struct flash_param_struct_0000 {
	Uint16 flash_struct_id;
	Uint16 board_id;
	Uint16 other_id;
	float AxisCalibrationSlopes[AXIS_CNT];
	float AxisCalibrationIntercepts[AXIS_CNT];
	int AxisHomePositions[AXIS_CNT];
};

extern struct flash_param_struct_0000 flash_params;

/**
 * initialize the flash, return positive on success, negative on failure
 *
 * Either way, the flash_param_struct needs to be initialized, if negative, to the default
 */
int init_flash(void);

/**
 * write what is in the current flash_params to flash, return a negative on failure
 */
int write_flash(void);


#endif /* FLASH_PARAMS_H_ */
