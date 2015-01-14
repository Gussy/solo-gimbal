/*
 * gimbal_mavlink.h
 *
 *  Created on: Jan 13, 2015
 *      Author: abamberger
 */

#ifndef GIMBAL_MAVLINK_H_
#define GIMBAL_MAVLINK_H_

// The sole purpose of this file is to only include the MAVLink headers after defining some integral data types
// that aren't defined by the C2000 compiler.  That way, this file can be included wherever the MAVLink headers
// are needed, and the defines don't need to be repeated everywhere they're included

#ifndef uint8_t
typedef Uint8 uint8_t;
#endif

#ifndef int8_t
typedef int8 int8_t;
#endif

#include <stdint.h>

#include "ardupilotmega/mavlink.h"

#endif /* GIMBAL_MAVLINK_H_ */
