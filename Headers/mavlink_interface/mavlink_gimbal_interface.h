/*
 * mavlink_interface.h
 *
 *  Created on: Oct 10, 2014
 *      Author: abamberger
 */

#ifndef MAVLINK_INTERFACE_H_
#define MAVLINK_INTERFACE_H_

#include "PeripheralHeaderIncludes.h"
#include "PM_Sensorless.h"
#include "mavlink_interface/gimbal_mavlink.h"

typedef enum {
	MAVLINK_STATE_PARSE_INPUT, MAVLINK_STATE_SEND_PARAM_LIST
} MavlinkProcessingState;

typedef struct {
	MAV_STATE mav_state;
	MAV_MODE_GIMBAL mav_mode;
} MavlinkGimbalInfo;

//TODO: System ID of 50 is temporary for now
#define MAVLINK_GIMBAL_SYSID 50
#define ATTITUDE_DATA_REFRESH_RATE 10
// This is defined in terms of 150ms periods, so 6 is the closest we can get to a 1Hz heartbeat
#define MAVLINK_HEARTBEAT_PERIOD 6

void init_mavlink();
void mavlink_state_machine();
void send_mavlink_heartbeat(MAV_STATE mav_state, MAV_MODE_GIMBAL mav_mode);
void send_mavlink_debug_data(DebugData* debug_data);
void send_mavlink_message(mavlink_message_t* msg);

#endif /* MAVLINK_INTERFACE_H_ */
