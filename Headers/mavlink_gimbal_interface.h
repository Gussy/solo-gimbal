/*
 * mavlink_interface.h
 *
 *  Created on: Oct 10, 2014
 *      Author: abamberger
 */

#ifndef MAVLINK_INTERFACE_H_
#define MAVLINK_INTERFACE_H_

#include "PeripheralHeaderIncludes.h"

#ifndef uint8_t
#define uint8_t Uint8
#endif

#ifndef int8_t
#define int8_t int8
#endif

#include "ardupilotmega/mavlink.h"

typedef enum {
    MAVLINK_STATE_PARSE_INPUT,
    MAVLINK_STATE_SEND_PARAM_LIST
} MavlinkProcessingState;

typedef struct {
    MAV_STATE mav_state;
    MAV_MODE_GIMBAL mav_mode;
} MavlinkGimbalInfo;

//TODO: System ID of 50 is temporary for now
#define MAVLINK_GIMBAL_SYSID 50

void init_mavlink();
void mavlink_state_machine();
void send_mavlink_heartbeat(MAV_STATE mav_state, MAV_MODE_GIMBAL mav_mode);
void handle_attitude(mavlink_message_t* received_msg);
void send_mavlink_message(mavlink_message_t* msg);

// This is defined in terms of 150ms periods, so 6 is the closest we can get to a 1Hz heartbeat
#define MAVLINK_HEARTBEAT_PERIOD 6

#endif /* MAVLINK_INTERFACE_H_ */
