/*
 * mavlink_parameter_interface.h
 *
 *  Created on: Dec 16, 2014
 *      Author: abamberger
 */

#ifndef MAVLINK_PARAMETER_INTERFACE_H_
#define MAVLINK_PARAMETER_INTERFACE_H_

#ifndef uint8_t
#define uint8_t Uint8
#endif

#ifndef int8_t
#define int8_t int8
#endif

#include "ardupilotmega/mavlink.h"

#define NUM_MAVLINK_PARAMS 14

//NOTE: I'm not using the MAVLink library's mavlink_param_union_t, because the C2000 compiler does not support anonymous unions
//It was easier just to add this custom version here than to modify the MAVLink library
typedef union {
    float param_float;
    uint32_t param_uint32;
} mavlink_param_union_c2000_t;

typedef struct {
    mavlink_param_union_c2000_t param;
    uint8_t param_type;
    char param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
} GimbalMavlinkParameter;

void init_default_mavlink_params();
void handle_param_set(mavlink_message_t* received_msg);
void send_gimbal_param(int param_num);

#endif /* MAVLINK_PARAMETER_INTERFACE_H_ */
