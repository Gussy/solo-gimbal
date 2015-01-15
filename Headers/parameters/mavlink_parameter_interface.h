/*
 * mavlink_parameter_interface.h
 *
 *  Created on: Dec 16, 2014
 *      Author: abamberger
 */

#ifndef MAVLINK_PARAMETER_INTERFACE_H_
#define MAVLINK_PARAMETER_INTERFACE_H_

#include "can/cand_BitFields.h"
#include "mavlink_interface/gimbal_mavlink.h"

typedef enum {
    MAVLINK_GIMBAL_PARAM_PID_YAW_P = 0,
    MAVLINK_GIMBAL_PARAM_PID_YAW_I,
    MAVLINK_GIMBAL_PARAM_PID_YAW_D,
    MAVLINK_GIMBAL_PARAM_PID_YAW_I_MAX,
    MAVLINK_GIMBAL_PARAM_PID_PITCH_P,
    MAVLINK_GIMBAL_PARAM_PID_PITCH_I,
    MAVLINK_GIMBAL_PARAM_PID_PITCH_D,
    MAVLINK_GIMBAL_PARAM_PID_PITCH_I_MAX,
    MAVLINK_GIMBAL_PARAM_PID_ROLL_P,
    MAVLINK_GIMBAL_PARAM_PID_ROLL_I,
    MAVLINK_GIMBAL_PARAM_PID_ROLL_D,
    MAVLINK_GIMBAL_PARAM_PID_ROLL_I_MAX,
    MAVLINK_GIMBAL_PARAM_POS_PID_YAW_P,
    MAVLINK_GIMBAL_PARAM_POS_PID_YAW_I,
    MAVLINK_GIMBAL_PARAM_POS_PID_YAW_D,
    MAVLINK_GIMBAL_PARAM_POS_PID_YAW_I_MAX,
    MAVLINK_GIMBAL_PARAM_POS_PID_PITCH_P,
    MAVLINK_GIMBAL_PARAM_POS_PID_PITCH_I,
    MAVLINK_GIMBAL_PARAM_POS_PID_PITCH_D,
    MAVLINK_GIMBAL_PARAM_POS_PID_PITCH_I_MAX,
    MAVLINK_GIMBAL_PARAM_POS_PID_ROLL_P,
    MAVLINK_GIMBAL_PARAM_POS_PID_ROLL_I,
    MAVLINK_GIMBAL_PARAM_POS_PID_ROLL_D,
    MAVLINK_GIMBAL_PARAM_POS_PID_ROLL_I_MAX,
    MAVLINK_GIMBAL_PARAM_GYRO_OFFSET_YAW,
    MAVLINK_GIMBAL_PARAM_GYRO_OFFSET_PITCH,
    MAVLINK_GIMBAL_PARAM_GYRO_OFFSET_ROLL,
    MAVLINK_GIMBAL_PARAM_SYSID_SWVER,
    MAVLINK_GIMBAL_PARAM_SERIAL_BAUD,
    MAVLINK_GIMBAL_PARAM_MAX
} GimbalMavlinkParameterID;

//NOTE: I'm not using the MAVLink library's mavlink_param_union_t, because the C2000 compiler does not support anonymous unions
//It was easier just to add this custom version here than to modify the MAVLink library
typedef union {
    float param_float;
    uint32_t param_uint32;
} mavlink_param_union_c2000_t;

typedef struct {
    uint8_t param_type;
    float* float_data_ptr;
    uint32_t* uint32_data_ptr;
    CAND_ParameterID can_parameter_id;
    CAND_DestinationID can_parameter_destination_axis;
    char param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
} GimbalMavlinkParameter;

void init_default_mavlink_params();
void handle_param_set(mavlink_message_t* received_msg);
void send_gimbal_param(int param_num);

#endif /* MAVLINK_PARAMETER_INTERFACE_H_ */