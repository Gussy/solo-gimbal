/*
 * mavlink_parameter_interface.c
 *
 *  Created on: Dec 16, 2014
 *      Author: abamberger
 */

#include "mavlink_gimbal_interface.h"
#include "mavlink_parameter_interface.h"

#include <string.h>

GimbalMavlinkParameter gimbal_params[NUM_MAVLINK_PARAMS];

void init_default_mavlink_params()
{
    strncpy(gimbal_params[0].param_id, "PID_YAW_P", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[0].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[0].param.param_float = 1.0;

    strncpy(gimbal_params[1].param_id, "PID_YAW_I", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[1].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[1].param.param_float = 0.0;

    strncpy(gimbal_params[2].param_id, "PID_YAW_I_MAX", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[2].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[2].param.param_float = 2000.0;

    strncpy(gimbal_params[3].param_id, "PID_YAW_D", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[3].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[3].param.param_float = 0.0;

    strncpy(gimbal_params[4].param_id, "PID_PITCH_P", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[4].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[4].param.param_float = 1.0;

    strncpy(gimbal_params[5].param_id, "PID_PITCH_I", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[5].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[5].param.param_float = 0.0;

    strncpy(gimbal_params[6].param_id, "PID_PITCH_I_MAX", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[6].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[6].param.param_float = 2000.0;

    strncpy(gimbal_params[7].param_id, "PID_PITCH_D", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[7].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[7].param.param_float = 0.0;

    strncpy(gimbal_params[8].param_id, "PID_ROLL_P", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[8].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[8].param.param_float = 1.0;

    strncpy(gimbal_params[9].param_id, "PID_ROLL_I", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[9].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[9].param.param_float = 0.0;

    strncpy(gimbal_params[10].param_id, "PID_ROLL_I_MAX", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[10].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[10].param.param_float = 2000.0;

    strncpy(gimbal_params[11].param_id, "PID_ROLL_D", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[11].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[11].param.param_float = 0.0;

    strncpy(gimbal_params[12].param_id, "SYSID_SWVER", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[12].param_type = MAV_PARAM_TYPE_INT32;
    gimbal_params[12].param.param_float = 1;

    strncpy(gimbal_params[13].param_id, "SERIAL_BAUD", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[13].param_type = MAV_PARAM_TYPE_INT32;
    gimbal_params[13].param.param_float = 115;
}

void handle_param_set(mavlink_message_t* received_msg)
{
    mavlink_param_set_t decoded_msg;
    mavlink_msg_param_set_decode(received_msg, &decoded_msg);

    // Search the onboard param list for the param id being updated
    int param_found = -1;
    int i;
    for(i = 0; i < NUM_MAVLINK_PARAMS; i++) {
        if (strncmp(decoded_msg.param_id, gimbal_params[i].param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN) == 0) {
            param_found = i;
            break;
        }
    }

    // If we found the param id in our param list, attempt to update the value
    if (param_found >= 0) {
        // First, make sure the type of the param being sent matches the type of the param being updated
        if (gimbal_params[param_found].param_type == decoded_msg.param_type) {
            // The float value of the param is always sent, regardless of what the actual type is, so update the param through the float value
            gimbal_params[param_found].param.param_float = decoded_msg.param_value;

            // Echo the new value of the param back to acknowledge receipt of the param
            send_gimbal_param(param_found);
        }
    }
}

void send_gimbal_param(int param_num)
{
    GimbalMavlinkParameter* param = &(gimbal_params[param_num]);

    static mavlink_message_t param_msg;
    mavlink_msg_param_value_pack(MAVLINK_GIMBAL_SYSID, MAV_COMP_ID_GIMBAL, &param_msg, param->param_id, param->param.param_float, param->param_type, NUM_MAVLINK_PARAMS, param_num);
    send_mavlink_message(&param_msg);
}
