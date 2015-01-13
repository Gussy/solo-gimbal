/*
 * mavlink_parameter_interface.c
 *
 *  Created on: Dec 16, 2014
 *      Author: abamberger
 */

#include "mavlink_gimbal_interface.h"
#include "parameters/mavlink_parameter_interface.h"

#include <string.h>

GimbalMavlinkParameter gimbal_params[MAVLINK_GIMBAL_PARAM_MAX];

void init_default_mavlink_params()
{
    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_P].param_id, "PID_YAW_P", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_P].can_parameter_id = CAND_PID_RATE_AZ_P;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_P].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_P].param.param_float = 1.0;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I].param_id, "PID_YAW_I", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I].can_parameter_id = CAND_PID_RATE_AZ_I;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I].param.param_float = 0.0;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_D].param_id, "PID_YAW_D", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_D].can_parameter_id = CAND_PID_RATE_AZ_D;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_D].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_D].param.param_float = 0.0;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I_MAX].param_id, "PID_YAW_I_MAX", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I_MAX].can_parameter_id = CAND_PID_RATE_AZ_WINDUP;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I_MAX].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I_MAX].param.param_float = 2000.0;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_P].param_id, "PID_PITCH_P", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_P].can_parameter_id = CAND_PID_RATE_EL_P;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_P].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_P].param.param_float = 1.0;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I].param_id, "PID_PITCH_I", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I].can_parameter_id = CAND_PID_RATE_EL_I;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I].param.param_float = 0.0;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_D].param_id, "PID_PITCH_D", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_D].can_parameter_id = CAND_PID_RATE_EL_D;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_D].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_D].param.param_float = 0.0;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I_MAX].param_id, "PID_PITCH_I_MAX", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I_MAX].can_parameter_id = CAND_PID_RATE_EL_WINDUP;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I_MAX].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I_MAX].param.param_float = 2000.0;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_P].param_id, "PID_ROLL_P", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_P].can_parameter_id = CAND_PID_RATE_RL_P;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_P].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_P].param.param_float = 1.0;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I].param_id, "PID_ROLL_I", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I].can_parameter_id = CAND_PID_RATE_RL_I;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I].param.param_float = 0.0;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_D].param_id, "PID_ROLL_D", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_D].can_parameter_id = CAND_PID_RATE_RL_D;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_D].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_D].param.param_float = 0.0;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I_MAX].param_id, "PID_ROLL_I_MAX", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I_MAX].can_parameter_id = CAND_PID_RATE_RL_WINDUP;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I_MAX].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I_MAX].param.param_float = 2000.0;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SWVER].param_id, "SYSID_SWVER", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SWVER].can_parameter_id = CAND_PID_INVALID;
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SWVER].param_type = MAV_PARAM_TYPE_INT32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SWVER].param.param_float = 1;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_SERIAL_BAUD].param_id, "SERIAL_BAUD", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_SERIAL_BAUD].can_parameter_id = CAND_PID_INVALID;
    gimbal_params[MAVLINK_GIMBAL_PARAM_SERIAL_BAUD].param_type = MAV_PARAM_TYPE_INT32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_SERIAL_BAUD].param.param_float = 115;
}

void handle_param_set(mavlink_message_t* received_msg)
{
    mavlink_param_set_t decoded_msg;
    mavlink_msg_param_set_decode(received_msg, &decoded_msg);

    // Search the onboard param list for the param id being updated
    int param_found = -1;
    int i;
    for(i = 0; i < MAVLINK_GIMBAL_PARAM_MAX; i++) {
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
    mavlink_msg_param_value_pack(MAVLINK_GIMBAL_SYSID, MAV_COMP_ID_GIMBAL, &param_msg, param->param_id, param->param.param_float, param->param_type, MAVLINK_GIMBAL_PARAM_MAX, param_num);
    send_mavlink_message(&param_msg);
}
