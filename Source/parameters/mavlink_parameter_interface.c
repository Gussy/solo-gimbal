/*
 * mavlink_parameter_interface.c
 *
 *  Created on: Dec 16, 2014
 *      Author: abamberger
 */

#include "mavlink_interface/mavlink_gimbal_interface.h"
#include "parameters/mavlink_parameter_interface.h"
#include "parameters/flash_params.h"
#include "can/cand.h"
#include "PM_Sensorless.h"

#include <string.h>

GimbalMavlinkParameter gimbal_params[MAVLINK_GIMBAL_PARAM_MAX];

void init_default_mavlink_params()
{
    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_P].param_id, "PID_YAW_P", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_P].can_parameter_id = CAND_PID_RATE_AZ_P;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_P].can_parameter_destination_axis = CAND_ID_EL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_P].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_P].float_data_ptr = &(flash_params.rate_pid_p[AZ]);

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I].param_id, "PID_YAW_I", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I].can_parameter_id = CAND_PID_RATE_AZ_I;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I].can_parameter_destination_axis = CAND_ID_EL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I].float_data_ptr = &(flash_params.rate_pid_i[AZ]);

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_D].param_id, "PID_YAW_D", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_D].can_parameter_id = CAND_PID_RATE_AZ_D;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_D].can_parameter_destination_axis = CAND_ID_EL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_D].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_D].float_data_ptr = &(flash_params.rate_pid_d[AZ]);

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I_MAX].param_id, "PID_YAW_I_MAX", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I_MAX].can_parameter_id = CAND_PID_RATE_AZ_WINDUP;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I_MAX].can_parameter_destination_axis = CAND_ID_EL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I_MAX].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I_MAX].float_data_ptr = &(flash_params.rate_pid_windup[AZ]);

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_P].param_id, "PID_PITCH_P", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_P].can_parameter_id = CAND_PID_RATE_EL_P;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_P].can_parameter_destination_axis = CAND_ID_EL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_P].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_P].float_data_ptr = &(flash_params.rate_pid_p[EL]);

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I].param_id, "PID_PITCH_I", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I].can_parameter_id = CAND_PID_RATE_EL_I;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I].can_parameter_destination_axis = CAND_ID_EL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I].float_data_ptr = &(flash_params.rate_pid_i[EL]);

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_D].param_id, "PID_PITCH_D", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_D].can_parameter_id = CAND_PID_RATE_EL_D;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_D].can_parameter_destination_axis = CAND_ID_EL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_D].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_D].float_data_ptr = &(flash_params.rate_pid_d[EL]);

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I_MAX].param_id, "PID_PITCH_I_MAX", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I_MAX].can_parameter_id = CAND_PID_RATE_EL_WINDUP;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I_MAX].can_parameter_destination_axis = CAND_ID_EL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I_MAX].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I_MAX].float_data_ptr = &(flash_params.rate_pid_windup[EL]);

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_P].param_id, "PID_ROLL_P", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_P].can_parameter_id = CAND_PID_RATE_RL_P;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_P].can_parameter_destination_axis = CAND_ID_EL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_P].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_P].float_data_ptr = &(flash_params.rate_pid_p[ROLL]);

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I].param_id, "PID_ROLL_I", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I].can_parameter_id = CAND_PID_RATE_RL_I;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I].can_parameter_destination_axis = CAND_ID_EL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I].float_data_ptr = &(flash_params.rate_pid_i[ROLL]);

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_D].param_id, "PID_ROLL_D", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_D].can_parameter_id = CAND_PID_RATE_RL_D;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_D].can_parameter_destination_axis = CAND_ID_EL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_D].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_D].float_data_ptr = &(flash_params.rate_pid_d[ROLL]);

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I_MAX].param_id, "PID_ROLL_I_MAX", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I_MAX].can_parameter_id = CAND_PID_RATE_RL_WINDUP;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I_MAX].can_parameter_destination_axis = CAND_ID_EL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I_MAX].param_type = MAV_PARAM_TYPE_REAL32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I_MAX].float_data_ptr = &(flash_params.rate_pid_windup[ROLL]);

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SWVER].param_id, "SYSID_SWVER", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SWVER].can_parameter_id = CAND_PID_INVALID;
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SWVER].param_type = MAV_PARAM_TYPE_INT32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SWVER].uint32_data_ptr = &(flash_params.sys_swver);

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_SERIAL_BAUD].param_id, "SERIAL_BAUD", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_SERIAL_BAUD].can_parameter_id = CAND_PID_INVALID;
    gimbal_params[MAVLINK_GIMBAL_PARAM_SERIAL_BAUD].param_type = MAV_PARAM_TYPE_INT32;
    gimbal_params[MAVLINK_GIMBAL_PARAM_SERIAL_BAUD].uint32_data_ptr = &(flash_params.mavlink_baud_rate);
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
        GimbalMavlinkParameter* param = &gimbal_params[param_found];
        // First, make sure the type of the param being sent matches the type of the param being updated
        if (param->param_type == decoded_msg.param_type) {
            // The float value of the param is always sent, regardless of the actual type
            // So if the param is actually a float, we can just use the param value
            // If the param is actually an integer, we need to convert it first
            if (decoded_msg.param_type == MAV_PARAM_TYPE_REAL32) {
                *(param->float_data_ptr) = decoded_msg.param_value;
            } else if (decoded_msg.param_type == MAV_PARAM_TYPE_INT32) {
                IntOrFloat float_converter;
                float_converter.float_val = decoded_msg.param_value;
                *(param->uint32_data_ptr) = float_converter.uint32_val;
            }

            // If the parameter has an associated CAN parameter, transmit the updated parameter over CAN
            // to the appropriate axis
            if (param->can_parameter_id != CAND_PID_INVALID) {
                if (param->param_type == MAV_PARAM_TYPE_REAL32) {
                    // If the parameter is a float, we first have to convert it to an integer to send over CAN
                    IntOrFloat float_converter;
                    float_converter.float_val = *(param->float_data_ptr);
                    cand_tx_param(param->can_parameter_destination_axis, param->can_parameter_id, float_converter.uint32_val);
                } else if (param->param_type == MAV_PARAM_TYPE_UINT32) {
                    // If the parameter is already an integer, we can just send it
                    cand_tx_param(param->can_parameter_destination_axis, param->can_parameter_id, *(param->uint32_data_ptr));
                }
            }

            // Echo the new value of the param back to acknowledge receipt of the param
            send_gimbal_param(param_found);
        }
    }
}

void send_gimbal_param(int param_num)
{
    GimbalMavlinkParameter* param = &(gimbal_params[param_num]);

    static mavlink_message_t param_msg;
    float param_val;
    IntOrFloat float_converter;

    // If the parameter is already a float, we can just send it.  Otherwise, it's an integer, so we have to convert it to a float first
    if (param->param_type == MAV_PARAM_TYPE_REAL32) {
        param_val = *(param->float_data_ptr);
    } else {
        float_converter.uint32_val = *(param->float_data_ptr);
        param_val = float_converter.float_val;
    }

    mavlink_msg_param_value_pack(MAVLINK_GIMBAL_SYSID, MAV_COMP_ID_GIMBAL, &param_msg, param->param_id, param_val, param->param_type, MAVLINK_GIMBAL_PARAM_MAX, param_num);
    send_mavlink_message(&param_msg);
}
