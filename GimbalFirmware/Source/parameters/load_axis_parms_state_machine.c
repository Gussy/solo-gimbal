/*
 * init_axis_parms_state_machine.c
 *
 *  Created on: Jan 9, 2015
 *      Author: abamberger
 */

#include "parameters/load_axis_parms_state_machine.h"
#include "can/cand.h"
#include "can/cand_BitFields.h"
#include "parameters/flash_params.h"
#include "hardware/device_init.h"
#include "PM_Sensorless-Settings.h"

void LoadAxisParmsStateMachine(LoadAxisParmsStateInfo* load_parms_state_info)
{
    // If we've received the current parameter we're requesting, go on to asking for the next parameter we need.
    // The parameter received flags are updated in can_message_processor.c when the parameter responses come in
    switch(load_parms_state_info->load_axis_parms_state) {
        case LOAD_AXIS_PARMS_STATE_REQUEST_TORQUE_KP:
            if (load_parms_state_info->init_param_recvd_flags_2 & INIT_PARAM_TORQUE_PID_KP_RECVD) {
                load_parms_state_info->load_axis_parms_state = LOAD_AXIS_PARMS_STATE_REQUEST_TORQUE_KI;
            } else {
                cand_tx_request(CAND_ID_AZ, CAND_PID_TORQUE_KP); // All parameter requests go to the AZ board
            }
            break;

        case LOAD_AXIS_PARMS_STATE_REQUEST_TORQUE_KI:
            if (load_parms_state_info->init_param_recvd_flags_2 & INIT_PARAM_TORQUE_PID_KI_RECVD) {
                load_parms_state_info->load_axis_parms_state = LOAD_AXIS_PARMS_STATE_REQUEST_TORQUE_KD;
            } else {
                cand_tx_request(CAND_ID_AZ, CAND_PID_TORQUE_KI); // All parameter requests go to the AZ board
            }
            break;

        case LOAD_AXIS_PARMS_STATE_REQUEST_TORQUE_KD:
            if (load_parms_state_info->init_param_recvd_flags_2 & INIT_PARAM_TORQUE_PID_KD_RECVD) {
                load_parms_state_info->load_axis_parms_state = LOAD_AXIS_PARMS_STATE_REQUEST_COMMUTATION_CALIBRATION_SLOPE;
            } else {
                cand_tx_request(CAND_ID_AZ, CAND_PID_TORQUE_KD); // All parameter requests go to the AZ board
            }
            break;

        case LOAD_AXIS_PARMS_STATE_REQUEST_COMMUTATION_CALIBRATION_SLOPE:
            if (load_parms_state_info->init_param_recvd_flags_2 & INIT_PARAM_COMMUTATION_CALIBRATION_SLOPE_RECVD) {
                load_parms_state_info->load_axis_parms_state = LOAD_AXIS_PARMS_STATE_REQUEST_COMMUTATION_CALIBRATION_INTERCEPT;
            } else {
                cand_tx_request(CAND_ID_AZ, CAND_PID_COMMUTATION_CALIBRATION_SLOPE); // All parameter requests go to the AZ board
            }
            break;

        case LOAD_AXIS_PARMS_STATE_REQUEST_COMMUTATION_CALIBRATION_INTERCEPT:
            if (load_parms_state_info->init_param_recvd_flags_2 & INIT_PARAM_COMMUTATION_CALIBRATION_INTERCEPT_RECVD) {
                load_parms_state_info->load_axis_parms_state = LOAD_AXIS_PARMS_STATE_REQUEST_COMMUTATION_CALIBRATION_HOME_OFFSET;
            } else {
                cand_tx_request(CAND_ID_AZ, CAND_PID_COMMUTATION_CALIBRATION_INTERCEPT); // All parameter requests go to the AZ board
            }
            break;

        case LOAD_AXIS_PARMS_STATE_REQUEST_COMMUTATION_CALIBRATION_HOME_OFFSET:
            if (load_parms_state_info->init_param_recvd_flags_2 & INIT_PARAM_COMMUTATION_CALIBRATION_HOME_OFFSET_RECVD) {
                if (GetBoardHWID() == EL) {
                    // If we're the EL axis, we need to request all of the rate loop PID params
                    load_parms_state_info->load_axis_parms_state = LOAD_AXIS_PARMS_STATE_REQUEST_RATE_PID_EL;
                } else {
                    // Otherwise (we're the roll axis, AZ doesn't request parameters), we've received all the parameters we need, so we signal being done
                    load_parms_state_info->axis_parms_load_complete = TRUE;
                }
            } else {
                cand_tx_request(CAND_ID_AZ, CAND_PID_COMMUTATION_CALIBRATION_HOME_OFFSET); // All parameter requests go to the AZ board
            }
            break;

        case LOAD_AXIS_PARMS_STATE_REQUEST_RATE_PID_EL:
            if ((load_parms_state_info->init_param_recvd_flags_1 & ALL_EL_RATE_PID_INIT_PARAMS_RECVD) == ALL_EL_RATE_PID_INIT_PARAMS_RECVD) {
                load_parms_state_info->load_axis_parms_state = LOAD_AXIS_PARMS_STATE_REQUEST_RATE_PID_AZ;
            } else {
                // For rate loop pid params, request all 4 of them for an axis at once
                CAND_ParameterID request_params[4];
                request_params[0] = CAND_PID_RATE_EL_P;
                request_params[1] = CAND_PID_RATE_EL_I;
                request_params[2] = CAND_PID_RATE_EL_D;
                request_params[3] = CAND_PID_RATE_EL_WINDUP;

                cand_tx_multi_request(CAND_ID_AZ, request_params, 4); // All parameter requests go to the AZ board
            }
            break;

        case LOAD_AXIS_PARMS_STATE_REQUEST_RATE_PID_AZ:
            if ((load_parms_state_info->init_param_recvd_flags_1 & ALL_AZ_RATE_PID_INIT_PARAMS_RECVD) == ALL_AZ_RATE_PID_INIT_PARAMS_RECVD) {
                load_parms_state_info->load_axis_parms_state = LOAD_AXIS_PARMS_STATE_REQUEST_RATE_PID_ROLL;
            } else {
                // For rate loop pid params, request all 4 of them for an axis at once
                CAND_ParameterID request_params[4];
                request_params[0] = CAND_PID_RATE_AZ_P;
                request_params[1] = CAND_PID_RATE_AZ_I;
                request_params[2] = CAND_PID_RATE_AZ_D;
                request_params[3] = CAND_PID_RATE_AZ_WINDUP;

                cand_tx_multi_request(CAND_ID_AZ, request_params, 4); // All parameter requests go to the AZ board
            }
            break;

        case LOAD_AXIS_PARMS_STATE_REQUEST_RATE_PID_ROLL:
            if ((load_parms_state_info->init_param_recvd_flags_1 & ALL_ROLL_RATE_PID_INIT_PARAMS_RECVD) == ALL_ROLL_RATE_PID_INIT_PARAMS_RECVD) {
                load_parms_state_info->load_axis_parms_state = LOAD_AXIS_PARMS_STATE_REQUEST_POS_PID_EL;
            } else {
                // For rate loop pid params, request all 4 of them for an axis at once
                CAND_ParameterID request_params[4];
                request_params[0] = CAND_PID_RATE_RL_P;
                request_params[1] = CAND_PID_RATE_RL_I;
                request_params[2] = CAND_PID_RATE_RL_D;
                request_params[3] = CAND_PID_RATE_RL_WINDUP;

                cand_tx_multi_request(CAND_ID_AZ, request_params, 4); // All parameter requests go to the AZ board
            }
            break;

        case LOAD_AXIS_PARMS_STATE_REQUEST_POS_PID_EL:
            if ((load_parms_state_info->init_param_recvd_flags_3 & ALL_EL_POS_PID_INIT_PARAMS_RECVD) == ALL_EL_POS_PID_INIT_PARAMS_RECVD) {
                load_parms_state_info->load_axis_parms_state = LOAD_AXIS_PARMS_STATE_REQUEST_POS_PID_AZ;
            } else {
                // For position loop pid params, request all 4 of them for an axis at once
                CAND_ParameterID request_params[4];
                request_params[0] = CAND_PID_POS_EL_P;
                request_params[1] = CAND_PID_POS_EL_I;
                request_params[2] = CAND_PID_POS_EL_D;
                request_params[3] = CAND_PID_POS_EL_WINDUP;

                cand_tx_multi_request(CAND_ID_AZ, request_params, 4); // All parameter requests go to the AZ board
            }
            break;

        case LOAD_AXIS_PARMS_STATE_REQUEST_POS_PID_AZ:
            if ((load_parms_state_info->init_param_recvd_flags_3 & ALL_AZ_POS_PID_INIT_PARAMS_RECVD) == ALL_AZ_POS_PID_INIT_PARAMS_RECVD) {
                load_parms_state_info->load_axis_parms_state = LOAD_AXIS_PARMS_STATE_REQUEST_POS_PID_RL;
            } else {
                // For position loop pid params, request all 4 of them for an axis at once
                CAND_ParameterID request_params[4];
                request_params[0] = CAND_PID_POS_AZ_P;
                request_params[1] = CAND_PID_POS_AZ_I;
                request_params[2] = CAND_PID_POS_AZ_D;
                request_params[3] = CAND_PID_POS_AZ_WINDUP;

                cand_tx_multi_request(CAND_ID_AZ, request_params, 4); // All parameter requests go to the AZ board
            }
            break;

        case LOAD_AXIS_PARMS_STATE_REQUEST_POS_PID_RL:
            if ((load_parms_state_info->init_param_recvd_flags_3 & ALL_RL_POS_PID_INIT_PARAMS_RECVD) == ALL_RL_POS_PID_INIT_PARAMS_RECVD) {
                //load_parms_state_info->load_axis_parms_state = LOAD_AXIS_PARMS_STATE_REQUEST_GYRO_OFFSETS;
                // We've now received all of the parameters we're looking for, so signal being done
                load_parms_state_info->axis_parms_load_complete = TRUE;
            } else {
                // For position loop pid params, request all 4 of them for an axis at once
                CAND_ParameterID request_params[4];
                request_params[0] = CAND_PID_POS_RL_P;
                request_params[1] = CAND_PID_POS_RL_I;
                request_params[2] = CAND_PID_POS_RL_D;
                request_params[3] = CAND_PID_POS_RL_WINDUP;

                cand_tx_multi_request(CAND_ID_AZ, request_params, 4); // All parameter requests go to the AZ board
            }
            break;

        //TODO: Remove this or fix it to work with the new 16-bit parameters
        /*
        case LOAD_AXIS_PARMS_STATE_REQUEST_GYRO_OFFSETS:
            if ((load_parms_state_info->init_param_recvd_flags_3 & ALL_GYRO_OFFSET_INIT_PARAMS_RECVD) == ALL_GYRO_OFFSET_INIT_PARAMS_RECVD) {
                // We've now received all of the parameters we're looking for, so signal being done
                load_parms_state_info->axis_parms_load_complete = TRUE;
            } else {
                // For gyro offsets, request all 3 of them at once
                CAND_ParameterID request_params[3];
                request_params[0] = CAND_PID_GYRO_OFFSET_EL;
                request_params[1] = CAND_PID_GYRO_OFFSET_AZ;
                request_params[2] = CAND_PID_GYRO_OFFSET_RL;

                cand_tx_multi_request(CAND_ID_AZ, request_params, 3); // All parameter requests go to the AZ board
            }
            break;
        */
    }
}
