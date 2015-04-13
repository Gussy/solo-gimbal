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

LoadParamEntry params_to_load[TOTAL_LOADABLE_PARAMS];

void InitAxisParmsLoader(LoadAxisParmsStateInfo* load_parms_state_info)
{
    // Populate the entries in the parameter load table
    params_to_load[0].request_param = CAND_PID_TORQUE_KP;
    params_to_load[0].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_2);
    params_to_load[0].recvd_flag_mask = INIT_PARAM_TORQUE_PID_KP_RECVD;

    params_to_load[1].request_param = CAND_PID_TORQUE_KI;
    params_to_load[1].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_2);
    params_to_load[1].recvd_flag_mask = INIT_PARAM_TORQUE_PID_KI_RECVD;

    params_to_load[2].request_param = CAND_PID_TORQUE_KD;
    params_to_load[2].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_2);
    params_to_load[2].recvd_flag_mask = INIT_PARAM_TORQUE_PID_KD_RECVD;

    params_to_load[3].request_param = CAND_PID_COMMUTATION_CALIBRATION_SLOPE;
    params_to_load[3].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_2);
    params_to_load[3].recvd_flag_mask = INIT_PARAM_COMMUTATION_CALIBRATION_SLOPE_RECVD;

    params_to_load[4].request_param = CAND_PID_COMMUTATION_CALIBRATION_INTERCEPT;
    params_to_load[4].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_2);
    params_to_load[4].recvd_flag_mask = INIT_PARAM_COMMUTATION_CALIBRATION_INTERCEPT_RECVD;

    params_to_load[5].request_param = CAND_PID_COMMUTATION_CALIBRATION_HOME_OFFSET;
    params_to_load[5].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_2);
    params_to_load[5].recvd_flag_mask = INIT_PARAM_COMMUTATION_CALIBRATION_HOME_OFFSET_RECVD;

    params_to_load[6].request_param = CAND_PID_RATE_EL_P;
    params_to_load[6].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_1);
    params_to_load[6].recvd_flag_mask = INIT_PARAM_RATE_PID_EL_P_RECVD;

    params_to_load[7].request_param = CAND_PID_RATE_EL_I;
    params_to_load[7].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_1);
    params_to_load[7].recvd_flag_mask = INIT_PARAM_RATE_PID_EL_I_RECVD;

    params_to_load[8].request_param = CAND_PID_RATE_EL_D;
    params_to_load[8].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_1);
    params_to_load[8].recvd_flag_mask = INIT_PARAM_RATE_PID_EL_D_RECVD;

    params_to_load[9].request_param = CAND_PID_RATE_EL_WINDUP;
    params_to_load[9].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_1);
    params_to_load[9].recvd_flag_mask = INIT_PARAM_RATE_PID_EL_WINDUP_RECVD;

    params_to_load[10].request_param = CAND_PID_RATE_AZ_P;
    params_to_load[10].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_1);
    params_to_load[10].recvd_flag_mask = INIT_PARAM_RATE_PID_AZ_P_RECVD;

    params_to_load[11].request_param = CAND_PID_RATE_AZ_I;
    params_to_load[11].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_1);
    params_to_load[11].recvd_flag_mask = INIT_PARAM_RATE_PID_AZ_I_RECVD;

    params_to_load[12].request_param = CAND_PID_RATE_AZ_D;
    params_to_load[12].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_1);
    params_to_load[12].recvd_flag_mask = INIT_PARAM_RATE_PID_AZ_D_RECVD;

    params_to_load[13].request_param = CAND_PID_RATE_AZ_WINDUP;
    params_to_load[13].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_1);
    params_to_load[13].recvd_flag_mask = INIT_PARAM_RATE_PID_AZ_WINDUP_RECVD;

    params_to_load[14].request_param = CAND_PID_RATE_RL_P;
    params_to_load[14].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_1);
    params_to_load[14].recvd_flag_mask = INIT_PARAM_RATE_PID_RL_P_RECVD;

    params_to_load[15].request_param = CAND_PID_RATE_RL_I;
    params_to_load[15].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_1);
    params_to_load[15].recvd_flag_mask = INIT_PARAM_RATE_PID_RL_I_RECVD;

    params_to_load[16].request_param = CAND_PID_RATE_RL_D;
    params_to_load[16].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_1);
    params_to_load[16].recvd_flag_mask = INIT_PARAM_RATE_PID_RL_D_RECVD;

    params_to_load[17].request_param = CAND_PID_RATE_RL_WINDUP;
    params_to_load[17].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_1);
    params_to_load[17].recvd_flag_mask = INIT_PARAM_RATE_PID_RL_WINDUP_RECVD;

    /*params_to_load[18].request_param = CAND_PID_POS_EL_P;
    params_to_load[18].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_3);
    params_to_load[18].recvd_flag_mask = INIT_PARAM_POS_PID_EL_P_RECVD;

    params_to_load[19].request_param = CAND_PID_POS_EL_I;
    params_to_load[19].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_3);
    params_to_load[19].recvd_flag_mask = INIT_PARAM_POS_PID_EL_I_RECVD;

    params_to_load[20].request_param = CAND_PID_POS_EL_D;
    params_to_load[20].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_3);
    params_to_load[20].recvd_flag_mask = INIT_PARAM_POS_PID_EL_D_RECVD;

    params_to_load[21].request_param = CAND_PID_POS_EL_WINDUP;
    params_to_load[21].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_3);
    params_to_load[21].recvd_flag_mask = INIT_PARAM_POS_PID_EL_WINDUP_RECVD;*/

    params_to_load[18].request_param = CAND_PID_POS_AZ_P;
    params_to_load[18].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_3);
    params_to_load[18].recvd_flag_mask = INIT_PARAM_POS_PID_AZ_P_RECVD;

    params_to_load[19].request_param = CAND_PID_POS_AZ_I;
    params_to_load[19].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_3);
    params_to_load[19].recvd_flag_mask = INIT_PARAM_POS_PID_AZ_I_RECVD;

    params_to_load[20].request_param = CAND_PID_POS_AZ_D;
    params_to_load[20].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_3);
    params_to_load[20].recvd_flag_mask = INIT_PARAM_POS_PID_AZ_D_RECVD;

    params_to_load[21].request_param = CAND_PID_POS_AZ_WINDUP;
    params_to_load[21].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_3);
    params_to_load[21].recvd_flag_mask = INIT_PARAM_POS_PID_AZ_WINDUP_RECVD;

    params_to_load[22].request_param = CAND_PID_POS_RL_P;
    params_to_load[22].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_3);
    params_to_load[22].recvd_flag_mask = INIT_PARAM_POS_PID_RL_P_RECVD;

    params_to_load[23].request_param = CAND_PID_POS_RL_I;
    params_to_load[23].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_3);
    params_to_load[23].recvd_flag_mask = INIT_PARAM_POS_PID_RL_I_RECVD;

    params_to_load[24].request_param = CAND_PID_POS_RL_D;
    params_to_load[24].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_3);
    params_to_load[24].recvd_flag_mask = INIT_PARAM_POS_PID_RL_D_RECVD;

    params_to_load[25].request_param = CAND_PID_POS_RL_WINDUP;
    params_to_load[25].recvd_flags_loc = &(load_parms_state_info->init_param_recvd_flags_3);
    params_to_load[25].recvd_flag_mask = INIT_PARAM_POS_PID_RL_WINDUP_RECVD;

    if (GetBoardHWID() == EL) {
        load_parms_state_info->total_params_to_load = EL_PARAMS_TO_LOAD;
    } else if (GetBoardHWID() == ROLL) {
        load_parms_state_info->total_params_to_load = RL_PARAMS_TO_LOAD;
    }
    // AZ doesn't load parameters over CAN
}

void LoadAxisParmsStateMachine(LoadAxisParmsStateInfo* load_parms_state_info)
{
    if (load_parms_state_info->current_param_to_load < load_parms_state_info->total_params_to_load) {
        LoadParamEntry* current_param_entry = &(params_to_load[load_parms_state_info->current_param_to_load]);

        // Check to see if we've received the current parameter we're asking for
        if (*(current_param_entry->recvd_flags_loc) & current_param_entry->recvd_flag_mask) {
            // We've received the parameter we're currently asking for, so increment the index of the parameter we're looking for
            load_parms_state_info->current_param_to_load++;
            // Preload the request retry counter so we immediately ask for the next parameter
            load_parms_state_info->request_retry_counter = REQUEST_RETRY_PERIOD;
        } else {
            if (load_parms_state_info->request_retry_counter++ >= REQUEST_RETRY_PERIOD) {
                // We haven't received the parameter we're currently asking for, so ask again
                cand_tx_request(CAND_ID_AZ, current_param_entry->request_param); // All parameter requests go to the AZ board
                load_parms_state_info->request_retry_counter = 0;
            }
        }
    } else {
        // If we've received all of the parameters in the parameter request list, we're done loading parameters
        load_parms_state_info->axis_parms_load_complete = TRUE;
    }
}
