#include "parameters/load_axis_parms_state_machine.h"
#include "can/cand.h"
#include "can/cand_BitFields.h"
#include "parameters/flash_params.h"
#include "hardware/device_init.h"
#include "hardware/led.h"
#include "can/cb.h"
#include "PM_Sensorless-Settings.h"

LoadParamEntry params_to_load[TOTAL_LOADABLE_PARAMS];

#define INIT_PARAM(i,flag,can_id,mask) params_to_load[i].request_param = can_id;params_to_load[i].recvd_flags_loc = &(load_parms_state_info->flag);params_to_load[i].recvd_flag_mask = mask;


void InitAxisParmsLoader(LoadAxisParmsStateInfo* load_parms_state_info)
{
    // Populate the entries in the parameter load table
	INIT_PARAM( 0,init_param_recvd_flags_2, CAND_PID_TORQUE_KP,INIT_PARAM_TORQUE_PID_KP_RECVD);
	INIT_PARAM( 1,init_param_recvd_flags_2, CAND_PID_TORQUE_KI,INIT_PARAM_TORQUE_PID_KI_RECVD);
	INIT_PARAM( 2,init_param_recvd_flags_2, CAND_PID_TORQUE_KD,INIT_PARAM_TORQUE_PID_KD_RECVD);
	INIT_PARAM( 3,init_param_recvd_flags_2, CAND_PID_COMMUTATION_CALIBRATION_SLOPE,INIT_PARAM_COMMUTATION_CALIBRATION_SLOPE_RECVD);
	INIT_PARAM( 4,init_param_recvd_flags_2, CAND_PID_COMMUTATION_CALIBRATION_INTERCEPT,INIT_PARAM_COMMUTATION_CALIBRATION_INTERCEPT_RECVD);
	INIT_PARAM( 5,init_param_recvd_flags_2, CAND_PID_COMMUTATION_CALIBRATION_HOME_OFFSET,INIT_PARAM_COMMUTATION_CALIBRATION_HOME_OFFSET_RECVD);
	INIT_PARAM( 6,init_param_recvd_flags_1, CAND_PID_RATE_EL_P,INIT_PARAM_RATE_PID_EL_P_RECVD);
	INIT_PARAM( 7,init_param_recvd_flags_1, CAND_PID_RATE_EL_I,INIT_PARAM_RATE_PID_EL_I_RECVD);
	INIT_PARAM( 8,init_param_recvd_flags_1, CAND_PID_RATE_EL_D,INIT_PARAM_RATE_PID_EL_D_RECVD);
	INIT_PARAM( 9,init_param_recvd_flags_1, CAND_PID_RATE_EL_WINDUP,INIT_PARAM_RATE_PID_EL_WINDUP_RECVD);
	INIT_PARAM(10,init_param_recvd_flags_1, CAND_PID_RATE_AZ_P,INIT_PARAM_RATE_PID_AZ_P_RECVD);
	INIT_PARAM(11,init_param_recvd_flags_1, CAND_PID_RATE_AZ_I,INIT_PARAM_RATE_PID_AZ_I_RECVD);
	INIT_PARAM(12,init_param_recvd_flags_1, CAND_PID_RATE_AZ_D,INIT_PARAM_RATE_PID_AZ_D_RECVD);
	INIT_PARAM(13,init_param_recvd_flags_1, CAND_PID_RATE_AZ_WINDUP,INIT_PARAM_RATE_PID_AZ_WINDUP_RECVD);
	INIT_PARAM(14,init_param_recvd_flags_1, CAND_PID_RATE_RL_P,INIT_PARAM_RATE_PID_RL_P_RECVD);
	INIT_PARAM(15,init_param_recvd_flags_1, CAND_PID_RATE_RL_I,INIT_PARAM_RATE_PID_RL_I_RECVD);
	INIT_PARAM(16,init_param_recvd_flags_1, CAND_PID_RATE_RL_D,INIT_PARAM_RATE_PID_RL_D_RECVD);
	INIT_PARAM(17,init_param_recvd_flags_1, CAND_PID_RATE_RL_WINDUP,INIT_PARAM_RATE_PID_RL_WINDUP_RECVD);

	load_parms_state_info->total_words_to_load = sizeof(flash_params);
	load_parms_state_info->current_load_offset = 0;
	load_parms_state_info->current_request_load_offset = 0;

    if (GetBoardHWID() == EL) {
        load_parms_state_info->total_params_to_load = TOTAL_LOADABLE_PARAMS;
    } else if (GetBoardHWID() == ROLL) {
        load_parms_state_info->total_params_to_load = TOTAL_LOADABLE_PARAMS;
    }
    // AZ doesn't load parameters over CAN
}

void LoadAxisParmsStateMachine(LoadAxisParmsStateInfo* load_parms_state_info)
{
	if (load_parms_state_info->current_load_offset < load_parms_state_info->total_words_to_load) {
		if (load_parms_state_info->current_load_offset != load_parms_state_info->current_request_load_offset) {
			// We've received the last batch of param data that we requested, so request the next batch
			load_parms_state_info->current_request_load_offset += MIN(2, load_parms_state_info->total_words_to_load - load_parms_state_info->current_load_offset);

			// Preload the request retry counter so we immediately ask for the next batch of param data
			load_parms_state_info->request_retry_counter = REQUEST_RETRY_PERIOD;
		} else {
			// We haven't received the batch of param data we've asked for yet.  Ask again at a periodic rate
			if (load_parms_state_info->request_retry_counter++ >= REQUEST_RETRY_PERIOD) {
				// Send request
				Uint8 params[3];
				params[0] = (load_parms_state_info->current_request_load_offset >> 8) & 0x00FF;
				params[1] = (load_parms_state_info->current_request_load_offset & 0x00FF);
				params[2] = GetBoardHWID();
				cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_PARAMS_LOAD, params, 3);

				load_parms_state_info->request_retry_counter = 0;
			}
		}
	} else if (!load_parms_state_info->axis_parms_checksum_verified) {
		// Keep periodically requesting the checksum until we get it
		if (load_parms_state_info->request_retry_counter++ >= REQUEST_RETRY_PERIOD) {
			Uint8 param = GetBoardHWID();
			cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_PARAMS_CHECKSUM, &param, 1);
			load_parms_state_info->request_retry_counter = 0;
		}
	} else {
		Uint8 params[3];
		params[0] = GetBoardHWID();
		params[1] = 0x12;
		params[2] = 0x34;
		cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_ARBITRARY_DEBUG, params, 3);
		load_parms_state_info->axis_parms_load_complete = TRUE;
	}

	/*
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
    */
}
