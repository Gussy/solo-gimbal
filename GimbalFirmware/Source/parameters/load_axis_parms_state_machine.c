#include "parameters/load_axis_parms_state_machine.h"
#include "can/cand.h"
#include "can/cand_BitFields.h"
#include "parameters/flash_params.h"
#include "hardware/device_init.h"
#include "hardware/led.h"
#include "can/cb.h"
#include "PM_Sensorless-Settings.h"

void InitAxisParmsLoader(LoadAxisParmsStateInfo* load_parms_state_info)
{
	load_parms_state_info->total_words_to_load = sizeof(flash_params);
	load_parms_state_info->current_load_offset = 0;
	load_parms_state_info->current_request_load_offset = 0;
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
		load_parms_state_info->axis_parms_load_complete = TRUE;
	}
}
