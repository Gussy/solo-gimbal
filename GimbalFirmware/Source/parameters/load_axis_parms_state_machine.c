#include "parameters/load_axis_parms_state_machine.h"
#include "can/cand.h"
#include "can/cand_BitFields.h"
#include "parameters/kvstore.h"
#include "hardware/device_init.h"
#include "hardware/led.h"
#include "can/cb.h"
#include "PM_Sensorless-Settings.h"

void InitAxisParmsLoader(LoadAxisParmsStateInfo* load_parms_state_info)
{
	load_parms_state_info->total_keys_to_load = FLASH_PARAM_KEY_COUNT;
	load_parms_state_info->current_key = 0;
	load_parms_state_info->current_request_key = 0;
}

void LoadAxisParmsStateMachine(LoadAxisParmsStateInfo* load_parms_state_info)
{
    // Step 1 - kvstore transmitting
	if (load_parms_state_info->current_key < load_parms_state_info->total_keys_to_load) {
		if (load_parms_state_info->current_key != load_parms_state_info->current_request_key) {
			// We've received the last batch of param data that we requested, so request the next batch
			load_parms_state_info->current_request_key++;

			// Pre-load the retry counter so we immediately ask for the next batch of param data
            load_parms_state_info->request_retry_counter = REQUEST_RETRY_PERIOD;
		}

        // We haven't received the batch of param data we've asked for yet. Ask again at a periodic rate.
        if (load_parms_state_info->request_retry_counter++ >= REQUEST_RETRY_PERIOD) {
            // Send a request for the key and value
            uint8_t params[3];
            params[0] = (load_parms_state_info->current_key >> 8) & 0x00FF;
            params[1] = (load_parms_state_info->current_key & 0x00FF);
            params[2] = GetBoardHWID();
            cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_KVSTORE_LOAD, params, ARRAY_LENGTH(params));

            load_parms_state_info->request_retry_counter = 0;
        }
    // Step 2 - kvstore header
	} else if (!load_parms_state_info->header_received) {
		// Keep periodically requesting the header until we get it
		if (load_parms_state_info->request_retry_counter++ >= REQUEST_RETRY_PERIOD) {
			uint8_t param = GetBoardHWID();
			cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_KVSTORE_HEADER, &param, 1);
			load_parms_state_info->request_retry_counter = 0;
		}
    // Step 3 - load complete
	} else {
		load_parms_state_info->load_complete = true;
	}
}
