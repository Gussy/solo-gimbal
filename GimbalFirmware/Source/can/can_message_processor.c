#include "can/can_message_processor.h"
#include "PM_Sensorless-Settings.h"
#include "PM_Sensorless.h"
#include "can/cb.h"
#include "can/cand.h"
#include "hardware/device_init.h"
#include "parameters/flash_params.h"
#include "hardware/HWSpecific.h"
#include "control/PID.h"
#include "gopro/gopro_interface.h"
#include "mavlink_interface/mavlink_gimbal_interface.h"
#include "helpers/fault_handling.h"
#include "hardware/watchdog.h"
#include "flash/flash.h"
#include "hardware/watchdog.h"
#include "hardware/led.h"

#include <string.h>
#include <stdio.h>

void Process_CAN_Messages(AxisParms* axis_parms,
		MotorDriveParms* md_parms,
		ControlBoardParms* cb_parms,
		ParamSet* param_set,
		LoadAxisParmsStateInfo* load_ap_state_info)
{
    static int fault_cnt = 0;
    static int cmd_cnt = 0;
    static int param_set_cnt = 0;
    static int param_query_cnt = 0;
    static int param_response_cnt = 0;
    static struct cand_message msg;
    static CAND_Result rx;

    rx = cand_rx(&msg);
    switch (rx) {
    case CAND_RX_FAULT:
        // Remember the last fault from each axis
        cb_parms->last_axis_fault[msg.sender_id] = msg.fault_code;
        // Based on the type of fault, either put us into the correct fault mode (disables current to the motor), or just send the MAVLink fault message
#ifndef AZ_TEST
        switch (msg.fault_type) {
            case CAND_FAULT_TYPE_RECOVERABLE:
                md_parms->motor_drive_state = STATE_RECOVERABLE_FAULT;
                break;

            case CAND_FAULT_TYPE_UNRECOVERABLE:
                md_parms->motor_drive_state = STATE_UNRECOVERABLE_FAULT;
                break;
        }
#endif
        // If we're the AZ axis, we need to send a MAVLink fault message to the copter
        if (GetBoardHWID() == AZ) {
            send_mavlink_axis_error(msg.sender_id, msg.fault_code, msg.fault_type);
        }

        fault_cnt++;
        break;

    case CAND_RX_COMMAND:
        switch (msg.command) {
        case CAND_CMD_INIT:
            axis_parms->enable_flag = TRUE;
            md_parms->motor_drive_state = STATE_INIT;
            break;

        case CAND_CMD_ENABLE:
            if (md_parms->md_initialized) {
                axis_parms->enable_flag = TRUE;
                md_parms->motor_drive_state = STATE_RUNNING;
            }
            break;

        case CAND_CMD_RELAX:
            /* If a relax command is sent before the motor driver state machine has initialised,
             * let it finish initialising before disabling it otherwise the Roll board will not
             * send encoder values or the Elevation board will not call RunRateLoops.
             * Without this initialisaiton the gimbal won't send feedback messages, even if it's
             * moved out of the disabled state into a running state at a later time */
            if (!md_parms->md_initialized) {
                md_parms->motor_drive_state_after_initialisation = STATE_DISABLED;
            } else {
                md_parms->motor_drive_state = STATE_DISABLED;
            }
            break;

        case CAND_CMD_RESET:
        	axis_parms->enable_flag = FALSE;
        	md_parms->motor_drive_state = STATE_DISABLED;
        	if (GetBoardHWID() != AZ) {
        		// just making sure we are off
        		power_down_motor();

        		// enable watchdog and wait until it goes off
        		watchdog_immediate_reset();
        	}
        	break;

        case CAND_CMD_CALIBRATE_AXES:
            md_parms->motor_drive_state = STATE_TAKE_COMMUTATION_CALIBRATION_DATA;
            break;

        case CAND_CMD_POS_MODE:
        	cb_parms->control_type = CONTROL_TYPE_POS;
        	break;

        case CAND_CMD_RATE_MODE:
        	cb_parms->control_type = CONTROL_TYPE_RATE;
        	break;

        case CAND_CMD_GP_CHARGE_DISABLE:
        	gp_disable_charging();
        	break;

        case CAND_CMD_GP_CHARGE_ENABLE:
        	gp_enable_charging();
        	break;

        default:
            AxisFault(CAND_FAULT_UNSUPPORTED_COMMAND, CAND_FAULT_TYPE_INFO, cb_parms, md_parms);
            break;
        }
        cmd_cnt++;
        break;

    case CAND_RX_PARAM_SET:
        param_set_cnt++;

        // Check whether it's an extended parameter or not
        if (msg.param_cnt > 0) {
            if (msg.param_id[0] == CAND_PID_EXTENDED) {
                // Extended parameter, parse as such
                switch (msg.extended_param_id) {
                    case CAND_EPID_ENCODER_TELEMETRY:
                        if (msg.extended_param_length == 6) {
                            int16 az_encoder = ((msg.extended_param[0] << 8) & 0xFF00) | (msg.extended_param[1] & 0x00FF);
                            int16 el_encoder = ((msg.extended_param[2] << 8) & 0xFF00) | (msg.extended_param[3] & 0x00FF);
                            int16 rl_encoder = ((msg.extended_param[4] << 8) & 0xFF00) | (msg.extended_param[5] & 0x00FF);
                            receive_encoder_telemetry(az_encoder, el_encoder, rl_encoder);
                        }
                        break;

                    case CAND_EPID_TORQUE_CMD_TELEMETRY:
                    	if (msg.extended_param_length == 6) {
                    		int16 az_torque_cmd = ((((Uint16)msg.extended_param[0]) << 8) & 0xFF00) | (((Uint16)msg.extended_param[1]) & 0x00FF);
                    		int16 el_torque_cmd = ((((Uint16)msg.extended_param[2]) << 8) & 0xFF00) | (((Uint16)msg.extended_param[3]) & 0x00FF);
                    		int16 rl_torque_cmd = ((((Uint16)msg.extended_param[4]) << 8) & 0xFF00) | (((Uint16)msg.extended_param[5]) & 0x00FF);
                    		receive_torque_cmd_telemetry(az_torque_cmd, el_torque_cmd, rl_torque_cmd);
                    	}
                    	break;

                    case CAND_EPID_GYRO_AZ_TELEMETRY:
                        if (msg.extended_param_length == 4) {
                            int32 az_gyro = (((int32)msg.extended_param[0] << 24) & 0xFF000000) |
                                    (((int32)msg.extended_param[1] << 16) & 0x00FF0000) |
                                    (((int32)msg.extended_param[2] << 8) & 0x0000FF00) |
                                    ((int32)msg.extended_param[3] & 0x000000FF);
                            receive_gyro_az_telemetry(az_gyro);
                        }
                        break;

                    case CAND_EPID_GYRO_EL_TELEMETRY:
                        if (msg.extended_param_length == 4) {
                            int32 el_gyro = (((int32)msg.extended_param[0] << 24) & 0xFF000000) |
                                    (((int32)msg.extended_param[1] << 16) & 0x00FF0000) |
                                    (((int32)msg.extended_param[2] << 8) & 0x0000FF00) |
                                    ((int32)msg.extended_param[3] & 0x000000FF);
                            receive_gyro_el_telemetry(el_gyro);
                        }
                        break;

                    case CAND_EPID_GYRO_RL_TELEMETRY:
                        if (msg.extended_param_length == 4) {
                            int32 rl_gyro = (((int32)msg.extended_param[0] << 24) & 0xFF000000) |
                                    (((int32)msg.extended_param[1] << 16) & 0x00FF0000) |
                                    (((int32)msg.extended_param[2] << 8) & 0x0000FF00) |
                                    ((int32)msg.extended_param[3] & 0x000000FF);
                            receive_gyro_rl_telemetry(rl_gyro);
                        }
                        break;

                    case CAND_EPID_ACCEL_AZ_TELEMETRY:
                        if (msg.extended_param_length == 4) {
                            int32 az_accel = (((int32)msg.extended_param[0] << 24) & 0xFF000000) |
                                    (((int32)msg.extended_param[1] << 16) & 0x00FF0000) |
                                    (((int32)msg.extended_param[2] << 8) & 0x0000FF00) |
                                    ((int32)msg.extended_param[3] & 0x000000FF);
                            receive_accel_az_telemetry(az_accel);
                        }
                        break;

                    case CAND_EPID_ACCEL_EL_TELEMETRY:
                        if (msg.extended_param_length == 4) {
                            int32 el_accel = (((int32)msg.extended_param[0] << 24) & 0xFF000000) |
                                    (((int32)msg.extended_param[1] << 16) & 0x00FF0000) |
                                    (((int32)msg.extended_param[2] << 8) & 0x0000FF00) |
                                    ((int32)msg.extended_param[3] & 0x000000FF);
                            receive_accel_el_telemetry(el_accel);
                        }
                        break;

                    case CAND_EPID_ACCEL_RL_TELEMETRY:
                        if (msg.extended_param_length == 4) {
                            int32 rl_accel = (((int32)msg.extended_param[0] << 24) & 0xFF000000) |
                                    (((int32)msg.extended_param[1] << 16) & 0x00FF0000) |
                                    (((int32)msg.extended_param[2] << 8) & 0x0000FF00) |
                                    ((int32)msg.extended_param[3] & 0x000000FF);
                            receive_accel_rl_telemetry(rl_accel);
                        }
                        break;

                    case CAND_EPID_ARBITRARY_DEBUG:
                    {
                        //NOTE: Using this for whatever debug info I'm looking for at the moment,
                        //so this parsing will change whenver I change whatever data I'm sending
                    	/*
                        char debug_msg[50];
                        int16 error = (((Uint16)msg.extended_param[0] << 8) & 0xFF00) | ((Uint16)msg.extended_param[1] & 0x00FF);
                        int16 min = (((Uint16)msg.extended_param[2] << 8) & 0xFF00) | ((Uint16)msg.extended_param[3] & 0x00FF);
                        int16 encoder = (((Uint16)msg.extended_param[4] << 8) & 0xFF00) | ((Uint16)msg.extended_param[5] & 0x00FF);
                        snprintf(debug_msg, 50, "Error: %d, Limit: %d, Enc: %d", error, min, encoder);
                        send_mavlink_statustext(debug_msg, MAV_SEVERITY_DEBUG);
                        */
                    }
                    break;

                    case CAND_EPID_CALIBRATION_PROGRESS_EL:
                        if (msg.extended_param_length == 2) {
                            Uint8 calibration_progress = msg.extended_param[0] & 0x00FF;
                            GIMBAL_AXIS_CALIBRATION_STATUS calibration_status = (GIMBAL_AXIS_CALIBRATION_STATUS)(msg.extended_param[1] & 0x00FF);
                            send_mavlink_calibration_progress(calibration_progress, GIMBAL_AXIS_PITCH, calibration_status);
                        }
                        break;

                    case CAND_EPID_CALIBRATION_PROGRESS_RL:
                        if (msg.extended_param_length == 2) {
                            Uint8 calibration_progress = msg.extended_param[0] & 0x00FF;
                            GIMBAL_AXIS_CALIBRATION_STATUS calibration_status = (GIMBAL_AXIS_CALIBRATION_STATUS)(msg.extended_param[1] & 0x00FF);
                            send_mavlink_calibration_progress(calibration_progress, GIMBAL_AXIS_ROLL, calibration_status);
                        }
                        break;

                    case CAND_EPID_CALIBRATION_REQUIRED_STATUS:
                        if (msg.extended_param_length == 2) {
                            GIMBAL_AXIS_CALIBRATION_REQUIRED calibration_required_status = (GIMBAL_AXIS_CALIBRATION_REQUIRED)(msg.extended_param[0] & 0x00FF);
                            GimbalAxis axis = (GimbalAxis)(msg.extended_param[1] & 0x00FF);
                            cb_parms->calibration_status[axis] = calibration_required_status;
                        }
                        break;

                    case CAND_EPID_BEACON_CONTROL:
                        {
                            LED_RGBA color;

                            // Only makes sense to do this on elevation (since that's where the beacon is)
                            // Protect against errant messages to the wrong axis
                            if (GetBoardHWID() == EL) {
                                if (msg.extended_param_length == 6) {
                                    axis_parms->blink_state = BLINK_OVERRIDE;
                                    color.red = msg.extended_param[1];
                                    color.green = msg.extended_param[2];
                                    color.blue = msg.extended_param[3];
                                    color.alpha = msg.extended_param[4];
                                    led_set_mode((LED_MODE)(msg.extended_param[0]), color, msg.extended_param[5]);
                                }
                            }
                        }
                    	break;

                    case CAND_EPID_MAX_TORQUE:
                    	if (msg.extended_param_length == 2) {
                    		int16 max_torque = ((((Uint16)msg.extended_param[0]) << 8) & 0xFF00) | (((Uint16)msg.extended_param[1]) & 0x00FF);
                    		cb_parms->max_allowed_torque = max_torque;
                    	}

                    case CAND_EPID_PARAMS_LOAD:
                    	if (GetBoardHWID() == AZ) {
                    	    Uint8 i;
                    		// This means the parameter was a request, so load up the necessary data and broadcast it to the other two boards
                    		Uint16 start_offset = ((((Uint16)msg.extended_param[0]) >> 8) & 0x00FF) | (((Uint16)msg.extended_param[1]) & 0x00FF);
                    		CAND_DestinationID sender_id = (CAND_DestinationID)msg.extended_param[2];
                    		Uint16 params_size = sizeof(flash_params);
                    		Uint8 words_to_send = MIN(2, params_size - start_offset);
                    		Uint8 params[7];
                    		params[0] = msg.extended_param[0];
                    		params[1] = msg.extended_param[1];
                    		params[2] = words_to_send;
                    		// We need to do this instead of a memcpy to account for the fact that Uint8's on this architecture are actually 16-bits
                    		for (i = 0; i < words_to_send; i++) {
                    			params[(2 * i) + 3] = ((((Uint16*)(&flash_params))[start_offset + i]) >> 8) & 0x00FF;
                    			params[(2 * i) + 4] = ((((Uint16*)(&flash_params))[start_offset + i]) & 0x00FF);
                    		}
                    		cand_tx_extended_param(sender_id, CAND_EPID_PARAMS_LOAD, params, (words_to_send * 2) + 3);
                    	} else {
                    		// This means the parameter was a response
                    		// First make sure this isn't data we've already received, and if not, load it into our copy of the flash params struct
                    		Uint16 start_offset = ((((Uint16)msg.extended_param[0]) << 8) & 0xFF00) | (((Uint16)msg.extended_param[1]) & 0x00FF);
                    		Uint8 words_received = msg.extended_param[2];
                    		if (load_ap_state_info->current_load_offset == start_offset && (start_offset + words_received) <= sizeof(flash_params)) {
                    			// We need to do this instead of a memcpy to account for the fact that Uint8's on this architecture are actually 16-bits
                    		    Uint8 i;
                    			for (i = 0; i < words_received; i++) {
                    				Uint16 received_word = ((((Uint16)msg.extended_param[(2 * i) + 3]) << 8) & 0xFF00) | (((Uint16)msg.extended_param[(2 * i) + 4]) & 0x00FF);
                    				((Uint16*)(&flash_params))[start_offset + i] = received_word;
                    			}
                    			load_ap_state_info->current_load_offset += words_received;
                    		}

                    		// If we have received all of the params, preload the request_retry_counter to ask for the checksum immediately on the next cycle
                    		if (load_ap_state_info->current_load_offset == sizeof(flash_params)) {
                    		    load_ap_state_info->request_retry_counter = REQUEST_RETRY_PERIOD;
                    		}
                    	}
                    	break;

                    case CAND_EPID_PARAMS_CHECKSUM:
                    	if (GetBoardHWID() == AZ) {
                    		// This means the parameter was a request, so compute the checksum of the flash params struct and broadcast it
                    		CAND_DestinationID sender_id = (CAND_DestinationID)msg.extended_param[0];
                    		Uint16 checksum = compute_flash_params_checksum();
                    		Uint8 params[2];
                    		params[0] = ((checksum >> 8) & 0x00FF);
                    		params[1] = (checksum & 0x00FF);
                    		cand_tx_extended_param(sender_id, CAND_EPID_PARAMS_CHECKSUM, params, 2);
                    	} else {
                    		// This means the parameter was a response, so extract the checksum reply,
                    		// compare it against our own checksum of the flash params, and set the result appropriately
                    		Uint16 received_checksum = ((((Uint16)msg.extended_param[0]) << 8) & 0xFF00) | (((Uint16)msg.extended_param[1]) & 0x00FF);
                    		Uint16 calculated_checksum = compute_flash_params_checksum();
                    		if (received_checksum == calculated_checksum) {
                    			load_ap_state_info->axis_parms_checksum_verified = TRUE;
                    		} else {
                    			// If we failed checksum check, re-request the entire flash params struct
                    			load_ap_state_info->current_load_offset = 0;
                    			load_ap_state_info->current_request_load_offset = 0;
                    		}
                    	}
                    	break;
                }
            } else {
                // Not an extended parameter, parse normally
                while(msg.param_cnt) {
                    if (msg.param_id[msg.param_cnt-1] < CAND_PID_LAST) {
                        param_set[msg.param_id[msg.param_cnt-1]].param = msg.param[msg.param_cnt-1];
                        *(param_set[msg.param_id[msg.param_cnt-1]].sema) = TRUE;
                    }
                    msg.param_cnt--;
                }
            }
        }
        break;

    case CAND_RX_PARAM_QUERY:
        while(msg.param_request_cnt) {
            // Query messages can contain up to 8 parameter requests, for now, handle
            // one at a time. (can can send multiple later if this becomes an issue)
            switch (msg.param_request_id[msg.param_request_cnt-1]) {
            case CAND_PID_CORETEMP:
                //cand_tx_response(msg.sender_id, CAND_PID_CORETEMP, DegreesC);
                // TODO: Implement
                break;

            case CAND_PID_VOLTAGE:
                //CBSendVoltage(DcBusVoltage);
                // TODO: Implement
                break;

            case CAND_PID_BIT:
                if (msg.param_repeat) {
                    axis_parms->BIT_heartbeat_enable = TRUE;
                    axis_parms->BIT_heartbeat_decimate = 0;
                } else {
                    CBSendStatus();
                    axis_parms->BIT_heartbeat_enable = FALSE;
                }
                break;

            case CAND_PID_VERSION:
                //IFBSendVersionV2(&our_version);
                //TODO: Implement
                break;

            default:
                AxisFault(CAND_FAULT_UNSUPPORTED_PARAMETER, CAND_FAULT_TYPE_INFO, cb_parms, md_parms);
                break;
            }
            msg.param_request_cnt--;
        }
        param_query_cnt++;
        break;

    case CAND_RX_PARAM_RESPONSE:
        while (msg.param_response_cnt) {

            // Response messages can contain up to 4 parameter responses
            switch (msg.param_response_id[msg.param_response_cnt - 1]) {
                case CAND_PID_POSITION:
                {
                    int16 encoder_value = msg.param_response[msg.param_response_cnt - 1];

                    cb_parms->encoder_readings[msg.sender_id] = encoder_value;
                    cb_parms->encoder_value_received[msg.sender_id] = TRUE;
                }
                break;

                case CAND_PID_BIT:
                {
                    Uint8 bit_value = msg.param_response[msg.param_response_cnt - 1];

                    if (!axis_parms->other_axis_hb_recvd[msg.sender_id]) {
                        axis_parms->other_axis_hb_recvd[msg.sender_id] = TRUE;
                    }

                    if (bit_value & CAND_BIT_AXIS_HOMED) {
                        cb_parms->axes_homed[msg.sender_id] = TRUE;
                    }

                    if (bit_value & CAND_BIT_AXIS_PARMS_RECVD) {
                        axis_parms->other_axis_init_params_recvd[msg.sender_id] = TRUE;
                    }
                }
                break;

                case CAND_PID_GOPRO_HEARTBEAT:
                {
                	GPHeartbeatStatus gp_heartbeat_state = (GPHeartbeatStatus)msg.param_response[msg.param_response_cnt - 1];
                    send_mavlink_gopro_heartbeat(gp_heartbeat_state);
                }
                break;

                case CAND_PID_GOPRO_GET_RESPONSE:
                {
                	GPGetResponse gp_get_response;
					Uint32 packed_param = msg.param_response[msg.param_response_cnt - 1];
					gp_get_response.cmd_id = (packed_param >> 8) & 0x000000FF;
					gp_get_response.value = (packed_param >> 0) & 0x000000FF;
					send_mavlink_gopro_get_response(gp_get_response);
                }
                break;

                case CAND_PID_GOPRO_SET_RESPONSE:
                {
                	GPSetResponse gp_set_response;
                    Uint32 packed_param = msg.param_response[msg.param_response_cnt - 1];
                    gp_set_response.cmd_id = (packed_param >> 8) & 0x000000FF;
                    gp_set_response.result = (packed_param >> 0) & 0x000000FF;
                    send_mavlink_gopro_set_response(gp_set_response);
                }
                break;

                case CAND_PID_COMMUTATION_CALIBRATION_SLOPE:
                	if (GetBoardHWID() == AZ) {
                		// If we're the AZ board, this is new commuation calibration parameters coming back from a completed
                		// commutation calibration.  These need to be updated in flash
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                		flash_params.commutation_slope[msg.sender_id] = float_converter.float_val;
                	}
                    break;

                case CAND_PID_COMMUTATION_CALIBRATION_INTERCEPT:
                	if (GetBoardHWID() == AZ) {
                		// If we're the AZ board, this is new commuation calibration parameters coming back from a completed
						// commutation calibration.  These need to be updated in flash
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                		flash_params.commutation_icept[msg.sender_id] = float_converter.float_val;
                		// intercept comes after slope, so we commit the new values to flash here
                		write_flash();
                	}
                    break;
            }

            msg.param_response_cnt--;
        }
        param_response_cnt++;
        break;

    default:
        break;
    }
}
