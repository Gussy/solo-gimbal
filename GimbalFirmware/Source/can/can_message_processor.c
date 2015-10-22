#include "can/can_message_processor.h"
#include "PM_Sensorless-Settings.h"
#include "PM_Sensorless.h"
#include "can/cb.h"
#include "can/cand.h"
#include "hardware/device_init.h"
#include "parameters/kvstore.h"
#include "hardware/HWSpecific.h"
#include "control/PID.h"
#include "gopro/gopro_interface.h"
#include "mavlink_interface/mavlink_gimbal_interface.h"
#include "parameters/mavlink_parameter_interface.h"
#include "helpers/fault_handling.h"
#include "hardware/watchdog.h"
#include "flash/flash.h"
#include "hardware/watchdog.h"
#include "hardware/led.h"
#include "motor/motor_drive_state_machine.h"

#include <string.h>
#include <stdio.h>

float float_from_uint8_t_array(uint8_t* in) {
    IntOrFloat float_converter;
    float_converter.uint32_val = (((Uint32)in[3] << 24) & 0xFF000000) | (((Uint32)in[2] << 16) & 0x00FF0000) | (((Uint32)in[1] << 8) & 0x0000FF00) | ((Uint32)in[0] & 0x000000FF);
    return float_converter.float_val;
}

void Process_CAN_Messages(AxisParms* axis_parms,
		MotorDriveParms* md_parms,
		ControlBoardParms* cb_parms,
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
            set_axis_enable(true);
            md_parms->motor_drive_state = STATE_INIT;
            break;

        case CAND_CMD_ENABLE:
            if (md_parms->md_initialized) {
                set_axis_enable(true);
                md_parms->motor_drive_state = STATE_RUNNING;
            } else {
                md_parms->motor_drive_state_after_initialisation = STATE_RUNNING;
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
            set_axis_enable(false);
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

                    case CAND_EPID_DEL_ANG_AZ_TELEMETRY:
                        if (msg.extended_param_length == 4) {
                            float del_ang = float_from_uint8_t_array(msg.extended_param);
                            receive_del_ang_az_telemetry(del_ang);
                        }
                        break;

                    case CAND_EPID_DEL_ANG_EL_TELEMETRY:
                        if (msg.extended_param_length == 4) {
                            float del_ang = float_from_uint8_t_array(msg.extended_param);
                            receive_del_ang_el_telemetry(del_ang);
                        }
                        break;

                    case CAND_EPID_DEL_ANG_RL_TELEMETRY:
                        if (msg.extended_param_length == 4) {
                            float del_ang = float_from_uint8_t_array(msg.extended_param);
                            receive_del_ang_rl_telemetry(del_ang);
                        }
                        break;

                    case CAND_EPID_DEL_VEL_AZ_TELEMETRY:
                        if (msg.extended_param_length == 4) {
                            float del_vel = float_from_uint8_t_array(msg.extended_param);
                            receive_del_vel_az_telemetry(del_vel);
                        }
                        break;

                    case CAND_EPID_DEL_VEL_EL_TELEMETRY:
                        if (msg.extended_param_length == 4) {
                            float del_vel = float_from_uint8_t_array(msg.extended_param);
                            receive_del_vel_el_telemetry(del_vel);
                        }
                        break;

                    case CAND_EPID_DEL_VEL_RL_TELEMETRY:
                        if (msg.extended_param_length == 4) {
                            float del_vel = float_from_uint8_t_array(msg.extended_param);
                            receive_del_vel_rl_telemetry(del_vel);
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

                        case CAND_EPID_KVSTORE_LOAD:
                            if (GetBoardHWID() == AZ) { // The parameter was a request, so load up the necessary data and broadcast it to the other two boards
                                uint16_t key_to_send = ((((Uint16)msg.extended_param[0]) >> 8) & 0x00FF) | (((Uint16)msg.extended_param[1]) & 0x00FF);
                                CAND_DestinationID sender_id = (CAND_DestinationID)msg.extended_param[2];
                                kv_value_t value_to_send = kvstore_get_value((flash_param_keys_t)key_to_send);

                                /* Packet format is as follows:
                                * byte:0...1   - uint8_t key[2]
                                * byte:2...5   - uint8_t value[4]
                                */

                                uint8_t params[7];
                                params[0] = msg.extended_param[0];
                                params[1] = msg.extended_param[1];
                                params[2] = value_to_send.as_bytes[0];
                                params[3] = value_to_send.as_bytes[1];
                                params[4] = value_to_send.as_bytes[2];
                                params[5] = value_to_send.as_bytes[3];

                                cand_tx_extended_param(sender_id, CAND_EPID_KVSTORE_LOAD, params, ARRAY_LENGTH(params));
                            } else { // This means the parameter was a response
                                // First make sure this isn't data we've already received, and if not, load it into our copy of the flash params struct
                                uint16_t received_key = ((((Uint16)msg.extended_param[0]) >> 8) & 0x00FF) | (((Uint16)msg.extended_param[1]) & 0x00FF);
                                if (load_ap_state_info->current_key == received_key && received_key <= load_ap_state_info->total_keys_to_load) {
                                    kv_value_t recevied_value;
                                    recevied_value.as_bytes[0] = msg.extended_param[2];
                                    recevied_value.as_bytes[0] = msg.extended_param[3];
                                    recevied_value.as_bytes[0] = msg.extended_param[4];
                                    recevied_value.as_bytes[0] = msg.extended_param[5];

                                    kvstore_put_value((flash_param_keys_t)received_key, recevied_value);

                                    load_ap_state_info->current_key++;
                                }

                                // If we have received all of the params, preload the request_retry_counter to ask for the checksum immediately on the next cycle
                                if (load_ap_state_info->current_key == load_ap_state_info->total_keys_to_load) {
                                    load_ap_state_info->request_retry_counter = REQUEST_RETRY_PERIOD;
                                }
                            }
                            break;

                        case CAND_EPID_KVSTORE_HEADER:
                            if (GetBoardHWID() == AZ) { // The parameter was a request
                                CAND_DestinationID sender_id = (CAND_DestinationID)msg.extended_param[0];

                                kvstore_header_t header;
                                kvstore_get_header(&header);

                                uint8_t params[7];

                                // Magic
                                params[0] = ((header.magic >> 8) & 0x00FF);
                                params[1] = ((header.magic >> 0) & 0x00FF);

                                // Unused[0]
                                params[2] = ((header.unused[0] >> 8) & 0x00FF);
                                params[3] = ((header.unused[0] >> 0) & 0x00FF);

                                // Unused[1]
                                params[4] = ((header.unused[1] >> 8) & 0x00FF);
                                params[5] = ((header.unused[1] >> 0) & 0x00FF);

                                cand_tx_extended_param(sender_id, CAND_EPID_KVSTORE_HEADER, params, ARRAY_LENGTH(params));
                            } else { // This means the parameter was a response
                                // TODO: Add a kvstore checksum the the header and check it here
                                load_ap_state_info->header_received = true;
                            }
                            break;

                    case CAND_EPID_MAVLINK_PARAM:
                        if (msg.extended_param_length == 5 && GetBoardHWID() != AZ) {
                            GimbalMavlinkParameterID param_id = (GimbalMavlinkParameterID)msg.extended_param[0];

                            Uint32 value = 0;
                            value |= ((Uint32)msg.extended_param[1] << 24) & 0xFF000000;
                            value |= ((Uint32)msg.extended_param[2] << 16) & 0x00FF0000;
                            value |= ((Uint32)msg.extended_param[3] <<  8) & 0x0000FF00;
                            value |= ((Uint32)msg.extended_param[4] <<  0) & 0x000000FF;

                            // Update the parameter in flash_params then copy it to the local values
                            gimbal_param_update(param_id, value, cb_parms);
                            update_local_params_from_kvstore(md_parms);
                        }
                        break;

                    case CAND_PID_GOPRO_SET_REQUEST:
                        if (msg.extended_param_length <= sizeof(gp_can_mav_set_req_t) && GetBoardHWID() == EL) {
                            gp_can_mav_set_req_t setreq;
                            memcpy(setreq.bytes, msg.extended_param, msg.extended_param_length);
                            gp_set_request(&setreq);
                        }
                        break;

                    case CAND_PID_GOPRO_GET_REQUEST:
                        if (msg.extended_param_length <= sizeof(gp_can_mav_get_req_t) && GetBoardHWID() == EL) {
                            gp_can_mav_get_req_t req;
                            memcpy(req.bytes, msg.extended_param, msg.extended_param_length);
                            gp_get_request(&req, false);
                        }
                        break;

                    case CAND_PID_GOPRO_GET_RESPONSE:
                        if (msg.extended_param_length <= sizeof(gp_can_mav_get_rsp_t) && GetBoardHWID() == AZ) {
                            gp_can_mav_get_rsp_t rsp;
                            memcpy(rsp.bytes, msg.extended_param, msg.extended_param_length);
                            send_mavlink_gopro_get_response(&rsp);
                        }
                        break;

                    case CAND_PID_GOPRO_SET_RESPONSE:
                        if (msg.extended_param_length <= sizeof(gp_can_mav_set_rsp_t) && GetBoardHWID() == AZ) {
                            gp_can_mav_set_rsp_t rsp;
                            memcpy(rsp.bytes, msg.extended_param, msg.extended_param_length);
                            send_mavlink_gopro_set_response(&rsp);
                        }
                        break;

                    case CAND_PID_GOPRO_HEARTBEAT:
                        if (msg.extended_param_length == sizeof(gp_can_mav_heartbeat_t) && GetBoardHWID() == AZ) {
                            gp_can_mav_heartbeat_t hb;
                            memcpy(hb.bytes, msg.extended_param, msg.extended_param_length);
                            send_mavlink_gopro_heartbeat(&hb);
                        }
                        break;
                }
            } else {
                // Not an extended parameter, parse normally
                while(msg.param_cnt) {
                    if (msg.param_id[msg.param_cnt-1] < CAND_PID_LAST) {
                        cb_parms->param_set[msg.param_id[msg.param_cnt-1]].param = msg.param[msg.param_cnt-1];
                        cb_parms->param_set[msg.param_id[msg.param_cnt-1]].sema = true;
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

                    // Only roll and elevation are used in update_joint_ang_trig,
                    // and elevation is not received on CAN
                    if(msg.sender_id == ROLL) {
                        update_joint_ang_trig(cb_parms->encoder_readings);
                    }
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

                case CAND_PID_COMMUTATION_CALIBRATION_SLOPE:
                	if (GetBoardHWID() == AZ) {
                		// If we're the AZ board, this is new commuation calibration parameters coming back from a completed
                		// commutation calibration.  These need to be updated in flash
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                		switch(msg.sender_id) {
                		    case AZ:
                		        kvstore_put_float(FLASH_PARAM_COMMUTATION_SLOPE_AZ, float_converter.float_val);
                		        break;
                		    case ROLL:
                                kvstore_put_float(FLASH_PARAM_COMMUTATION_SLOPE_ROLL, float_converter.float_val);
                                break;
                		    case EL:
                                kvstore_put_float(FLASH_PARAM_COMMUTATION_SLOPE_EL, float_converter.float_val);
                                break;
                		}
                	}
                    break;

                case CAND_PID_COMMUTATION_CALIBRATION_INTERCEPT:
                	if (GetBoardHWID() == AZ) {
                		// If we're the AZ board, this is new commuation calibration parameters coming back from a completed
						// commutation calibration.  These need to be updated in flash
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                		switch(msg.sender_id) {
                            case AZ:
                                kvstore_put_float(FLASH_PARAM_COMMUTATION_ICEPT_AZ, float_converter.float_val);
                                break;
                            case ROLL:
                                kvstore_put_float(FLASH_PARAM_COMMUTATION_ICEPT_ROLL, float_converter.float_val);
                                break;
                            case EL:
                                kvstore_put_float(FLASH_PARAM_COMMUTATION_ICEPT_EL, float_converter.float_val);
                                break;
                        }
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
