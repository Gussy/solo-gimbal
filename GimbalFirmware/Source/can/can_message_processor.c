/*
 * can_message_processor.c
 *
 *  Created on: Jan 7, 2015
 *      Author: abamberger
 */

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

void WDogEnable(void)
{
    EALLOW;
    SysCtrlRegs.WDCR = 0x0028;               // Enable watchdog module
    SysCtrlRegs.WDKEY = 0x55;                // Clear the WD counter
    SysCtrlRegs.WDKEY = 0xAA;
    EDIS;
}

void Process_CAN_Messages(AxisParms* axis_parms, MotorDriveParms* md_parms, ControlBoardParms* cb_parms, ParamSet* param_set, LoadAxisParmsStateInfo* load_ap_state_info)
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
        // Put us into fault mode (disables current to the motor)
#ifndef AZ_TEST
        md_parms->motor_drive_state = STATE_FAULT;
#endif
        fault_cnt++;
        break;

    case CAND_RX_COMMAND:
        switch (msg.command) {
        case CAND_CMD_ENABLE:
            axis_parms->enable_flag = TRUE;
            md_parms->motor_drive_state = STATE_INIT;
            axis_parms->blink_state = BLINK_INIT;
            break;

        case CAND_CMD_RELAX:
            //EnableFlag = 0;
        	axis_parms->enable_flag = FALSE;
            md_parms->motor_drive_state = STATE_DISABLED;
            axis_parms->blink_state = BLINK_READY;
    		power_down_motor();
            break;

        case CAND_CMD_RESET:
        	/* only reset if in relaxed state */
        	if ((axis_parms->enable_flag == FALSE)&&
        		(md_parms->motor_drive_state == STATE_DISABLED)&&
            	(axis_parms->blink_state == BLINK_READY)&&
            	(GetBoardHWID() != AZ)) {
        		// just making sure we are off
        		power_down_motor();
        		// enable watchdog and wait until it goes off
        		WDogEnable();
        		while (1);
        	}
        	break;

        case CAND_CMD_GOPRO_ON:
            gp_request_power_on();
            break;

        case CAND_CMD_GOPRO_OFF:
            gp_request_power_off();
            break;

        default:
            AxisFault(CAND_FAULT_UNSUPPORTED_COMMAND);
            break;
        }
        cmd_cnt++;
        break;

    case CAND_RX_PARAM_SET:
        param_set_cnt++;
        while(msg.param_cnt) {
            if (msg.param_id[msg.param_cnt-1] < CAND_PID_LAST) {
                param_set[msg.param_id[msg.param_cnt-1]].param = msg.param[msg.param_cnt-1];
                *(param_set[msg.param_id[msg.param_cnt-1]].sema) = TRUE;
            }
            msg.param_cnt--;
        }
        cb_parms->angle_targets[AZ]   = param_set[CAND_PID_TARGET_ANGLES_AZ].param;
        cb_parms->angle_targets[EL]   = param_set[CAND_PID_TARGET_ANGLES_EL].param;
        cb_parms->angle_targets[ROLL] = param_set[CAND_PID_TARGET_ANGLES_ROLL].param;
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

            case CAND_PID_TORQUE_KP:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    float_converter.float_val = flash_params.torque_pid_kp[msg.sender_id];
                    cand_tx_response(msg.sender_id, CAND_PID_TORQUE_KP, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_TORQUE_KI:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    float_converter.float_val = flash_params.torque_pid_ki[msg.sender_id];
                    cand_tx_response(msg.sender_id, CAND_PID_TORQUE_KI, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_TORQUE_KD:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    float_converter.float_val = flash_params.torque_pid_kd[msg.sender_id];
                    cand_tx_response(msg.sender_id, CAND_PID_TORQUE_KD, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_COMMUTATION_CALIBRATION_SLOPE:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    float_converter.float_val = flash_params.AxisCalibrationSlopes[msg.sender_id];
                    cand_tx_response(msg.sender_id, CAND_PID_COMMUTATION_CALIBRATION_SLOPE, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_COMMUTATION_CALIBRATION_INTERCEPT:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    float_converter.float_val = flash_params.AxisCalibrationIntercepts[msg.sender_id];
                    cand_tx_response(msg.sender_id, CAND_PID_COMMUTATION_CALIBRATION_INTERCEPT, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_COMMUTATION_CALIBRATION_HOME_OFFSET:
            {
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    cand_tx_response(msg.sender_id, CAND_PID_COMMUTATION_CALIBRATION_HOME_OFFSET, (int)(flash_params.AxisHomePositions[msg.sender_id]));
                }
            }
            break;

            case CAND_PID_RATE_EL_P:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Rate loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.rate_pid_p[EL];
                    cand_tx_response(msg.sender_id, CAND_PID_RATE_EL_P, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_RATE_EL_I:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Rate loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.rate_pid_i[EL];
                    cand_tx_response(msg.sender_id, CAND_PID_RATE_EL_I, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_RATE_EL_D:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Rate loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.rate_pid_d[EL];
                    cand_tx_response(msg.sender_id, CAND_PID_RATE_EL_D, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_RATE_EL_WINDUP:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Rate loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.rate_pid_windup[EL];
                    cand_tx_response(msg.sender_id, CAND_PID_RATE_EL_WINDUP, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_RATE_AZ_P:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Rate loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.rate_pid_p[AZ];
                    cand_tx_response(msg.sender_id, CAND_PID_RATE_AZ_P, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_RATE_AZ_I:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Rate loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.rate_pid_i[AZ];
                    cand_tx_response(msg.sender_id, CAND_PID_RATE_AZ_I, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_RATE_AZ_D:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Rate loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.rate_pid_d[AZ];
                    cand_tx_response(msg.sender_id, CAND_PID_RATE_AZ_D, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_RATE_AZ_WINDUP:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Rate loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.rate_pid_windup[AZ];
                    cand_tx_response(msg.sender_id, CAND_PID_RATE_AZ_WINDUP, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_RATE_RL_P:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Rate loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.rate_pid_p[ROLL];
                    cand_tx_response(msg.sender_id, CAND_PID_RATE_RL_P, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_RATE_RL_I:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Rate loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.rate_pid_i[ROLL];
                    cand_tx_response(msg.sender_id, CAND_PID_RATE_RL_I, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_RATE_RL_D:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Rate loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.rate_pid_d[ROLL];
                    cand_tx_response(msg.sender_id, CAND_PID_RATE_RL_D, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_RATE_RL_WINDUP:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Rate loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.rate_pid_windup[ROLL];
                    cand_tx_response(msg.sender_id, CAND_PID_RATE_RL_WINDUP, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_POS_EL_P:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Position loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.pos_pid_p[EL];
                    cand_tx_response(msg.sender_id, CAND_PID_POS_EL_P, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_POS_EL_I:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Position loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.pos_pid_i[EL];
                    cand_tx_response(msg.sender_id, CAND_PID_POS_EL_I, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_POS_EL_D:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Position loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.pos_pid_d[EL];
                    cand_tx_response(msg.sender_id, CAND_PID_POS_EL_D, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_POS_EL_WINDUP:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Position loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.pos_pid_windup[EL];
                    cand_tx_response(msg.sender_id, CAND_PID_POS_EL_WINDUP, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_POS_AZ_P:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Position loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.pos_pid_p[AZ];
                    cand_tx_response(msg.sender_id, CAND_PID_POS_AZ_P, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_POS_AZ_I:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Position loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.pos_pid_i[AZ];
                    cand_tx_response(msg.sender_id, CAND_PID_POS_AZ_I, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_POS_AZ_D:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Position loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.pos_pid_d[AZ];
                    cand_tx_response(msg.sender_id, CAND_PID_POS_AZ_D, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_POS_AZ_WINDUP:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Position loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.pos_pid_windup[AZ];
                    cand_tx_response(msg.sender_id, CAND_PID_POS_AZ_WINDUP, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_POS_RL_P:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Position loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.pos_pid_p[ROLL];
                    cand_tx_response(msg.sender_id, CAND_PID_POS_RL_P, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_POS_RL_I:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Position loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.pos_pid_i[ROLL];
                    cand_tx_response(msg.sender_id, CAND_PID_POS_RL_I, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_POS_RL_D:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Position loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.pos_pid_d[ROLL];
                    cand_tx_response(msg.sender_id, CAND_PID_POS_RL_D, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_POS_RL_WINDUP:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Position loop parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.pos_pid_windup[ROLL];
                    cand_tx_response(msg.sender_id, CAND_PID_POS_RL_WINDUP, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_GYRO_OFFSET_EL:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Gyro offset parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.gyro_offsets[EL];
                    cand_tx_response(msg.sender_id, CAND_PID_GYRO_OFFSET_EL, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_GYRO_OFFSET_AZ:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Gyro offset parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.gyro_offsets[AZ];
                    cand_tx_response(msg.sender_id, CAND_PID_GYRO_OFFSET_AZ, float_converter.uint32_val);
                }
            }
            break;

            case CAND_PID_GYRO_OFFSET_RL:
            {
                IntOrFloat float_converter;
                // Flash paramters only live on the AZ board, so only AZ should be responding to requests for parameters
                if (GetBoardHWID() == AZ) {
                    // Gyro offset parameter requests all come from EL, so can't use the sender ID to lookup the proper value
                    float_converter.float_val = flash_params.gyro_offsets[ROLL];
                    cand_tx_response(msg.sender_id, CAND_PID_GYRO_OFFSET_RL, float_converter.uint32_val);
                }
            }
            break;

            default:
                AxisFault(CAND_FAULT_UNSUPPORTED_PARAMETER);
                break;
            }
            msg.param_request_cnt--;
        }
        param_query_cnt++;
        break;

    case CAND_RX_PARAM_RESPONSE:
        while (msg.param_response_cnt) {
            // GoPro response messages come as 2 parameter IDs packed into the same param response, so we need a temporary GoPro response to assemble the two parts into,
            // in case this param reponse happens to be a GoPro command response
            GPCmdResponse gp_cmd_response;

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

                case CAND_PID_TARGET_ANGLES_AZ:
                    cb_parms->angle_targets[AZ] = msg.param_response[msg.param_response_cnt-1];
                    break;

                case CAND_PID_TARGET_ANGLES_EL:
                    cb_parms->angle_targets[EL] = msg.param_response[msg.param_response_cnt-1];
                    break;

                case CAND_PID_TARGET_ANGLES_ROLL:
                    cb_parms->angle_targets[ROLL] = msg.param_response[msg.param_response_cnt-1];
                    break;

                case CAND_PID_GP_CMD:
                {
                    Uint32 packed_param = msg.param_response[msg.param_response_cnt - 1];
                    gp_cmd_response.cmd[0] = (packed_param >> 24) & 0x000000FF;
                    gp_cmd_response.cmd[1] = (packed_param >> 16) & 0x000000FF;
                    gp_cmd_response.cmd_status = (packed_param >> 8) & 0x000000FF;
                    gp_cmd_response.cmd_response = (packed_param & 0x000000FF);
                }
                break;

                case CAND_PID_GP_LAST_CMD_RESULT:
                    gp_cmd_response.cmd_result = (GPCmdResult)msg.param_response[msg.param_response_cnt - 1];

                    // This is the 2nd half of the GoPro result, so we can now send the result over the MAVLink interface
                    send_mavlink_gopro_response(&gp_cmd_response);
                    break;

                case CAND_PID_TORQUE_KP:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_2 & INIT_PARAM_TORQUE_PID_KP_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        md_parms->pid_id.param.Kp = float_converter.float_val;
                        md_parms->pid_iq.param.Kp = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_2 |= INIT_PARAM_TORQUE_PID_KP_RECVD;
                    }
                    break;

                case CAND_PID_TORQUE_KI:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_2 & INIT_PARAM_TORQUE_PID_KI_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        md_parms->pid_id.param.Ki = float_converter.float_val;
                        md_parms->pid_iq.param.Ki = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_2 |= INIT_PARAM_TORQUE_PID_KI_RECVD;
                    }
                    break;

                case CAND_PID_TORQUE_KD:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_2 & INIT_PARAM_TORQUE_PID_KD_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        md_parms->pid_id.param.Kd = float_converter.float_val;
                        md_parms->pid_iq.param.Kd = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_2 |= INIT_PARAM_TORQUE_PID_KD_RECVD;
                    }
                    break;

                case CAND_PID_COMMUTATION_CALIBRATION_SLOPE:
                	if (GetBoardHWID() == AZ) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                		flash_params.AxisCalibrationSlopes[msg.sender_id] = float_converter.float_val;
                		//write_flash();
                	} else {
                        // Only load the parameter once (because we request parameters until we get them, there's a possibility
                        // of getting multiple responses for the same parameter)
                        if (!(load_ap_state_info->init_param_recvd_flags_2 & INIT_PARAM_COMMUTATION_CALIBRATION_SLOPE_RECVD)) {
                            IntOrFloat float_converter;
                            float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                            AxisCalibrationSlopes[GetBoardHWID()] = float_converter.float_val;
                            load_ap_state_info->init_param_recvd_flags_2 |= INIT_PARAM_COMMUTATION_CALIBRATION_SLOPE_RECVD;
                        }
                	}
                    break;

                case CAND_PID_COMMUTATION_CALIBRATION_INTERCEPT:
                	if (GetBoardHWID() == AZ) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                		flash_params.AxisCalibrationIntercepts[msg.sender_id] = float_converter.float_val;
                		// intercept comes after slope
                		write_flash();
                	} else {
                        // Only load the parameter once (because we request parameters until we get them, there's a possibility
                        // of getting multiple responses for the same parameter)
                        if (!(load_ap_state_info->init_param_recvd_flags_2 & INIT_PARAM_COMMUTATION_CALIBRATION_INTERCEPT_RECVD)) {
                            IntOrFloat float_converter;
                            float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                            AxisCalibrationIntercepts[GetBoardHWID()] = float_converter.float_val;
                            load_ap_state_info->init_param_recvd_flags_2 |= INIT_PARAM_COMMUTATION_CALIBRATION_INTERCEPT_RECVD;
                        }
                	}
                    break;

                case CAND_PID_COMMUTATION_CALIBRATION_HOME_OFFSET:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_2 & INIT_PARAM_COMMUTATION_CALIBRATION_HOME_OFFSET_RECVD)) {
                        AxisHomePositions[GetBoardHWID()] = (float)(msg.param_response[msg.param_response_cnt - 1]);
                        load_ap_state_info->init_param_recvd_flags_2 |= INIT_PARAM_COMMUTATION_CALIBRATION_HOME_OFFSET_RECVD;
                    }
                    break;

                case CAND_PID_RATE_EL_P:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_1 & INIT_PARAM_RATE_PID_EL_P_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        rate_pid_loop_float[EL].gainP = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_EL_P_RECVD;
                    }
                    break;

                case CAND_PID_RATE_EL_I:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_1 & INIT_PARAM_RATE_PID_EL_I_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        rate_pid_loop_float[EL].gainI = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_EL_I_RECVD;
                    }
                    break;

                case CAND_PID_RATE_EL_D:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_1 & INIT_PARAM_RATE_PID_EL_D_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        rate_pid_loop_float[EL].gainD = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_EL_D_RECVD;
                    }
                    break;

                case CAND_PID_RATE_EL_WINDUP:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_1 & INIT_PARAM_RATE_PID_EL_WINDUP_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        rate_pid_loop_float[EL].integralMax = float_converter.float_val;
                        rate_pid_loop_float[EL].integralMin = -float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_EL_WINDUP_RECVD;
                    }
                    break;

                case CAND_PID_RATE_AZ_P:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_1 & INIT_PARAM_RATE_PID_AZ_P_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        rate_pid_loop_float[AZ].gainP = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_AZ_P_RECVD;
                    }
                    break;

                case CAND_PID_RATE_AZ_I:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_1 & INIT_PARAM_RATE_PID_AZ_I_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        rate_pid_loop_float[AZ].gainI = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_AZ_I_RECVD;
                    }
                    break;

                case CAND_PID_RATE_AZ_D:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_1 & INIT_PARAM_RATE_PID_AZ_D_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        rate_pid_loop_float[AZ].gainD = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_AZ_D_RECVD;
                    }
                    break;

                case CAND_PID_RATE_AZ_WINDUP:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_1 & INIT_PARAM_RATE_PID_AZ_WINDUP_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        rate_pid_loop_float[AZ].integralMax = float_converter.float_val;
                        rate_pid_loop_float[AZ].integralMin = -float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_AZ_WINDUP_RECVD;
                    }
                    break;

                case CAND_PID_RATE_RL_P:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_1 & INIT_PARAM_RATE_PID_RL_P_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        rate_pid_loop_float[ROLL].gainP = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_RL_P_RECVD;
                    }
                    break;

                case CAND_PID_RATE_RL_I:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_1 & INIT_PARAM_RATE_PID_RL_I_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        rate_pid_loop_float[ROLL].gainI = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_RL_I_RECVD;
                    }
                    break;

                case CAND_PID_RATE_RL_D:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_1 & INIT_PARAM_RATE_PID_RL_D_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        rate_pid_loop_float[ROLL].gainD = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_RL_D_RECVD;
                    }
                    break;

                case CAND_PID_RATE_RL_WINDUP:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_1 & INIT_PARAM_RATE_PID_RL_WINDUP_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        rate_pid_loop_float[ROLL].integralMax = float_converter.float_val;
                        rate_pid_loop_float[ROLL].integralMin = -float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_1 |= INIT_PARAM_RATE_PID_RL_WINDUP_RECVD;
                    }
                    break;

                case CAND_PID_POS_EL_P:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_3 & INIT_PARAM_POS_PID_EL_P_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        pos_pid_loop_float[EL].gainP = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_3 |= INIT_PARAM_POS_PID_EL_P_RECVD;
                    }
                    break;

                case CAND_PID_POS_EL_I:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_3 & INIT_PARAM_POS_PID_EL_I_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        pos_pid_loop_float[EL].gainI = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_3 |= INIT_PARAM_POS_PID_EL_I_RECVD;
                    }
                    break;

                case CAND_PID_POS_EL_D:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_3 & INIT_PARAM_POS_PID_EL_D_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        pos_pid_loop_float[EL].gainD = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_3 |= INIT_PARAM_POS_PID_EL_D_RECVD;
                    }
                    break;

                case CAND_PID_POS_EL_WINDUP:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_3 & INIT_PARAM_POS_PID_EL_WINDUP_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        pos_pid_loop_float[EL].integralMax = float_converter.float_val;
                        pos_pid_loop_float[EL].integralMin = -float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_3 |= INIT_PARAM_POS_PID_EL_WINDUP_RECVD;
                    }
                    break;

                case CAND_PID_POS_AZ_P:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_3 & INIT_PARAM_POS_PID_AZ_P_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        pos_pid_loop_float[AZ].gainP = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_3 |= INIT_PARAM_POS_PID_AZ_P_RECVD;
                    }
                    break;

                case CAND_PID_POS_AZ_I:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_3 & INIT_PARAM_POS_PID_AZ_I_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        pos_pid_loop_float[AZ].gainI = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_3 |= INIT_PARAM_POS_PID_AZ_I_RECVD;
                    }
                    break;

                case CAND_PID_POS_AZ_D:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_3 & INIT_PARAM_POS_PID_AZ_D_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        pos_pid_loop_float[AZ].gainD = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_3 |= INIT_PARAM_POS_PID_AZ_D_RECVD;
                    }
                    break;

                case CAND_PID_POS_AZ_WINDUP:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_3 & INIT_PARAM_POS_PID_AZ_WINDUP_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        pos_pid_loop_float[AZ].integralMax = float_converter.float_val;
                        pos_pid_loop_float[AZ].integralMin = -float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_3 |= INIT_PARAM_POS_PID_AZ_WINDUP_RECVD;
                    }
                    break;

                case CAND_PID_POS_RL_P:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_3 & INIT_PARAM_POS_PID_RL_P_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        pos_pid_loop_float[ROLL].gainP = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_3 |= INIT_PARAM_POS_PID_RL_P_RECVD;
                    }
                    break;

                case CAND_PID_POS_RL_I:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_3 & INIT_PARAM_POS_PID_RL_I_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        pos_pid_loop_float[ROLL].gainI = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_3 |= INIT_PARAM_POS_PID_RL_I_RECVD;
                    }
                    break;

                case CAND_PID_POS_RL_D:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_3 & INIT_PARAM_POS_PID_RL_D_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        pos_pid_loop_float[ROLL].gainD = float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_3 |= INIT_PARAM_POS_PID_RL_D_RECVD;
                    }
                    break;

                case CAND_PID_POS_RL_WINDUP:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_3 & INIT_PARAM_POS_PID_RL_WINDUP_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        pos_pid_loop_float[ROLL].integralMax = float_converter.float_val;
                        pos_pid_loop_float[ROLL].integralMin = -float_converter.float_val;
                        load_ap_state_info->init_param_recvd_flags_3 |= INIT_PARAM_POS_PID_RL_WINDUP_RECVD;
                    }
                    break;

                case CAND_PID_GYRO_OFFSET_EL:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_3 & INIT_PARAM_GYRO_OFFSET_EL_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        cb_parms->gyro_offsets[EL] = (int16)(float_converter.float_val);
                        load_ap_state_info->init_param_recvd_flags_3 |= INIT_PARAM_GYRO_OFFSET_EL_RECVD;
                    }
                    break;

                case CAND_PID_GYRO_OFFSET_AZ:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_3 & INIT_PARAM_GYRO_OFFSET_AZ_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        cb_parms->gyro_offsets[AZ] = (int16)(float_converter.float_val);
                        load_ap_state_info->init_param_recvd_flags_3 |= INIT_PARAM_GYRO_OFFSET_AZ_RECVD;
                    }
                    break;

                case CAND_PID_GYRO_OFFSET_RL:
                    // Only load the parameter once (because we request parameters until we get them, there's a possibility
                    // of getting multiple responses for the same parameter)
                    if (!(load_ap_state_info->init_param_recvd_flags_3 & INIT_PARAM_GYRO_OFFSET_RL_RECVD)) {
                        IntOrFloat float_converter;
                        float_converter.uint32_val = msg.param_response[msg.param_response_cnt - 1];
                        cb_parms->gyro_offsets[ROLL] = (int16)(float_converter.float_val);
                        load_ap_state_info->init_param_recvd_flags_3 |= INIT_PARAM_GYRO_OFFSET_RL_RECVD;
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
