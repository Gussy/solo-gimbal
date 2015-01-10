/*
 * can_message_processor.c
 *
 *  Created on: Jan 7, 2015
 *      Author: abamberger
 */

#include "can_message_processor.h"
#include "PM_Sensorless-Settings.h"
#include "PM_Sensorless.h"
#include "cb.h"
#include "cand.h"

void Process_CAN_Messages(AxisParms* axis_parms, MotorDriveParms* md_parms, ControlBoardParms* cb_parms, ParamSet* param_set)
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
        md_parms->motor_drive_state = STATE_FAULT;

        fault_cnt++;
        break;

    case CAND_RX_COMMAND:
        switch (msg.command) {
        case CAND_CMD_INIT:
            axis_parms->blink_state = BLINK_INIT;
            break;

        case CAND_CMD_ENABLE:
            axis_parms->enable_flag = TRUE;
            md_parms->motor_drive_state = STATE_INIT;
            axis_parms->blink_state = BLINK_INIT;
            break;

        case CAND_CMD_RELAX:
            //EnableFlag = 0;
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
            }

            msg.param_response_cnt--;
        }
        param_response_cnt++;
        break;

    default:
        break;
    }
}
