#include "PM_Sensorless.h"
#include "PM_Sensorless-Settings.h"

#include "can/cand.h"
#include "can/cand_BitFields.h"
#include "hardware/uart.h"
#include "parameters/mavlink_parameter_interface.h"
#include "parameters/load_axis_parms_state_machine.h"
#include "parameters/flash_params.h"
#include "mavlink_interface/mavlink_gimbal_interface.h"
#include "motor/motor_drive_state_machine.h"
#include "gopro/gopro_interface.h"
#include "hardware/watchdog.h"
#include "flash/flash.h"
#include "version_git.h"
#include "hardware/watchdog.h"
#include "hardware/encoder.h"

#include <stdio.h>

static void process_mavlink_input(MavlinkGimbalInfo* mavlink_info, ControlBoardParms* cb_parms, MotorDriveParms* md_parms, EncoderParms* encoder_parms, LoadAxisParmsStateInfo* load_ap_state_info);
static void handle_data_transmission_handshake(mavlink_message_t *msg);
static void handle_reset_gimbal();
static void handle_full_reset_gimbal(void);
static void handle_request_axis_calibration(MotorDriveParms* md_parms);
static void handle_gopro_get_request(mavlink_message_t* received_msg);
static void handle_gopro_set_request(mavlink_message_t* received_msg);
static void handle_gimbal_control(mavlink_message_t* received_msg, MavlinkGimbalInfo* mavlink_info, ControlBoardParms* cb_parms);
void send_cmd_long_ack(uint16_t cmd_id, uint8_t result);

static uint8_t message_buffer[MAVLINK_MAX_PACKET_LEN];

unsigned char gimbal_sysid = 0;

int messages_received = 0;
int heartbeats_received = 0;

// Encoder, Gyro, and Accelerometer telemetry
float latest_encoder_telemetry[3] = {0, 0, 0};
float latest_gyro_telemetry[3] = {0, 0, 0};
float latest_accel_telemetry[3] = {0, 0, 0};
Uint16 telem_received = 0;

int last_parameter_sent = 0;

extern float send_torques;

void init_mavlink() {
	// Reset the gimbal's communication channel so the parse state machine starts out in a known state
	mavlink_reset_channel_status(MAVLINK_COMM_0);

	// Initialize the default parameters for the parameter interface
	init_default_mavlink_params();
}

void update_mavlink_sysid(Uint8 new_sysid)
{
	gimbal_sysid = new_sysid;
}

void mavlink_state_machine(MavlinkGimbalInfo* mavlink_info, ControlBoardParms* cb_parms, MotorDriveParms* md_parms, EncoderParms* encoder_parms, LoadAxisParmsStateInfo* load_ap_state_info) {
	switch (mavlink_info->mavlink_processing_state) {
	case MAVLINK_STATE_PARSE_INPUT:
		process_mavlink_input(mavlink_info, cb_parms, md_parms, encoder_parms, load_ap_state_info);
		break;

	case MAVLINK_STATE_SEND_PARAM_LIST:
		send_gimbal_param(last_parameter_sent++);
		if (last_parameter_sent >= MAVLINK_GIMBAL_PARAM_MAX) {
		    mavlink_info->mavlink_processing_state = MAVLINK_STATE_PARSE_INPUT;
		}
		break;
	}
}

static void handle_data_transmission_handshake(mavlink_message_t *msg)
{
	/* does this need to be a state machine? */
	if (msg->compid == MAV_COMP_ID_GIMBAL) {
		// make sure this message is for us
		// stop this axis
		power_down_motor();
		// reset other axis
		cand_tx_command(CAND_ID_ALL_AXES,CAND_CMD_RESET);
		// erase our flash
		if (erase_our_flash() < 0) {
			// something went wrong... but what do I do?
		}

		// reset now
		watchdog_immediate_reset();
	}
}

static void process_mavlink_input(MavlinkGimbalInfo* mavlink_info, ControlBoardParms* cb_parms, MotorDriveParms* md_parms, EncoderParms* encoder_parms, LoadAxisParmsStateInfo* load_ap_state_info) {
	static mavlink_message_t received_msg;
	mavlink_status_t parse_status;

	while (uart_chars_available() > 0) {
		uint8_t c = uart_get_char();
		if (mavlink_parse_char(MAVLINK_COMM_0, c, &received_msg,
				&parse_status)) {
			messages_received++;
			switch (received_msg.msgid) {
			case MAVLINK_MSG_ID_HEARTBEAT:
				heartbeats_received++;
				break;

			case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
				last_parameter_sent = 0;
				mavlink_info->mavlink_processing_state = MAVLINK_STATE_SEND_PARAM_LIST;
			    send_mavlink_statustext(GitVersionString, MAV_SEVERITY_INFO);
				break;

			case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
			    handle_param_read(&received_msg);
			    break;

			case MAVLINK_MSG_ID_PARAM_SET:
				handle_param_set(&received_msg);
				break;

			case MAVLINK_MSG_ID_GOPRO_GET_REQUEST:
				handle_gopro_get_request(&received_msg);
				break;

			case MAVLINK_MSG_ID_GOPRO_SET_REQUEST:
				handle_gopro_set_request(&received_msg);
				break;

			case MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE:
				handle_data_transmission_handshake(&received_msg);
				break;

			case MAVLINK_MSG_ID_GIMBAL_CONTROL:
			    handle_gimbal_control(&received_msg, mavlink_info, cb_parms);
			    break;


			case MAVLINK_MSG_ID_COMMAND_LONG:
			    switch(mavlink_msg_command_long_get_command(&received_msg)){
			    case 42501:
			    	handle_reset_gimbal();
			    	break;
			    case 42503:
			    	handle_request_axis_calibration(md_parms);
			    	break;
			    case 42505:
			        // Check all the params have the right keys
			        if(mavlink_msg_command_long_get_param1(&received_msg) == 42.0 &&
			                mavlink_msg_command_long_get_param2(&received_msg) == 49.0 &&
			                mavlink_msg_command_long_get_param3(&received_msg) == 12.0 &&
			                mavlink_msg_command_long_get_param4(&received_msg) == 26.0 &&
			                mavlink_msg_command_long_get_param5(&received_msg) == 64.0 &&
			                mavlink_msg_command_long_get_param6(&received_msg) == 85.0 &&
			                mavlink_msg_command_long_get_param7(&received_msg) == 42.0) {
			            handle_full_reset_gimbal();
			        }
			    default:
					break;
			    }
			    break;
			default:
				break;
			}
		}
	}
}

void receive_encoder_telemetry(int16 az_encoder, int16 el_encoder, int16 rl_encoder)
{
	// Undo joint angle offset cal here (copter applies its own joint offset angle correction, don't want to do it twice)
    latest_encoder_telemetry[AZ] = ENCODER_FORMAT_TO_RAD(az_encoder + getAxisJointOffset(AZ));
    latest_encoder_telemetry[EL] = ENCODER_FORMAT_TO_RAD(el_encoder + getAxisJointOffset(EL));
    latest_encoder_telemetry[ROLL] = ENCODER_FORMAT_TO_RAD(rl_encoder + getAxisJointOffset(ROLL));

    telem_received |= ENCODER_TELEM_RECEIVED;

    if (telem_received == ALL_TELEM_RECEIVED) {
        send_mavlink_gimbal_feedback();

        telem_received = 0;
    }
}

void receive_torque_cmd_telemetry(int16 az_torque_cmd, int16 el_torque_cmd, int16 rl_torque_cmd)
{
    if(send_torques == 1.0) {
	    send_mavlink_torque_cmd_feedback(az_torque_cmd, el_torque_cmd, rl_torque_cmd);
    }
}

void receive_gyro_az_telemetry(int32 az_gyro)
{
    latest_gyro_telemetry[AZ] = GYRO_FORMAT_TO_RAD_S(az_gyro) / 1000.0;

    telem_received |= GYRO_AZ_TELEM_RECEIVED;

    if (telem_received == ALL_TELEM_RECEIVED) {
        send_mavlink_gimbal_feedback();

        telem_received = 0;
    }
}

void receive_gyro_el_telemetry(int32 el_gyro)
{
    latest_gyro_telemetry[EL] = GYRO_FORMAT_TO_RAD_S(el_gyro) / 1000.0;

    telem_received |= GYRO_EL_TELEM_RECEIVED;

    if (telem_received == ALL_TELEM_RECEIVED) {
        send_mavlink_gimbal_feedback();

        telem_received = 0;
    }
}

void receive_gyro_rl_telemetry(int32 rl_gyro)
{
    latest_gyro_telemetry[ROLL] = GYRO_FORMAT_TO_RAD_S(rl_gyro) / 1000.0;

    telem_received |= GYRO_RL_TELEM_RECEIVED;

    if (telem_received == ALL_TELEM_RECEIVED) {
        send_mavlink_gimbal_feedback();

        telem_received = 0;
    }
}

void receive_accel_az_telemetry(int32 az_accel)
{
    latest_accel_telemetry[AZ] = ACCEL_FORMAT_TO_M_S_S(az_accel) / 1000.0;

    telem_received |= ACCEL_AZ_TELEM_RECEIVED;

    if (telem_received == ALL_TELEM_RECEIVED) {
        send_mavlink_gimbal_feedback();

        telem_received = 0;
    }
}

void receive_accel_el_telemetry(int32 el_accel)
{
    latest_accel_telemetry[EL] = ACCEL_FORMAT_TO_M_S_S(el_accel) / 1000.0;

    telem_received |= ACCEL_EL_TELEM_RECEIVED;

    if (telem_received == ALL_TELEM_RECEIVED) {
        send_mavlink_gimbal_feedback();

        telem_received = 0;
    }
}

void receive_accel_rl_telemetry(int32 rl_accel)
{
    latest_accel_telemetry[ROLL] = ACCEL_FORMAT_TO_M_S_S(rl_accel) / 1000.0;

    telem_received |= ACCEL_RL_TELEM_RECEIVED;

    if (telem_received == ALL_TELEM_RECEIVED) {
        send_mavlink_gimbal_feedback();

        telem_received = 0;
    }
}


static void handle_gopro_get_request(mavlink_message_t* received_msg)
{
	mavlink_gopro_get_request_t decoded_msg;
    mavlink_msg_gopro_get_request_decode(received_msg, &decoded_msg);

    // Make sure the message was for us.  If it was, package up the command and send it over CAN
    if ((decoded_msg.target_component == MAV_COMP_ID_GIMBAL)) {
        cand_tx_param(CAND_ID_EL, CAND_PID_GOPRO_GET_REQUEST, (Uint32)decoded_msg.cmd_id);
    }
}

static void handle_gopro_set_request(mavlink_message_t* received_msg)
{
	mavlink_gopro_set_request_t decoded_msg;
    mavlink_msg_gopro_set_request_decode(received_msg, &decoded_msg);

    // Make sure the message was for us.  If it was, package up the command and send it over CAN
    if ((decoded_msg.target_component == MAV_COMP_ID_GIMBAL)) {
        Uint32 parameter = 0;
        parameter |= (((Uint32)decoded_msg.cmd_id) << 8) & 0x0000FF00;
        parameter |= (((Uint32)decoded_msg.value) << 0) & 0x000000FF;
        cand_tx_param(CAND_ID_EL, CAND_PID_GOPRO_SET_REQUEST, parameter);
    }
}

static void handle_gimbal_control(mavlink_message_t* received_msg, MavlinkGimbalInfo* mavlink_info, ControlBoardParms* cb_parms)
{
    mavlink_gimbal_control_t decoded_msg;
    mavlink_msg_gimbal_control_decode(received_msg, &decoded_msg);

    // Make sure the message was for us.  If it was, send the various parameters over CAN
    if ((decoded_msg.target_component == MAV_COMP_ID_GIMBAL)) {
        CAND_ParameterID rate_cmd_pids[3] = {CAND_PID_RATE_CMD_AZ, CAND_PID_RATE_CMD_EL, CAND_PID_RATE_CMD_RL};
        Uint32 rate_cmds[3] = {0, 0, 0};

        // Copter mapping is X roll, Y el, Z az
        rate_cmds[EL] = (int16)CLAMP_TO_BOUNDS(RAD_S_TO_GYRO_FORMAT(decoded_msg.demanded_rate_z), (float)INT16_MIN, (float)INT16_MAX);
        rate_cmds[AZ] = (int16)CLAMP_TO_BOUNDS(RAD_S_TO_GYRO_FORMAT(decoded_msg.demanded_rate_y), (float)INT16_MIN, (float)INT16_MAX);
        rate_cmds[ROLL] = (int16)CLAMP_TO_BOUNDS(RAD_S_TO_GYRO_FORMAT(decoded_msg.demanded_rate_x), (float)INT16_MIN, (float)INT16_MAX);

        cand_tx_multi_param(CAND_ID_EL, rate_cmd_pids, rate_cmds, 3);

        // If we've received a rate command, reset the rate command timeout counter
        mavlink_info->rate_cmd_timeout_counter = 0;

        // If the gimbal is not enabled, we want to enable it when we receive a rate command (we disable the gimbal if
        // we lose rate commands, and we want to re-enable it when we re-acquire rate commands)
        if (!(mavlink_info->gimbal_active)) {
            // Enable the other two axes
            cand_tx_command(CAND_ID_ALL_AXES, CAND_CMD_ENABLE);
            // Enable ourselves
            EnableAZAxis();
            mavlink_info->gimbal_active = TRUE;
        }
    }
}

static void handle_reset_gimbal()
{
		send_cmd_long_ack(42501, MAV_CMD_ACK_OK);

        // stop this axis
        RelaxAZAxis();
        power_down_motor();

        // reset other axes
        cand_tx_command(CAND_ID_ALL_AXES, CAND_CMD_RESET);

        // reset this axis
        watchdog_reset();
}

static void handle_full_reset_gimbal(void)
{
    // stop this axis
    power_down_motor();

    // reset other axis
    cand_tx_command(CAND_ID_ALL_AXES, CAND_CMD_RESET);

    // erase program and param flash
    erase_our_flash();
    erase_param_flash();

    // reset now
    watchdog_immediate_reset();
}

static void handle_request_axis_calibration(MotorDriveParms* md_parms)
{
    // Make sure we're in the waiting to calibrate state before commanding calibration
    if (md_parms->motor_drive_state == STATE_WAIT_FOR_AXIS_CALIBRATION_COMMAND) {
        // Tell all axes to start calibrating
        cand_tx_command(CAND_ID_ALL_AXES, CAND_CMD_CALIBRATE_AXES);
        md_parms->motor_drive_state = STATE_TAKE_COMMUTATION_CALIBRATION_DATA;
    }
}

void send_mavlink_heartbeat(MAV_STATE mav_state, MAV_MODE_GIMBAL mav_mode) {
    mavlink_message_t heartbeat_msg;
    mavlink_msg_heartbeat_pack(gimbal_sysid,
                                MAV_COMP_ID_GIMBAL,
                                &heartbeat_msg,
                                MAV_TYPE_GIMBAL,
                                MAV_AUTOPILOT_INVALID,
                                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                mav_mode,
                                mav_state);
    send_mavlink_message(&heartbeat_msg);
}

void send_mavlink_gimbal_feedback() {
    mavlink_message_t feedback_msg;

	// Copter mapping is X roll, Y el, Z az
	mavlink_msg_gimbal_report_pack(gimbal_sysid, MAV_COMP_ID_GIMBAL,
			&feedback_msg,
			gimbal_sysid, // We assume the system we're talking to has the same sysid as us (or we're broadcasting)
			0,
			0.01f,
			latest_gyro_telemetry[ROLL],
			latest_gyro_telemetry[EL],
			latest_gyro_telemetry[AZ],
			-latest_accel_telemetry[ROLL],
			latest_accel_telemetry[EL],
			latest_accel_telemetry[AZ],
			latest_encoder_telemetry[ROLL],
			latest_encoder_telemetry[EL],
			latest_encoder_telemetry[AZ]);
	send_mavlink_message(&feedback_msg);
}

void send_mavlink_torque_cmd_feedback(int16 az_torque_cmd, int16 el_torque_cmd, int16 rl_torque_cmd) {
    mavlink_message_t torque_cmd_fb_msg;

	mavlink_msg_gimbal_torque_cmd_report_pack(gimbal_sysid,
		MAV_COMP_ID_GIMBAL,
		&torque_cmd_fb_msg,
		gimbal_sysid,
		0,
		rl_torque_cmd,
		el_torque_cmd,
		az_torque_cmd);

	send_mavlink_message(&torque_cmd_fb_msg);
}

void send_cmd_long_ack(uint16_t cmd_id, uint8_t result)
{
    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(gimbal_sysid,
                                    MAV_COMP_ID_GIMBAL,
                                    &msg,
                                    cmd_id,
									result);
    send_mavlink_message(&msg);
}

void send_mavlink_gopro_heartbeat(GPHeartbeatStatus status)
{
    mavlink_message_t gopro_heartbeat_msg;
    mavlink_msg_gopro_heartbeat_pack(gimbal_sysid,
                                    MAV_COMP_ID_GIMBAL,
                                    &gopro_heartbeat_msg,
                                    (uint8_t)status);
    send_mavlink_message(&gopro_heartbeat_msg);
}

void send_mavlink_gopro_get_response(GPGetResponse response)
{
    mavlink_message_t gopro_get_response_msg;
    mavlink_msg_gopro_get_response_pack(gimbal_sysid,
                                    MAV_COMP_ID_GIMBAL,
                                    &gopro_get_response_msg,
                                    response.cmd_id,
                                    response.value);
    send_mavlink_message(&gopro_get_response_msg);
}

void send_mavlink_gopro_set_response(GPSetResponse response)
{
    mavlink_message_t gopro_set_response_msg;
    mavlink_msg_gopro_set_response_pack(gimbal_sysid,
                                    MAV_COMP_ID_GIMBAL,
                                    &gopro_set_response_msg,
                                    response.cmd_id,
                                    response.result);
    send_mavlink_message(&gopro_set_response_msg);
}

void send_mavlink_debug_data(DebugData* debug_data) {
    mavlink_message_t debug_msg;
	mavlink_msg_debug_vect_pack(gimbal_sysid,
	        MAV_COMP_ID_GIMBAL,
			&debug_msg,
			"Debug 1",
			global_timestamp_counter * 100, // Global timestamp counter is in units of 100us
			(float) debug_data->debug_1,
			(float) debug_data->debug_2,
			(float) debug_data->debug_3);
	send_mavlink_message(&debug_msg);
}

void send_mavlink_axis_error(CAND_DestinationID axis, CAND_FaultCode fault_code, CAND_FaultType fault_type)
{
    char* axis_str = "Unknown";
    switch (axis) {
        case CAND_ID_AZ:
            axis_str = "Yaw";
            break;

        case CAND_ID_EL:
            axis_str = "Pitch";
            break;

        case CAND_ID_ROLL:
            axis_str = "Roll";
            break;
    }

    char* fault_str = "None";

    switch (fault_code) {
        case CAND_FAULT_CALIBRATING_POT:
            fault_str = "Calibrating Encoder";
            break;

        case CAND_FAULT_FIND_STOP_TIMEOUT:
            fault_str = "Timeout while finding mechanical stop";
            break;

        case CAND_FAULT_UNSUPPORTED_COMMAND:
            fault_str = "Received unsupported command";
            break;

        case CAND_FAULT_UNSUPPORTED_PARAMETER:
            fault_str = "Received unsupported parameter";
            break;

        case CAND_FAULT_UNKNOWN_AXIS_ID:
            fault_str = "Unable to determine axis ID";
            break;

        case CAND_FAULT_OVER_CURRENT:
            fault_str = "Over current";
            break;

        case CAND_FAULT_MOTOR_DRIVER_FAULT:
            fault_str = "Motor driver fault";
            break;

        case CAND_FAULT_CURRENT_SENSOR_FAULT:
            fault_str = "Current sensor fault";
            break;
    }

    MAV_SEVERITY severity = MAV_SEVERITY_ENUM_END;
    switch (fault_type) {
        case CAND_FAULT_TYPE_INFO:
            severity = MAV_SEVERITY_INFO;
            break;

        case CAND_FAULT_TYPE_RECOVERABLE:
            severity = MAV_SEVERITY_ALERT;
            break;

        case CAND_FAULT_TYPE_UNRECOVERABLE:
            severity = MAV_SEVERITY_CRITICAL;
            break;
    }

    char error_msg[100];
    snprintf(error_msg, 100, "Axis %s indicated fault: %s", axis_str, fault_str);
    send_mavlink_statustext(error_msg, severity);
}

void send_mavlink_statustext(char* message, MAV_SEVERITY severity)
{
    mavlink_message_t status_msg;
    mavlink_msg_statustext_pack(gimbal_sysid,
            MAV_COMP_ID_GIMBAL,
            &status_msg,
            severity,
            message);

    send_mavlink_message(&status_msg);
}

void send_mavlink_calibration_progress(Uint8 progress, GIMBAL_AXIS axis,
		GIMBAL_AXIS_CALIBRATION_STATUS calibration_status) {
    mavlink_message_t msg;
	mavlink_msg_command_long_pack(gimbal_sysid, MAV_COMP_ID_GIMBAL, &msg, 0, 0,
			42502, 0, (float) axis, (float) progress,
			(float) calibration_status, 0, 0, 0, 0);
	send_mavlink_message(&msg);
}

void send_mavlink_message(mavlink_message_t* msg) {
	uint16_t message_len = mavlink_msg_to_send_buffer(message_buffer, msg);

	uart_send_data(message_buffer, message_len);
}
