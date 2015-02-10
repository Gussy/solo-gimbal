/*
 * mavlink_interface.c
 *
 *  Created on: Oct 10, 2014
 *      Author: abamberger
 */

#include "PM_Sensorless.h"
#include "PM_Sensorless-Settings.h"

#include "can/cand.h"
#include "can/cand_BitFields.h"
#include "hardware/uart.h"
#include "parameters/mavlink_parameter_interface.h"
#include "mavlink_interface/mavlink_gimbal_interface.h"
#include "gopro/gopro_interface.h"

static void process_mavlink_input();
static void send_mavlink_request_stream();
static void handle_attitude(mavlink_message_t* received_msg);
static void handle_mount_control(mavlink_message_t* received_msg);
static void handle_gopro_power_on(mavlink_message_t* received_msg);
static void handle_gopro_power_off(mavlink_message_t* received_msg);
static void handle_gopro_command(mavlink_message_t* received_msg);
static void handle_gimbal_control(mavlink_message_t* received_msg);

mavlink_system_t mavlink_system;
uint8_t message_buffer[MAVLINK_MAX_PACKET_LEN];

unsigned char feedback_id;

int messages_received = 0;
int heartbeats_received = 0;
int attitude_received = 0;

float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

// Encoder, Gyro, and Accelerometer telemetry
float latest_encoder_telemetry[3] = {0, 0, 0};
float latest_gyro_telemetry[3] = {0, 0, 0};
float latest_accel_telemetry[3] = {0, 0, 0};
Uint16 telem_received = 0;

Uint16 rate_cmd_received = FALSE;

float targets[3] = { 0, 0, 0 };

MavlinkProcessingState mavlink_state = MAVLINK_STATE_PARSE_INPUT;

int last_parameter_sent = 0;

void init_mavlink() {
	// Reset the gimbal's communication channel so the parse state machine starts out in a known state
	mavlink_reset_channel_status(MAVLINK_COMM_0);

	// Initialize the default parameters for the parameter interface
	init_default_mavlink_params();
}

DebugData attitude_debug_data = {
	0,
	0,
	0
};

void mavlink_state_machine() {
	switch (mavlink_state) {
	case MAVLINK_STATE_PARSE_INPUT:
		process_mavlink_input();
		break;

	case MAVLINK_STATE_SEND_PARAM_LIST:
		send_gimbal_param(last_parameter_sent++);
		if (last_parameter_sent >= MAVLINK_GIMBAL_PARAM_MAX) {
			mavlink_state = MAVLINK_STATE_PARSE_INPUT;
		}
		break;
	}
	if (attitude_received) {
		static int16 pos[3] = { 0, 0, 0 };
		CAND_ParameterID pids[3] = { CAND_PID_TARGET_ANGLES_AZ,
				CAND_PID_TARGET_ANGLES_EL, CAND_PID_TARGET_ANGLES_ROLL };

		// azimuth
		pos[0] = -1 * yaw / (3.14159 / 2) * ENCODER_COUNTS_PER_REV;
		// set azimuth to zero since it points north, zero keeps it trying to point forward
		pos[0] = targets[AZ] * ENCODER_COUNTS_PER_REV;
		if (pos[0] < 0) {
			pos[0] += ENCODER_COUNTS_PER_REV;
		} else if (pos[0] > ENCODER_COUNTS_PER_REV) {
			pos[0] -= ENCODER_COUNTS_PER_REV;
		}

		// elevation
		pos[1] = targets[EL] * ENCODER_COUNTS_PER_REV
				- 1 * pitch / (3.14159 * 2) * ENCODER_COUNTS_PER_REV;
		if (pos[1] < 0) {
			pos[1] += ENCODER_COUNTS_PER_REV;
		} else if (pos[1] > ENCODER_COUNTS_PER_REV) {
			pos[1] -= ENCODER_COUNTS_PER_REV;
		}
		// roll
		pos[2] = targets[ROLL] * ENCODER_COUNTS_PER_REV
				- 1 * roll / (3.14159 * 2) * ENCODER_COUNTS_PER_REV;
		if (pos[2] < 0) {
			pos[2] += ENCODER_COUNTS_PER_REV;
		} else if (pos[2] > ENCODER_COUNTS_PER_REV) {
			pos[2] -= ENCODER_COUNTS_PER_REV;
		}

		//TODO: For debugging pixhawk attitude drift
		/*
		attitude_debug_data.debug_1 = pos[0];
		attitude_debug_data.debug_2 = pos[1];
		attitude_debug_data.debug_3 = pos[2];

		send_mavlink_debug_data(&attitude_debug_data);
		*/

		// Inhibit transmission of new target angles if we're running the balance procedure,
		// so we don't overwrite the balance angles
#ifndef ENABLE_BALANCE_PROCEDURE
		Uint32 pos_payload[3];
		pos_payload[0] = pos[0];
		pos_payload[1] = pos[1];
		pos_payload[2] = pos[2];
		cand_tx_multi_param(CAND_ID_EL, pids, pos_payload, 3);
#endif
		attitude_received = 0;
	}
}

static void handle_data_transmission_handshake(mavlink_message_t *msg)
{
	/* does this need to be a state machine? */
	if ((msg->sysid == MAVLINK_GIMBAL_SYSID)&&
		(msg->compid == MAV_COMP_ID_GIMBAL)) {
		// make sure this message is for us
		// stop this axis
		power_down_motor();
		// stop the other axis
		cand_tx_command(CAND_ID_ALL_AXES,CAND_CMD_RELAX);
		// reset other axis
		cand_tx_command(CAND_ID_ALL_AXES,CAND_CMD_RESET);
		// erase our flash
		extern int erase_our_flash();
		if (erase_our_flash() < 0) {
			// something went wrong... but what do I do?
		}
		// reset
		extern void WDogEnable(void);
		WDogEnable();
		while(1);
	}
}

static void process_mavlink_input() {
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
				mavlink_state = MAVLINK_STATE_SEND_PARAM_LIST;
				break;

			case MAVLINK_MSG_ID_PARAM_SET:
				handle_param_set(&received_msg);
				break;

			case MAVLINK_MSG_ID_ATTITUDE:
				attitude_received++;
				handle_attitude(&received_msg);
				break;

			case MAVLINK_MSG_ID_MOUNT_CONTROL:
				handle_mount_control(&received_msg);
				break;

			case MAVLINK_MSG_ID_GOPRO_POWER_ON:
			    handle_gopro_power_on(&received_msg);
			    break;

			case MAVLINK_MSG_ID_GOPRO_POWER_OFF:
			    handle_gopro_power_off(&received_msg);
			    break;

			case MAVLINK_MSG_ID_GOPRO_COMMAND:
			    handle_gopro_command(&received_msg);
			    break;

			case MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE:
				handle_data_transmission_handshake(&received_msg);
				break;

			case MAVLINK_MSG_ID_GIMBAL_CONTROL:
			    handle_gimbal_control(&received_msg);
			    break;

			default: {
				Uint8 msgid = received_msg.msgid;
			}
				break;
			}
		}
	}
}

void receive_encoder_telemetry(int16 az_encoder, int16 el_encoder, int16 rl_encoder)
{
    latest_encoder_telemetry[AZ] = ENCODER_FORMAT_TO_RAD(az_encoder);
    latest_encoder_telemetry[EL] = ENCODER_FORMAT_TO_RAD(el_encoder);
    latest_encoder_telemetry[ROLL] = ENCODER_FORMAT_TO_RAD(rl_encoder);

    telem_received |= ENCODER_TELEM_RECEIVED;

    if (telem_received == ALL_TELEM_RECEIVED) {
        send_mavlink_gimbal_feedback();

        telem_received = 0;
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

static void handle_attitude(mavlink_message_t* received_msg) {
	mavlink_attitude_t decoded_msg;
	mavlink_msg_attitude_decode(received_msg, &decoded_msg);

	roll = decoded_msg.roll;
	pitch = decoded_msg.pitch;
	yaw = decoded_msg.yaw;
}

static void handle_mount_control(mavlink_message_t* received_msg) {
	mavlink_mount_control_t decoded_msg;
	mavlink_msg_mount_control_decode(received_msg, &decoded_msg);
	targets[EL] = decoded_msg.input_a / (360 * 100.0); ///< pitch(deg*100) or lat, depending on mount mode
	targets[ROLL] = decoded_msg.input_b / (360 * 100.0); ///< roll(deg*100) or lon depending on mount mode
	targets[AZ] = decoded_msg.input_c / (360 * 100.0); ///< yaw(deg*100) or alt (in cm) depending on mount mode
}

static void handle_gopro_power_on(mavlink_message_t* received_msg)
{
    mavlink_gopro_power_on_t decoded_msg;
    mavlink_msg_gopro_power_on_decode(received_msg, &decoded_msg);

    // Make sure the message was for us.  If it was, send the GoPro power on command over CAN
    if ((decoded_msg.target_system == MAVLINK_GIMBAL_SYSID) && (decoded_msg.target_component == MAV_COMP_ID_GIMBAL)) {
        cand_tx_command(CAND_ID_EL, CAND_CMD_GOPRO_ON);
    }
}

static void handle_gopro_power_off(mavlink_message_t* received_msg)
{
    mavlink_gopro_power_off_t decoded_msg;
    mavlink_msg_gopro_power_off_decode(received_msg, &decoded_msg);

    // Make sure the message was for us.  If it was, send the GoPro power off command over CAN
    if ((decoded_msg.target_system == MAVLINK_GIMBAL_SYSID) && (decoded_msg.target_component == MAV_COMP_ID_GIMBAL)) {
        cand_tx_command(CAND_ID_EL, CAND_CMD_GOPRO_OFF);
    }
}

static void handle_gopro_command(mavlink_message_t* received_msg)
{
    mavlink_gopro_command_t decoded_msg;
    mavlink_msg_gopro_command_decode(received_msg, &decoded_msg);

    // Make sure the message was for us.  If it was, package up the command and send it over CAN
    if ((decoded_msg.target_system == MAVLINK_GIMBAL_SYSID) && (decoded_msg.target_component == MAV_COMP_ID_GIMBAL)) {
        Uint32 parameter = 0;
        parameter |= (((Uint32)decoded_msg.gp_cmd_name_1) << 24) & 0xFF000000;
        parameter |= (((Uint32)decoded_msg.gp_cmd_name_2) << 16) & 0x00FF0000;
        parameter |= (((Uint32)decoded_msg.gp_cmd_parm) << 8) & 0x0000FF00;
        cand_tx_param(CAND_ID_EL, CAND_PID_GP_CMD, parameter);
    }
}

static void handle_gimbal_control(mavlink_message_t* received_msg)
{
    mavlink_gimbal_control_t decoded_msg;
    mavlink_msg_gimbal_control_decode(received_msg, &decoded_msg);

    // Make sure the message was for us.  If it was, send the various parameters over CAN
    if ((decoded_msg.target_system == MAVLINK_GIMBAL_SYSID) && (decoded_msg.target_component == MAV_COMP_ID_GIMBAL)) {
        CAND_ParameterID rate_cmd_pids[3] = {CAND_PID_RATE_CMD_AZ, CAND_PID_RATE_CMD_EL, CAND_PID_RATE_CMD_RL};
        Uint32 rate_cmds[3] = {0, 0, 0};

        // Copter mapping is X roll, Y el, Z az

        //TODO: Make the axis mapping more robust and less confusing
        // Gyro X rate
        rate_cmds[AZ] = (int16)RAD_S_TO_GYRO_FORMAT(CLAMP_TO_BOUNDS(decoded_msg.ratez, (float)INT16_MIN, (float)INT16_MAX));

        // Gyro Y rate
        rate_cmds[EL] = (int16)RAD_S_TO_GYRO_FORMAT(CLAMP_TO_BOUNDS(decoded_msg.ratey, (float)INT16_MIN, (float)INT16_MAX));

        // Gyro Z rate
        rate_cmds[ROLL] = (int16)RAD_S_TO_GYRO_FORMAT(CLAMP_TO_BOUNDS(decoded_msg.ratex, (float)INT16_MIN, (float)INT16_MAX));

        cand_tx_multi_param(CAND_ID_EL, rate_cmd_pids, rate_cmds, 3);

        // The first time we get a rate command, we want to enable the gimbal axes (we start out disabled)
        if (!rate_cmd_received) {
            // Enable the other two axes
            cand_tx_command(CAND_ID_ALL_AXES, CAND_CMD_ENABLE);
            // Enable ourselves
            EnableAZAxis();
            rate_cmd_received = TRUE;
        }
    }
}

void send_mavlink_heartbeat(MAV_STATE mav_state, MAV_MODE_GIMBAL mav_mode) {
    static mavlink_message_t heartbeat_msg;
    mavlink_msg_heartbeat_pack(MAVLINK_GIMBAL_SYSID,
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
	static mavlink_message_t feedback_msg;
	// Copter mapping is X roll, Y el, Z az
	mavlink_msg_gimbal_feedback_pack(MAVLINK_GIMBAL_SYSID, MAV_COMP_ID_GIMBAL,
			&feedback_msg,
			0,
			0,
			feedback_id++,
			latest_accel_telemetry[ROLL],
			latest_accel_telemetry[EL],
			latest_accel_telemetry[AZ],
			latest_gyro_telemetry[ROLL],
			latest_gyro_telemetry[EL],
			latest_gyro_telemetry[AZ],
			latest_encoder_telemetry[AZ],
			latest_encoder_telemetry[ROLL],
			latest_encoder_telemetry[EL]);
	send_mavlink_message(&feedback_msg);
}

void send_mavlink_gopro_response(GPCmdResponse* response)
{
    static mavlink_message_t gopro_response_msg;
    mavlink_msg_gopro_response_pack(MAVLINK_GIMBAL_SYSID,
                                    MAV_COMP_ID_GIMBAL,
                                    &gopro_response_msg,
                                    response->cmd[0],
                                    response->cmd[1],
                                    response->cmd_status,
                                    response->cmd_response,
                                    response->cmd_result);
    send_mavlink_message(&gopro_response_msg);
}

void send_mavlink_debug_data(DebugData* debug_data) {
	static mavlink_message_t debug_msg;
	mavlink_msg_debug_vect_pack(MAVLINK_GIMBAL_SYSID,
	        MAV_COMP_ID_GIMBAL,
			&debug_msg,
			"Debug 1",
			global_timestamp_counter * 100, // Global timestamp counter is in units of 100us
			(float) debug_data->debug_1,
			(float) debug_data->debug_2,
			(float) debug_data->debug_3);
	send_mavlink_message(&debug_msg);
}

void send_mavlink_statustext(char* message)
{
    static mavlink_message_t status_msg;
    mavlink_msg_statustext_pack(MAVLINK_GIMBAL_SYSID,
            MAV_COMP_ID_GIMBAL,
            &status_msg,
            MAV_SEVERITY_DEBUG,
            message);

    send_mavlink_message(&status_msg);
}

void send_mavlink_message(mavlink_message_t* msg) {
	uint16_t message_len = mavlink_msg_to_send_buffer(message_buffer, msg);

	uart_send_data(message_buffer, message_len);
}