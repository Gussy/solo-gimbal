/*
 * mavlink_interface.c
 *
 *  Created on: Oct 10, 2014
 *      Author: abamberger
 */

#include "hardware/uart.h"
#include "parameters/mavlink_parameter_interface.h"
#include "mavlink_gimbal_interface.h"
#include "can/cand.h"
#include "PM_Sensorless-Settings.h"

static void process_mavlink_input();

#define ATTITUDE_DATA_REFRESH_RATE 10

mavlink_system_t mavlink_system;
uint8_t message_buffer[MAVLINK_MAX_PACKET_LEN];

int messages_received = 0;
int heartbeats_received = 0;
int heartbeats_detected = 0;
int attitude_received = 0;

float roll = 0.0;
float pitch = 0.0;
float yaw = 0.0;

MavlinkProcessingState mavlink_state = MAVLINK_STATE_PARSE_INPUT;

int last_parameter_sent = 0;

void init_mavlink()
{
    // Reset the gimbal's communication channel so the parse state machine starts out in a known state
    mavlink_reset_channel_status(MAVLINK_COMM_0);

    // Initialize the default parameters for the parameter interface
    init_default_mavlink_params();
}

void send_mavlink_heartbeat(MAV_STATE mav_state, MAV_MODE_GIMBAL mav_mode)
{
    static mavlink_message_t heartbeat_msg;
    mavlink_msg_heartbeat_pack(MAVLINK_GIMBAL_SYSID, MAV_COMP_ID_GIMBAL, &heartbeat_msg, MAV_TYPE_GIMBAL, MAV_AUTOPILOT_INVALID, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mav_mode, mav_state);
    send_mavlink_message(&heartbeat_msg);
}

void request_mavlink_updates()
{
    static mavlink_message_t msg_request_data_stream;
    mavlink_msg_request_data_stream_pack(MAVLINK_GIMBAL_SYSID, MAV_COMP_ID_GIMBAL, &msg_request_data_stream,1,1,MAV_DATA_STREAM_EXTRA1,ATTITUDE_DATA_REFRESH_RATE,1);
    send_mavlink_message(&msg_request_data_stream);
}

void mavlink_state_machine()
{
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
    	static int16 pos[3] = {0,0,0};
    	CAND_ParameterID pids[3] = {CAND_PID_TARGET_ANGLES_AZ,CAND_PID_TARGET_ANGLES_EL,CAND_PID_TARGET_ANGLES_ROLL};

    	// azimuth
    	pos[0] = -1*yaw/(3.14159/2)*ENCODER_COUNTS_PER_REV;
    	// set azimuth to zero since it points north, zero keeps it trying to point forward
    	pos[0] = 0;
    	if (pos[0] < 0) {
    	    pos[0] += ENCODER_COUNTS_PER_REV;
    	} else if(pos[0] > ENCODER_COUNTS_PER_REV) {
    	    pos[0] -= ENCODER_COUNTS_PER_REV;
    	}
    	// @todo: change to the behavior desired by 3DR
    	// elevation
    	pos[1] = -1*pitch/(3.14159*2)*ENCODER_COUNTS_PER_REV;
    	if (pos[1] < 0) {
            pos[1] += ENCODER_COUNTS_PER_REV;
        } else if(pos[1] > ENCODER_COUNTS_PER_REV) {
            pos[1] -= ENCODER_COUNTS_PER_REV;
        }
    	// roll
    	pos[2] = -1*roll/(3.14159*2)*ENCODER_COUNTS_PER_REV;
    	if (pos[2] < 0) {
            pos[2] += ENCODER_COUNTS_PER_REV;
        } else if(pos[2] > ENCODER_COUNTS_PER_REV) {
            pos[2] -= ENCODER_COUNTS_PER_REV;
        }

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

static void process_mavlink_input()
{
    static mavlink_message_t received_msg;
    mavlink_status_t parse_status;

    while (uart_chars_available() > 0) {
        uint8_t c = uart_get_char();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &received_msg, &parse_status)) {
            messages_received++;
            switch (received_msg.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                heartbeats_received++;
                if (!heartbeats_detected) {
                	heartbeats_detected = 1;
                	request_mavlink_updates();
               	}
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
            }
        }
    }
}

void handle_attitude(mavlink_message_t* received_msg)
{
    mavlink_attitude_t decoded_msg;
    mavlink_msg_attitude_decode(received_msg, &decoded_msg);

    roll = decoded_msg.roll;
    pitch = decoded_msg.pitch;
    yaw = decoded_msg.yaw;
}

void send_mavlink_message(mavlink_message_t* msg)
{
    uint16_t message_len = mavlink_msg_to_send_buffer(message_buffer, msg);

    uart_send_data(message_buffer, message_len);
}
