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
#include "parameters/load_axis_parms_state_machine.h"
#include "parameters/flash_params.h"
#include "mavlink_interface/mavlink_gimbal_interface.h"
#include "motor/motor_drive_state_machine.h"
#include "gopro/gopro_interface.h"

#include <stdio.h>

static void process_mavlink_input(MavlinkGimbalInfo* mavlink_info, ControlBoardParms* cb_parms, MotorDriveParms* md_parms, EncoderParms* encoder_parms, LoadAxisParmsStateInfo* load_ap_state_info);
static void send_mavlink_request_stream();
static void handle_attitude(mavlink_message_t* received_msg);
static void handle_mount_control(mavlink_message_t* received_msg);
static void handle_gopro_power_on(mavlink_message_t* received_msg);
static void handle_gopro_power_off(mavlink_message_t* received_msg);
static void handle_gopro_command(mavlink_message_t* received_msg);
static void handle_data_transmission_handshake(mavlink_message_t *msg);
static void handle_reset_gimbal(mavlink_message_t* received_msg);
static void handle_gimbal_control(mavlink_message_t* received_msg, MavlinkGimbalInfo* mavlink_info);
static void handle_set_home_offsets(MotorDriveParms* md_parms, EncoderParms* encoder_parms, LoadAxisParmsStateInfo* load_ap_state_info);
static void handle_set_factory_params(mavlink_message_t* msg);
static void handle_gimbal_erase_flash(mavlink_message_t* msg);
static void handle_perform_factory_tests(mavlink_message_t* msg, ControlBoardParms* cb_parms);
static void handle_request_axis_calibration(mavlink_message_t* msg, MotorDriveParms* md_parms);
static void handle_request_axis_calibration_status(mavlink_message_t* msg, ControlBoardParms* cb_parms);

mavlink_system_t mavlink_system;
uint8_t message_buffer[MAVLINK_MAX_PACKET_LEN];

unsigned char feedback_id;

unsigned char gimbal_sysid = 0;

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

float targets[3] = { 0, 0, 0 };

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
	if (msg->compid == MAV_COMP_ID_GIMBAL) {
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
		
		EALLOW;
		// Cause a device reset by writing incorrect values into WDCHK
		SysCtrlRegs.WDCR = 0x0010;
		EDIS;

		// This should never be reached.
		while(1);
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
				if (heartbeats_received ==0){ // Grab the compid from the first heartbeat message received
					gimbal_sysid = received_msg.sysid;
				}				
				heartbeats_received++;
				break;

			case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
				last_parameter_sent = 0;
				mavlink_info->mavlink_processing_state = MAVLINK_STATE_SEND_PARAM_LIST;
				break;

			case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
			    handle_param_read(&received_msg);
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
			    handle_gimbal_control(&received_msg, mavlink_info);
			    break;

			case MAVLINK_MSG_ID_GIMBAL_RESET:
			    handle_reset_gimbal(&received_msg);
			    break;

			case MAVLINK_MSG_ID_GIMBAL_SET_HOME_OFFSETS:
			    handle_set_home_offsets(md_parms, encoder_parms, load_ap_state_info);
			    break;

			case MAVLINK_MSG_ID_GIMBAL_SET_FACTORY_PARAMETERS:
			    handle_set_factory_params(&received_msg);
			    break;

			case MAVLINK_MSG_ID_GIMBAL_ERASE_FIRMWARE_AND_CONFIG:
			    handle_gimbal_erase_flash(&received_msg);
			    break;

			case MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS:
			    handle_perform_factory_tests(&received_msg, cb_parms);
			    break;

			case MAVLINK_MSG_ID_GIMBAL_REQUEST_AXIS_CALIBRATION:
			    handle_request_axis_calibration(&received_msg, md_parms);
			    break;

			case MAVLINK_MSG_ID_GIMBAL_REQUEST_AXIS_CALIBRATION_STATUS:
			    handle_request_axis_calibration_status(&received_msg, cb_parms);
			    break;

			default:
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
    if ((decoded_msg.target_component == MAV_COMP_ID_GIMBAL)) {
        cand_tx_command(CAND_ID_EL, CAND_CMD_GOPRO_ON);
    }
}

static void handle_gopro_power_off(mavlink_message_t* received_msg)
{
    mavlink_gopro_power_off_t decoded_msg;
    mavlink_msg_gopro_power_off_decode(received_msg, &decoded_msg);

    // Make sure the message was for us.  If it was, send the GoPro power off command over CAN
    if ((decoded_msg.target_component == MAV_COMP_ID_GIMBAL)) {
        cand_tx_command(CAND_ID_EL, CAND_CMD_GOPRO_OFF);
    }
}

static void handle_gopro_command(mavlink_message_t* received_msg)
{
    mavlink_gopro_command_t decoded_msg;
    mavlink_msg_gopro_command_decode(received_msg, &decoded_msg);

    // Make sure the message was for us.  If it was, package up the command and send it over CAN
    if ((decoded_msg.target_component == MAV_COMP_ID_GIMBAL)) {
        Uint32 parameter = 0;
        parameter |= (((Uint32)decoded_msg.gp_cmd_name_1) << 24) & 0xFF000000;
        parameter |= (((Uint32)decoded_msg.gp_cmd_name_2) << 16) & 0x00FF0000;
        parameter |= (((Uint32)decoded_msg.gp_cmd_parm) << 8) & 0x0000FF00;
        cand_tx_param(CAND_ID_EL, CAND_PID_GP_CMD, parameter);
    }
}

static void handle_gimbal_control(mavlink_message_t* received_msg, MavlinkGimbalInfo* mavlink_info)
{
    mavlink_gimbal_control_t decoded_msg;
    mavlink_msg_gimbal_control_decode(received_msg, &decoded_msg);

    // Make sure the message was for us.  If it was, send the various parameters over CAN
    if ((decoded_msg.target_component == MAV_COMP_ID_GIMBAL)) {
        CAND_ParameterID rate_cmd_pids[3] = {CAND_PID_RATE_CMD_AZ, CAND_PID_RATE_CMD_EL, CAND_PID_RATE_CMD_RL};
        Uint32 rate_cmds[3] = {0, 0, 0};

        // Copter mapping is X roll, Y el, Z az

        //TODO: Make the axis mapping more robust and less confusing
        // Gyro X rate
        rate_cmds[EL] = (int16)RAD_S_TO_GYRO_FORMAT(CLAMP_TO_BOUNDS(decoded_msg.demanded_rate_z, (float)INT16_MIN, (float)INT16_MAX));

        // Gyro Y rate
        rate_cmds[AZ] = (int16)RAD_S_TO_GYRO_FORMAT(CLAMP_TO_BOUNDS(decoded_msg.demanded_rate_y, (float)INT16_MIN, (float)INT16_MAX));

        // Gyro Z rate
        rate_cmds[ROLL] = (int16)RAD_S_TO_GYRO_FORMAT(CLAMP_TO_BOUNDS(decoded_msg.demanded_rate_x, (float)INT16_MIN, (float)INT16_MAX));

        cand_tx_multi_param(CAND_ID_EL, rate_cmd_pids, rate_cmds, 3);

        // If we've received a rate command, reset the rate command timeout counter
        mavlink_info->rate_cmd_timeout_counter = 0;

        // If the gimbal is not enabled, we want to enable it when we receive a rate command (we start out disabled)
        if (!(mavlink_info->gimbal_active)) {
            // Enable the other two axes
            cand_tx_command(CAND_ID_ALL_AXES, CAND_CMD_ENABLE);
            // Enable ourselves
            EnableAZAxis();
            mavlink_info->gimbal_active = TRUE;
        }
    }
}

static void handle_reset_gimbal(mavlink_message_t* received_msg)
{
    mavlink_gimbal_reset_t decoded_msg;
    mavlink_msg_gimbal_reset_decode(received_msg, &decoded_msg);

    // Make sure the message is for us
    if ((decoded_msg.target_component == MAV_COMP_ID_GIMBAL)) {
        // stop this axis
        RelaxAZAxis();
        power_down_motor();

        // stop the other axes
        cand_tx_command(CAND_ID_ALL_AXES, CAND_CMD_RELAX);

        // reset other axes
        cand_tx_command(CAND_ID_ALL_AXES, CAND_CMD_RESET);

        // reset this axes
        extern void WDogEnable(void);
        WDogEnable();
        while(1)
        {}
    }
}

static void handle_set_home_offsets(MotorDriveParms* md_parms, EncoderParms* encoder_parms, LoadAxisParmsStateInfo* load_ap_state_info)
{
    // Reset the flags used to indicate when the other axes have recalibrated their own home offsets
    load_ap_state_info->init_param_recvd_flags_2 &= ~ALL_NEW_HOME_OFFSETS_RECVD;

    // Zero out our own home offset accumulator and sample counter
    encoder_parms->home_offset_calibration_accumulator = 0;
    encoder_parms->home_offset_calibration_samples_accumulated = 0;

    // Command the other axes to start calibrating their home offsets
    cand_tx_command(CAND_ID_ALL_AXES, CAND_CMD_SET_HOME_OFFSETS);

    // Start calibrating our own home offset
    md_parms->motor_drive_state = STATE_CALIBRATE_HOME_OFFSETS;
}

static void handle_set_factory_params(mavlink_message_t* msg)
{
    mavlink_gimbal_set_factory_parameters_t decoded_msg;
    mavlink_msg_gimbal_set_factory_parameters_decode(msg, &decoded_msg);

    // Compose the assembly date and assembly time into the internal 32-bit format
    Uint32 assy_date = 0;
    Uint32 assy_time = 0;
    assy_date |= ((Uint32)decoded_msg.assembly_year << 16);
    assy_date |= ((Uint32)decoded_msg.assembly_month << 8);
    assy_date |= ((Uint32)decoded_msg.assembly_day);
    assy_time |= ((Uint32)decoded_msg.assembly_hour << 24);
    assy_time |= ((Uint32)decoded_msg.assembly_minute << 16);
    assy_time |= ((Uint32)decoded_msg.assembly_second << 8);

    // Compute the "checksums" using the 3 magic numbers to determine if this is a valid factory parameter set
    if (((assy_date + decoded_msg.magic_1) == FACTORY_PARAM_CHECK_MAGIC_1) &&
        ((assy_time + decoded_msg.magic_2) == FACTORY_PARAM_CHECK_MAGIC_2) &&
        ((decoded_msg.serial_number_pt_1 + decoded_msg.magic_3) == FACTORY_PARAM_CHECK_MAGIC_3)) {
        // This means the parameters are valid and we can go ahead and set them.  Otherwise, we just ignore the parameters
        flash_params.assy_date = assy_date;
        flash_params.assy_time = assy_time;
        flash_params.ser_num_1 = decoded_msg.serial_number_pt_1;
        flash_params.ser_num_2 = decoded_msg.serial_number_pt_2;
        flash_params.ser_num_3 = decoded_msg.serial_number_pt_3;
        write_flash();
        // Indicate that we've loaded the new parameters
        send_mavlink_factory_parameters_loaded();
    }
}

static void handle_gimbal_erase_flash(mavlink_message_t* msg)
{
    mavlink_gimbal_erase_firmware_and_config_t decoded_msg;
    mavlink_msg_gimbal_erase_firmware_and_config_decode(msg, &decoded_msg);

    // Make sure this message is for us
    if ((decoded_msg.target_component == MAV_COMP_ID_GIMBAL)) {
        // Make sure the knock value is correct
        if (decoded_msg.knock == GIMBAL_FIRMWARE_ERASE_KNOCK) {
            // stop this axis
            power_down_motor();
            // stop the other axis
            cand_tx_command(CAND_ID_ALL_AXES,CAND_CMD_RELAX);
            // reset other axis
            cand_tx_command(CAND_ID_ALL_AXES,CAND_CMD_RESET);

            // Erase firmware and calibration
            extern int erase_firmware_and_config();
            erase_firmware_and_config();

            // reset
            extern void WDogEnable(void);
            WDogEnable();

            EALLOW;
            // Cause a device reset by writing incorrect values into WDCHK
            SysCtrlRegs.WDCR = 0x0010;
            EDIS;

            // This should never be reached.
            while(1);
        }
    }
}

static void handle_perform_factory_tests(mavlink_message_t* msg, ControlBoardParms* cb_parms)
{
    mavlink_gimbal_perform_factory_tests_t decoded_msg;
    mavlink_msg_gimbal_perform_factory_tests_decode(msg, &decoded_msg);

    // Make sure this is for us
    if (decoded_msg.target_component == MAV_COMP_ID_GIMBAL) {
        // Remember that we're performing tests
        cb_parms->running_tests = TRUE;

        // Enable all of the axes
        // Enable the other two axes
        cand_tx_command(CAND_ID_ALL_AXES, CAND_CMD_ENABLE);
        // Enable ourselves
        EnableAZAxis();

        // Command the EL axis to start the factory tests
        cand_tx_command(CAND_ID_EL, CAND_CMD_START_FACTORY_TESTS);
    }
}

static void handle_request_axis_calibration(mavlink_message_t* msg, MotorDriveParms* md_parms)
{
    // First, decode the message to see which axes are being requested to calibrate
    mavlink_gimbal_request_axis_calibration_t decoded_msg;
    mavlink_msg_gimbal_request_axis_calibration_decode(msg, &decoded_msg);

    // Make sure the message is intended for us
    if (decoded_msg.target_component == MAV_COMP_ID_GIMBAL) {
        // Make sure we're in the waiting to calibrate state before commanding calibration
        if (md_parms->motor_drive_state == STATE_WAIT_FOR_AXIS_CALIBRATION_COMMAND) {
            // Tell all axes to start calibrating
            cand_tx_command(CAND_ID_ALL_AXES, CAND_CMD_CALIBRATE_AXES);
            md_parms->motor_drive_state = STATE_TAKE_COMMUTATION_CALIBRATION_DATA;
        }
    }
}

static void handle_request_axis_calibration_status(mavlink_message_t* msg, ControlBoardParms* cb_parms)
{
    mavlink_gimbal_request_axis_calibration_status_t decoded_msg;
    mavlink_msg_gimbal_request_axis_calibration_status_decode(msg, &decoded_msg);

    if (decoded_msg.target_component == MAV_COMP_ID_GIMBAL) {
        /*
        send_mavlink_axis_calibration_status(cb_parms->calibration_status[AZ] == CALIBRATION_REQUIRED,
                                             cb_parms->calibration_status[EL] == CALIBRATION_REQUIRED,
                                             cb_parms->calibration_status[ROLL] == CALIBRATION_REQUIRED);
        */
        send_mavlink_axis_calibration_status(cb_parms->calibration_status[AZ],
                                             cb_parms->calibration_status[EL],
                                             cb_parms->calibration_status[ROLL]);
    }
}

void send_mavlink_heartbeat(MAV_STATE mav_state, MAV_MODE_GIMBAL mav_mode) {
    static mavlink_message_t heartbeat_msg;
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
	static mavlink_message_t feedback_msg;

	// Copter mapping is X roll, Y el, Z az
	mavlink_msg_gimbal_report_pack(gimbal_sysid, MAV_COMP_ID_GIMBAL,
			&feedback_msg,
			0,
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

void send_mavlink_gopro_response(GPCmdResponse* response)
{
    static mavlink_message_t gopro_response_msg;
    mavlink_msg_gopro_response_pack(gimbal_sysid,
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
    static mavlink_message_t status_msg;
    mavlink_msg_statustext_pack(gimbal_sysid,
            MAV_COMP_ID_GIMBAL,
            &status_msg,
            severity,
            message);

    send_mavlink_message(&status_msg);
}

void send_mavlink_calibration_progress(Uint8 progress, GIMBAL_AXIS axis, GIMBAL_AXIS_CALIBRATION_STATUS calibration_status)
{
    static mavlink_message_t calibration_progress_msg;
    mavlink_msg_gimbal_axis_calibration_progress_pack(gimbal_sysid,
            MAV_COMP_ID_GIMBAL,
            &calibration_progress_msg,
            axis,
            progress,
            calibration_status);

    send_mavlink_message(&calibration_progress_msg);
}

void send_mavlink_factory_test_progress(FACTORY_TEST test, Uint8 section, Uint8 progress, Uint8 status)
{
    static mavlink_message_t factory_test_progress_msg;
    mavlink_msg_gimbal_report_factory_tests_progress_pack(gimbal_sysid,
            MAV_COMP_ID_GIMBAL,
            &factory_test_progress_msg,
            test,
            section,
            progress,
            status);
    send_mavlink_message(&factory_test_progress_msg);
}

void send_mavlink_home_offset_calibration_result(GIMBAL_AXIS_CALIBRATION_STATUS result)
{
    static mavlink_message_t home_offset_cal_result_msg;
    mavlink_msg_gimbal_home_offset_calibration_result_pack(gimbal_sysid,
            MAV_COMP_ID_GIMBAL,
            &home_offset_cal_result_msg,
            result);

    send_mavlink_message(&home_offset_cal_result_msg);
}

void send_mavlink_factory_parameters_loaded()
{
    static mavlink_message_t factory_params_loaded_msg;
    mavlink_msg_gimbal_factory_parameters_loaded_pack(gimbal_sysid,
            MAV_COMP_ID_GIMBAL,
            &factory_params_loaded_msg,
            0);
    send_mavlink_message(&factory_params_loaded_msg);
}

void send_mavlink_axis_calibration_status(Uint8 az_needs_calibration, Uint8 el_needs_calibration, Uint8 rl_needs_calibration)
{
    static mavlink_message_t axis_calibration_status_msg;
    mavlink_msg_gimbal_report_axis_calibration_status_pack(gimbal_sysid,
            MAV_COMP_ID_GIMBAL,
            &axis_calibration_status_msg,
            az_needs_calibration,
            el_needs_calibration,
            rl_needs_calibration);
    send_mavlink_message(&axis_calibration_status_msg);
}

void send_mavlink_message(mavlink_message_t* msg) {
	uint16_t message_len = mavlink_msg_to_send_buffer(message_buffer, msg);

	uart_send_data(message_buffer, message_len);
}
