/*
 * mavlink_interface.h
 *
 *  Created on: Oct 10, 2014
 *      Author: abamberger
 */

#ifndef MAVLINK_INTERFACE_H_
#define MAVLINK_INTERFACE_H_

#include "PeripheralHeaderIncludes.h"
#include "PM_Sensorless.h"
#include "mavlink_interface/gimbal_mavlink.h"
#include "gopro/gopro_interface.h"
#include "control/gyro_kinematics_correction.h"

typedef enum {
	MAVLINK_STATE_PARSE_INPUT, MAVLINK_STATE_SEND_PARAM_LIST
} MavlinkProcessingState;

typedef struct {
	MAV_STATE mav_state;
	MAV_MODE_GIMBAL mav_mode;
} MavlinkGimbalInfo;

#define GYRO_AZ_TELEM_RECEIVED 0x0001
#define GYRO_EL_TELEM_RECEIVED 0x0002
#define GYRO_RL_TELEM_RECEIVED 0x0004
#define ACCEL_AZ_TELEM_RECEIVED 0x0008
#define ACCEL_EL_TELEM_RECEIVED 0x0010
#define ACCEL_RL_TELEM_RECEIVED 0x0020
#define ENCODER_TELEM_RECEIVED 0x0040

#define ALL_TELEM_RECEIVED 0x007F

#define GYRO_FULL_SCALE_DEG_S (500.0)
#define ACCEL_FULL_SCALE_G (4.0)
#define ENCODER_FULL_SCALE (5000.0)

#define DEG_TO_RAD(deg) ((deg) * (M_PI / 180.0))
#define RAD_TO_DEG(rad) ((rad) * (180.0 / M_PI))
#define G_TO_M_S_S(g) ((g) * 9.80665)

#define ENCODER_FORMAT_TO_RAD(encoder) (M_PI * (float)(encoder) / ENCODER_FULL_SCALE)

#define GYRO_FORMAT_TO_RAD_S(gyro) DEG_TO_RAD((((float)(gyro) / (float)INT_MAX) * GYRO_FULL_SCALE_DEG_S))
#define RAD_S_TO_GYRO_FORMAT(rad) (RAD_TO_DEG((float)rad) * (((float)INT_MAX) / GYRO_FULL_SCALE_DEG_S))

#define ACCEL_FORMAT_TO_M_S_S(accel) G_TO_M_S_S(((((float)(accel)) / (float)INT_MAX) * ACCEL_FULL_SCALE_G))

//TODO: System ID of 50 is temporary for now
#define MAVLINK_GIMBAL_SYSID 50
#define ATTITUDE_DATA_REFRESH_RATE 50
// This is defined in terms of 150ms periods, so 6 is the closest we can get to a 1Hz heartbeat
#define MAVLINK_HEARTBEAT_PERIOD 6

void init_mavlink();
void mavlink_state_machine();
void send_mavlink_heartbeat(MAV_STATE mav_state, MAV_MODE_GIMBAL mav_mode);
void send_mavlink_gimbal_feedback();
void send_mavlink_debug_data(DebugData* debug_data);
void send_mavlink_gopro_response(GPCmdResponse* response);
void send_mavlink_axis_error(CAND_DestinationID axis, CAND_FaultCode fault);
void send_mavlink_statustext(char* message, MAV_SEVERITY severity);
void send_mavlink_message(mavlink_message_t* msg);

void receive_encoder_telemetry(int16 az_encoder, int16 el_encoder, int16 rl_encoder);
void receive_gyro_az_telemetry(int32 az_gyro);
void receive_gyro_el_telemetry(int32 el_gyro);
void receive_gyro_rl_telemetry(int32 rl_gyro);
void receive_accel_az_telemetry(int32 az_accel);
void receive_accel_el_telemetry(int32 el_accel);
void receive_accel_rl_telemetry(int32 rl_accel);

#endif /* MAVLINK_INTERFACE_H_ */
