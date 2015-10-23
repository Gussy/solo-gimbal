#ifndef MAVLINK_INTERFACE_H_
#define MAVLINK_INTERFACE_H_

#include "PeripheralHeaderIncludes.h"
#include "PM_Sensorless.h"
#include "mavlink_interface/gimbal_mavlink.h"
#include "gopro/gopro_interface.h"
#include "control/gyro_kinematics_correction.h"
#include "motor/motor_drive_state_machine.h"
#include "parameters/load_axis_parms_state_machine.h"

typedef enum {
	MAVLINK_STATE_PARSE_INPUT, MAVLINK_STATE_SEND_PARAM_LIST
} MavlinkProcessingState;

typedef struct {
	MAV_STATE mav_state;
	MAV_MODE_GIMBAL mav_mode;
	MavlinkProcessingState mavlink_processing_state;
	int rate_cmd_timeout_counter;
	Uint16 gimbal_active;
} MavlinkGimbalInfo;

#define FACTORY_PARAM_CHECK_MAGIC_1 189496049
#define FACTORY_PARAM_CHECK_MAGIC_2 775861598
#define FACTORY_PARAM_CHECK_MAGIC_3 950903575
#define GIMBAL_FIRMWARE_ERASE_KNOCK 484383340

#define GYRO_AZ_TELEM_RECEIVED 0x0001
#define GYRO_EL_TELEM_RECEIVED 0x0002
#define GYRO_RL_TELEM_RECEIVED 0x0004
#define ACCEL_AZ_TELEM_RECEIVED 0x0008
#define ACCEL_EL_TELEM_RECEIVED 0x0010
#define ACCEL_RL_TELEM_RECEIVED 0x0020
#define ENCODER_TELEM_RECEIVED 0x0040

#define ALL_TELEM_RECEIVED 0x007F

#define ENCODER_FULL_SCALE (5000.0)

#define DEG_TO_RAD(deg) ((deg) * (M_PI / 180.0))
#define RAD_TO_DEG(rad) ((rad) * (180.0 / M_PI))

#define ENCODER_FORMAT_TO_RAD(encoder) (M_PI * (float)(encoder) / ENCODER_FULL_SCALE)
#define RAD_TO_ENCODER_FORMAT(angle) ((int16) (ENCODER_FULL_SCALE * angle / M_PI))

// This is defined in terms of 150ms periods, so 6 is the closest we can get to a 1Hz heartbeat
#define MAVLINK_HEARTBEAT_PERIOD 6

void init_mavlink();
void mavlink_state_machine(MavlinkGimbalInfo* mavlink_info, MotorDriveParms* md_parms);
void send_mavlink_heartbeat(MAV_STATE mav_state, MAV_MODE_GIMBAL mav_mode);
void send_mavlink_gimbal_feedback();
void send_mavlink_torque_cmd_feedback(int16 az_torque_cmd, int16 el_torque_cmd, int16 rl_torque_cmd);
void send_mavlink_debug_data(DebugData* debug_data);
void send_mavlink_gopro_heartbeat(const gp_can_mav_heartbeat_t *hb);
void send_mavlink_gopro_get_response(const gp_can_mav_get_rsp_t *rsp);
void send_mavlink_gopro_set_response(const gp_can_mav_set_rsp_t *rsp);
void send_mavlink_axis_error(CAND_DestinationID axis, CAND_FaultCode fault_code, CAND_FaultType fault_type);
void send_mavlink_statustext(char* message, MAV_SEVERITY severity);
void send_mavlink_calibration_progress(Uint8 progress, GIMBAL_AXIS axis, GIMBAL_AXIS_CALIBRATION_STATUS calibration_status);
void send_mavlink_home_offset_calibration_result(GIMBAL_AXIS_CALIBRATION_STATUS result);
void send_mavlink_message(mavlink_message_t* msg);
void update_mavlink_sysid(Uint8 new_sysid);

void receive_encoder_telemetry(int16 az_encoder, int16 el_encoder, int16 rl_encoder);
void receive_torque_cmd_telemetry(int16 az_torque_cmd, int16 el_torque_cmd, int16 rl_torque_cmd);
void receive_del_ang_az_telemetry(float az_del_ang);
void receive_del_ang_el_telemetry(float el_del_ang);
void receive_del_ang_rl_telemetry(float rl_del_ang);
void receive_del_vel_az_telemetry(float az_del_vel);
void receive_del_vel_el_telemetry(float el_del_vel);
void receive_del_vel_rl_telemetry(float rl_del_vel);

#endif /* MAVLINK_INTERFACE_H_ */
