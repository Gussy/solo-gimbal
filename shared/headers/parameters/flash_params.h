#ifndef FLASH_PARAMS_H_
#define FLASH_PARAMS_H_

#include "hardware/HWSpecific.h"
#include "Flash2806x_API_Library.h"

#define FINAL_FLASH_PARAM_STRUCT_ID 9

struct flash_param_struct_0009 {
    Uint16 flash_struct_id;
    float assy_time;
    float ser_num_1;
    float ser_num_2;
    float ser_num_3;
    float k_rate;
    float commutation_slope[AXIS_CNT];
    float commutation_icept[AXIS_CNT];
    float torque_pid_kp;
    float torque_pid_ki;
    float torque_pid_kd;
    float torque_pid_kr;
    float torque_pid_km;
    float torque_pid_c1;
    float torque_pid_c2;
    float rate_pid_p[AXIS_CNT];
    float rate_pid_i[AXIS_CNT];
    float rate_pid_d[AXIS_CNT];
    float rate_pid_d_alpha[AXIS_CNT];
    float offset_joint[AXIS_CNT];
    float offset_gyro[AXIS_CNT];
    float offset_accelerometer[AXIS_CNT];
    float gain_accelerometer[AXIS_CNT];
    float alignment_accelerometer[AXIS_CNT];
    float gopro_charging_enabled;
    float use_custom_gains;
    float gopro_enabled;
};

struct flash_param_struct_0008 {
    Uint16 flash_struct_id;
    float assy_time;
    float ser_num_1;
    float ser_num_2;
    float ser_num_3;
    float k_rate;
    float commutation_slope[AXIS_CNT];
    float commutation_icept[AXIS_CNT];
    float torque_pid_kp;
    float torque_pid_ki;
    float torque_pid_kd;
    float torque_pid_kr;
    float torque_pid_km;
    float torque_pid_c1;
    float torque_pid_c2;
    float rate_pid_p[AXIS_CNT];
    float rate_pid_i[AXIS_CNT];
    float rate_pid_d[AXIS_CNT];
    float rate_pid_d_alpha[AXIS_CNT];
    float offset_joint[AXIS_CNT];
    float offset_gyro[AXIS_CNT];
    float offset_accelerometer[AXIS_CNT];
    float gain_accelerometer[AXIS_CNT];
    float alignment_accelerometer[AXIS_CNT];
    float gopro_charging_enabled;
    float use_custom_gains;
};

struct flash_param_struct_0007 {
    Uint16 flash_struct_id;
    float assy_time;
    float ser_num_1;
    float ser_num_2;
    float ser_num_3;
    float k_rate;
    float commutation_slope[AXIS_CNT];
    float commutation_icept[AXIS_CNT];
    float torque_pid_kp[AXIS_CNT];
    float torque_pid_ki[AXIS_CNT];
    float torque_pid_kd[AXIS_CNT];
    float rate_pid_p[AXIS_CNT];
    float rate_pid_i[AXIS_CNT];
    float rate_pid_d[AXIS_CNT];
    float rate_pid_d_alpha[AXIS_CNT];
    float offset_joint[AXIS_CNT];
    float offset_gyro[AXIS_CNT];
    float offset_accelerometer[AXIS_CNT];
    float gain_accelerometer[AXIS_CNT];
    float alignment_accelerometer[AXIS_CNT];
    float gopro_charging_enabled;
    float use_custom_gains;
};

struct flash_param_struct_0006 {
    Uint16 flash_struct_id;
    float assy_time;
    float ser_num_1;
    float ser_num_2;
    float ser_num_3;
    float k_rate;
    float commutation_slope[AXIS_CNT];
    float commutation_icept[AXIS_CNT];
    float torque_pid_kp[AXIS_CNT];
    float torque_pid_ki[AXIS_CNT];
    float torque_pid_kd[AXIS_CNT];
    float rate_pid_p[AXIS_CNT];
    float rate_pid_i[AXIS_CNT];
    float rate_pid_d[AXIS_CNT];
    float rate_pid_windup[AXIS_CNT];
    float offset_joint[AXIS_CNT];
    float offset_gyro[AXIS_CNT];
    float offset_accelerometer[AXIS_CNT];
    float gain_accelerometer[AXIS_CNT];
    float alignment_accelerometer[AXIS_CNT];
    float gopro_charging_enabled;
    float use_custom_gains;
};

struct flash_param_struct_0005 {
    Uint16 flash_struct_id;
    float sys_swver;
    float assy_time;
    float ser_num_1;
    float ser_num_2;
    float ser_num_3;
    float k_rate;
    float commutation_slope[AXIS_CNT];
    float commutation_icept[AXIS_CNT];
    float torque_pid_kp[AXIS_CNT];
    float torque_pid_ki[AXIS_CNT];
    float torque_pid_kd[AXIS_CNT];
    float rate_pid_p[AXIS_CNT];
    float rate_pid_i[AXIS_CNT];
    float rate_pid_d[AXIS_CNT];
    float rate_pid_windup[AXIS_CNT];
    float offset_joint[AXIS_CNT];
    float offset_gyro[AXIS_CNT];
    float offset_accelerometer[AXIS_CNT];
    float gain_accelerometer[AXIS_CNT];
    float alignment_accelerometer[AXIS_CNT];
    float gopro_charging_enabled;
    float use_custom_gains;
};

struct flash_param_struct_0004 {
	Uint16 flash_struct_id;
	float sys_swver;
	float assy_time;
	float ser_num_1;
	float ser_num_2;
	float ser_num_3;
	float k_rate;
	float commutation_slope[AXIS_CNT];
	float commutation_icept[AXIS_CNT];
	float torque_pid_kp[AXIS_CNT];
	float torque_pid_ki[AXIS_CNT];
	float torque_pid_kd[AXIS_CNT];
	float rate_pid_p[AXIS_CNT];
	float rate_pid_i[AXIS_CNT];
	float rate_pid_d[AXIS_CNT];
	float rate_pid_windup[AXIS_CNT];
	float offset_joint[AXIS_CNT];
	float offset_gyro[AXIS_CNT];
	float offset_accelerometer[AXIS_CNT];
	float gain_accelerometer[AXIS_CNT];
	float alignment_accelerometer[AXIS_CNT];
	float gopro_charging_enabled;
};

struct flash_param_struct_0003 {
	Uint16 flash_struct_id;
	float sys_swver;
	float assy_time;
	float ser_num_1;
	float ser_num_2;
	float ser_num_3;
	float broadcast_msgs;
	float k_rate;
	float commutation_slope[AXIS_CNT];
	float commutation_icept[AXIS_CNT];
	float torque_pid_kp[AXIS_CNT];
	float torque_pid_ki[AXIS_CNT];
	float torque_pid_kd[AXIS_CNT];
	float rate_pid_p[AXIS_CNT];
	float rate_pid_i[AXIS_CNT];
	float rate_pid_d[AXIS_CNT];
	float rate_pid_windup[AXIS_CNT];
	float offset_joint[AXIS_CNT];
	float offset_gyro[AXIS_CNT];
	float offset_accelerometer[AXIS_CNT];
	float gain_accelerometer[AXIS_CNT];
	float alignment_accelerometer[AXIS_CNT];
	float gopro_charging_enabled;
};

struct flash_param_struct_0002 {
	Uint16 flash_struct_id;
	float sys_swver;
	float assy_time;
	float ser_num_1;
	float ser_num_2;
	float ser_num_3;
	float broadcast_msgs;
	float k_rate;
	float commutation_slope[AXIS_CNT];
	float commutation_icept[AXIS_CNT];
	float torque_pid_kp[AXIS_CNT];
	float torque_pid_ki[AXIS_CNT];
	float torque_pid_kd[AXIS_CNT];
	float rate_pid_p[AXIS_CNT];
	float rate_pid_i[AXIS_CNT];
	float rate_pid_d[AXIS_CNT];
	float rate_pid_windup[AXIS_CNT];
	float offset_joint[AXIS_CNT];
	float offset_gyro[AXIS_CNT];
	float offset_accelerometer[AXIS_CNT];
	float gain_accelerometer[AXIS_CNT];
	float alignment_accelerometer[AXIS_CNT];
};

struct flash_param_struct_0001 {
	Uint16 flash_struct_id;
	float sys_swver;
	float assy_time;
	float ser_num_1;
	float ser_num_2;
	float ser_num_3;
	float broadcast_msgs;
	float k_rate;
	float commutation_slope[AXIS_CNT];
	float commutation_icept[AXIS_CNT];
	float torque_pid_kp[AXIS_CNT];
	float torque_pid_ki[AXIS_CNT];
	float torque_pid_kd[AXIS_CNT];
	float rate_pid_p[AXIS_CNT];
	float rate_pid_i[AXIS_CNT];
	float rate_pid_d[AXIS_CNT];
	float rate_pid_windup[AXIS_CNT];
	float offset_joint[AXIS_CNT];
	float offset_accelerometer[AXIS_CNT];
	float offset_gyro[AXIS_CNT];
};

struct flash_param_struct_0000 {
	Uint16 flash_struct_id;
	Uint16 board_id;
	Uint16 other_id;
	Uint32 sys_swver;
	Uint32 assy_date;
	Uint32 assy_time;
	Uint32 ser_num_1;
	Uint32 ser_num_2;
	Uint32 ser_num_3;
	Uint32 mavlink_baud_rate;
	float AxisCalibrationSlopes[AXIS_CNT];
	float AxisCalibrationIntercepts[AXIS_CNT];
	float AxisHomePositions[AXIS_CNT];
	float rate_pid_p[AXIS_CNT];
	float rate_pid_i[AXIS_CNT];
	float rate_pid_d[AXIS_CNT];
	float rate_pid_windup[AXIS_CNT];
	float pos_pid_p[AXIS_CNT];
	float pos_pid_i[AXIS_CNT];
	float pos_pid_d[AXIS_CNT];
	float pos_pid_windup[AXIS_CNT];
	float gyro_offsets[AXIS_CNT];
	float torque_pid_kp[AXIS_CNT];
	float torque_pid_ki[AXIS_CNT];
	float torque_pid_kd[AXIS_CNT];
	float offset_joint[AXIS_CNT];
	float offset_accelerometer[AXIS_CNT];
	float offset_gyro[AXIS_CNT];
	float k_rate;
	float broadcast_msgs;
	float balance_axis;
	float balance_step_duration;
};

#endif /* FLASH_PARAMS_H_ */
