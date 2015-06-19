#ifndef FLASH_PARAMS_H_
#define FLASH_PARAMS_H_

#include "hardware/HWSpecific.h"
#include "Flash2806x_API_Library.h"

struct flash_param_struct_0000 {
	Uint16 flash_struct_id;
	Uint32 sys_swver;
	Uint32 assy_time;
	Uint32 ser_num_1;
	Uint32 ser_num_2;
	Uint32 ser_num_3;
	float commutation_slope[AXIS_CNT];
	float commutation_icept[AXIS_CNT];
	float rate_pid_p[AXIS_CNT];
	float rate_pid_i[AXIS_CNT];
	float rate_pid_d[AXIS_CNT];
	float rate_pid_windup[AXIS_CNT];
	float torque_pid_kp[AXIS_CNT];
	float torque_pid_ki[AXIS_CNT];
	float torque_pid_kd[AXIS_CNT];
	float offset_joint[AXIS_CNT];
	float offset_accelerometer[AXIS_CNT];
	float offset_gyro[AXIS_CNT];
	float k_rate;
	float broadcast_msgs;
};

extern struct flash_param_struct_0000 flash_params;


#endif /* FLASH_PARAMS_H_ */
