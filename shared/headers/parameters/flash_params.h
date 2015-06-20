#ifndef FLASH_PARAMS_H_
#define FLASH_PARAMS_H_

#include "hardware/HWSpecific.h"
#include "Flash2806x_API_Library.h"

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

extern struct flash_param_struct_0001 flash_params;


#endif /* FLASH_PARAMS_H_ */
