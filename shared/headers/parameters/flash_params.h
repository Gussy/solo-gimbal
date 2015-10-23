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

#endif /* FLASH_PARAMS_H_ */
