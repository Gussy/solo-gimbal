#include "parameters/flash_params.h"

// ***************************************************************
// NOTE: These differ per gimbal, and are loaded from flash at boot
// time, so we default these to 0 here to force and auto-calibration
// if there are no previously saved

struct CURRENT_FLASH_PARAM_STRUCT flash_params =
{
        .flash_struct_id = PPCAT(1,CURRENT_FLASH_PARAM_STRUCT_VERSION) - 10000,
        .assy_time = 0.0,   // Unix timestamp (seconds since Jan 01 1970 UTC)
        .ser_num_1 = 0.0,   // Serial number part 1 (part code, design, language/country)
        .ser_num_2 = 0.0,   // Serial number part 2 (option, year, month)
        .ser_num_3 = 0.0,   // Serial number part 3 (incrementing serial number per month)

        .k_rate = 0.0,

        //  {EL, AZ, ROLL}
        .commutation_slope = { 0.0, 0.0, 0.0 },
        .commutation_icept = { 0.0, 0.0, 0.0 },

        //  {EL, AZ, ROLL}
        .torque_pid_kp = 0.80,
        .torque_pid_ki = 0.75,
        .torque_pid_kd = 0.00,
        .torque_pid_kr = 1.00,
        .torque_pid_km = 1.00,
        .torque_pid_c1 = 1.00,
        .torque_pid_c2 = 0.00,

        //  {EL, AZ, ROLL}
        .rate_pid_p = { 0.0, 0.0, 0.0 },
        .rate_pid_i = { 0.0, 0.0, 0.0 },
        .rate_pid_d = { 0.0, 0.0, 0.0 },
        .rate_pid_d_alpha = { 0.0, 0.0, 0.0 },

        //  { X, Y, Z }
        .offset_joint = { 0.0, 0.0, 0.0 },
        .offset_gyro = { 0.0, 0.0, 0.0 },
        .offset_accelerometer = { 0.0, 0.0, 0.0 },
        .gain_accelerometer = { 0.0, 0.0, 0.0 },
        .alignment_accelerometer = { 0.0, 0.0, 0.0 },

        .gopro_charging_enabled = 1.0,
        .use_custom_gains = 0.0
};
