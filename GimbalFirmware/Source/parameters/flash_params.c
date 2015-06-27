#include "parameters/flash_params.h"

// ***************************************************************
// NOTE: These differ per gimbal, and are loaded from flash at boot
// time, so we default these to 0 here to force and auto-calibration
// if there are no previously saved

struct flash_param_struct_0001 flash_params =
{
        .flash_struct_id = 0x0001,                          // Flash Struct ID
        .sys_swver = 0.0,                                   // Software version number, loaded from compiled in version information at boot time
        .assy_time = 0.0,                                   // Unix timestamp (seconds since Jan 01 1970 UTC)
        .ser_num_1 = 0.0,                                   // Serial number part 1 (part code, design, language/country)
        .ser_num_2 = 0.0,                                   // Serial number part 2 (option, year, month)
        .ser_num_3 = 0.0,                                   // Serial number part 3 (incrementing serial number per month)

        .broadcast_msgs = 0.0,                              // Message brodcasting
        .k_rate = 0.0,                                      // Pointing loop gain

        //  {EL, AZ, ROLL}
        .commutation_slope = { 0.0, 0.0, 0.0 },             // Axis calibration slopes
        .commutation_icept = { 0.0, 0.0, 0.0 },             // Axis calibration intercepts

        //  {EL, AZ, ROLL}
        .torque_pid_kp = { 0.80, 0.80, 0.80 },              // Torque Loop PID Kp
        .torque_pid_ki = { 0.75, 0.75, 0.75 },              // Torque Loop PID Ki
        .torque_pid_kd = { 0.00, 0.00, 0.00 },              // Torque Loop PID Kd

        //  {EL, AZ, ROLL}
        .rate_pid_p = { 0.0, 0.0, 0.0 },                    // Rate PID P gains
        .rate_pid_i = { 0.0, 0.0, 0.0 },                    // Rate PID I gains
        .rate_pid_d = { 0.0, 0.0, 0.0 },                    // Rate PID D gains
        .rate_pid_windup = { 32768.0, 32768.0, 32768.0 },   // Rate PID windup limits

        //  { X,   Y,   Z   }
        .offset_joint = { 0.0, 0.0, 0.0 },                  // offset_joint
        .offset_accelerometer = { 0.0, 0.0, 0.0 },          // offset_accelerometers
        .offset_gyro = { 0.0, 0.0, 0.0 },                   // offset_gyro
};
