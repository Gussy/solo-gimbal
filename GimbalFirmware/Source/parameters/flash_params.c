#include "parameters/flash_params.h"
#include "version_git.h"

struct flash_param_struct_0000 flash_params =
{
    0x0000,                     // Flash Struct ID
	0x00000000,                 // Software version number, loaded from compiled in version information at boot time
    0x00000000,                 // Assembly time
    0x00000000,                 // Serial number part 1 (part code, design, language/country)
    0x00000000,                 // Serial number part 2 (option, year, month)
    0x00000000,                 // Serial number part 3 (incrementing serial number per month)
    // ***************************************************************
    // NOTE: These differ per gimbal, and are loaded from flash at boot
    // time, so we default these to 0 here to force and auto-calibration
    // if there are no previously saved
    // Axis calibration slopes
    {
        0.0,       // EL
        0.0,       // AZ
        0.0        // ROLL
    },
    // Axis calibration intercepts
    {
        0.0,      // EL
        0.0,      // AZ
        0.0       // ROLL
    },
    // Rate PID P gains
    {
        3.5,    // EL
        4.0,    // AZ
        4.0     // ROLL
    },
    // Rate PID I gains
    {
        0.25,   // EL
        1.0,    // AZ
        0.75    // ROLL
    },
    // Rate PID D gains
    {
        0.1,    // EL
        0.1,    // AZ
        1.0     // ROLL
    },
    // Rate PID windup limits
    {
        32768.0,// EL
        32768.0,// AZ
        32768.0 // ROLL
    },
    // Torque Loop PID Kp
    {
        0.8,    // EL
        0.8,    // AZ
        0.8     // ROLL
    },
    // Torque Loop PID Ki
    {
        0.75,   // EL
        0.75,   // AZ
        0.75    // ROLL
    },
    // Torque Loop PID Kd
    {
        0.0,    // EL
        0.0,    // AZ
        0.0     // ROLL
    },
    // offset_joint
    {
        0.0,    // X
        0.0,    // Y
        0.0     // Z
    },
    // offset_accelerometers
    {
        0.0,    // X
        0.0,    // Y
        0.0     // Z
    },
    // offset_gyro
    {
        0.0,    // X
        0.0,    // Y
        0.0     // Z
    },
    0.0,			// Pointing loop gain
	0.0,				// Message brodcasting
};
