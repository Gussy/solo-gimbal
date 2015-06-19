#include "parameters/flash_params.h"

// ***************************************************************
// NOTE: These differ per gimbal, and are loaded from flash at boot
// time, so we default these to 0 here to force and auto-calibration
// if there are no previously saved

struct flash_param_struct_0001 flash_params =
{
		0x0001,                     // Flash Struct ID
		0x00000000,                 // Software version number, loaded from compiled in version information at boot time
		0x00000000,                 // Unix timestamp (seconds since Jan 01 1970 UTC)
		0x00000000,                 // Serial number part 1 (part code, design, language/country)
		0x00000000,                 // Serial number part 2 (option, year, month)
		0x00000000,                 // Serial number part 3 (incrementing serial number per month)

		0.0,							// Message brodcasting
		0.0,						// Pointing loop gain

	//	{EL, AZ, ROLL}
		{ 0.0, 0.0, 0.0 },   		// Axis calibration slopes
		{ 0.0, 0.0, 0.0 },			// Axis calibration intercepts

	//	{EL, AZ, ROLL}
    	{ 0.80, 0.80, 0.80 },		// Torque Loop PID Kp
    	{ 0.75, 0.75, 0.75 },		// Torque Loop PID Ki
    	{ 0.00, 0.00, 0.00 },		// Torque Loop PID Kd

	//	{EL, AZ, ROLL}
		{ 0.0, 0.0, 0.0 },			// Rate PID P gains
		{ 0.0, 0.0, 0.0 },			// Rate PID I gains
    	{ 0.0, 0.0, 0.0 },			// Rate PID D gains
    	{ 32768.0,32768.0,32768.0 },// Rate PID windup limits

	// 	{ X,   Y,   Z   }
		{ 0.0, 0.0, 0.0 },			// offset_joint
		{ 0.0, 0.0, 0.0 },			// offset_accelerometers
		{ 0.0, 0.0, 0.0 },			// offset_gyro
};
