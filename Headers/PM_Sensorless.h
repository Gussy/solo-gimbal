/* ==============================================================================
System Name:  	PM_Sensorless

File Name:		PM_Sensorless.h

Description:	Primary system header file for the Real Implementation of Sensorless  
          		Field Orientation Control for a Three Phase Permanent-Magnet
          		Synchronous Motor 

Originator:		Digital control systems Group - Texas Instruments

Note: In this software, the default inverter is supposed to be DRV8312-EVM. 
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 02-07-2011	Version 1.0
=================================================================================  */

#ifndef _PM_SENSORLESS_H
#define _PM_SENSORLESS_H

/*-------------------------------------------------------------------------------
Next, Include project specific include files.
-------------------------------------------------------------------------------*/

#include "PeripheralHeaderIncludes.h"

// Headers for TI libraries
#define MATH_TYPE 1 // Select floating point math before we include the IQmathLib header
#include "IQmathLib.h"

#include "park.h"                           // Include header for the PARK object
#include "ipark.h"                          // Include header for the IPARK object
#include "control/pid_grando_aes_modified.h"        // Include header for the PID_GRANDO_CONTROLLER object.  Using an AES modified version of this header to fix several bugs in the original implementation
#include "clarke.h"                         // Include header for the CLARKE object
#include "control/svgen_dq_aes_modified.h"          // Include header for the SVGENDQ object.  Using an AES modified version of this header to fix an issue with global variables in the original header
#include "rampgen.h"                        // Include header for the RAMPGEN object
#include "control/rmp_cntl_aes_modified.h"          // Include header for the RMPCNTL object.  Using an AES modified version of this header to fix an issue with global variables in the original header

#ifdef DSP2803x_DEVICE_H
#include "f2803xileg_vdc_PM.h"              // Include header for the ILEG2DCBUSMEAS object
#include "f2803xpwm_PM.h"                   // Include header for the PWMGEN object
#include "f2803xpwmdac_PM.h"                // Include header for the PWMGEN object
#include "f2803xqep_PM.h"                   // Include header for the QEP object
#endif

#ifdef F2806x_DEVICE_H
#include "f2806/f2806xileg_vdc_PM.h"              // Include header for the ILEG2DCBUSMEAS object
#include "f2806/f2806xpwm_PM_aes_modified.h"      // Include header for the PWMGEN object.  Using an AES modified version of this header to fix an issue with global variables in the original header
#endif

#include "can/cand_BitFields.h"
#include "hardware/HWSpecific.h"

typedef enum {
    BLINK_NO_COMM,
    BLINK_INIT,
    BLINK_READY,
    BLINK_ERROR
} BlinkState;

typedef enum {
    BEACON_RED,
    BEACON_GREEN,
    BEACON_BLUE
} BeaconState;

typedef enum {
    READ_GYRO_PASS,
    READ_GYRO_PASS_1,
    READ_GYRO_PASS_2,
    KINEMATICS_PASS,
    ERROR_AZ_PASS,
    ERROR_EL_PASS,
    ERROR_ROLL_PASS,
    TORQUE_OUT_PASS
} RateLoopPass;

typedef struct {
    Uint32 param;
    Uint8 *sema;
} ParamSet;

#define BALANCE_PROCEDURE_ANGLE_COUNT 12

typedef struct {
    int balance_angles[AXIS_CNT][BALANCE_PROCEDURE_ANGLE_COUNT];
    int current_balance_angle_index;
    int balance_angle_counter;
    int balance_angle_counter_max;
    int current_direction;
    GimbalAxis balance_axis;
} BalanceProcedureParms;

typedef struct {
    int16 raw_theta;
    int16 virtual_counts;
    int16 virtual_counts_offset;
    float mech_theta;
    float corrected_mech_theta;
    float elec_theta;
    float calibration_mech_y0;
    float calibration_mech_y1;
    float calibration_slope;
    float calibration_intercept;
} EncoderParms;

typedef struct {
    BlinkState blink_state;
    volatile Uint16 enable_flag;
    Uint16 run_motor;
    Uint8 BIT_heartbeat_enable;
    int BIT_heartbeat_decimate;
    Uint16 all_init_params_recvd;
    Uint16 other_axis_hb_recvd[AXIS_CNT];
    Uint16 other_axis_init_params_recvd[AXIS_CNT];
} AxisParms;

typedef struct {
    int16 gyro_readings[AXIS_CNT];
    int16 corrected_gyro_readings[AXIS_CNT];
    int16 gyro_offsets[AXIS_CNT];
    int16 encoder_readings[AXIS_CNT];
    int16 motor_torques[AXIS_CNT];
    int16 unfiltered_position_errors[AXIS_CNT];
    int16 filtered_position_errors[AXIS_CNT];
    Uint8 out_of_position_deadband_positive[AXIS_CNT];
    Uint8 out_of_position_deadband_negative[AXIS_CNT];
    int16 position_deadband_hysteresis_positive[AXIS_CNT];
    int16 position_deadband_hysteresis_negative[AXIS_CNT];
    int16 axis_errors[AXIS_CNT];
    Uint16 angle_targets[AXIS_CNT];
    CAND_FaultCode last_axis_fault[AXIS_CNT];
    Uint8 encoder_value_received[AXIS_CNT];
    Uint16 axes_homed[AXIS_CNT];
    int pos_loop_2nd_stage_decimation_count;
    int16 tuning_rate_inject[AXIS_CNT];
    RateLoopPass rate_loop_pass;
    Uint8 initialized;
    Uint8 enabled;
} ControlBoardParms;

typedef struct {
    int16 debug_1;
    int16 debug_2;
    int16 debug_3;
} DebugData;

typedef union {
    Uint32 uint32_val;
    float float_val;
} IntOrFloat;

#define ROUND(x) (((x) > (floor(x) + 0.5f)) ? ceil(x) : floor(x))
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define IndexTimeOutLimit 268

int GetIndexTimeOut(void);
int GetAxisHomed(void);
Uint16 GetEnableFlag(void);
Uint16 GetAxisParmsLoaded(void);
void AxisFault(CAND_FaultCode fault_code);
int16 CorrectEncoderError(int16 raw_error);
interrupt void MainISR(void);
interrupt void GyroIntISR(void);

#endif

//===========================================================================
// No more.
//===========================================================================
