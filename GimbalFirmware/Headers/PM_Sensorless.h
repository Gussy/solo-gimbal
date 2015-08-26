#ifndef _PM_SENSORLESS_H
#define _PM_SENSORLESS_H

#include "f2806x_int8.h"
#include "PeripheralHeaderIncludes.h"

// Headers for TI libraries
#define MATH_TYPE FLOAT_MATH // Select floating point math before we include the IQmathLib header
#include "IQmathLib.h"

#include "park.h"                           // Include header for the PARK object
#include "ipark.h"                          // Include header for the IPARK object
#include "control/current_controller.h"
#include "clarke.h"                         // Include header for the CLARKE object
#include "control/svgen_dq_aes_modified.h"          // Include header for the SVGENDQ object.  Using an AES modified version of this header to fix an issue with global variables in the original header
#include "rampgen.h"                        // Include header for the RAMPGEN object
#include "control/rmp_cntl_aes_modified.h"          // Include header for the RMPCNTL object.  Using an AES modified version of this header to fix an issue with global variables in the original header

#include "f2806/f2806xileg_vdc_PM.h"              // Include header for the ILEG2DCBUSMEAS object
#include "f2806/f2806xpwm_PM_aes_modified.h"      // Include header for the PWMGEN object.  Using an AES modified version of this header to fix an issue with global variables in the original header

#include "can/cand_BitFields.h"
#include "hardware/HWSpecific.h"
#include "ardupilotmega/mavlink.h"

#include <stdbool.h>

#define USE_EL_DEBUG_UART

#ifdef USE_EL_DEBUG_UART
#define DEBUG_ON    {}
#define DEBUG_OFF   {}
#else
#define DEBUG_ON    {GpioDataRegs.GPASET.bit.GPIO29 = 1;}
#define DEBUG_OFF   {GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;}
#endif

// Units are 11915/Amp (+/-32767 counts full scale over +/-2.75A full scale)
#define LOW_TORQUE_MODE_MAX (5000.0)

// Voltagte divider R1=51k, R2=5.1K, Multiplier ratio is: (5100 + 51000) / 5100 = 11
#define VBUS_DIV_MULTIPLIER 11.0f

#define DEFAULT_GMB_PITCH_P 1.650000
#define DEFAULT_GMB_PITCH_I 0.288000
#define DEFAULT_GMB_PITCH_D 0.212000
#define DEFAULT_GMB_PITCH_D_A 0.5

#define DEFAULT_GMB_ROLL_P 6.000000
#define DEFAULT_GMB_ROLL_I 0.700000
#define DEFAULT_GMB_ROLL_D 0.000000
#define DEFAULT_GMB_ROLL_D_A 0.5

#define DEFAULT_GMB_YAW_P 6.500000
#define DEFAULT_GMB_YAW_I 0.600000
#define DEFAULT_GMB_YAW_D 0.000000
#define DEFAULT_GMB_YAW_D_A 0.5

#define DEFAULT_GMB_TRQ_P 3.500000
#define DEFAULT_GMB_TRQ_I 3600.000000
#define DEFAULT_GMB_TRQ_R 7.000000

typedef enum {
    BLINK_NO_COMM,
    BLINK_INIT,
	BLINK_RUNNING,
    BLINK_ERROR,
    BLINK_ERROR_UNRECOVERABLE,
    BLINK_CALIBRATING,
    BLINK_OVERRIDE
} BlinkState;

typedef enum {
	CONTROL_TYPE_RATE,
	CONTROL_TYPE_POS
} ControlType;

typedef struct {
    Uint32 param;
    bool sema;
} ParamSet;

typedef struct {
    int16 raw_theta;
    int16 virtual_counts;
    int32 virtual_counts_accumulator;
    Uint16 virtual_counts_accumulated;
    float mech_theta;
    float corrected_mech_theta;
    float elec_theta;
    float calibration_slope;
    float calibration_intercept;
} EncoderParms;

typedef struct {
    BlinkState blink_state;
    Uint16 enable_flag;
    Uint16 all_init_params_recvd;
    Uint16 other_axis_hb_recvd[AXIS_CNT];
    Uint16 other_axis_init_params_recvd[AXIS_CNT];
    int other_axis_enable_retry_counter;
} AxisParms;

typedef struct {
    float gyro_readings[AXIS_CNT];
    int32 integrated_raw_gyro_readings[AXIS_CNT];
    int32 integrated_raw_accel_readings[AXIS_CNT];
    int16 encoder_readings[AXIS_CNT];
    CAND_FaultCode last_axis_fault[AXIS_CNT];
    Uint8 encoder_value_received[AXIS_CNT];
    Uint16 axes_homed[AXIS_CNT];
    GIMBAL_AXIS_CALIBRATION_REQUIRED calibration_status[AXIS_CNT];
    float rate_cmd_inject[AXIS_CNT];
    float rate_cmd_inject_filtered[AXIS_CNT];
    Uint8 rate_loop_step;
    ControlType control_type;
    // As a special case, a value of 0 in max allowed torque means unlimited allowed torque
    int16 max_allowed_torque;
    Uint8 initialized;
    Uint8 enabled;
    ParamSet param_set[CAND_PID_LAST];
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
#define CLAMP_TO_BOUNDS(x, bound_lower, bound_upper) (((x) < (bound_lower)) ? (bound_lower) : (((x) > (bound_upper)) ? (bound_upper) : (x)))

#define IndexTimeOutLimit 268

interrupt void MainISR(void);
interrupt void GyroIntISR(void);
interrupt void MotorDriverFaultIntISR();
interrupt void eCAN0INT_ISR(void);

int GetIndexTimeOut(void);
int GetAxisHomed(void);
bool get_axis_enable(void);
void set_axis_enable(bool enable_flag);
Uint16 GetAxisParmsLoaded(void);
void power_down_motor(void);
void EnableAZAxis(void);
void RelaxAZAxis(void);
void SetMavlinkGimbalEnabled(void);
void SetMavlinkGimbalDisabled(void);

#endif
