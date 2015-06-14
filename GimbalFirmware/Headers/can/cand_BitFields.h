#ifndef CAN_BITFIELDS_H
#define CAN_BITFIELDS_H

#include "hardware/HWSpecific.h"

/**
 * CAN API
 * Message IDs
 */
typedef enum {
    CAND_MID_FAULT =            0,
    CAND_MID_COMMAND,
    CAND_MID_PARAMETER_SET,
    CAND_MID_PARAMETER_QUERY
} CAND_MessageID;

/**
 * CAN API
 * Destination IDs
 */
typedef enum  {
    CAND_ID_EL = EL,
    CAND_ID_AZ = AZ,
    CAND_ID_ROLL = ROLL,
    CAND_ID_ALL_AXES
} CAND_DestinationID;

typedef CAND_DestinationID CAND_SenderID;

/**
 * CAN API
 * Parameter IDs
 */
typedef enum  {
    CAND_PID_INVALID =                                  0,
    // Four Byte Parameters
    CAND_PID_RATE_EL_P =                                1,
    CAND_PID_RATE_EL_I,
    CAND_PID_RATE_EL_D,
    CAND_PID_RATE_EL_WINDUP,
    CAND_PID_RATE_AZ_P,
    CAND_PID_RATE_AZ_I,
    CAND_PID_RATE_AZ_D,
    CAND_PID_RATE_AZ_WINDUP,
    CAND_PID_RATE_RL_P,
    CAND_PID_RATE_RL_I,
    CAND_PID_RATE_RL_D,
    CAND_PID_RATE_RL_WINDUP,
    CAND_PID_COMMUTATION_CALIBRATION_SLOPE,
    CAND_PID_COMMUTATION_CALIBRATION_INTERCEPT,
    CAND_PID_TORQUE_KP,
    CAND_PID_TORQUE_KI,
    CAND_PID_TORQUE_KD,
	CAND_PID_GOPRO_GET_RESPONSE,
	CAND_PID_GOPRO_SET_REQUEST,
	CAND_PID_GOPRO_SET_RESPONSE,
	CAND_PID_GOPRO_GET_REQUEST,
	CAND_PID_BALANCE_AXIS,
	CAND_PID_BALANCE_STEP_DURATION,
	CAND_PID_GP_CMD,
    // Two Byte Parameters
    CAND_PID_4_BYTE_CUTOFF =                            36,
    CAND_PID_COMMUTATION_CALIBRATION_HOME_OFFSET,
    CAND_PID_CORETEMP,
    CAND_PID_TORQUE,
    CAND_PID_POSITION,
    CAND_PID_RATE_CMD_EL,
    CAND_PID_RATE_CMD_AZ,
    CAND_PID_RATE_CMD_RL,
    CAND_PID_DEBUG_1,
    CAND_PID_DEBUG_2,
    CAND_PID_DEBUG_3,

    // One Byte Parameters
    CAND_PID_2_BYTE_CUTOFF =                            54,
    CAND_PID_BIT,
    CAND_PID_VOLTAGE,
    CAND_PID_USER_CONTROL_FLAGS,
    CAND_PID_BEACON_MODE,
    CAND_PID_BEACON_BRIGHTNESS,
    CAND_PID_VERSION,
	CAND_PID_GOPRO_HEARTBEAT,
    CAND_PID_EXTENDED,
    CAND_PID_LAST =                                     63
} CAND_ParameterID;

typedef enum {
    CAND_EPID_GYRO_AZ_TELEMETRY = 1,
    CAND_EPID_GYRO_EL_TELEMETRY,
    CAND_EPID_GYRO_RL_TELEMETRY,
    CAND_EPID_ACCEL_AZ_TELEMETRY,
    CAND_EPID_ACCEL_EL_TELEMETRY,
    CAND_EPID_ACCEL_RL_TELEMETRY,
    CAND_EPID_ENCODER_TELEMETRY,
    CAND_EPID_ARBITRARY_DEBUG,
    CAND_EPID_CALIBRATION_PROGRESS_EL,
    CAND_EPID_CALIBRATION_PROGRESS_RL,
    CAND_EPID_FACTORY_TEST_PROGRESS,
    CAND_EPID_FACTORY_TESTS_COMPLETE,
    CAND_EPID_CALIBRATION_REQUIRED_STATUS
} CAND_ExtendedParameterID;

#define CAND_DIR_RESPONSE   0
#define CAND_DIR_QUERY      1

#define CAND_ADDR_MODE_IMMEDIATE  0
#define CAND_ADDR_MODE_EXTENDED   1

/**
 * CAND API
 * Fault codes
 */
typedef enum  {
    CAND_FAULT_NONE = 0,
    CAND_FAULT_CALIBRATING_POT,
    CAND_FAULT_FIND_STOP_TIMEOUT,
    CAND_FAULT_UNSUPPORTED_COMMAND,
    CAND_FAULT_UNSUPPORTED_PARAMETER,
    CAND_FAULT_UNKNOWN_AXIS_ID,
    CAND_FAULT_OVER_CURRENT,
    CAND_FAULT_MOTOR_DRIVER_FAULT,
    CAND_FAULT_MAX = 31
} CAND_FaultCode;

/**
 * CAN API
 * Fault types
 */
typedef enum {
    CAND_FAULT_TYPE_INFO,
    CAND_FAULT_TYPE_RECOVERABLE,
    CAND_FAULT_TYPE_UNRECOVERABLE,
    CAND_FAULT_TYPE_TBD
} CAND_FaultType;

/**
 * CAN API
 * Commands
 */
typedef enum  {
    CAND_CMD_ENABLE = 0,
    CAND_CMD_RELAX,
    CAND_CMD_GOPRO_HEARTBEAT,
    CAND_CMD_INIT,
    CAND_CMD_SET_HOME_OFFSETS,
    CAND_CMD_START_FACTORY_TESTS,
    CAND_CMD_RESET,
    CAND_CMD_CALIBRATE_AXES
} CAND_Command;

/**
 *  CAN API Built In Test Flags
 */
typedef enum {
    CAND_BIT_GOOD,
    CAND_BIT_CH1_FAULT 	        = 0x01,
    CAND_BIT_OTW	            = 0x02,
    CAND_BIT_IDEXTMOUT          = 0x04,
    CAND_BIT_INDEXNF            = 0x08,
    CAND_BIT_NOT_ENABLED        = 0x10,
    CAND_BIT_AXIS_HOMED         = 0x20,
    CAND_BIT_AXIS_PARMS_RECVD   = 0x40,
    CAND_BIT_F8                 = 0x80
} CAND_BITReg;

typedef enum {
    CAND_BEACON_OFF         = 0x00,
    CAND_BEACON_RED         = 0x01,
    CAND_BEACON_BLUE        = 0x02,
    CAND_BEACON_GREEN       = 0x04,
    CAND_BEACON_SOLID       = 0x08,
    CAND_BEACON_FAST_FLASH  = 0x10,
    CAND_BEACON_SLOW_FLASH  = 0x20,
    CAND_BEACON_ERR_FLASH   = 0x40,
    CAND_BEACON_PARTY       = 0x80
} CAND_BeaconMode;

/**
 *  CAN API Return Codes
 */
typedef enum {
    CAND_SUCCESS,
    CAND_RX_COMPLETE,
    CAND_RX_EMPTY,
    CAND_RX_PARSE_ERROR,
    CAND_RX_COMMAND,
    CAND_RX_PARAM_SET,
    CAND_RX_PARAM_QUERY,
    CAND_RX_PARAM_RESPONSE,
    CAND_RX_FAULT,
    CAND_RX_TIMEOUT,
    CAND_TX_COMPLETE,
    CAND_TX_UNSUPPORTED_PARAM,
    CAND_TX_TOO_MANY_PARAM_REQUEST_PIDS,
    CAND_INIT_NO_FIFO,
    CAND_INIT_BAD_HW_ID
} CAND_Result;


#endif /* CAN_BITFIELDS_H */
