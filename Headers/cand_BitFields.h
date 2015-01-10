
#ifndef CAN_BITFIELDS_H
#define CAN_BITFIELDS_H

#include "HWSpecific.h"

/**
 * CAN API
 * Message IDs
 */
typedef enum {
    CAND_MID_FAULT =            0,
    CAND_MID_COMMAND,
    CAND_MID_PARAMETER_SET,
    CAND_MID_PARAMETER_QUERY,
    CAND_MID_DEBUG =            3
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

    // Two Byte Parameters
    CAND_PID_4_BYTE_CUTOFF =                            31,
    CAND_PID_COMMUTATION_CALIBRATION_HOME_OFFSET,
    CAND_PID_CORETEMP,
    CAND_PID_TORQUE,
    CAND_PID_POSITION,
    CAND_PID_TARGET_ANGLES_AZ,
    CAND_PID_TARGET_ANGLES_EL,
    CAND_PID_TARGET_ANGLES_ROLL,

    // One Byte Parameters
    CAND_PID_2_BYTE_CUTOFF =                            47,
    CAND_PID_BIT,
    CAND_PID_VOLTAGE,
    CAND_PID_USER_CONTROL_FLAGS,
    CAND_PID_BEACON_MODE,
    CAND_PID_BEACON_BRIGHTNESS,
    CAND_PID_VERSION,
    CAND_PID_LAST =                                     63
} CAND_ParameterID;

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
    CAND_FAULT_OVER_CURRENT
} CAND_FaultCode;

/**
 * CAN API
 * Commands
 */
typedef enum  {
    CAND_CMD_INIT =     0,
    CAND_CMD_ENABLE,
    CAND_CMD_RELAX,
    CAND_CMD_FIND_INDEX,
    CAND_CMD_CURRENT_MODE,
    CAND_CMD_CAL_CURRENT_OFFSET,
    CAND_CMD_ALIGN_ENCODER_TO_PHASE,
    CAND_CMD_TBD7
} CAND_Command;

#define CAND_CMD_VELOCITY_MODE CAND_CMD_FIND_INDEX

typedef enum {
    CAND_CB_CMD_INIT = 0,
    CAND_CB_CMD_ENABLE,
    CAND_CB_CMD_RELAX,
    CAND_CB_CMD_POS_MODE,
    CAND_CB_CMD_RATE_MODE,
    CAND_CB_CMD_TBD5,
    CAND_CB_CMD_TBD6,
    CAND_CB_CMD_TBD7
} CAND_CB_Command;

typedef enum {
    CAND_IFB_CMD_INIT = 0,
    CAND_IFB_CMD_ENABLE,
    CAND_IFB_CMD_TBD2,
    CAND_IFB_CMD_TBD3,
    CAND_IFB_CMD_TBD4,
    CAND_IFB_CMD_TBD5,
    CAND_IFB_CMD_TBD6,
    CAND_IFB_CMD_TBD7
} CAND_IFB_Command;

/**
 *  CAN API Built In Test Flags
 */
typedef enum {
    CAND_BIT_GOOD,
    CAND_BIT_CH1_FAULT 	 =       0x01,
    CAND_BIT_OTW	     =       0x02,
    CAND_BIT_IDEXTMOUT   =       0x04,
    CAND_BIT_INDEXNF     =       0x08,
    CAND_BIT_NOT_ENABLED =       0x10,
    CAND_BIT_AXIS_HOMED  =       0x20,
    CAND_BIT_F7          =       0x40,
    CAND_BIT_F8          =       0x80
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

#if 0
typedef enum {
    CAND_UC_STABE_HOME_MODE = 0x01,
    CAND_UC_STABE_POS_MODE  = 0x02,
    CAND_UC_STABE_MODE      = 0x04,
    CAND_UC_RECORD          = 0x08,
    CAND_UC_HOME_MODE       = 0x10,
    CAND_UC_STABE_LOS_MODE  = 0x20,
    CAND_UC_RELAX           = 0x40,
    CAND_UC_TBD             = 0x80
} CAND_UserControlFlags;
#endif

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
    CAND_INIT_NO_FIFO
} CAND_Result;


#endif /* CAN_BITFIELDS_H */
