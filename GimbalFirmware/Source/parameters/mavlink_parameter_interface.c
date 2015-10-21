#include "mavlink_interface/mavlink_gimbal_interface.h"
#include "parameters/mavlink_parameter_interface.h"
#include "parameters/flash_params.h"
#include "PM_Sensorless.h"
#include "flash/flash.h"
#include "hardware/led.h"
#include "hardware/device_init.h"
#include "can/cb.h"
#include "can/cand.h"
#include "control/PID.h"
#include "gopro/gopro_interface.h"
#include "version_git.h"

#include <string.h>
#include <math.h>

GimbalMavlinkParameter gimbal_params[MAVLINK_GIMBAL_PARAM_MAX];

extern unsigned char gimbal_sysid;

// Volatile parameters (these aren't saved in the flash params struct)
float sys_swver = GitVersionFloat;
float commit_to_flash_status = 0.0;
float pos_hold = CONTROL_TYPE_POS;
float max_torque = LOW_TORQUE_MODE_MAX;
float sysid = 0.0;
float k_rate = 2.0;
float send_torques = 1.0;

void init_default_mavlink_params()
{
    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_P].param_id, "GMB_YAW_P", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_P].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_P].kvstore_key = FLASH_PARAM_RATE_PID_P_AZ;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_P].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I].param_id, "GMB_YAW_I", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I].kvstore_key = FLASH_PARAM_RATE_PID_I_AZ;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_I].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_D].param_id, "GMB_YAW_D", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_D].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_D].kvstore_key = FLASH_PARAM_RATE_PID_D_AZ;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_D].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_D_A].param_id, "GMB_YAW_D_A", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_D_A].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_D_A].kvstore_key = FLASH_PARAM_RATE_PID_D_ALPHA_AZ;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_YAW_D_A].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_P].param_id, "GMB_PITCH_P", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_P].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_P].kvstore_key = FLASH_PARAM_RATE_PID_P_EL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_P].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I].param_id, "GMB_PITCH_I", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I].kvstore_key = FLASH_PARAM_RATE_PID_I_EL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_I].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_D].param_id, "GMB_PITCH_D", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_D].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_D].kvstore_key = FLASH_PARAM_RATE_PID_D_EL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_D].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_D_A].param_id, "GMB_PITCH_D_A", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_D_A].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_D_A].kvstore_key = FLASH_PARAM_RATE_PID_D_ALPHA_EL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_PITCH_D_A].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_P].param_id, "GMB_ROLL_P", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_P].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_P].kvstore_key = FLASH_PARAM_RATE_PID_P_ROLL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_P].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I].param_id, "GMB_ROLL_I", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I].kvstore_key = FLASH_PARAM_RATE_PID_I_ROLL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_I].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_D].param_id, "GMB_ROLL_D", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_D].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_D].kvstore_key = FLASH_PARAM_RATE_PID_D_ROLL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_D].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_D_A].param_id, "GMB_ROLL_D_A", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_D_A].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_D_A].kvstore_key = FLASH_PARAM_RATE_PID_D_ALPHA_ROLL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_PID_ROLL_D_A].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_YAW_SLOPE].param_id, "GMB_YAW_SLOPE", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_YAW_SLOPE].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_YAW_SLOPE].kvstore_key = FLASH_PARAM_COMMUTATION_SLOPE_AZ;
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_YAW_SLOPE].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_YAW_INTERCEPT].param_id, "GMB_YAW_ICEPT", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_YAW_INTERCEPT].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_YAW_INTERCEPT].kvstore_key = FLASH_PARAM_COMMUTATION_ICEPT_AZ;
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_YAW_INTERCEPT].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_PITCH_SLOPE].param_id, "GMB_PITCH_SLOPE", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_PITCH_SLOPE].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_PITCH_SLOPE].kvstore_key = FLASH_PARAM_COMMUTATION_SLOPE_EL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_PITCH_SLOPE].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_PITCH_INTERCEPT].param_id, "GMB_PITCH_ICEPT", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_PITCH_INTERCEPT].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_PITCH_INTERCEPT].kvstore_key = FLASH_PARAM_COMMUTATION_ICEPT_EL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_PITCH_INTERCEPT].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_ROLL_SLOPE].param_id, "GMB_ROLL_SLOPE", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_ROLL_SLOPE].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_ROLL_SLOPE].kvstore_key = FLASH_PARAM_COMMUTATION_SLOPE_ROLL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_ROLL_SLOPE].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_ROLL_INTERCEPT].param_id, "GMB_ROLL_ICEPT", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_ROLL_INTERCEPT].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_ROLL_INTERCEPT].kvstore_key = FLASH_PARAM_COMMUTATION_ICEPT_ROLL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMUTATION_CALIBRATION_ROLL_INTERCEPT].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_TRQ_P].param_id, "GMB_TRQ_P", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_TRQ_P].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_TRQ_P].kvstore_key = FLASH_PARAM_TORQUE_PID_KP;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_TRQ_P].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_TRQ_I].param_id, "GMB_TRQ_I", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_TRQ_I].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_TRQ_I].kvstore_key = FLASH_PARAM_TORQUE_PID_KI;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_TRQ_I].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_TRQ_R].param_id, "GMB_TRQ_R", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_TRQ_R].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_TRQ_R].kvstore_key = FLASH_PARAM_TORQUE_PID_KR;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_TRQ_R].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_ASSEMBLY_DATE].param_id, "GMB_ASM_TIME", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_ASSEMBLY_DATE].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_ASSEMBLY_DATE].kvstore_key = FLASH_PARAM_ASSY_TIME;
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_ASSEMBLY_DATE].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SERIAL_NUM_PART_1].param_id, "GMB_SER_NUM_1", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SERIAL_NUM_PART_1].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SERIAL_NUM_PART_1].kvstore_key = FLASH_PARAM_SER_NUM_1;
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SERIAL_NUM_PART_1].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SERIAL_NUM_PART_2].param_id, "GMB_SER_NUM_2", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SERIAL_NUM_PART_2].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SERIAL_NUM_PART_2].kvstore_key = FLASH_PARAM_SER_NUM_2;
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SERIAL_NUM_PART_2].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SERIAL_NUM_PART_3].param_id, "GMB_SER_NUM_3", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SERIAL_NUM_PART_3].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SERIAL_NUM_PART_3].kvstore_key = FLASH_PARAM_SER_NUM_3;
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SERIAL_NUM_PART_3].access_type = GIMBAL_PARAM_READ_WRITE;

	strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GP_CHARGE].param_id, "GMB_GP_CHARGE", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
	gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GP_CHARGE].param_type = GIMBAL_PARAM_NON_VOLATILE;
	gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GP_CHARGE].kvstore_key = FLASH_PARAM_GOPRO_CHARGING_ENABLED;
	gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GP_CHARGE].access_type = GIMBAL_PARAM_READ_WRITE;

	strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_CUST_GAINS].param_id, "GMB_CUST_GAINS", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_CUST_GAINS].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_CUST_GAINS].kvstore_key = FLASH_PARAM_USE_CUSTOM_GAINS;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_CUST_GAINS].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GP_CTRL].param_id, "GMB_GP_CTRL", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GP_CTRL].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GP_CTRL].kvstore_key = FLASH_PARAM_GOPRO_CONTROL;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GP_CTRL].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_JNT_X].param_id, "GMB_OFF_JNT_X", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_JNT_X].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_JNT_X].kvstore_key = FLASH_PARAM_OFFSET_JOINT_X;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_JNT_X].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_JNT_Y].param_id, "GMB_OFF_JNT_Y", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_JNT_Y].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_JNT_Y].kvstore_key = FLASH_PARAM_OFFSET_JOINT_Y;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_JNT_Y].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_JNT_Z].param_id, "GMB_OFF_JNT_Z", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_JNT_Z].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_JNT_Z].kvstore_key = FLASH_PARAM_OFFSET_JOINT_Z;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_JNT_Z].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_ACC_X].param_id, "GMB_OFF_ACC_X", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_ACC_X].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_ACC_X].kvstore_key = FLASH_PARAM_OFFSET_ACCELEROMETER_X;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_ACC_X].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_ACC_Y].param_id, "GMB_OFF_ACC_Y", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_ACC_Y].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_ACC_Y].kvstore_key = FLASH_PARAM_OFFSET_ACCELEROMETER_Y;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_ACC_Y].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_ACC_Z].param_id, "GMB_OFF_ACC_Z", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_ACC_Z].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_ACC_Z].kvstore_key = FLASH_PARAM_OFFSET_ACCELEROMETER_Z;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_ACC_Z].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GN_ACC_X].param_id, "GMB_GN_ACC_X", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GN_ACC_X].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GN_ACC_X].kvstore_key = FLASH_PARAM_GAIN_ACCELEROMETER_X;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GN_ACC_X].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GN_ACC_Y].param_id, "GMB_GN_ACC_Y", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GN_ACC_Y].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GN_ACC_Y].kvstore_key = FLASH_PARAM_GAIN_ACCELEROMETER_Y;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GN_ACC_Y].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GN_ACC_Z].param_id, "GMB_GN_ACC_Z", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GN_ACC_Z].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GN_ACC_Z].kvstore_key = FLASH_PARAM_GAIN_ACCELEROMETER_Z;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_GN_ACC_Z].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_ALN_ACC_X].param_id, "GMB_ALN_ACC_X", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_ALN_ACC_X].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_ALN_ACC_X].kvstore_key = FLASH_PARAM_ALIGNMENT_ACCELEROMETER_X;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_ALN_ACC_X].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_ALN_ACC_Y].param_id, "GMB_ALN_ACC_Y", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_ALN_ACC_Y].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_ALN_ACC_Y].kvstore_key = FLASH_PARAM_ALIGNMENT_ACCELEROMETER_Y;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_ALN_ACC_Y].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_ALN_ACC_Z].param_id, "GMB_ALN_ACC_Z", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_ALN_ACC_Z].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_ALN_ACC_Z].kvstore_key = FLASH_PARAM_ALIGNMENT_ACCELEROMETER_Z;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_ALN_ACC_Z].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_GYRO_X].param_id, "GMB_OFF_GYRO_X", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_GYRO_X].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_GYRO_X].kvstore_key = FLASH_PARAM_OFFSET_GYRO_X;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_GYRO_X].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_GYRO_Y].param_id, "GMB_OFF_GYRO_Y", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_GYRO_Y].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_GYRO_Y].kvstore_key = FLASH_PARAM_OFFSET_GYRO_Y;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_GYRO_Y].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_GYRO_Z].param_id, "GMB_OFF_GYRO_Z", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_GYRO_Z].param_type = GIMBAL_PARAM_NON_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_GYRO_Z].kvstore_key = FLASH_PARAM_OFFSET_GYRO_Z;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_OFF_GYRO_Z].access_type = GIMBAL_PARAM_READ_WRITE;

    // Volatile parameters (key of FLASH_PARAM_KEY_COUNT indicates it is a volatile parameter)
    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SWVER].param_id, "GMB_SWVER", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SWVER].param_type = GIMBAL_PARAM_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SWVER].float_data_ptr = &sys_swver;
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SWVER].kvstore_key = FLASH_PARAM_KEY_COUNT;
    gimbal_params[MAVLINK_GIMBAL_PARAM_SYSID_SWVER].access_type = GIMBAL_PARAM_READ_ONLY;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_COMMIT_TO_FLASH].param_id, "GMB_FLASH", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMIT_TO_FLASH].param_type = GIMBAL_PARAM_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMIT_TO_FLASH].float_data_ptr = &commit_to_flash_status;
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMIT_TO_FLASH].kvstore_key = FLASH_PARAM_KEY_COUNT;
    gimbal_params[MAVLINK_GIMBAL_PARAM_COMMIT_TO_FLASH].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_POS_HOLD].param_id, "GMB_POS_HOLD", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_POS_HOLD].param_type = GIMBAL_PARAM_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_POS_HOLD].float_data_ptr = &pos_hold;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_POS_HOLD].kvstore_key = FLASH_PARAM_KEY_COUNT;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_POS_HOLD].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_MAX_TORQUE].param_id, "GMB_MAX_TORQUE", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_MAX_TORQUE].param_type = GIMBAL_PARAM_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_MAX_TORQUE].float_data_ptr = &max_torque;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_MAX_TORQUE].kvstore_key = FLASH_PARAM_KEY_COUNT;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_MAX_TORQUE].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_SND_TORQUE].param_id, "GMB_SND_TORQUE", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_SND_TORQUE].param_type = GIMBAL_PARAM_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_SND_TORQUE].float_data_ptr = &send_torques;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_SND_TORQUE].kvstore_key = FLASH_PARAM_KEY_COUNT;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_SND_TORQUE].access_type = GIMBAL_PARAM_READ_WRITE;

    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_SYSID].param_id, "GMB_SYSID", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_SYSID].param_type = GIMBAL_PARAM_VOLATILE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_SYSID].float_data_ptr = &sysid;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_SYSID].access_type = GIMBAL_PARAM_READ_WRITE;

    // Hybrid volatile/non-volatile parameters
    strncpy(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_K_RATE].param_id, "GMB_K_RATE", MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_K_RATE].float_data_ptr = &k_rate;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_K_RATE].kvstore_key = FLASH_PARAM_K_RATE;
    gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_K_RATE].access_type = GIMBAL_PARAM_READ_WRITE;
}

void handle_param_set(mavlink_message_t* received_msg, MotorDriveParms* md_parms)
{
    int param_found = -1;
    int i;

    mavlink_param_set_t decoded_msg;
    mavlink_msg_param_set_decode(received_msg, &decoded_msg);

    // Search the onboard param list for the param id being updated
    for(i = 0; i < MAVLINK_GIMBAL_PARAM_MAX; i++) {
        if (strncmp(decoded_msg.param_id, gimbal_params[i].param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN) == 0) {
            param_found = i;
            break;
        }
    }

    // If we found the param id in our param list, attempt to update the value
    if (param_found >= 0) {
        GimbalMavlinkParameter* param = &gimbal_params[param_found];

        // First, make sure we're not trying to update a read-only parameter
        if (param->access_type == GIMBAL_PARAM_READ_WRITE) {
            if (param_found == MAVLINK_GIMBAL_PARAM_COMMIT_TO_FLASH) {
            	// Special case the commit to flash param since we need to check for a specific value
                if (decoded_msg.param_value == 69.0) {
                    commit_to_flash_status = (float)write_flash();
                    send_gimbal_param(param_found);
                }
            } else if (param_found == MAVLINK_GIMBAL_PARAM_GMB_POS_HOLD) {
            	// Special case the position hold flag param since we need to validate it against the valid settings
            	// and need to send it to the EL board over CAN
            	if ((decoded_msg.param_value == CONTROL_TYPE_POS) || (decoded_msg.param_value == CONTROL_TYPE_RATE)) {
            	    *(param->float_data_ptr) = decoded_msg.param_value;
            		if (decoded_msg.param_value == CONTROL_TYPE_POS) {
            			// If we switch to position mode, we want to hold position no matter if we're receiving rate commands or not,
            			// so mark the gimbal as inactive for the purposes of mavlink control here
            			SetMavlinkGimbalDisabled();
            			// Also enable the axes here, just in case they were disabled when we entered position hold mode
            			// Enable the other axes
            			cand_tx_command(CAND_ID_ALL_AXES, CAND_CMD_ENABLE);
						// Enable ourselves
            			EnableAZAxis();
            			cand_tx_command(CAND_ID_EL, CAND_CMD_POS_MODE);
            		} else if (decoded_msg.param_value == CONTROL_TYPE_RATE) {
            			// In rate mode, we want to disable the gimbal if we lose rate commands from the copter, so we mark
            			// the gimbal as active for the purposes of mavlink control here
            			SetMavlinkGimbalEnabled();
            			cand_tx_command(CAND_ID_EL, CAND_CMD_RATE_MODE);
            		}

            		send_gimbal_param(param_found);
            	}
            } else if (param_found == MAVLINK_GIMBAL_PARAM_GMB_SYSID) {
            	// Special case sysid configuration, since it has to be range limited to an unsigned byte,
				// and floored to an integer
				Uint8 new_sysid = 0;
				if (decoded_msg.param_value < 0.0) {
					new_sysid = 0;
				} else if (decoded_msg.param_value > 255.0) {
					new_sysid = 255;
				} else {
					new_sysid = floor(decoded_msg.param_value);
				}
				*(param->float_data_ptr) = decoded_msg.param_value;
				update_mavlink_sysid(new_sysid);
				send_gimbal_param(param_found);
            } else {
                // First, make sure the type of the param being sent matches the type of the param being updated
                if (decoded_msg.param_type == MAV_PARAM_TYPE_REAL32) {
                    IntOrFloat float_converter;
                    Uint8 payload[5];

                    if (param_found == MAVLINK_GIMBAL_PARAM_GMB_K_RATE) {
                        // Special case K_RATE configuration, since it can be either volatile or non-volatile
                        float kvstore_cust_gains = kvstore_get_float(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_CUST_GAINS].kvstore_key);

                        // K_RATE is non-volatile if CUST_GAINS is 0.0 in the key-value store
                        if(kvstore_cust_gains == 0.0f) {
                            kvstore_put_float(param->kvstore_key, decoded_msg.param_value);
                        } else {
                            *(param->float_data_ptr) = decoded_msg.param_value;
                        }
                    } else {
                        float_converter.float_val = decoded_msg.param_value;
                        if(param->param_type == GIMBAL_PARAM_NON_VOLATILE) {
                            kvstore_put_float(param->kvstore_key, decoded_msg.param_value);
                        } else {
                            *(param->float_data_ptr) = decoded_msg.param_value;
                        }
                    }

                    // Send the param out over CAN to the other two boards
                    payload[0] = param_found;
                    payload[1] = (float_converter.uint32_val >> 24) & 0x000000FF;
                    payload[2] = (float_converter.uint32_val >> 16) & 0x000000FF;
                    payload[3] = (float_converter.uint32_val >>  8) & 0x000000FF;
                    payload[4] = (float_converter.uint32_val >>  0) & 0x000000FF;
                    cand_tx_extended_param(CAND_ID_ALL_AXES, CAND_EPID_MAVLINK_PARAM, payload, sizeof(payload));

                    update_local_params_from_kvstore(md_parms);

                    // Echo the new value of the param back to acknowledge receipt of the param
                    send_gimbal_param(param_found);
                }
            }
        } else {
            // If this is a read only parameter, echo back the current value to indicate that we're not going to update it
            send_gimbal_param(param_found);
        }
    }
}

void handle_param_read(mavlink_message_t* received_msg)
{
    mavlink_param_request_read_t decoded_msg;
    mavlink_msg_param_request_read_decode(received_msg, &decoded_msg);

    // First check if the parameter was requested by index
    if ((decoded_msg.param_index >= 0) && (decoded_msg.param_index < MAVLINK_GIMBAL_PARAM_MAX)) {
        send_gimbal_param(decoded_msg.param_index);
    } else {
        // Search the onboard param list for the param id being requested
        int param_found = -1;
        int i;
        for(i = 0; i < MAVLINK_GIMBAL_PARAM_MAX; i++) {
            if (strncmp(decoded_msg.param_id, gimbal_params[i].param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN) == 0) {
                param_found = i;
                break;
            }
        }

        if (param_found >= 0) {
            send_gimbal_param(param_found);
        }
    }
}

void send_gimbal_param(int param_num)
{
    GimbalMavlinkParameter* param = &(gimbal_params[param_num]);

    static mavlink_message_t param_msg;
    float param_val;

    if (param_num == MAVLINK_GIMBAL_PARAM_GMB_K_RATE) {
        // Special case K_RATE configuration, since it can be either volatile or non-volatile
        float kvstore_cust_gains = kvstore_get_float(gimbal_params[MAVLINK_GIMBAL_PARAM_GMB_CUST_GAINS].kvstore_key);

        // K_RATE is non-volatile if CUST_GAINS is 0.0 in the key-value store
        if(kvstore_cust_gains == 0.0f) {
            param->param_type = GIMBAL_PARAM_NON_VOLATILE;
        } else {
            param->param_type = GIMBAL_PARAM_VOLATILE;
        }
    }

    if(param->param_type == GIMBAL_PARAM_NON_VOLATILE) {
        param_val = kvstore_get_float(param->kvstore_key);
    } else {
        param_val = *(param->float_data_ptr);
    }

    mavlink_msg_param_value_pack(gimbal_sysid, MAV_COMP_ID_GIMBAL, &param_msg, param->param_id, param_val, param->param_type, MAVLINK_GIMBAL_PARAM_MAX, param_num);
    send_mavlink_message(&param_msg);
}

void gimbal_param_update(GimbalMavlinkParameterID param_id, Uint32 value, ControlBoardParms *cb_parms)
{
    IntOrFloat float_converter;
    GimbalMavlinkParameter* param = &(gimbal_params[param_id]);

    // Update the flash_param field with the new value
    float_converter.uint32_val = value;
    if(param->param_type == GIMBAL_PARAM_NON_VOLATILE) {
        kvstore_put_float(param->kvstore_key, float_converter.float_val);
    } else {
        *(param->float_data_ptr) = float_converter.float_val;
    }

    // Special case handling of mavlink params on the EL board
    if(GetBoardHWID() == EL) {
        // If the mavlink param is a PID we should reset the PID values
        if(param_id <= MAVLINK_GIMBAL_PARAM_PID_YAW_D_A) {
            rate_pid_loop_float[AZ].integralCumulative = 0.0;
            rate_pid_loop_float[AZ].processVarPrevious = 0.0;
        } else if(param_id >= MAVLINK_GIMBAL_PARAM_PID_PITCH_P && param_id <= MAVLINK_GIMBAL_PARAM_PID_PITCH_D_A) {
            rate_pid_loop_float[EL].integralCumulative = 0.0;
            rate_pid_loop_float[EL].processVarPrevious = 0.0;
        } else if(param_id >= MAVLINK_GIMBAL_PARAM_PID_ROLL_P && param_id <= MAVLINK_GIMBAL_PARAM_PID_ROLL_D_A) {
            rate_pid_loop_float[ROLL].integralCumulative = 0.0;
            rate_pid_loop_float[ROLL].processVarPrevious = 0.0;
        } else if(param_id == MAVLINK_GIMBAL_PARAM_GMB_MAX_TORQUE) {
            // Special case max allowed torque, since we need to bounds limit it
            if (float_converter.float_val > 32767.0) {
                cb_parms->max_allowed_torque = 32767;
            } else if (float_converter.float_val < 0) {
                cb_parms->max_allowed_torque = 0;
            } else {
                cb_parms->max_allowed_torque = (int16)floor(float_converter.float_val);
            }
        }
    }
}
