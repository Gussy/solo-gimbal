#include "PM_Sensorless.h"
#include "memory_map.h"
#include "flash/flash.h"
#include "flash/flash_migrations.h"
#include "parameters/flash_params.h"
#include "parameters/kvstore.h"
#include "hardware/watchdog.h"
#include "hardware/led.h"
#include "can/cand.h"
#include "can/cb.h"

static void flash_migration_from_0009(void);
static void flash_migration_from_0008(void);
static void flash_migration_from_0007(void);
static void flash_migration_from_0006(void);
static void flash_migration_not_possible(void);

static const LED_RGBA rgba_green_dim = {0, 0xff, 0, 1};

void flash_migration_run(const Uint16 from_rev) {
    // Override the "axis_parms.blink_state" on the elevation board to avoid moving into "BLINK_NO_COMM" too early
    CANUpdateBeaconState(LED_MODE_SOLID, rgba_green_dim, 0);

    // Handle flash param migrations *from* the id stored in flash *to* this version of the compiled firmware
    switch(from_rev) {
        // Last seen in v1.1.9
        case 0x0009:
            flash_migration_from_0009();
            break;

        // Last seen in v1.1.8
        case 0x0008:
            flash_migration_from_0008();
            break;

        // Last seen in v1.0.1
        case 0x0007:
            flash_migration_from_0007();
            break;

        // Last seen in v0.28.1
        case 0x0006:
            flash_migration_from_0006();
            break;

        // Migrations are one only forwards compatible. When loading an older firmware it's
        //  not possible to determine the unknown newer struct layout, so instead of
        //  potentially harming hardware, the software will erase the incompatible firmware
        //  and reset into the bootloader.
        // This could be handled better, but downgrades should be rare
        default:
            flash_migration_not_possible();
            break;
    }
}

static void flash_migration_not_possible(void) {
    erase_our_flash();

    // Reset other axes then ourselves
    cand_tx_command(CAND_ID_ALL_AXES, CAND_CMD_RESET);
    watchdog_reset();
}

static void flash_migration_from_0009(void) {
    // Load the struct from flash into the old struct layout
    struct flash_param_struct_0009 flash_params_0009 = {0};
    memcpy(&flash_params_0009, (Uint16 *)PARAMS_START, sizeof(flash_params_0009));

    kvstore_put_float(FLASH_PARAM_ASSY_TIME, flash_params_0009.assy_time);
    kvstore_put_float(FLASH_PARAM_SER_NUM_1, flash_params_0009.ser_num_1);
    kvstore_put_float(FLASH_PARAM_SER_NUM_2, flash_params_0009.ser_num_2);
    kvstore_put_float(FLASH_PARAM_SER_NUM_3, flash_params_0009.ser_num_3);
    kvstore_put_float(FLASH_PARAM_K_RATE, flash_params_0009.k_rate);

    kvstore_put_float(FLASH_PARAM_COMMUTATION_SLOPE_AZ, flash_params_0009.commutation_slope[AZ]);
    kvstore_put_float(FLASH_PARAM_COMMUTATION_SLOPE_ROLL, flash_params_0009.commutation_slope[ROLL]);
    kvstore_put_float(FLASH_PARAM_COMMUTATION_SLOPE_EL, flash_params_0009.commutation_slope[EL]);

    kvstore_put_float(FLASH_PARAM_COMMUTATION_ICEPT_AZ, flash_params_0009.commutation_icept[AZ]);
    kvstore_put_float(FLASH_PARAM_COMMUTATION_ICEPT_ROLL, flash_params_0009.commutation_icept[ROLL]);
    kvstore_put_float(FLASH_PARAM_COMMUTATION_ICEPT_EL, flash_params_0009.commutation_icept[EL]);

    kvstore_put_float(FLASH_PARAM_TORQUE_PID_KP, flash_params_0009.torque_pid_kp);
    kvstore_put_float(FLASH_PARAM_TORQUE_PID_KI, flash_params_0009.torque_pid_ki);
    kvstore_put_float(FLASH_PARAM_TORQUE_PID_KR, flash_params_0009.torque_pid_kr);

    kvstore_put_float(FLASH_PARAM_RATE_PID_P_AZ, flash_params_0009.rate_pid_p[AZ]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_P_ROLL, flash_params_0009.rate_pid_p[ROLL]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_P_EL, flash_params_0009.rate_pid_p[EL]);

    kvstore_put_float(FLASH_PARAM_RATE_PID_I_AZ, flash_params_0009.rate_pid_i[AZ]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_I_ROLL, flash_params_0009.rate_pid_i[ROLL]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_I_EL, flash_params_0009.rate_pid_i[EL]);

    kvstore_put_float(FLASH_PARAM_RATE_PID_D_AZ, flash_params_0009.rate_pid_d[AZ]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_D_ROLL, flash_params_0009.rate_pid_d[ROLL]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_D_EL, flash_params_0009.rate_pid_d[EL]);

    kvstore_put_float(FLASH_PARAM_RATE_PID_D_ALPHA_AZ, flash_params_0009.rate_pid_d_alpha[AZ]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_D_ALPHA_ROLL, flash_params_0009.rate_pid_d_alpha[ROLL]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_D_ALPHA_EL, flash_params_0009.rate_pid_d_alpha[EL]);

    kvstore_put_float(FLASH_PARAM_OFFSET_JOINT_X, flash_params_0009.offset_joint[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_JOINT_Y, flash_params_0009.offset_joint[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_JOINT_Z, flash_params_0009.offset_joint[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_OFFSET_GYRO_X, flash_params_0009.offset_gyro[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_GYRO_Y, flash_params_0009.offset_gyro[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_GYRO_Z, flash_params_0009.offset_gyro[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_OFFSET_ACCELEROMETER_X, flash_params_0009.offset_accelerometer[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_ACCELEROMETER_Y, flash_params_0009.offset_accelerometer[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_ACCELEROMETER_Z, flash_params_0009.offset_accelerometer[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_GAIN_ACCELEROMETER_X, flash_params_0009.gain_accelerometer[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_GAIN_ACCELEROMETER_Y, flash_params_0009.gain_accelerometer[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_GAIN_ACCELEROMETER_Z, flash_params_0009.gain_accelerometer[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_ALIGNMENT_ACCELEROMETER_X, flash_params_0009.alignment_accelerometer[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_ALIGNMENT_ACCELEROMETER_Y, flash_params_0009.alignment_accelerometer[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_ALIGNMENT_ACCELEROMETER_Z, flash_params_0009.alignment_accelerometer[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_GOPRO_CHARGING_ENABLED, flash_params_0009.gopro_charging_enabled);
    kvstore_put_float(FLASH_PARAM_USE_CUSTOM_GAINS, flash_params_0009.use_custom_gains);
    kvstore_put_float(FLASH_PARAM_GOPRO_CONTROL, flash_params_0009.gopro_enabled);
}

static void flash_migration_from_0008(void) {
    // Load the struct from flash into the old struct layout
    struct flash_param_struct_0008 flash_params_0008 = {0};
    memcpy(&flash_params_0008, (Uint16 *)PARAMS_START, sizeof(flash_params_0008));

    kvstore_put_float(FLASH_PARAM_ASSY_TIME, flash_params_0008.assy_time);
    kvstore_put_float(FLASH_PARAM_SER_NUM_1, flash_params_0008.ser_num_1);
    kvstore_put_float(FLASH_PARAM_SER_NUM_2, flash_params_0008.ser_num_2);
    kvstore_put_float(FLASH_PARAM_SER_NUM_3, flash_params_0008.ser_num_3);
    kvstore_put_float(FLASH_PARAM_K_RATE, flash_params_0008.k_rate);

    kvstore_put_float(FLASH_PARAM_COMMUTATION_SLOPE_AZ, flash_params_0008.commutation_slope[AZ]);
    kvstore_put_float(FLASH_PARAM_COMMUTATION_SLOPE_ROLL, flash_params_0008.commutation_slope[ROLL]);
    kvstore_put_float(FLASH_PARAM_COMMUTATION_SLOPE_EL, flash_params_0008.commutation_slope[EL]);

    kvstore_put_float(FLASH_PARAM_COMMUTATION_ICEPT_AZ, flash_params_0008.commutation_icept[AZ]);
    kvstore_put_float(FLASH_PARAM_COMMUTATION_ICEPT_ROLL, flash_params_0008.commutation_icept[ROLL]);
    kvstore_put_float(FLASH_PARAM_COMMUTATION_ICEPT_EL, flash_params_0008.commutation_icept[EL]);

    kvstore_put_float(FLASH_PARAM_TORQUE_PID_KP, flash_params_0008.torque_pid_kp);
    kvstore_put_float(FLASH_PARAM_TORQUE_PID_KI, flash_params_0008.torque_pid_ki);
    kvstore_put_float(FLASH_PARAM_TORQUE_PID_KR, flash_params_0008.torque_pid_kr);

    kvstore_put_float(FLASH_PARAM_RATE_PID_P_AZ, flash_params_0008.rate_pid_p[AZ]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_P_ROLL, flash_params_0008.rate_pid_p[ROLL]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_P_EL, flash_params_0008.rate_pid_p[EL]);

    kvstore_put_float(FLASH_PARAM_RATE_PID_I_AZ, flash_params_0008.rate_pid_i[AZ]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_I_ROLL, flash_params_0008.rate_pid_i[ROLL]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_I_EL, flash_params_0008.rate_pid_i[EL]);

    kvstore_put_float(FLASH_PARAM_RATE_PID_D_AZ, flash_params_0008.rate_pid_d[AZ]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_D_ROLL, flash_params_0008.rate_pid_d[ROLL]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_D_EL, flash_params_0008.rate_pid_d[EL]);

    kvstore_put_float(FLASH_PARAM_RATE_PID_D_ALPHA_AZ, flash_params_0008.rate_pid_d_alpha[AZ]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_D_ALPHA_ROLL, flash_params_0008.rate_pid_d_alpha[ROLL]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_D_ALPHA_EL, flash_params_0008.rate_pid_d_alpha[EL]);

    kvstore_put_float(FLASH_PARAM_OFFSET_JOINT_X, flash_params_0008.offset_joint[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_JOINT_Y, flash_params_0008.offset_joint[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_JOINT_Z, flash_params_0008.offset_joint[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_OFFSET_GYRO_X, flash_params_0008.offset_gyro[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_GYRO_Y, flash_params_0008.offset_gyro[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_GYRO_Z, flash_params_0008.offset_gyro[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_OFFSET_ACCELEROMETER_X, flash_params_0008.offset_accelerometer[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_ACCELEROMETER_Y, flash_params_0008.offset_accelerometer[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_ACCELEROMETER_Z, flash_params_0008.offset_accelerometer[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_GAIN_ACCELEROMETER_X, flash_params_0008.gain_accelerometer[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_GAIN_ACCELEROMETER_Y, flash_params_0008.gain_accelerometer[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_GAIN_ACCELEROMETER_Z, flash_params_0008.gain_accelerometer[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_ALIGNMENT_ACCELEROMETER_X, flash_params_0008.alignment_accelerometer[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_ALIGNMENT_ACCELEROMETER_Y, flash_params_0008.alignment_accelerometer[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_ALIGNMENT_ACCELEROMETER_Z, flash_params_0008.alignment_accelerometer[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_GOPRO_CHARGING_ENABLED, flash_params_0008.gopro_charging_enabled);
    kvstore_put_float(FLASH_PARAM_USE_CUSTOM_GAINS, flash_params_0008.use_custom_gains);

    /* Added parameters:
     *  gopro_enabled
     */
}

static void flash_migration_from_0007(void) {
    // Load the struct from flash into the old struct layout
    struct flash_param_struct_0007 flash_params_0007 = {0};
    memcpy(&flash_params_0007, (Uint16 *)PARAMS_START, sizeof(flash_params_0007));

    kvstore_put_float(FLASH_PARAM_ASSY_TIME, flash_params_0007.assy_time);
    kvstore_put_float(FLASH_PARAM_SER_NUM_1, flash_params_0007.ser_num_1);
    kvstore_put_float(FLASH_PARAM_SER_NUM_2, flash_params_0007.ser_num_2);
    kvstore_put_float(FLASH_PARAM_SER_NUM_3, flash_params_0007.ser_num_3);
    kvstore_put_float(FLASH_PARAM_K_RATE, flash_params_0007.k_rate);

    kvstore_put_float(FLASH_PARAM_COMMUTATION_SLOPE_AZ, flash_params_0007.commutation_slope[AZ]);
    kvstore_put_float(FLASH_PARAM_COMMUTATION_SLOPE_ROLL, flash_params_0007.commutation_slope[ROLL]);
    kvstore_put_float(FLASH_PARAM_COMMUTATION_SLOPE_EL, flash_params_0007.commutation_slope[EL]);

    kvstore_put_float(FLASH_PARAM_COMMUTATION_ICEPT_AZ, flash_params_0007.commutation_icept[AZ]);
    kvstore_put_float(FLASH_PARAM_COMMUTATION_ICEPT_ROLL, flash_params_0007.commutation_icept[ROLL]);
    kvstore_put_float(FLASH_PARAM_COMMUTATION_ICEPT_EL, flash_params_0007.commutation_icept[EL]);

    kvstore_put_float(FLASH_PARAM_TORQUE_PID_KP, flash_params_0007.torque_pid_kp[0]);
    kvstore_put_float(FLASH_PARAM_TORQUE_PID_KI, flash_params_0007.torque_pid_ki[0]);

    kvstore_put_float(FLASH_PARAM_RATE_PID_P_AZ, flash_params_0007.rate_pid_p[AZ]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_P_ROLL, flash_params_0007.rate_pid_p[ROLL]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_P_EL, flash_params_0007.rate_pid_p[EL]);

    kvstore_put_float(FLASH_PARAM_RATE_PID_I_AZ, flash_params_0007.rate_pid_i[AZ]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_I_ROLL, flash_params_0007.rate_pid_i[ROLL]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_I_EL, flash_params_0007.rate_pid_i[EL]);

    kvstore_put_float(FLASH_PARAM_RATE_PID_D_AZ, flash_params_0007.rate_pid_d[AZ]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_D_ROLL, flash_params_0007.rate_pid_d[ROLL]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_D_EL, flash_params_0007.rate_pid_d[EL]);

    kvstore_put_float(FLASH_PARAM_RATE_PID_D_ALPHA_AZ, flash_params_0007.rate_pid_d_alpha[AZ]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_D_ALPHA_ROLL, flash_params_0007.rate_pid_d_alpha[ROLL]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_D_ALPHA_EL, flash_params_0007.rate_pid_d_alpha[EL]);

    kvstore_put_float(FLASH_PARAM_OFFSET_JOINT_X, flash_params_0007.offset_joint[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_JOINT_Y, flash_params_0007.offset_joint[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_JOINT_Z, flash_params_0007.offset_joint[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_OFFSET_GYRO_X, flash_params_0007.offset_gyro[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_GYRO_Y, flash_params_0007.offset_gyro[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_GYRO_Z, flash_params_0007.offset_gyro[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_OFFSET_ACCELEROMETER_X, flash_params_0007.offset_accelerometer[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_ACCELEROMETER_Y, flash_params_0007.offset_accelerometer[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_ACCELEROMETER_Z, flash_params_0007.offset_accelerometer[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_GAIN_ACCELEROMETER_X, flash_params_0007.gain_accelerometer[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_GAIN_ACCELEROMETER_Y, flash_params_0007.gain_accelerometer[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_GAIN_ACCELEROMETER_Z, flash_params_0007.gain_accelerometer[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_ALIGNMENT_ACCELEROMETER_X, flash_params_0007.alignment_accelerometer[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_ALIGNMENT_ACCELEROMETER_Y, flash_params_0007.alignment_accelerometer[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_ALIGNMENT_ACCELEROMETER_Z, flash_params_0007.alignment_accelerometer[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_GOPRO_CHARGING_ENABLED, flash_params_0007.gopro_charging_enabled);
    kvstore_put_float(FLASH_PARAM_USE_CUSTOM_GAINS, flash_params_0007.use_custom_gains);

    //kvstore_put_float(FLASH_PARAM_TORQUE_PID_KR, flash_params_0007.torque_pid_kr[0]);

    /* Removed parameters:
     *  rate_pid_windup
     * Added parameters:
     *  rate_pid_d_alpha
     */
}

static void flash_migration_from_0006(void) {
    // Load the struct from flash into the old struct layout
    struct flash_param_struct_0006 flash_params_0006 = {0};
    memcpy(&flash_params_0006, (Uint16 *)PARAMS_START, sizeof(flash_params_0006));

    kvstore_put_float(FLASH_PARAM_ASSY_TIME, flash_params_0006.assy_time);
    kvstore_put_float(FLASH_PARAM_SER_NUM_1, flash_params_0006.ser_num_1);
    kvstore_put_float(FLASH_PARAM_SER_NUM_2, flash_params_0006.ser_num_2);
    kvstore_put_float(FLASH_PARAM_SER_NUM_3, flash_params_0006.ser_num_3);
    kvstore_put_float(FLASH_PARAM_K_RATE, flash_params_0006.k_rate);

    kvstore_put_float(FLASH_PARAM_COMMUTATION_SLOPE_AZ, flash_params_0006.commutation_slope[AZ]);
    kvstore_put_float(FLASH_PARAM_COMMUTATION_SLOPE_ROLL, flash_params_0006.commutation_slope[ROLL]);
    kvstore_put_float(FLASH_PARAM_COMMUTATION_SLOPE_EL, flash_params_0006.commutation_slope[EL]);

    kvstore_put_float(FLASH_PARAM_COMMUTATION_ICEPT_AZ, flash_params_0006.commutation_icept[AZ]);
    kvstore_put_float(FLASH_PARAM_COMMUTATION_ICEPT_ROLL, flash_params_0006.commutation_icept[ROLL]);
    kvstore_put_float(FLASH_PARAM_COMMUTATION_ICEPT_EL, flash_params_0006.commutation_icept[EL]);

    kvstore_put_float(FLASH_PARAM_TORQUE_PID_KP, flash_params_0006.torque_pid_kp[0]);
    kvstore_put_float(FLASH_PARAM_TORQUE_PID_KI, flash_params_0006.torque_pid_ki[0]);


    kvstore_put_float(FLASH_PARAM_RATE_PID_P_AZ, flash_params_0006.rate_pid_p[AZ]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_P_ROLL, flash_params_0006.rate_pid_p[ROLL]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_P_EL, flash_params_0006.rate_pid_p[EL]);

    kvstore_put_float(FLASH_PARAM_RATE_PID_I_AZ, flash_params_0006.rate_pid_i[AZ]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_I_ROLL, flash_params_0006.rate_pid_i[ROLL]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_I_EL, flash_params_0006.rate_pid_i[EL]);

    kvstore_put_float(FLASH_PARAM_RATE_PID_D_AZ, flash_params_0006.rate_pid_d[AZ]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_D_ROLL, flash_params_0006.rate_pid_d[ROLL]);
    kvstore_put_float(FLASH_PARAM_RATE_PID_D_EL, flash_params_0006.rate_pid_d[EL]);

    kvstore_put_float(FLASH_PARAM_OFFSET_JOINT_X, flash_params_0006.offset_joint[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_JOINT_Y, flash_params_0006.offset_joint[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_JOINT_Z, flash_params_0006.offset_joint[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_OFFSET_GYRO_X, flash_params_0006.offset_gyro[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_GYRO_Y, flash_params_0006.offset_gyro[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_GYRO_Z, flash_params_0006.offset_gyro[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_OFFSET_ACCELEROMETER_X, flash_params_0006.offset_accelerometer[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_ACCELEROMETER_Y, flash_params_0006.offset_accelerometer[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_OFFSET_ACCELEROMETER_Z, flash_params_0006.offset_accelerometer[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_GAIN_ACCELEROMETER_X, flash_params_0006.gain_accelerometer[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_GAIN_ACCELEROMETER_Y, flash_params_0006.gain_accelerometer[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_GAIN_ACCELEROMETER_Z, flash_params_0006.gain_accelerometer[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_ALIGNMENT_ACCELEROMETER_X, flash_params_0006.alignment_accelerometer[X_AXIS]);
    kvstore_put_float(FLASH_PARAM_ALIGNMENT_ACCELEROMETER_Y, flash_params_0006.alignment_accelerometer[Y_AXIS]);
    kvstore_put_float(FLASH_PARAM_ALIGNMENT_ACCELEROMETER_Z, flash_params_0006.alignment_accelerometer[Z_AXIS]);

    kvstore_put_float(FLASH_PARAM_GOPRO_CHARGING_ENABLED, 1.0f);
    kvstore_put_float(FLASH_PARAM_USE_CUSTOM_GAINS, flash_params_0006.use_custom_gains);

    //kvstore_put_float(FLASH_PARAM_TORQUE_PID_KR, flash_params_0006.torque_pid_kr[0]);

    //kvstore_put_float(FLASH_PARAM_RATE_PID_D_ALPHA_AZ, flash_params_0006.rate_pid_d_alpha[AZ]);
    //kvstore_put_float(FLASH_PARAM_RATE_PID_D_ALPHA_ROLL, flash_params_0006.rate_pid_d_alpha[ROLL]);
    //kvstore_put_float(FLASH_PARAM_RATE_PID_D_ALPHA_EL, flash_params_0006.rate_pid_d_alpha[EL]);

    /* Removed parameters:
     *  rate_pid_windup
     * Added parameters:
     *  rate_pid_d_alpha
     */
}
