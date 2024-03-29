#include "PM_Sensorless.h"
#include "memory_map.h"
#include "flash/flash.h"
#include "flash/flash_migrations.h"
#include "parameters/flash_params.h"
#include "hardware/watchdog.h"
#include "hardware/led.h"
#include "can/cand.h"
#include "can/cb.h"

static void flash_migration_from_0008(void);
static void flash_migration_from_0007(void);
static void flash_migration_from_0006(void);
static void flash_migration_from_0005(void);
static void flash_migration_from_0004(void);
static void flash_migration_from_0003(void);
static void flash_migration_from_0002(void);
static void flash_migration_from_0001(void);
static void flash_migration_from_0000(void);
static void flash_migration_not_possible(void);

static const LED_RGBA rgba_green_dim = {0, 0xff, 0, 1};

void flash_migration_run(const Uint16 from_rev) {
    // Override the "axis_parms.blink_state" on the elevation board to avoid moving into "BLINK_NO_COMM" too early
    CANUpdateBeaconState(LED_MODE_SOLID, rgba_green_dim, 0);

    // Handle flash param migrations *from* the id stored in flash *to* this version of the compiled firmware
    switch(from_rev) {
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

        // Last seen in v0.27.1
        case 0x0005:
            flash_migration_from_0005();
            break;

        // Last seen in v0.26.4
        case 0x0004:
            flash_migration_from_0004();
            break;

        // Last seen in aaea77faa48a592d1d0c12a6d1eb5beb3dda1b18
        case 0x0003:
            flash_migration_from_0003();
            break;

        // Last seen in 33e9f961e1f258fb2ba300c092bfd9b80d726dac
        case 0x0002:
            flash_migration_from_0002();
            break;

        // Last seen in 2e89f5a10768188863f3e456638c7966e72723d3
        case 0x0001:
            flash_migration_from_0001();
            break;

        // Last seen in 047d3dfda2072b3d7c8d4143330aabe7e7c72bb0
        case 0x0000:
            flash_migration_from_0000();
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

static void flash_migration_from_0008(void) {
    // Load the struct from flash into the old struct layout
    struct flash_param_struct_0008 flash_params_0008 = {0};
    memcpy(&flash_params_0008, (Uint16 *)PARAMS_START, sizeof(flash_params_0008));

    // Copy floats
    flash_params.ser_num_1 = flash_params_0008.ser_num_1;
    flash_params.ser_num_2 = flash_params_0008.ser_num_2;
    flash_params.ser_num_3 = flash_params_0008.ser_num_3;
    flash_params.assy_time = flash_params_0008.assy_time;
    flash_params.k_rate = flash_params_0008.k_rate;
    flash_params.gopro_charging_enabled = flash_params_0008.gopro_charging_enabled;
    flash_params.use_custom_gains = flash_params_0008.use_custom_gains;

    // Copy arrays
    memcpy(flash_params.commutation_slope, flash_params_0008.commutation_slope, sizeof(flash_params_0008.commutation_slope));
    memcpy(flash_params.commutation_icept, flash_params_0008.commutation_icept, sizeof(flash_params_0008.commutation_icept));

    flash_params.torque_pid_kp = flash_params_0008.torque_pid_kp;
    flash_params.torque_pid_ki = flash_params_0008.torque_pid_ki;
    flash_params.torque_pid_kd = flash_params_0008.torque_pid_kd;

    memcpy(flash_params.rate_pid_p, flash_params_0008.rate_pid_p, sizeof(flash_params_0008.rate_pid_p));
    memcpy(flash_params.rate_pid_i, flash_params_0008.rate_pid_i, sizeof(flash_params_0008.rate_pid_i));
    memcpy(flash_params.rate_pid_d, flash_params_0008.rate_pid_d, sizeof(flash_params_0008.rate_pid_d));

    memcpy(flash_params.offset_joint, flash_params_0008.offset_joint, sizeof(flash_params_0008.offset_joint));
    memcpy(flash_params.offset_gyro, flash_params_0008.offset_gyro, sizeof(flash_params_0008.offset_gyro));

    memcpy(flash_params.offset_accelerometer, flash_params_0008.offset_accelerometer, sizeof(flash_params_0008.offset_accelerometer));
    memcpy(flash_params.gain_accelerometer, flash_params_0008.gain_accelerometer, sizeof(flash_params_0008.gain_accelerometer));
    memcpy(flash_params.alignment_accelerometer, flash_params_0008.alignment_accelerometer, sizeof(flash_params_0008.alignment_accelerometer));

    /* Added parameters:
     *  gopro_enabled
     */
}

static void flash_migration_from_0007(void) {
    // Load the struct from flash into the old struct layout
    struct flash_param_struct_0007 flash_params_0007 = {0};
    memcpy(&flash_params_0007, (Uint16 *)PARAMS_START, sizeof(flash_params_0007));

    // Copy floats
    flash_params.ser_num_1 = flash_params_0007.ser_num_1;
    flash_params.ser_num_2 = flash_params_0007.ser_num_2;
    flash_params.ser_num_3 = flash_params_0007.ser_num_3;
    flash_params.assy_time = flash_params_0007.assy_time;
    flash_params.k_rate = flash_params_0007.k_rate;
    flash_params.gopro_charging_enabled = flash_params_0007.gopro_charging_enabled;
    flash_params.use_custom_gains = flash_params_0007.use_custom_gains;

    // Copy arrays
    memcpy(flash_params.commutation_slope, flash_params_0007.commutation_slope, sizeof(flash_params_0007.commutation_slope));
    memcpy(flash_params.commutation_icept, flash_params_0007.commutation_icept, sizeof(flash_params_0007.commutation_icept));

    flash_params.torque_pid_kp = flash_params_0007.torque_pid_kp[0];
    flash_params.torque_pid_ki = flash_params_0007.torque_pid_ki[0];
    flash_params.torque_pid_kd = flash_params_0007.torque_pid_kd[0];

    memcpy(flash_params.rate_pid_p, flash_params_0007.rate_pid_p, sizeof(flash_params_0007.rate_pid_p));
    memcpy(flash_params.rate_pid_i, flash_params_0007.rate_pid_i, sizeof(flash_params_0007.rate_pid_i));
    memcpy(flash_params.rate_pid_d, flash_params_0007.rate_pid_d, sizeof(flash_params_0007.rate_pid_d));

    memcpy(flash_params.offset_joint, flash_params_0007.offset_joint, sizeof(flash_params_0007.offset_joint));
    memcpy(flash_params.offset_gyro, flash_params_0007.offset_gyro, sizeof(flash_params_0007.offset_gyro));

    memcpy(flash_params.offset_accelerometer, flash_params_0007.offset_accelerometer, sizeof(flash_params_0007.offset_accelerometer));
    memcpy(flash_params.gain_accelerometer, flash_params_0007.gain_accelerometer, sizeof(flash_params_0007.gain_accelerometer));
    memcpy(flash_params.alignment_accelerometer, flash_params_0007.alignment_accelerometer, sizeof(flash_params_0007.alignment_accelerometer));

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

    // Copy floats
    flash_params.ser_num_1 = flash_params_0006.ser_num_1;
    flash_params.ser_num_2 = flash_params_0006.ser_num_2;
    flash_params.ser_num_3 = flash_params_0006.ser_num_3;
    flash_params.assy_time = flash_params_0006.assy_time;
    flash_params.k_rate = flash_params_0006.k_rate;
    flash_params.gopro_charging_enabled = 1.0;
    flash_params.use_custom_gains = flash_params_0006.use_custom_gains;

    // Copy arrays
    memcpy(flash_params.commutation_slope, flash_params_0006.commutation_slope, sizeof(flash_params_0006.commutation_slope));
    memcpy(flash_params.commutation_icept, flash_params_0006.commutation_icept, sizeof(flash_params_0006.commutation_icept));

    flash_params.torque_pid_kp = flash_params_0006.torque_pid_kp[0];
    flash_params.torque_pid_ki = flash_params_0006.torque_pid_ki[0];
    flash_params.torque_pid_kd = flash_params_0006.torque_pid_kd[0];

    memcpy(flash_params.rate_pid_p, flash_params_0006.rate_pid_p, sizeof(flash_params_0006.rate_pid_p));
    memcpy(flash_params.rate_pid_i, flash_params_0006.rate_pid_i, sizeof(flash_params_0006.rate_pid_i));
    memcpy(flash_params.rate_pid_d, flash_params_0006.rate_pid_d, sizeof(flash_params_0006.rate_pid_d));

    memcpy(flash_params.offset_joint, flash_params_0006.offset_joint, sizeof(flash_params_0006.offset_joint));
    memcpy(flash_params.offset_gyro, flash_params_0006.offset_gyro, sizeof(flash_params_0006.offset_gyro));

    memcpy(flash_params.offset_accelerometer, flash_params_0006.offset_accelerometer, sizeof(flash_params_0006.offset_accelerometer));
    memcpy(flash_params.gain_accelerometer, flash_params_0006.gain_accelerometer, sizeof(flash_params_0006.gain_accelerometer));
    memcpy(flash_params.alignment_accelerometer, flash_params_0006.alignment_accelerometer, sizeof(flash_params_0006.alignment_accelerometer));

    /* Removed parameters:
     *  rate_pid_windup
     * Added parameters:
     *  rate_pid_d_alpha
     */
}

static void flash_migration_from_0005(void) {
    // Load the struct from flash into the old struct layout
    struct flash_param_struct_0005 flash_params_0005 = {0};
    memcpy(&flash_params_0005, (Uint16 *)PARAMS_START, sizeof(flash_params_0005));

    // Copy floats
    flash_params.ser_num_1 = flash_params_0005.ser_num_1;
    flash_params.ser_num_2 = flash_params_0005.ser_num_2;
    flash_params.ser_num_3 = flash_params_0005.ser_num_3;
    flash_params.assy_time = flash_params_0005.assy_time;
    flash_params.k_rate = flash_params_0005.k_rate;
    flash_params.gopro_charging_enabled = 1.0;
    flash_params.use_custom_gains = flash_params_0005.use_custom_gains;

    // Copy arrays
    memcpy(flash_params.commutation_slope, flash_params_0005.commutation_slope, sizeof(flash_params_0005.commutation_slope));
    memcpy(flash_params.commutation_icept, flash_params_0005.commutation_icept, sizeof(flash_params_0005.commutation_icept));

    flash_params.torque_pid_kp = flash_params_0005.torque_pid_kp[0];
    flash_params.torque_pid_ki = flash_params_0005.torque_pid_ki[0];
    flash_params.torque_pid_kd = flash_params_0005.torque_pid_kd[0];

    memcpy(flash_params.rate_pid_p, flash_params_0005.rate_pid_p, sizeof(flash_params_0005.rate_pid_p));
    memcpy(flash_params.rate_pid_i, flash_params_0005.rate_pid_i, sizeof(flash_params_0005.rate_pid_i));
    memcpy(flash_params.rate_pid_d, flash_params_0005.rate_pid_d, sizeof(flash_params_0005.rate_pid_d));

    memcpy(flash_params.offset_joint, flash_params_0005.offset_joint, sizeof(flash_params_0005.offset_joint));
    memcpy(flash_params.offset_gyro, flash_params_0005.offset_gyro, sizeof(flash_params_0005.offset_gyro));

    memcpy(flash_params.offset_accelerometer, flash_params_0005.offset_accelerometer, sizeof(flash_params_0005.offset_accelerometer));
    memcpy(flash_params.gain_accelerometer, flash_params_0005.gain_accelerometer, sizeof(flash_params_0005.gain_accelerometer));
    memcpy(flash_params.alignment_accelerometer, flash_params_0005.alignment_accelerometer, sizeof(flash_params_0005.alignment_accelerometer));

    /* Removed parameters:
     *  sys_swver
     */
}

static void flash_migration_from_0004(void) {
    // Load the struct from flash into the old struct layout
    struct flash_param_struct_0004 flash_params_0004 = {0};
    memcpy(&flash_params_0004, (Uint16 *)PARAMS_START, sizeof(flash_params_0004));

    // Copy floats
    flash_params.ser_num_1 = flash_params_0004.ser_num_1;
    flash_params.ser_num_2 = flash_params_0004.ser_num_2;
    flash_params.ser_num_3 = flash_params_0004.ser_num_3;
    flash_params.assy_time = flash_params_0004.assy_time;
    flash_params.k_rate = flash_params_0004.k_rate;
    flash_params.gopro_charging_enabled = 1.0;

    // Copy arrays
    memcpy(flash_params.commutation_slope, flash_params_0004.commutation_slope, sizeof(flash_params_0004.commutation_slope));
    memcpy(flash_params.commutation_icept, flash_params_0004.commutation_icept, sizeof(flash_params_0004.commutation_icept));

    flash_params.torque_pid_kp = flash_params_0004.torque_pid_kp[0];
    flash_params.torque_pid_ki = flash_params_0004.torque_pid_ki[0];
    flash_params.torque_pid_kd = flash_params_0004.torque_pid_kd[0];

    memcpy(flash_params.rate_pid_p, flash_params_0004.rate_pid_p, sizeof(flash_params_0004.rate_pid_p));
    memcpy(flash_params.rate_pid_i, flash_params_0004.rate_pid_i, sizeof(flash_params_0004.rate_pid_i));
    memcpy(flash_params.rate_pid_d, flash_params_0004.rate_pid_d, sizeof(flash_params_0004.rate_pid_d));

    memcpy(flash_params.offset_joint, flash_params_0004.offset_joint, sizeof(flash_params_0004.offset_joint));
    memcpy(flash_params.offset_gyro, flash_params_0004.offset_gyro, sizeof(flash_params_0004.offset_gyro));

    memcpy(flash_params.offset_accelerometer, flash_params_0004.offset_accelerometer, sizeof(flash_params_0004.offset_accelerometer));
    memcpy(flash_params.gain_accelerometer, flash_params_0004.gain_accelerometer, sizeof(flash_params_0004.gain_accelerometer));
    memcpy(flash_params.alignment_accelerometer, flash_params_0004.alignment_accelerometer, sizeof(flash_params_0004.alignment_accelerometer));

    /* Added parameters:
     *  use_custom_gains
     */
}

static void flash_migration_from_0003(void) {
    // Load the struct from flash into the old struct layout
    struct flash_param_struct_0003 flash_params_0003 = {0};
    memcpy(&flash_params_0003, (Uint16 *)PARAMS_START, sizeof(flash_params_0003));

    // Copy floats
    flash_params.ser_num_1 = flash_params_0003.ser_num_1;
    flash_params.ser_num_2 = flash_params_0003.ser_num_2;
    flash_params.ser_num_3 = flash_params_0003.ser_num_3;
    flash_params.assy_time = flash_params_0003.assy_time;
    flash_params.k_rate = flash_params_0003.k_rate;
    flash_params.gopro_charging_enabled = 1.0;

    // Copy arrays
    memcpy(flash_params.commutation_slope, flash_params_0003.commutation_slope, sizeof(flash_params_0003.commutation_slope));
    memcpy(flash_params.commutation_icept, flash_params_0003.commutation_icept, sizeof(flash_params_0003.commutation_icept));

    flash_params.torque_pid_kp = flash_params_0003.torque_pid_kp[0];
    flash_params.torque_pid_ki = flash_params_0003.torque_pid_ki[0];
    flash_params.torque_pid_kd = flash_params_0003.torque_pid_kd[0];

    memcpy(flash_params.rate_pid_p, flash_params_0003.rate_pid_p, sizeof(flash_params_0003.rate_pid_p));
    memcpy(flash_params.rate_pid_i, flash_params_0003.rate_pid_i, sizeof(flash_params_0003.rate_pid_i));
    memcpy(flash_params.rate_pid_d, flash_params_0003.rate_pid_d, sizeof(flash_params_0003.rate_pid_d));

    memcpy(flash_params.offset_joint, flash_params_0003.offset_joint, sizeof(flash_params_0003.offset_joint));
    memcpy(flash_params.offset_gyro, flash_params_0003.offset_gyro, sizeof(flash_params_0003.offset_gyro));

    memcpy(flash_params.offset_accelerometer, flash_params_0003.offset_accelerometer, sizeof(flash_params_0003.offset_accelerometer));
    memcpy(flash_params.gain_accelerometer, flash_params_0003.gain_accelerometer, sizeof(flash_params_0003.gain_accelerometer));
    memcpy(flash_params.alignment_accelerometer, flash_params_0003.alignment_accelerometer, sizeof(flash_params_0003.alignment_accelerometer));

    /* Removed parameters:
     *  broadcast_msgs (was made volatile)
     */
}

static void flash_migration_from_0002(void) {
    // Load the struct from flash into the old struct layout
    struct flash_param_struct_0002 flash_params_0002 = {0};
    memcpy(&flash_params_0002, (Uint16 *)PARAMS_START, sizeof(flash_params_0002));

    // Copy floats
    flash_params.ser_num_1 = flash_params_0002.ser_num_1;
    flash_params.ser_num_2 = flash_params_0002.ser_num_2;
    flash_params.ser_num_3 = flash_params_0002.ser_num_3;
    flash_params.assy_time = flash_params_0002.assy_time;
    flash_params.k_rate = flash_params_0002.k_rate;

    // Copy arrays
    memcpy(flash_params.commutation_slope, flash_params_0002.commutation_slope, sizeof(flash_params_0002.commutation_slope));
    memcpy(flash_params.commutation_icept, flash_params_0002.commutation_icept, sizeof(flash_params_0002.commutation_icept));

    flash_params.torque_pid_kp = flash_params_0002.torque_pid_kp[0];
    flash_params.torque_pid_ki = flash_params_0002.torque_pid_ki[0];
    flash_params.torque_pid_kd = flash_params_0002.torque_pid_kd[0];

    memcpy(flash_params.rate_pid_p, flash_params_0002.rate_pid_p, sizeof(flash_params_0002.rate_pid_p));
    memcpy(flash_params.rate_pid_i, flash_params_0002.rate_pid_i, sizeof(flash_params_0002.rate_pid_i));
    memcpy(flash_params.rate_pid_d, flash_params_0002.rate_pid_d, sizeof(flash_params_0002.rate_pid_d));

    memcpy(flash_params.offset_joint, flash_params_0002.offset_joint, sizeof(flash_params_0002.offset_joint));
    memcpy(flash_params.offset_gyro, flash_params_0002.offset_gyro, sizeof(flash_params_0002.offset_gyro));

    memcpy(flash_params.offset_accelerometer, flash_params_0002.offset_accelerometer, sizeof(flash_params_0002.offset_accelerometer));
    memcpy(flash_params.gain_accelerometer, flash_params_0002.gain_accelerometer, sizeof(flash_params_0002.gain_accelerometer));
    memcpy(flash_params.alignment_accelerometer, flash_params_0002.alignment_accelerometer, sizeof(flash_params_0002.alignment_accelerometer));

    /* Added parameters:
     *  gopro_charging_enabled
     */
}

static void flash_migration_from_0001(void) {
    // Load the struct from flash into the old struct layout
    struct flash_param_struct_0001 flash_params_0001 = {0};
    memcpy(&flash_params_0001, (Uint16 *)PARAMS_START, sizeof(flash_params_0001));

    // Copy floats
    flash_params.ser_num_1 = flash_params_0001.ser_num_1;
    flash_params.ser_num_2 = flash_params_0001.ser_num_2;
    flash_params.ser_num_3 = flash_params_0001.ser_num_3;
    flash_params.assy_time = flash_params_0001.assy_time;
    flash_params.k_rate = flash_params_0001.k_rate;

    // Copy arrays
    memcpy(flash_params.commutation_slope, flash_params_0001.commutation_slope, sizeof(flash_params_0001.commutation_slope));
    memcpy(flash_params.commutation_icept, flash_params_0001.commutation_icept, sizeof(flash_params_0001.commutation_icept));

    flash_params.torque_pid_kp = flash_params_0001.torque_pid_kp[0];
    flash_params.torque_pid_ki = flash_params_0001.torque_pid_ki[0];
    flash_params.torque_pid_kd = flash_params_0001.torque_pid_kd[0];

    memcpy(flash_params.rate_pid_p, flash_params_0001.rate_pid_p, sizeof(flash_params_0001.rate_pid_p));
    memcpy(flash_params.rate_pid_i, flash_params_0001.rate_pid_i, sizeof(flash_params_0001.rate_pid_i));
    memcpy(flash_params.rate_pid_d, flash_params_0001.rate_pid_d, sizeof(flash_params_0001.rate_pid_d));

    memcpy(flash_params.offset_joint, flash_params_0001.offset_joint, sizeof(flash_params_0001.offset_joint));
    memcpy(flash_params.offset_accelerometer, flash_params_0001.offset_accelerometer, sizeof(flash_params_0001.offset_accelerometer));
    memcpy(flash_params.offset_gyro, flash_params_0001.offset_gyro, sizeof(flash_params_0001.offset_gyro));

    /* Added parameters:
     *  gain_accelerometer
     *  alignment_accelerometer
     */
}

static void flash_migration_from_0000(void) {
    IntOrFloat float_converter;

    // Load the struct from flash into the old struct layout
    struct flash_param_struct_0000 flash_params_0000 = {0};
    memcpy(&flash_params_0000, (Uint16 *)PARAMS_START, sizeof(flash_params_0000));

    // The following parameters were stored as uint32_t in the version 0x0000 struct
    float_converter.uint32_val = flash_params_0000.ser_num_1;
    flash_params.ser_num_1 = float_converter.float_val;

    float_converter.uint32_val = flash_params_0000.ser_num_2;
    flash_params.ser_num_2 = float_converter.float_val;

    float_converter.uint32_val = flash_params_0000.ser_num_3;
    flash_params.ser_num_3 = float_converter.float_val;

    float_converter.uint32_val = flash_params_0000.assy_time;
    flash_params.assy_time = float_converter.float_val;

    // Copy floats
    flash_params.k_rate = flash_params_0000.k_rate;

    // Copy arrays
    memcpy(flash_params.commutation_slope, flash_params_0000.AxisCalibrationSlopes, sizeof(flash_params_0000.AxisCalibrationSlopes));
    memcpy(flash_params.commutation_icept, flash_params_0000.AxisCalibrationIntercepts, sizeof(flash_params_0000.AxisCalibrationIntercepts));

    flash_params.torque_pid_kp = flash_params_0000.torque_pid_kp[0];
    flash_params.torque_pid_ki = flash_params_0000.torque_pid_ki[0];
    flash_params.torque_pid_kd = flash_params_0000.torque_pid_kd[0];

    memcpy(flash_params.rate_pid_p, flash_params_0000.rate_pid_p, sizeof(flash_params_0000.rate_pid_p));
    memcpy(flash_params.rate_pid_i, flash_params_0000.rate_pid_i, sizeof(flash_params_0000.rate_pid_i));
    memcpy(flash_params.rate_pid_d, flash_params_0000.rate_pid_d, sizeof(flash_params_0000.rate_pid_d));

    memcpy(flash_params.offset_joint, flash_params_0000.offset_joint, sizeof(flash_params_0000.offset_joint));
    memcpy(flash_params.offset_accelerometer, flash_params_0000.offset_accelerometer, sizeof(flash_params_0000.offset_accelerometer));
    memcpy(flash_params.offset_gyro, flash_params_0000.offset_gyro, sizeof(flash_params_0000.offset_gyro));

    /* Removed parameters:
     *  board_id
     *  other_id
     *  assy_date
     *  mavlink_baud_rate
     *  AxisHomePositions
     *  pos_pid_p
     *  pos_pid_i
     *  pos_pid_d
     *  pos_pid_windup
     *  balance_axis
     *  balance_step_duration
     */
}
