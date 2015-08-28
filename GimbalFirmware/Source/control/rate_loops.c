#include "control/rate_loops.h"
#include "can/cb.h"
#include "can/cand.h"
#include "control/PID.h"
#include "hardware/gyro.h"
#include "hardware/encoder.h"
#include "control/filt2p.h"
#include "control/gyro_kinematics_correction.h"
#include "PM_Sensorless-Settings.h"

#define RATE_LOOP_KHZ 2 // valid settings: 1, 2, 4

static const int TorqueSignMap[AXIS_CNT] = {
        1, // EL
        -1, // AZ
        -1  // ROLL
};

static const int IMUSignMap[AXIS_CNT] = {
        1, // EL
        1, // AZ
        -1  // ROLL
};

// Map gyro axes to gimbal axes
static const GimbalAxis GyroAxisMap[AXIS_CNT] = {
        AZ,
        EL,
        ROLL
};

static const double RATE_UPSAMPLING_ALPHA = 0.1;

static void SendDeltaAngleTelemetry(uint32_t az_gyro, uint32_t el_gyro, uint32_t rl_gyro);
static void SendDeltaVelocityTelemetry(uint32_t az_accel, uint32_t el_accel, uint32_t rl_accel);
static void SendEncoderTelemetry(int16 az_encoder, int16 el_encoder, int16 rl_encoder);
static void SendTorqueCmdTelemetry(int16 az_torque_cmd, int16 el_torque_cmd, int16 rl_torque_cmd);

static float CorrectEncoderError(float raw_error);
static float CalculatePosHoldGain(float encoder_error);

// Compensation for gimbal  structural dyamics
#define ROLL_FILTER_NUM_SECTIONS 10
struct Filt2p_Params roll_torque_filt_params[ROLL_FILTER_NUM_SECTIONS] = {
{.b0 =1.000000e+00 , .b1 =-8.566533e-01 , .b2 =3.501994e-01 , .a1 =-1.282965e+00 , .a2 =4.633703e-01},
{.b0 =1.000000e+00 , .b1 =5.400214e-01 , .b2 =8.281463e-01 , .a1 =5.425471e-01 , .a2 =6.595636e-01},
{.b0 =1.000000e+00 , .b1 =8.918936e-01 , .b2 =7.655044e-01 , .a1 =9.236715e-01 , .a2 =7.708803e-01},
{.b0 =1.000000e+00 , .b1 =-1.168482e+00 , .b2 =7.838612e-01 , .a1 =-7.123734e-01 , .a2 =7.858003e-01},
{.b0 =1.000000e+00 , .b1 =-1.566138e+00 , .b2 =8.970805e-01 , .a1 =-1.577659e+00 , .a2 =8.070947e-01},
{.b0 =1.000000e+00 , .b1 =-2.242559e-02 , .b2 =7.685448e-01 , .a1 =-2.472262e-01 , .a2 =8.165589e-01},
{.b0 =1.000000e+00 , .b1 =-1.665028e+00 , .b2 =8.269451e-01 , .a1 =-1.702052e+00 , .a2 =8.510257e-01},
{.b0 =1.000000e+00 , .b1 =1.095402e+00 , .b2 =8.778571e-01 , .a1 =1.099350e+00 , .a2 =8.821301e-01},
{.b0 =1.000000e+00 , .b1 =-4.184922e-03 , .b2 =8.825162e-01 , .a1 =-5.658610e-02 , .a2 =8.842939e-01},
{.b0 =3.314497e-01 , .b1 =-3.427625e-01 , .b2 =2.982497e-01 , .a1 =-1.045457e+00 , .a2 =8.987932e-01}
};
struct Filt2p_State roll_torque_filt_state[ROLL_FILTER_NUM_SECTIONS];

// Compensation for gimbal stuctural dynamics
#define YAW_FILTER_NUM_SECTIONS 15
struct Filt2p_Params yaw_torque_filt_params[YAW_FILTER_NUM_SECTIONS] = {
{.b0 =1.000000e+00 , .b1 =1.214647e+00 , .b2 =2.520989e-01 , .a1 =8.061597e-02 , .a2 =-1.471932e-02},
{.b0 =1.000000e+00 , .b1 =1.055248e+00 , .b2 =2.747366e-01 , .a1 =1.020022e+00 , .a2 =2.508800e-01},
{.b0 =1.000000e+00 , .b1 =7.923983e-01 , .b2 =-9.998669e-04 , .a1 =5.234268e-02 , .a2 =-5.317662e-01},
{.b0 =1.000000e+00 , .b1 =-6.936477e-02 , .b2 =-7.905107e-01 , .a1 =3.153714e-02 , .a2 =-7.258027e-01},
{.b0 =1.000000e+00 , .b1 =-1.149175e+00 , .b2 =3.559917e-01 , .a1 =5.086368e-02 , .a2 =-8.481562e-01},
{.b0 =1.000000e+00 , .b1 =-1.155860e+00 , .b2 =8.412208e-01 , .a1 =-1.169164e+00 , .a2 =8.410038e-01},
{.b0 =1.000000e+00 , .b1 =-1.575801e+00 , .b2 =6.456572e-01 , .a1 =-1.726068e+00 , .a2 =8.411660e-01},
{.b0 =1.000000e+00 , .b1 =-1.236329e+00 , .b2 =6.645562e-01 , .a1 =-1.652017e+00 , .a2 =8.540979e-01},
{.b0 =1.000000e+00 , .b1 =-1.634496e+00 , .b2 =8.296250e-01 , .a1 =-1.566939e+00 , .a2 =8.700252e-01},
{.b0 =1.000000e+00 , .b1 =-1.871202e+00 , .b2 =9.484462e-01 , .a1 =-1.764457e+00 , .a2 =8.793505e-01},
{.b0 =1.000000e+00 , .b1 =-1.477828e+00 , .b2 =9.369412e-01 , .a1 =-1.486371e+00 , .a2 =8.873036e-01},
{.b0 =1.000000e+00 , .b1 =-1.416191e+00 , .b2 =8.508510e-01 , .a1 =-1.436453e+00 , .a2 =8.896694e-01},
{.b0 =1.000000e+00 , .b1 =-1.925149e+00 , .b2 =9.588014e-01 , .a1 =-1.896677e+00 , .a2 =9.149202e-01},
{.b0 =1.000000e+00 , .b1 =-1.139846e+00 , .b2 =9.224731e-01 , .a1 =-1.139118e+00 , .a2 =9.200954e-01},
{.b0 =2.268441e-01 , .b1 =4.489771e-01 , .b2 =2.228900e-01 , .a1 =1.978764e+00 , .a2 =9.821094e-01}
};
struct Filt2p_State yaw_torque_filt_state[YAW_FILTER_NUM_SECTIONS];

// Compensaton for gimbal stuctural dyamics
#define PITCH_FILTER_NUM_SECTIONS 5
struct Filt2p_Params pitch_torque_filt_params[PITCH_FILTER_NUM_SECTIONS] = {
{.b0 =1.000000e+00 , .b1 =3.947891e-01 , .b2 =6.352391e-02 , .a1 =2.233081e-01 , .a2 =2.708713e-02},
{.b0 =1.000000e+00 , .b1 =-1.777528e-01 , .b2 =5.092682e-01 , .a1 =-3.354577e-01 , .a2 =4.698488e-01},
{.b0 =1.000000e+00 , .b1 =2.970381e-01 , .b2 =7.841214e-01 , .a1 =1.414730e-01 , .a2 =5.498336e-01},
{.b0 =1.000000e+00 , .b1 =8.981163e-01 , .b2 =6.995164e-01 , .a1 =9.343154e-01 , .a2 =7.226448e-01},
{.b0 =6.089209e-01 , .b1 =-4.512564e-01 , .b2 =4.368166e-01 , .a1 =-7.568603e-01 , .a2 =7.358947e-01},
};
struct Filt2p_State pitch_torque_filt_state[PITCH_FILTER_NUM_SECTIONS];

struct Filt2p_Params gyro_filt_params;
struct Filt2p_State gyro_filt_state[AXIS_CNT];

Uint16 telemetry_decimation_count = 0;
Uint16 torque_cmd_telemetry_decimation_count = 5; // Start this at 5 so it's staggered with respect to the rest of the telemetry

static int16 raw_accel_measurement[AXIS_CNT] = {0, 0, 0}; // raw accelerometer measurements in body frame

static int16 raw_gyro_measurement[AXIS_CNT] = {0, 0, 0}; // raw gyro measurements in body frame
static float filtered_gyro_measurement[AXIS_CNT] = {0, 0, 0}; // filtered gyro measurements in body frame
static float filtered_gyro_measurement_jf[AXIS_CNT] = {0, 0, 0}; // euler angle derivatives
static float setpoints[AXIS_CNT] = {0, 0, 0}; // position targets

static int32 summed_raw_gyro[AXIS_CNT] = {0, 0, 0};
static int32 summed_raw_accel[AXIS_CNT] = {0, 0, 0};

static int32 summed_torque_cmd[AXIS_CNT] = {0, 0, 0};
static Uint16 summed_torque_cmd_count = 0;

static Uint16 rate_loop_step = 0;
static Uint16 telem_step = 0;

void InitRateLoops(void)
{
    calc_butter2p(8000, 1000, &gyro_filt_params);
    memset(&gyro_filt_state, 0, sizeof(gyro_filt_state));

#ifdef ROLL_FILTER_NUM_SECTIONS
    memset(&roll_torque_filt_state, 0, sizeof(roll_torque_filt_state));
#endif
#ifdef PITCH_FILTER_NUM_SECTIONS
    memset(&pitch_torque_filt_state, 0, sizeof(pitch_torque_filt_state));
#endif
#ifdef YAW_FILTER_NUM_SECTIONS
    memset(&yaw_torque_filt_state, 0, sizeof(yaw_torque_filt_state));
#endif
}

void RunRateLoops(ControlBoardParms* cb_parms)
{
    Uint8 i;
    ReadGyro(&(raw_gyro_measurement[GyroAxisMap[X_AXIS]]), &(raw_gyro_measurement[GyroAxisMap[Y_AXIS]]), &(raw_gyro_measurement[GyroAxisMap[Z_AXIS]]));

    for (i=0; i<AXIS_CNT; i++) {
        raw_gyro_measurement[i] *= IMUSignMap[i];
        summed_raw_gyro[i] += raw_gyro_measurement[i];
        filtered_gyro_measurement[i] = update_filt2p(&gyro_filt_params, &(gyro_filt_state[i]), raw_gyro_measurement[i]);
    }
    rate_loop_step++;

    switch(rate_loop_step) {
        case 1: { // gyro and control loops
            float torque_limit = cb_parms->max_allowed_torque != 0?cb_parms->max_allowed_torque : 32767.0;
            float torque_out[AXIS_CNT];

            // Do gyro kinematics correction
            transform_ang_vel_to_joint_rate(filtered_gyro_measurement, filtered_gyro_measurement_jf);

            if (cb_parms->control_type == CONTROL_TYPE_POS) {
                for (i=0; i<AXIS_CNT; i++) {
                    float encoder_error = CorrectEncoderError(-cb_parms->encoder_readings[i]);
                    float pos_gain = CalculatePosHoldGain(encoder_error);
                    setpoints[i] = pos_gain * encoder_error;
                }
            } else {
                for (i=0; i<AXIS_CNT; i++) {
                    // low-pass filter to do the upsampling between the 100Hz telemetry and 1kHz rate loop
                    cb_parms->rate_cmd_inject_filtered[i] += (cb_parms->rate_cmd_inject[i] - cb_parms->rate_cmd_inject_filtered[i]) * RATE_UPSAMPLING_ALPHA;
                    setpoints[i] = cb_parms->rate_cmd_inject_filtered[i];
                }
            }

            // Compute the new motor torque commands
            torque_out[EL] = UpdatePID_Float(EL, setpoints[EL], filtered_gyro_measurement_jf[EL], torque_limit, 1.0f, 0.0f);
            torque_out[AZ] = UpdatePID_Float(AZ, setpoints[AZ], filtered_gyro_measurement_jf[AZ], torque_limit, cos_phi*cos_phi, 0.0f);
            torque_out[ROLL] = UpdatePID_Float(ROLL, setpoints[ROLL], filtered_gyro_measurement_jf[ROLL], torque_limit, 1.0f, 0.0f);

            #ifdef PITCH_FILTER_NUM_SECTIONS
            for(i=0; i<PITCH_FILTER_NUM_SECTIONS; i++) {
                torque_out[EL] = update_filt2p(&(pitch_torque_filt_params[i]), &(pitch_torque_filt_state[i]), torque_out[EL]);
            }
            #endif

            #ifdef ROLL_FILTER_NUM_SECTIONS
            for(i=0; i<ROLL_FILTER_NUM_SECTIONS; i++) {
                torque_out[ROLL] = update_filt2p(&(roll_torque_filt_params[i]), &(roll_torque_filt_state[i]), torque_out[ROLL]);
            }
            #endif

            #ifdef YAW_FILTER_NUM_SECTIONS
            for(i=0; i<YAW_FILTER_NUM_SECTIONS; i++) {
                torque_out[AZ] = update_filt2p(&(yaw_torque_filt_params[i]), &(yaw_torque_filt_state[i]), torque_out[AZ]);
            }
            #endif

            torque_out[AZ] += torque_out[EL] * sin_phi;

            for (i=0; i<AXIS_CNT; i++) {
                if (torque_out[i] > torque_limit) {
                    torque_out[i] = torque_limit;
                } else if (torque_out[i] < -torque_limit) {
                    torque_out[i] = -torque_limit;
                }
                torque_out[i] *= TorqueSignMap[i];
                summed_torque_cmd[i] += (int16)torque_out[i];
            }
            summed_torque_cmd_count++;

            // Send out motor torques
            MDBSendTorques(torque_out[AZ], torque_out[ROLL]);

            // Also update our own torque (fake like we got a value over CAN)
            cb_parms->param_set[CAND_PID_TORQUE].param = (int16)torque_out[EL];
            cb_parms->param_set[CAND_PID_TORQUE].sema = true;
            break;
        }
        case 2: { // accel and telem
            ReadAccel(&(raw_accel_measurement[GyroAxisMap[X_AXIS]]), &(raw_accel_measurement[GyroAxisMap[Y_AXIS]]), &(raw_accel_measurement[GyroAxisMap[Z_AXIS]]));
            for (i=0; i<AXIS_CNT; i++) {
                raw_accel_measurement[i] *= IMUSignMap[i];
                summed_raw_accel[i] += raw_accel_measurement[i];
            }

            telem_step++;
            switch(telem_step) {
                case 1: { // gyro
                    IntOrFloat delta_angle[AXIS_CNT];
                    memset(&delta_angle, 0, sizeof(delta_angle));
                    for (i=0; i<AXIS_CNT; i++) {
                        delta_angle[i].float_val = GYRO_FORMAT_TO_RAD_S(summed_raw_gyro[i]) / (float)RATE_LOOP_FREQUENCY_HZ;
                    }
                    memset(&summed_raw_gyro, 0, sizeof(summed_raw_gyro));

                    SendDeltaAngleTelemetry(delta_angle[AZ].uint32_val,
                                            delta_angle[EL].uint32_val,
                                            delta_angle[ROLL].uint32_val);

                    break;
                }
                case 2: { // accel
                    IntOrFloat delta_velocity[AXIS_CNT] = {0, 0, 0};
                    for (i=0; i<AXIS_CNT; i++) {
                        delta_velocity[i].float_val = ACCEL_FORMAT_TO_MSS(summed_raw_accel[i]) / ((float)RATE_LOOP_FREQUENCY_HZ/8.0);
                    }
                    memset(&summed_raw_accel, 0, sizeof(summed_raw_accel));

                    SendDeltaVelocityTelemetry(delta_velocity[AZ].uint32_val,
                                               delta_velocity[EL].uint32_val,
                                               delta_velocity[ROLL].uint32_val);

                    break;
                }
                case 3: { // encoder
                    SendEncoderTelemetry(cb_parms->encoder_readings[AZ], cb_parms->encoder_readings[EL], cb_parms->encoder_readings[ROLL]);
                    break;
                }
                case 4: { // torque
                    SendTorqueCmdTelemetry(summed_torque_cmd[AZ] / summed_torque_cmd_count,
                                           summed_torque_cmd[EL] / summed_torque_cmd_count,
                                           summed_torque_cmd[ROLL] / summed_torque_cmd_count);
                    memset(&summed_torque_cmd, 0, sizeof(summed_torque_cmd));
                    summed_torque_cmd_count = 0;
                    break;
                }
                case 10*RATE_LOOP_KHZ: {
                    telem_step = 0;
                    break;
                }
            };
            break;
        }
    };

    if (rate_loop_step >= 8/RATE_LOOP_KHZ) {
        rate_loop_step = 0;
    }
}

static void SendDeltaAngleTelemetry(uint32_t az_gyro, uint32_t el_gyro, uint32_t rl_gyro)
{
    Uint8 gyro_az_readings[4];
    Uint8 gyro_el_readings[4];
    Uint8 gyro_rl_readings[4];
    gyro_az_readings[0] = (az_gyro >> 24) & 0x000000FF;
    gyro_az_readings[1] = (az_gyro >> 16) & 0x000000FF;
    gyro_az_readings[2] = (az_gyro >> 8) & 0x000000FF;
    gyro_az_readings[3] = (az_gyro & 0x000000FF);
    gyro_el_readings[0] = (el_gyro >> 24) & 0x000000FF;
    gyro_el_readings[1] = (el_gyro >> 16) & 0x000000FF;
    gyro_el_readings[2] = (el_gyro >> 8) & 0x000000FF;
    gyro_el_readings[3] = (el_gyro & 0x000000FF);
    gyro_rl_readings[0] = (rl_gyro >> 24) & 0x000000FF;
    gyro_rl_readings[1] = (rl_gyro >> 16) & 0x000000FF;
    gyro_rl_readings[2] = (rl_gyro >> 8) & 0x000000FF;
    gyro_rl_readings[3] = (rl_gyro & 0x000000FF);
    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_DEL_ANG_AZ_TELEMETRY, gyro_az_readings, 4);
    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_DEL_ANG_EL_TELEMETRY, gyro_el_readings, 4);
    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_DEL_ANG_RL_TELEMETRY, gyro_rl_readings, 4);
}

static void SendDeltaVelocityTelemetry(uint32_t az_accel, uint32_t el_accel, uint32_t rl_accel)
{
    Uint8 accel_az_readings[4];
    Uint8 accel_el_readings[4];
    Uint8 accel_rl_readings[4];
    accel_az_readings[0] = (az_accel >> 24) & 0x000000FF;
    accel_az_readings[1] = (az_accel >> 16) & 0x000000FF;
    accel_az_readings[2] = (az_accel >> 8) & 0x000000FF;
    accel_az_readings[3] = (az_accel & 0x000000FF);
    accel_el_readings[0] = (el_accel >> 24) & 0x000000FF;
    accel_el_readings[1] = (el_accel >> 16) & 0x000000FF;
    accel_el_readings[2] = (el_accel >> 8) & 0x000000FF;
    accel_el_readings[3] = (el_accel & 0x000000FF);
    accel_rl_readings[0] = (rl_accel >> 24) & 0x000000FF;
    accel_rl_readings[1] = (rl_accel >> 16) & 0x000000FF;
    accel_rl_readings[2] = (rl_accel >> 8) & 0x000000FF;
    accel_rl_readings[3] = (rl_accel & 0x000000FF);
    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_DEL_VEL_AZ_TELEMETRY, accel_az_readings, 4);
    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_DEL_VEL_EL_TELEMETRY, accel_el_readings, 4);
    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_DEL_VEL_RL_TELEMETRY, accel_rl_readings, 4);
}

static void SendEncoderTelemetry(int16 az_encoder, int16 el_encoder, int16 rl_encoder)
{
    Uint8 encoder_readings[6];
    encoder_readings[0] = (az_encoder >> 8) & 0x00FF;
    encoder_readings[1] = (az_encoder & 0x00FF);
    encoder_readings[2] = (el_encoder >> 8) & 0x00FF;
    encoder_readings[3] = (el_encoder & 0x00FF);
    encoder_readings[4] = (rl_encoder >> 8) & 0x00FF;
    encoder_readings[5] = (rl_encoder & 0x00FF);

    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_ENCODER_TELEMETRY, encoder_readings, 6);
}

static void SendTorqueCmdTelemetry(int16 az_torque_cmd, int16 el_torque_cmd, int16 rl_torque_cmd)
{
    Uint8 torque_cmds[6];
    torque_cmds[0] = (az_torque_cmd >> 8) & 0x00FF;
    torque_cmds[1] = (az_torque_cmd & 0x00FF);
    torque_cmds[2] = (el_torque_cmd >> 8) & 0x00FF;
    torque_cmds[3] = (el_torque_cmd & 0x00FF);
    torque_cmds[4] = (rl_torque_cmd >> 8) & 0x00FF;
    torque_cmds[5] = (rl_torque_cmd & 0x00FF);

    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_TORQUE_CMD_TELEMETRY, torque_cmds, 6);
}

static float CorrectEncoderError(float raw_error)
{
    if (raw_error < -(ENCODER_COUNTS_PER_REV / 2)) {
        return raw_error + ENCODER_COUNTS_PER_REV;
    } else if (raw_error > (ENCODER_COUNTS_PER_REV / 2)) {
        return raw_error - ENCODER_COUNTS_PER_REV;
    } else {
        return raw_error;
    }
}

static float CalculatePosHoldGain(float encoder_error)
{
    float ret = 1.0f + 0.00004f * encoder_error*encoder_error;
    if (ret > 8) {
        ret = 8;
    }
    return ret;
}
