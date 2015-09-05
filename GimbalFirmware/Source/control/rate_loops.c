#include "control/rate_loops.h"
#include "can/cb.h"
#include "can/cand.h"
#include "control/PID.h"
#include "hardware/gyro.h"
#include "hardware/encoder.h"
#include "control/filt2p.h"
#include "control/gyro_kinematics_correction.h"
#include "PM_Sensorless-Settings.h"

#define RATE_CTRL_KHZ 2 // valid settings: 1, 2, 4

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

// [0  120  180  280  350  550  700 1000]
// [0    0  -12  -12    0  -21   -6   -6]
#define PITCH_FILTER_NUM_SECTIONS 10
struct Filt2p_Params pitch_torque_filt_params[PITCH_FILTER_NUM_SECTIONS] = {
    {.b0 =  1.000000000000000e+00, .b1 =  4.458538918141721e-02, .b2 =  1.579555545365659e-01, .a1 = -5.611560479817667e-01, .a2 =  2.945358109457595e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  3.510243257656501e-01, .b2 =  7.622726835633040e-01, .a1 =  3.182588961162357e-01, .a2 =  4.103549394368090e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.236493568490428e+00, .b2 =  7.597677588316720e-01, .a1 = -9.077532433034299e-01, .a2 =  5.970889557247674e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  8.412149036716350e-01, .b2 =  6.463492799026873e-01, .a1 =  8.869523050169887e-01, .a2 =  6.735884105059068e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.546626233277172e+00, .b2 =  8.537288303945658e-01, .a1 = -1.512440046358469e+00, .a2 =  7.392256461582323e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.210230621294905e+00, .b2 =  8.661284776171438e-01, .a1 = -1.156582905105887e+00, .a2 =  7.824093628530484e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  3.101350700329667e-01, .b2 =  9.038045442836927e-01, .a1 =  3.204547184777277e-01, .a2 =  8.307068633160497e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -8.592379696400776e-01, .b2 =  8.198743207416612e-01, .a1 = -8.623525499503710e-01, .a2 =  8.490503676583914e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  1.075259101358715e+00, .b2 =  8.496957224181675e-01, .a1 =  1.080312364723463e+00, .a2 =  8.568345327157516e-01},
    {.b0 =  4.676355071618404e-01, .b1 = -8.010955019917212e-01, .b2 =  4.008655244895742e-01, .a1 = -1.729014953143746e+00, .a2 =  8.701343253352108e-01}
};

// [0  120  180  280  330  380  430  480  600  700 1000]
// [0    0  -20  -26  -20    0    0  -15  -20  -10  -10]
#define ROLL_FILTER_NUM_SECTIONS 10
struct Filt2p_Params roll_torque_filt_params[ROLL_FILTER_NUM_SECTIONS] = {
    {.b0 =  1.000000000000000e+00, .b1 = -1.088281193012037e+00, .b2 =  4.691072666336631e-01, .a1 = -1.343294259886622e+00, .a2 =  4.959095425993567e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  5.362952243826644e-01, .b2 =  8.185422184055270e-01, .a1 =  5.610114124987819e-01, .a2 =  6.445743124158697e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  9.223297370956391e-01, .b2 =  7.816351542445440e-01, .a1 =  9.495630621804413e-01, .a2 =  7.894958021605872e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.140133880738714e+00, .b2 =  8.670524370196023e-01, .a1 = -7.250543609254737e-01, .a2 =  8.146198158694153e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -7.457064584498330e-02, .b2 =  7.757595097010316e-01, .a1 = -3.435532790842695e-01, .a2 =  8.217291810012237e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.540977645700661e+00, .b2 =  8.751070579592117e-01, .a1 = -1.686159114608366e+00, .a2 =  8.390563373563111e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.632160670864418e+00, .b2 =  8.117675224906538e-01, .a1 = -1.611050995854007e+00, .a2 =  8.413895518043293e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.007281810542044e+00, .b2 =  9.135347600580412e-01, .a1 = -9.651490445864275e-01, .a2 =  8.819161575515235e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  1.107708851235774e+00, .b2 =  8.898965677704936e-01, .a1 =  1.110768092878029e+00, .a2 =  8.930384682450432e-01},
    {.b0 =  2.910961300223849e-01, .b1 = -3.927848292068935e-02, .b2 =  2.712713608212377e-01, .a1 = -1.523021774822244e-01, .a2 =  9.284066540684429e-01}
};

// [0   41   42   70   90  100  150  240  300 1000]
// [0    0  -15  -14  -10   10   10   -5  -15  -20]
#define YAW_FILTER_NUM_SECTIONS 20
struct Filt2p_Params yaw_torque_filt_params[YAW_FILTER_NUM_SECTIONS] = {
    {.b0 =  1.000000000000000e+00, .b1 =  6.931525559427758e-01, .b2 =  2.270514003379302e-01, .a1 =  1.376599021276814e-01, .a2 = -8.147391623191762e-03},
    {.b0 =  1.000000000000000e+00, .b1 =  1.382382920801625e+00, .b2 =  5.159608841667093e-01, .a1 =  9.261784234260552e-01, .a2 =  2.079242043590670e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -6.332490331922037e-01, .b2 =  1.755699239300257e-01, .a1 = -6.385351371043624e-02, .a2 = -4.990465144579650e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  1.732384781535391e+00, .b2 =  7.563129985291042e-01, .a1 =  1.641828712702738e+00, .a2 =  6.722425196052463e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  1.910551902325041e+00, .b2 =  9.126578507281505e-01, .a1 =  1.888655848369805e+00, .a2 =  8.912751075923364e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.036518829605319e+00, .b2 =  3.908356776525246e-01, .a1 = -1.470903502479668e+00, .a2 =  7.309358861816269e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.156586426767938e+00, .b2 =  6.886003149873365e-01, .a1 = -1.346560783466611e+00, .a2 =  7.503946163759387e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.501464249436705e+00, .b2 =  7.171186113828578e-01, .a1 = -1.583636481382110e+00, .a2 =  7.712284990099241e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  6.768655779151702e-01, .b2 =  7.843414182821385e-01, .a1 =  6.787434919507640e-01, .a2 =  7.845818080562674e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.145346159724908e+00, .b2 =  8.433627088201481e-01, .a1 = -1.173634231552678e+00, .a2 =  8.391188529346782e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.878642351260564e+00, .b2 =  9.182147043112912e-01, .a1 = -1.783343042770987e+00, .a2 =  8.737363464377675e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.914238552823795e+00, .b2 =  9.780641898732079e-01, .a1 = -1.834877977430187e+00, .a2 =  9.104640790027245e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.410646950774568e+00, .b2 =  9.308040815317506e-01, .a1 = -1.402527876663043e+00, .a2 =  9.162705395917954e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.713213562377781e+00, .b2 =  9.165466541683015e-01, .a1 = -1.716276721472346e+00, .a2 =  9.205571765229206e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.153761600390361e+00, .b2 =  9.281323364153053e-01, .a1 = -1.159306335443584e+00, .a2 =  9.205797400752164e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.812669091308156e+00, .b2 =  9.116560302531087e-01, .a1 = -1.830835247141745e+00, .a2 =  9.235988884163656e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.933148862857146e+00, .b2 =  9.580302183326660e-01, .a1 = -1.927663671908659e+00, .a2 =  9.438602461307023e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.129342299414930e+00, .b2 =  9.585855198751476e-01, .a1 = -1.129246313183860e+00, .a2 =  9.590677442315108e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.243002449970539e+00, .b2 =  9.678086099979986e-01, .a1 = -1.243067373175512e+00, .a2 =  9.677055469132700e-01},
    {.b0 =  2.442459929613207e-01, .b1 =  4.871361482237471e-01, .b2 =  2.429919652213591e-01, .a1 =  1.994184528206350e+00, .a2 =  9.946131709952525e-01}
};

#ifdef ROLL_FILTER_NUM_SECTIONS
struct Filt2p_State roll_torque_filt_state[ROLL_FILTER_NUM_SECTIONS];
#endif
#ifdef PITCH_FILTER_NUM_SECTIONS
struct Filt2p_State pitch_torque_filt_state[PITCH_FILTER_NUM_SECTIONS];
#endif
#ifdef YAW_FILTER_NUM_SECTIONS
struct Filt2p_State yaw_torque_filt_state[YAW_FILTER_NUM_SECTIONS];
#endif

struct Filt2p_Params gyro_filt_params;
struct Filt2p_State gyro_filt_state[AXIS_CNT];

uint16_t telemetry_decimation_count = 0;
uint16_t torque_cmd_telemetry_decimation_count = 5; // Start this at 5 so it's staggered with respect to the rest of the telemetry

static int16_t raw_accel_measurement[AXIS_CNT] = {0, 0, 0}; // raw accelerometer measurements in body frame

static int16_t raw_gyro_measurement[AXIS_CNT] = {0, 0, 0}; // raw gyro measurements in body frame
static float filtered_gyro_measurement[AXIS_CNT] = {0, 0, 0}; // filtered gyro measurements in body frame
static float filtered_gyro_measurement_jf[AXIS_CNT] = {0, 0, 0}; // euler angle derivatives
static float setpoints[AXIS_CNT] = {0, 0, 0}; // position targets

static int32_t summed_raw_gyro[AXIS_CNT] = {0, 0, 0};
static int32_t summed_raw_accel[AXIS_CNT] = {0, 0, 0};

static int32_t summed_torque_cmd[AXIS_CNT] = {0, 0, 0};
static uint16_t summed_torque_cmd_count = 0;

static uint16_t steps_since_rate_loop = 0;
static uint16_t steps_since_telem = 0;
static uint16_t steps_since_accel = 0;

static bool need_rate_loop = false;
static bool need_accel_read = false;
static bool need_send_telem = false;

static uint16_t telem_step = 0;

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
    steps_since_rate_loop++;
    steps_since_accel++;
    steps_since_telem++;

    if (steps_since_rate_loop >= GYRO_READ_KHZ/RATE_CTRL_KHZ) {
        need_rate_loop = true;
        steps_since_rate_loop = 0;
    }

    if (steps_since_accel >= GYRO_READ_KHZ/ACCEL_READ_KHZ) {
        need_accel_read = true;
        steps_since_accel = 0;
    }

    // send telemetry every 10ms
    if (steps_since_telem >= GYRO_READ_KHZ*10) {
        need_send_telem = true;
        steps_since_telem = 0;
    }

    if (need_rate_loop) {
        need_rate_loop = false;
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
    } else if (need_accel_read) {
        need_accel_read = false;
        ReadAccel(&(raw_accel_measurement[GyroAxisMap[X_AXIS]]), &(raw_accel_measurement[GyroAxisMap[Y_AXIS]]), &(raw_accel_measurement[GyroAxisMap[Z_AXIS]]));
        for (i=0; i<AXIS_CNT; i++) {
            raw_accel_measurement[i] *= IMUSignMap[i];
            summed_raw_accel[i] += raw_accel_measurement[i];
        }
    } else if (need_send_telem) {
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
                    delta_velocity[i].float_val = ACCEL_FORMAT_TO_MSS(summed_raw_accel[i]) / (ACCEL_READ_KHZ*1000);
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

                // done sending telem
                telem_step = 0;
                need_send_telem = false;
                break;
            }
        };
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
