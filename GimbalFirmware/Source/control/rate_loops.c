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

static void float_to_byte_array(float in, Uint8* ret);
static void SendDeltaAngleTelemetry(float az_gyro, float el_gyro, float rl_gyro);
static void SendDeltaVelocityTelemetry(float az_accel, float el_accel, float rl_accel);
static void SendEncoderTelemetry(int16 az_encoder, int16 el_encoder, int16 rl_encoder);
static void SendTorqueCmdTelemetry(int16 az_torque_cmd, int16 el_torque_cmd, int16 rl_torque_cmd);

static float CorrectEncoderError(float raw_error);
static float CalculatePosHoldGain(float encoder_error);

// [0  120  180  280  350  440  490 1000]
// [0    0  -12  -12   -3   -3  -14  -14]
#define PITCH_FILTER_NUM_SECTIONS 10
struct Filt2p_Params pitch_torque_filt_params[PITCH_FILTER_NUM_SECTIONS] = {
    {.b0 =  1.000000000000000e+00, .b1 = -7.104542184398694e-01, .b2 =  1.480289926514557e-01, .a1 = -9.967059128347604e-01, .a2 =  2.512922521536697e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.317205949299502e+00, .b2 =  6.509645544445982e-01, .a1 = -1.487001650715800e+00, .a2 =  6.622488573394284e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -5.989446342808280e-01, .b2 =  5.480498164629326e-01, .a1 = -5.986032451685918e-01, .a2 =  7.437197088121786e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -6.939275777834731e-01, .b2 =  7.223670852415167e-01, .a1 = -8.210267821089017e-01, .a2 =  7.936830005450466e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -2.034394670078251e-01, .b2 =  7.196177732524465e-01, .a1 = -3.152197932578217e-01, .a2 =  8.207072961874218e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.185581739958383e+00, .b2 =  8.644585472527319e-01, .a1 = -1.139219869106457e+00, .a2 =  8.335063597529839e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.582582076839510e+00, .b2 =  8.852252658294744e-01, .a1 = -1.547307463707331e+00, .a2 =  8.336296520311292e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.695418086426230e+00, .b2 =  8.286028441908765e-01, .a1 = -1.714111797055207e+00, .a2 =  8.487957852988264e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -6.926270952785296e-02, .b2 =  8.702551671061489e-01, .a1 = -1.167272187781488e-01, .a2 =  8.570233882156695e-01},
    {.b0 =  3.272210275758354e-01, .b1 =  3.555916862490311e-04, .b2 =  3.002847091337406e-01, .a1 =  4.059561385744981e-03, .a2 =  9.144767360461138e-01}
};

// [0   52   63   80   87  162  220  270  330  390  445  530 1000]
// [0    0   -6   -6    0  -23  -26  -26  -17   -3  -14  -25  -25]
#define ROLL_FILTER_NUM_SECTIONS 20
struct Filt2p_Params roll_torque_filt_params[ROLL_FILTER_NUM_SECTIONS] = {
    {.b0 =  1.000000000000000e+00, .b1 = -1.085349799741241e+00, .b2 =  3.226066420537685e-01, .a1 = -1.661181311855751e+00, .a2 =  7.025853842119368e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.446187857911075e+00, .b2 =  7.874191001538540e-01, .a1 = -1.613344833527078e+00, .a2 =  7.653347633937793e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -8.679056778766798e-01, .b2 =  6.148045484228689e-01, .a1 = -7.627796713715436e-01, .a2 =  7.757950381004152e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -2.446293632823028e-01, .b2 =  6.748420369589816e-01, .a1 = -4.353781767413455e-01, .a2 =  7.849881713178345e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -5.003561418654077e-01, .b2 =  5.847312015099501e-01, .a1 = -6.051475171552736e-01, .a2 =  7.956222862960128e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.495036541964068e-02, .b2 =  7.921999220354091e-01, .a1 = -4.956918539667512e-02, .a2 =  8.275444078656051e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.415715524024078e-01, .b2 =  8.393442638832477e-01, .a1 = -1.530840801582251e-01, .a2 =  8.491884850131727e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.827998043869145e+00, .b2 =  8.916586075450571e-01, .a1 = -1.793985910439603e+00, .a2 =  8.672672202418008e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  1.817473922808283e+00, .b2 =  8.686357417182621e-01, .a1 =  1.817569251317682e+00, .a2 =  8.687209890211492e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  1.063636816797249e-01, .b2 =  8.786808442031872e-01, .a1 =  8.385330091502669e-02, .a2 =  8.748271305049727e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.259666683671663e+00, .b2 =  9.056347089413581e-01, .a1 = -1.227985985074216e+00, .a2 =  8.789058983176867e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.512639863160278e+00, .b2 =  8.655815606122854e-01, .a1 = -1.524053678506391e+00, .a2 =  8.822541089988789e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.690829076701517e+00, .b2 =  9.431708084604126e-01, .a1 = -1.664638848525594e+00, .a2 =  8.967740312274698e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -9.881350125096207e-01, .b2 =  9.320143281084597e-01, .a1 = -9.689225207186170e-01, .a2 =  9.117552187133979e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -6.456738882452717e-01, .b2 =  9.030569484720031e-01, .a1 = -6.511261892530451e-01, .a2 =  9.182161053861129e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -3.386129303642930e-01, .b2 =  9.332858764054008e-01, .a1 = -3.401948456199501e-01, .a2 =  9.205588428593747e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.887609082032153e+00, .b2 =  9.218820464852464e-01, .a1 = -1.905751540499719e+00, .a2 =  9.342492712474355e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  1.609559557405764e-01, .b2 =  9.382374825177239e-01, .a1 =  1.539052385964873e-01, .a2 =  9.344435627373402e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.859458604498048e+00, .b2 =  9.170390633030443e-01, .a1 = -1.873689167797753e+00, .a2 =  9.420209714354976e-01},
    {.b0 =  1.142674815597577e-01, .b1 =  2.512661163315913e-02, .b2 =  1.114206456248232e-01, .a1 =  2.203112133455365e-01, .a2 =  9.756900457947583e-01}
};

// [0   35   50   66   99  140  150  210  350  396  404  418 1000]
// [0    0  -14  -14   -6    0   13  -10  -11   -7   -8  -16  -16]
#define YAW_FILTER_NUM_SECTIONS 20
struct Filt2p_Params yaw_torque_filt_params[YAW_FILTER_NUM_SECTIONS] = {
    {.b0 =  1.000000000000000e+00, .b1 =  1.240080408682241e-01, .b2 = -6.466509180355096e-01, .a1 = -8.203640765078513e-03, .a2 = -7.584140345106759e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.034725320771494e+00, .b2 =  6.155248971280263e-01, .a1 = -1.086428423726030e+00, .a2 =  6.311827819433893e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -7.515695598798848e-01, .b2 =  6.099659855341838e-01, .a1 = -8.767765587632891e-01, .a2 =  6.816398764370930e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -6.702234759311891e-01, .b2 =  7.614419973603470e-01, .a1 = -6.869127416819009e-01, .a2 =  8.323924760787123e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.445722173132183e+00, .b2 =  6.043270688550714e-01, .a1 = -1.585128930324808e+00, .a2 =  8.556056907906535e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.494353195937983e+00, .b2 =  8.894588589725223e-01, .a1 = -1.537102833315133e+00, .a2 =  8.884258901290736e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.873973779808824e+00, .b2 =  9.196367148123099e-01, .a1 = -1.856440756538808e+00, .a2 =  9.055099027096360e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -5.708473351810351e-01, .b2 =  8.676300691190856e-01, .a1 = -6.001026116016012e-01, .a2 =  9.082572494063577e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  1.350895392508264e+00, .b2 =  9.128961344463284e-01, .a1 =  1.350182335663937e+00, .a2 =  9.124102018602801e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -8.736979716138775e-01, .b2 =  9.236783291325412e-01, .a1 = -8.700837268825012e-01, .a2 =  9.190626214306827e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.397588176221005e+00, .b2 =  7.051955974626077e-01, .a1 = -1.723154455636625e+00, .a2 =  9.238828166764360e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.918455198025677e+00, .b2 =  9.459118967027623e-01, .a1 = -1.909806591943764e+00, .a2 =  9.246433718503397e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.793179066565055e+00, .b2 =  9.609778771668365e-01, .a1 = -1.748220753447298e+00, .a2 =  9.251044039553602e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -5.185996813036043e-01, .b2 =  9.385339105741728e-01, .a1 = -5.393614893448533e-01, .a2 =  9.386247121614870e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.545087969402682e+00, .b2 =  9.563302192862917e-01, .a1 = -1.539064916946239e+00, .a2 =  9.394414840219076e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  1.761596554033251e+00, .b2 =  9.442429151752283e-01, .a1 =  1.761207879798664e+00, .a2 =  9.438382670489393e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.784553497553610e+00, .b2 =  9.500635588781761e-01, .a1 = -1.785850848355626e+00, .a2 =  9.507887516509621e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  1.942032775197617e+00, .b2 =  9.530458606011714e-01, .a1 =  1.942161844625497e+00, .a2 =  9.531328344548821e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -4.374704086623967e-01, .b2 =  9.592261466183123e-01, .a1 = -4.397783719816157e-01, .a2 =  9.585726303373685e-01},
    {.b0 =  2.619426451308640e-01, .b1 = -1.281521101849041e-01, .b2 =  2.540114296451966e-01, .a1 = -4.898869369062586e-01, .a2 =  9.648211282814494e-01}
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
                float delta_angle[AXIS_CNT];
                memset(&delta_angle, 0, sizeof(delta_angle));
                for (i=0; i<AXIS_CNT; i++) {
                    delta_angle[i] = GYRO_FORMAT_TO_RAD_S(summed_raw_gyro[i]) / (float)RATE_LOOP_FREQUENCY_HZ;
                }
                memset(&summed_raw_gyro, 0, sizeof(summed_raw_gyro));

                SendDeltaAngleTelemetry(delta_angle[AZ],
                                        delta_angle[EL],
                                        delta_angle[ROLL]);

                break;
            }
            case 2: { // accel
                float delta_velocity[AXIS_CNT] = {0, 0, 0};
                for (i=0; i<AXIS_CNT; i++) {
                    delta_velocity[i] = ACCEL_FORMAT_TO_MSS(summed_raw_accel[i]) / (ACCEL_READ_KHZ*1000);
                }
                memset(&summed_raw_accel, 0, sizeof(summed_raw_accel));

                SendDeltaVelocityTelemetry(delta_velocity[AZ],
                                           delta_velocity[EL],
                                           delta_velocity[ROLL]);

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

static void float_to_byte_array(float in, Uint8* ret) {
    IntOrFloat float_converter;
    float_converter.float_val = in;
    ret[0] = (float_converter.uint32_val >> 24) & 0x000000FF;
    ret[1] = (float_converter.uint32_val >> 16) & 0x000000FF;
    ret[2] = (float_converter.uint32_val >> 8) & 0x000000FF;
    ret[3] = (float_converter.uint32_val & 0x000000FF);
}

static void SendDeltaAngleTelemetry(float az_gyro, float el_gyro, float rl_gyro)
{
    Uint8 gyro_az_readings[4];
    Uint8 gyro_el_readings[4];
    Uint8 gyro_rl_readings[4];
    float_to_byte_array(az_gyro, gyro_az_readings);
    float_to_byte_array(el_gyro, gyro_el_readings);
    float_to_byte_array(rl_gyro, gyro_rl_readings);
    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_DEL_ANG_AZ_TELEMETRY, gyro_az_readings, 4);
    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_DEL_ANG_EL_TELEMETRY, gyro_el_readings, 4);
    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_DEL_ANG_RL_TELEMETRY, gyro_rl_readings, 4);
}

static void SendDeltaVelocityTelemetry(float az_accel, float el_accel, float rl_accel)
{
    Uint8 accel_az_readings[4];
    Uint8 accel_el_readings[4];
    Uint8 accel_rl_readings[4];
    float_to_byte_array(az_accel, accel_az_readings);
    float_to_byte_array(el_accel, accel_el_readings);
    float_to_byte_array(rl_accel, accel_rl_readings);
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
