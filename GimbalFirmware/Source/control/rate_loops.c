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

// [0   52   80   87  162  220  270  330  390  445  530  750 1000]
// [0    0   -3    0  -23  -26  -26  -17   -3  -14  -22  -19  -19]
#define ROLL_FILTER_NUM_SECTIONS 20
struct Filt2p_Params roll_torque_filt_params[ROLL_FILTER_NUM_SECTIONS] = {
    {.b0 =  1.000000000000000e+00, .b1 = -1.031933160449974e+00, .b2 =  1.968239116039782e-01, .a1 =  6.960436823251352e-01, .a2 =  4.406704013390225e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  6.638457746804158e-01, .b2 =  4.358210925410625e-01, .a1 = -1.409369352825855e+00, .a2 =  5.510155531046206e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.361287457458192e+00, .b2 =  7.478716918668297e-01, .a1 = -1.489339555004284e+00, .a2 =  5.629009082619001e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  1.025929209321772e+00, .b2 =  6.331730954020613e-01, .a1 =  1.034293730769485e+00, .a2 =  6.377130499475460e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  8.010176699718075e-02, .b2 =  6.189210460157603e-01, .a1 = -1.700628802115107e-01, .a2 =  6.621038449282082e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -2.888900918230939e-01, .b2 =  7.758512790982799e-01, .a1 = -4.615968484426870e-01, .a2 =  7.616068651340623e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -9.944194997946636e-01, .b2 =  6.809735148598320e-01, .a1 = -6.903659988932503e-01, .a2 =  7.630416556369962e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.612182475297607e+00, .b2 =  8.329399489104691e-01, .a1 = -1.610289233758454e+00, .a2 =  7.917735961618696e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  1.551478511915253e-01, .b2 =  8.202530630628664e-01, .a1 =  1.371803983458049e-01, .a2 =  7.938820155840030e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  1.226340062924682e+00, .b2 =  7.975166599194536e-01, .a1 =  1.228949170719727e+00, .a2 =  7.996590678203486e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.262408719401904e+00, .b2 =  9.098624594039502e-01, .a1 = -1.234239936442695e+00, .a2 =  8.861210846665735e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -9.869904877361841e-01, .b2 =  9.291776807736069e-01, .a1 = -9.645530120360948e-01, .a2 =  9.061892145094588e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.686556586603286e+00, .b2 =  9.366155177532477e-01, .a1 = -1.683354626714479e+00, .a2 =  9.082038775615171e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.876155842725304e+00, .b2 =  9.033780840731593e-01, .a1 = -1.883367041241208e+00, .a2 =  9.103037143345029e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -6.449690663400290e-01, .b2 =  8.963798821461584e-01, .a1 = -6.508475291929352e-01, .a2 =  9.153277893612769e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -3.397114251678954e-01, .b2 =  9.325932227446146e-01, .a1 = -3.426951049566437e-01, .a2 =  9.194531598110045e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  1.344429325580268e+00, .b2 =  9.194737335255621e-01, .a1 =  1.344922609004720e+00, .a2 =  9.200295697082737e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  1.673811283882295e-01, .b2 =  9.282020148546747e-01, .a1 =  1.657970985868459e-01, .a2 =  9.208192544918032e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.881434797082979e+00, .b2 =  9.423183097775254e-01, .a1 = -1.873437122780392e+00, .a2 =  9.393375314541172e-01},
    {.b0 =  1.558026224981670e-01, .b1 =  2.235389059681591e-01, .b2 =  1.486387213517654e-01, .a1 =  1.434756636155907e+00, .a2 =  9.540166630495137e-01}
};

// [0   35   50   66   99  140  150  157  217  350  396  404  418 1000]
// [0    0  -14  -14   -6    0   13   13  -10  -11   -7   -8  -16  -16]
#define YAW_FILTER_NUM_SECTIONS 20
struct Filt2p_Params yaw_torque_filt_params[YAW_FILTER_NUM_SECTIONS] = {
    {.b0 =  1.000000000000000e+00, .b1 =  2.260238435577094e-01, .b2 = -7.410601121848244e-01, .a1 =  9.697579800496448e-02, .a2 = -8.674378778743114e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.394854158419358e+00, .b2 =  5.294787821563820e-01, .a1 = -1.130766410126454e+00, .a2 =  6.813393142136605e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -7.582522460387264e-01, .b2 =  6.361737794934219e-01, .a1 = -8.891265017806776e-01, .a2 =  6.846083068060723e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.124434127845316e+00, .b2 =  6.924861096498913e-01, .a1 = -1.533891860706926e+00, .a2 =  8.186504957197578e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -6.632216484710944e-01, .b2 =  7.452626645778947e-01, .a1 = -6.864855106686751e-01, .a2 =  8.309320864294232e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.447460586172731e+00, .b2 =  8.793122488776862e-01, .a1 = -1.486324974830894e+00, .a2 =  8.631181002176521e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.290802474314453e+00, .b2 =  6.378974344747625e-01, .a1 = -1.687273013197603e+00, .a2 =  8.919170664773968e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -5.739469470705985e-01, .b2 =  8.685981428662068e-01, .a1 = -5.997008750216514e-01, .a2 =  9.081827157656722e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -8.732578995593361e-01, .b2 =  9.229373327165795e-01, .a1 = -8.704581693612132e-01, .a2 =  9.187053490205295e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.922902248530602e+00, .b2 =  9.508118244291526e-01, .a1 = -1.908212754348547e+00, .a2 =  9.230167592846552e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.511832884695178e+00, .b2 =  9.536280283157598e-01, .a1 = -1.501558801760853e+00, .a2 =  9.291083557943498e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  1.764315825314206e+00, .b2 =  9.302978928416572e-01, .a1 =  1.763872524802796e+00, .a2 =  9.299058239310273e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.784061779657819e+00, .b2 =  9.531168290940961e-01, .a1 = -1.746941091722787e+00, .a2 =  9.334210588139089e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -5.194322212366834e-01, .b2 =  9.417832137295901e-01, .a1 = -5.390632512313338e-01, .a2 =  9.389366138484820e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.906682218638630e+00, .b2 =  9.511695816195945e-01, .a1 = -1.901050687850474e+00, .a2 =  9.457154600045743e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -1.795538432579343e+00, .b2 =  9.583072280042478e-01, .a1 = -1.792832341094148e+00, .a2 =  9.548326221474562e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -4.407606657886858e-01, .b2 =  9.541655760228338e-01, .a1 = -4.441270984762857e-01, .a2 =  9.551134193012368e-01},
    {.b0 =  1.000000000000000e+00, .b1 = -4.877955449664852e-01, .b2 =  9.706778247146687e-01, .a1 = -4.898356075487262e-01, .a2 =  9.657199052836480e-01},
    {.b0 =  1.000000000000000e+00, .b1 =  1.897116185562300e+00, .b2 =  9.895652234143574e-01, .a1 =  1.897063218738187e+00, .a2 =  9.895073759151165e-01},
    {.b0 =  2.675061552818381e-01, .b1 =  3.944228327832957e-01, .b2 =  2.675007346144773e-01, .a1 =  1.474362283929398e+00, .a2 =  9.998300536128212e-01}
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
