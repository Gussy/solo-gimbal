#include "control/rate_loops.h"
#include "can/cb.h"
#include "can/cand.h"
#include "control/PID.h"
#include "hardware/gyro.h"
#include "hardware/encoder.h"
#include "control/filt2p.h"
#include "control/gyro_kinematics_correction.h"
#include "PM_Sensorless-Settings.h"
#define MAX_NUM_CHIRPS 5
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

static const int8_t roll_ang[] = { 35,0,-35};
static const int8_t el_ang[] = { 35, 0, -45, -90, -120};

#define NUM_ROLL_ANGLES 3
#define NUM_EL_ANGLES 5


float t = 0;
float f = 0;
float phi = 0;
static uint8_t num_chirps = 0;
static uint8_t el_ang_cnt =0;
static uint8_t roll_ang_cnt =0;
GimbalAxis torque_type = AZ;


static float SpringMassDamperPass(int16_t ang_vel, int16_t abs_ang,float k,float c);
static float ChirpSignalPass(uint16_t Magnitude);


// structural mode at ~240hz on roll arm->camera carriage connection
// 180hz 12dB chebyshev type 2 4th-order low pass
#define ROLL_FILTER_NUM_SECTIONS 2
struct Filt2p_Params roll_torque_filt_params[ROLL_FILTER_NUM_SECTIONS] = {
    {.b0= 0.2600345100000000, .b1= 0.2427056873942225, .b2= 0.2600345100000003, .a1=-0.0446326229788293, .a2= 0.1414793891043662},
    {.b0= 1.0000000000000000, .b1=-0.7176845003927455, .b2= 1.0000000000000009, .a1=-0.8191282670211697, .a2= 0.7108825577823754}
};
struct Filt2p_State roll_torque_filt_state[ROLL_FILTER_NUM_SECTIONS];

#define YAW_FILTER_NUM_SECTIONS 10
struct Filt2p_Params yaw_torque_filt_params[YAW_FILTER_NUM_SECTIONS] = {
    {.b0 = 1, .b1 = 0.225529040556968,  .b2=0.576675671830002, .a1=-0.298288222856009, .a2=0.253745603180684},
    {.b0 = 1, .b1 = 0.742834969121970,  .b2=0.491861412716521, .a1=0.898730782721131 , .a2=0.504159809676648},
    {.b0 = 1, .b1 = 1.16806049846683,   .b2=0.641170929188255, .a1=1.19810337939554  , .a2=0.649042183987252},
    {.b0 = 1, .b1 = 0.760277827677753,  .b2=0.823618104853667, .a1=0.744184129421869 , .a2=0.777640277180757},
    {.b0 = 1, .b1 = 0.0157023472943767, .b2=0.843770835560726, .a1=0.0186116292713203, .a2=0.790517395894700},
    {.b0 = 1, .b1 = -1.70128682157733,  .b2=0.912955206806351, .a1=-1.23965743943720 , .a2=0.845778550407367},
    {.b0 = 1, .b1 = 1.46488744115071,   .b2=0.840203249235700, .a1=1.47282070398360  , .a2=0.847236562244339},
    {.b0 = 1, .b1 = -1.24300053658244,  .b2=0.859866627637780, .a1=-1.45073725525697 , .a2=0.857457565144410},
    {.b0 = 1, .b1 = -1.81473946965913,  .b2=0.937102507769830, .a1=-1.80184762288135 , .a2=0.887972394896855},
    {.b0 = 1*0.724094261930275, .b1 = -1.65856685103887*0.724094261930275,  .b2=0.957234397860325*0.724094261930275, .a1=-1.61078498581705 , .a2=0.906192363385310}
};
struct Filt2p_State yaw_torque_filt_state[YAW_FILTER_NUM_SECTIONS];

struct Filt2p_Params gyro_filt_params = { .b0 = 0.097631072937817, .b1 = 0.195262145875635, .b2 = 0.097631072937817, .a1 = -0.942809041582063, .a2 = 0.333333333333333 };
struct Filt2p_State gyro_filt_state[AXIS_CNT];

Uint16 telemetry_decimation_count = 0;
Uint16 torque_cmd_telemetry_decimation_count = 5; // Start this at 5 so it's staggered with respect to the rest of the telemetry

static int16 raw_accel_measurement[AXIS_CNT] = {0, 0, 0}; // raw accelerometer measurements in body frame

static int16 raw_gyro_measurement[AXIS_CNT] = {0, 0, 0}; // raw gyro measurements in body frame
static float filtered_gyro_measurement[AXIS_CNT] = {0, 0, 0}; // filtered gyro measurements in body frame
static float filtered_gyro_measurement_jf[AXIS_CNT] = {0, 0, 0}; // euler angle derivatives


static int32 summed_raw_gyro[AXIS_CNT] = {0, 0, 0};
static int32 summed_raw_accel[AXIS_CNT] = {0, 0, 0};

static int32 summed_torque_cmd[AXIS_CNT] = {0, 0, 0};
static Uint16 summed_torque_cmd_count = 0;

static Uint16 rate_loop_step = 0;


void InitRateLoops(void)
{
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
        case 0: MDBSendTorques(0,torque_type, roll_ang[roll_ang_cnt], el_ang[el_ang_cnt]);
                break;
        case 1: { // gyro and control loops
            float torque_limit = cb_parms->max_allowed_torque != 0?cb_parms->max_allowed_torque : 32767.0;
            float torque_out[AXIS_CNT];

            // Do gyro kinematics correction
            transform_ang_vel_to_joint_rate(filtered_gyro_measurement, filtered_gyro_measurement_jf);

            // Compute the new motor torque commands
            float roll_ang_in_enc,el_ang_in_enc;
            el_ang_in_enc = (el_ang[el_ang_cnt]/360.0)*ENCODER_COUNTS_PER_REV;
            roll_ang_in_enc = (roll_ang[roll_ang_cnt]/360.0)*ENCODER_COUNTS_PER_REV;
            switch(torque_type){
                case AZ:
                    torque_out[EL] = SpringMassDamperPass(filtered_gyro_measurement_jf[EL], filtered_gyro_measurement_jf[EL]-el_ang_in_enc,2000.0,0.01);
                    torque_out[ROLL] = SpringMassDamperPass(filtered_gyro_measurement_jf[ROLL], filtered_gyro_measurement_jf[ROLL]-roll_ang_in_enc,5000.0,0.05);
                    torque_out[AZ] =  ChirpSignalPass(5000);
                    if(num_chirps == MAX_NUM_CHIRPS && el_ang_cnt < (NUM_EL_ANGLES-1)) {
                        num_chirps = 0;
                        el_ang_cnt++;
                    } else if(num_chirps == MAX_NUM_CHIRPS && roll_ang_cnt < (NUM_ROLL_ANGLES-1)) {
                        num_chirps = 0;
                        el_ang_cnt = 0;
                        roll_ang_cnt++;
                    } else if(num_chirps == MAX_NUM_CHIRPS) {
                        torque_type = ROLL;
                        num_chirps = 0;
                        el_ang_cnt = 0;
                        roll_ang_cnt = 0;
                    }
                    break;
                case ROLL:
                    torque_out[EL] = SpringMassDamperPass(filtered_gyro_measurement_jf[EL], filtered_gyro_measurement_jf[EL] - el_ang_in_enc,2000.0,0.01);
                    torque_out[ROLL] = ChirpSignalPass(5000);
                    torque_out[AZ] =  0;
                    if(num_chirps == MAX_NUM_CHIRPS && el_ang_cnt < (NUM_EL_ANGLES-1)) {
                        num_chirps = 0;
                        el_ang_cnt++;
                    } else if(num_chirps == MAX_NUM_CHIRPS){
                        num_chirps = 0;
                        el_ang_cnt=0;
                        torque_type = EL;
                    }
                    break;
                case EL:
                    torque_out[EL] = ChirpSignalPass(3000);
                    torque_out[ROLL] = SpringMassDamperPass(filtered_gyro_measurement_jf[ROLL], filtered_gyro_measurement_jf[ROLL],5000.0,0.05);
                    torque_out[AZ] =  0;
                    if(num_chirps == MAX_NUM_CHIRPS) {
                        num_chirps = 0;
                        torque_type = AZ;
                    }
                    break;
            }

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
            MDBSendTorques(1,torque_out[ROLL],torque_out[EL],torque_out[AZ]);
            // Also update our own torque (fake like we got a value over CAN)
            cb_parms->param_set[CAND_PID_TORQUE].param = (int16)torque_out[EL];
            cb_parms->param_set[CAND_PID_TORQUE].sema = true;
            break;
        }
        case 2: { // accel
            ReadAccel(&(raw_accel_measurement[GyroAxisMap[X_AXIS]]), &(raw_accel_measurement[GyroAxisMap[Y_AXIS]]), &(raw_accel_measurement[GyroAxisMap[Z_AXIS]]));
            for (i=0; i<AXIS_CNT; i++) {
                raw_accel_measurement[i] *= IMUSignMap[i];
                summed_raw_accel[i] += raw_accel_measurement[i];
            }
            MDBSendTorques(3,raw_accel_measurement[GyroAxisMap[X_AXIS]],raw_accel_measurement[GyroAxisMap[Y_AXIS]],raw_accel_measurement[GyroAxisMap[Z_AXIS]]);
            break;
        }
        case 3:
        case 8: {
            rate_loop_step = 0;
            break;
        }
    };
}

static float SpringMassDamperPass(int16_t ang_vel, int16_t abs_ang,float k,float c) {
    float generated_torque;
    generated_torque =-k*2.0*M_PI*abs_ang/ENCODER_COUNTS_PER_REV -c*ang_vel;
    return generated_torque;
}

static float ChirpSignalPass(uint16_t Magnitude) {
    float torque;
    t += 0.001;
    if (t > 12.0) {
        t = 0.0;
        num_chirps++;
    }
    if (t < 10) {
        f = exp(log(5.0) * (1.0 - t/10.0) +
                log(500.0) * (t/10.0));
        phi += 2.0*M_PI*f*0.001;
        if (phi > 2.0*M_PI) {
            phi -= 2.0*M_PI;
        }
        torque = Magnitude*sin(phi);
    } else {
        torque = 0;
        phi  = 0;
    }
    return torque;
}

