#include "control/rate_loops.h"
#include "can/cb.h"
#include "can/cand.h"
#include "control/PID.h"
#include "hardware/gyro.h"
#include "control/filt2p.h"
#include "control/gyro_kinematics_correction.h"
#include "PM_Sensorless-Settings.h"

static const double RATE_UPSAMPLING_ALPHA = 0.1;

static void SendEncoderTelemetry(int16 az_encoder, int16 el_encoder, int16 rl_encoder);
static void SendGyroTelemetry(int32 az_gyro, int32 el_gyro, int32 rl_gyro);
static void SendAccelTelemetry(int32 az_accel, int32 el_accel, int32 rl_accel);
static void SendTorqueCmdTelemetry(int16 az_torque_cmd, int16 el_torque_cmd, int16 rl_torque_cmd);

// structural mode at ~240hz on roll arm->camera carriage connection
// 180hz 12dB chebyshev type 2 4th-order low pass
struct Filt2p_Params roll_torque_filt_params[2] = {
    {.b0= 0.2600345100000000, .b1= 0.2427056873942225, .b2= 0.2600345100000003, .a1=-0.0446326229788293, .a2= 0.1414793891043662},
    {.b0= 1.0000000000000000, .b1=-0.7176845003927455, .b2= 1.0000000000000009, .a1=-0.8191282670211697, .a2= 0.7108825577823754}
};

struct Filt2p_State roll_torque_filt_state[2] = {
    {0.0f,0.0f},
    {0.0f,0.0f}
};

Uint16 telemetry_decimation_count = 0;
Uint16 torque_cmd_telemetry_decimation_count = 5; // Start this at 5 so it's staggered with respect to the rest of the telemetry

#define POS_LOOP_GAIN_1 1
#define POS_LOOP_GAIN_2 2
#define POS_LOOP_GAIN_3 4
#define POS_LOOP_GAIN_4 8
#define POS_LOOP_GAIN_1_LIMIT 139
#define POS_LOOP_GAIN_2_LIMIT 278
#define POS_LOOP_GAIN_3_LIMIT 417

void RunRateLoops(ControlBoardParms* cb_parms, ParamSet* param_set)
{
    static int16 raw_gyro_readings[AXIS_CNT] = {0, 0, 0};
    static int16 raw_accel_readings[AXIS_CNT] = {0, 0, 0};
    float torque_limit = cb_parms->max_allowed_torque != 0?cb_parms->max_allowed_torque : 32767.0;
    float torque_out[AXIS_CNT];

    switch (cb_parms->rate_loop_pass) {
    case READ_GYRO_PASS:
        ReadGyro(&(raw_gyro_readings[GyroAxisMap[X_AXIS]]), &(raw_gyro_readings[GyroAxisMap[Y_AXIS]]), &(raw_gyro_readings[GyroAxisMap[Z_AXIS]]));
        cb_parms->rate_loop_pass = READ_ACCEL_PASS;
        break;

    case READ_ACCEL_PASS:
        ReadAccel(&(raw_accel_readings[GyroAxisMap[X_AXIS]]), &(raw_accel_readings[GyroAxisMap[Y_AXIS]]), &(raw_accel_readings[GyroAxisMap[Z_AXIS]]));
        cb_parms->rate_loop_pass = KINEMATICS_PASS;
        break;

    case KINEMATICS_PASS:
        // Unpack the gyro data into the correct axes, and apply the gyro offsets, Do gyro sign correction
        cb_parms->gyro_readings[AZ] = (raw_gyro_readings[AZ] * GyroSignMap[AZ]);
        cb_parms->gyro_readings[EL] = (raw_gyro_readings[EL] * GyroSignMap[EL]);
        cb_parms->gyro_readings[ROLL] = (raw_gyro_readings[ROLL] * GyroSignMap[ROLL]);

        // Do the 10-cycle integration of the raw gyro readings for the 100Hz gyro telemetry
        cb_parms->integrated_raw_gyro_readings[AZ] += cb_parms->gyro_readings[AZ];
        cb_parms->integrated_raw_gyro_readings[EL] += cb_parms->gyro_readings[EL];
        cb_parms->integrated_raw_gyro_readings[ROLL] += cb_parms->gyro_readings[ROLL];

        // Do the 10-cycle integration of the raw accelerometer readings for the 100Hz accelerometer telemetry
        cb_parms->integrated_raw_accel_readings[AZ] += raw_accel_readings[AZ];
        cb_parms->integrated_raw_accel_readings[EL] += raw_accel_readings[EL];
        cb_parms->integrated_raw_accel_readings[ROLL] += raw_accel_readings[ROLL];

        // Do gyro kinematics correction
        update_joint_ang_trig(cb_parms->encoder_readings); // TODO: determine where to run this such that it only updates as often as necessary
        transform_ang_vel_to_joint_rate(cb_parms->gyro_readings, cb_parms->corrected_gyro_readings);

        // Set up the next rate loop pass to be the az error computation pass
        cb_parms->rate_loop_pass = ERROR_AZ_PASS;
        break;

        case ERROR_AZ_PASS:
        	if ((Uint16)cb_parms->control_type == CONTROL_TYPE_POS) {
        		Uint16 pos_gain = 0;
        		int16 encoder_error = CorrectEncoderError(-cb_parms->encoder_readings[AZ]);
        		if ((encoder_error <= -POS_LOOP_GAIN_3_LIMIT) || (encoder_error >= POS_LOOP_GAIN_3_LIMIT)) {
        			pos_gain = POS_LOOP_GAIN_4;
        		} else if ((encoder_error <= -POS_LOOP_GAIN_2_LIMIT) || (encoder_error >= POS_LOOP_GAIN_2_LIMIT)) {
        			pos_gain = POS_LOOP_GAIN_3;
        		} else if ((encoder_error <= -POS_LOOP_GAIN_1_LIMIT) || (encoder_error >= POS_LOOP_GAIN_1_LIMIT)) {
        			pos_gain = POS_LOOP_GAIN_2;
        		} else {
        			pos_gain = POS_LOOP_GAIN_1;
        		}
        		cb_parms->setpoints[AZ] = (pos_gain * encoder_error);
        		cb_parms->process_vars[AZ] = cb_parms->corrected_gyro_readings[AZ];
        		//cb_parms->axis_errors[AZ] = (pos_gain * encoder_error) - cb_parms->corrected_gyro_readings[AZ];
        	} else {
        		// low-pass filter to do the upsampling between the 100Hz telemetry and 1kHz rate loop
				cb_parms->rate_cmd_inject_filtered[AZ] = cb_parms->rate_cmd_inject_filtered[AZ]	+ RATE_UPSAMPLING_ALPHA * (cb_parms->rate_cmd_inject[AZ] - cb_parms->rate_cmd_inject_filtered[AZ]);
				cb_parms->setpoints[AZ] = cb_parms->rate_cmd_inject_filtered[AZ];
				cb_parms->process_vars[AZ] = cb_parms->corrected_gyro_readings[AZ];
				//cb_parms->axis_errors[AZ] = (int16)CLAMP_TO_BOUNDS(((int32)cb_parms->rate_cmd_inject_filtered[AZ] - (int32)cb_parms->corrected_gyro_readings[AZ]), INT16_MIN, INT16_MAX);
        	}

        	// Set up the next rate loop pass to be the el error computation pass
        	cb_parms->rate_loop_pass = ERROR_EL_PASS;
            break;

        case ERROR_EL_PASS:
        	if ((Uint16)cb_parms->control_type == CONTROL_TYPE_POS) {
        		Uint16 pos_gain = 0;
				int16 encoder_error = CorrectEncoderError(-cb_parms->encoder_readings[EL]);
				if ((encoder_error <= -POS_LOOP_GAIN_3_LIMIT) || (encoder_error >= POS_LOOP_GAIN_3_LIMIT)) {
					pos_gain = POS_LOOP_GAIN_4;
				} else if ((encoder_error <= -POS_LOOP_GAIN_2_LIMIT) || (encoder_error >= POS_LOOP_GAIN_2_LIMIT)) {
					pos_gain = POS_LOOP_GAIN_3;
				} else if ((encoder_error <= -POS_LOOP_GAIN_1_LIMIT) || (encoder_error >= POS_LOOP_GAIN_1_LIMIT)) {
					pos_gain = POS_LOOP_GAIN_2;
				} else {
					pos_gain = POS_LOOP_GAIN_1;
				}
				cb_parms->setpoints[EL] = (pos_gain * encoder_error);
				cb_parms->process_vars[EL] = cb_parms->corrected_gyro_readings[EL];
				//cb_parms->axis_errors[EL] = (pos_gain * encoder_error) - cb_parms->corrected_gyro_readings[EL];
        	} else {
        		// low-pass filter to do the upsampling between the 100Hz telemetry and 1kHz rate loop
				cb_parms->rate_cmd_inject_filtered[EL] = cb_parms->rate_cmd_inject_filtered[EL]	+ RATE_UPSAMPLING_ALPHA * (cb_parms->rate_cmd_inject[EL] - cb_parms->rate_cmd_inject_filtered[EL]);
        		cb_parms->setpoints[EL] = cb_parms->rate_cmd_inject_filtered[EL];
        		cb_parms->process_vars[EL] = cb_parms->corrected_gyro_readings[EL];
				//cb_parms->axis_errors[EL] = (int16)CLAMP_TO_BOUNDS(((int32)cb_parms->rate_cmd_inject_filtered[EL] - (int32)cb_parms->corrected_gyro_readings[EL]), INT16_MIN, INT16_MAX);
        	}

			// Set up the next rate loop pass to be the roll error computation pass
        	cb_parms->rate_loop_pass = ERROR_ROLL_PASS;
            break;

        case ERROR_ROLL_PASS:
        	if ((Uint16)cb_parms->control_type == CONTROL_TYPE_POS) {
        		Uint16 pos_gain = 0;
				int16 encoder_error = CorrectEncoderError(-cb_parms->encoder_readings[ROLL]);
				if ((encoder_error <= -POS_LOOP_GAIN_3_LIMIT) || (encoder_error >= POS_LOOP_GAIN_3_LIMIT)) {
					pos_gain = POS_LOOP_GAIN_4;
				} else if ((encoder_error <= -POS_LOOP_GAIN_2_LIMIT) || (encoder_error >= POS_LOOP_GAIN_2_LIMIT)) {
					pos_gain = POS_LOOP_GAIN_3;
				} else if ((encoder_error <= -POS_LOOP_GAIN_1_LIMIT) || (encoder_error >= POS_LOOP_GAIN_1_LIMIT)) {
					pos_gain = POS_LOOP_GAIN_2;
				} else {
					pos_gain = POS_LOOP_GAIN_1;
				}
				cb_parms->setpoints[ROLL] = (pos_gain * encoder_error);
				cb_parms->process_vars[ROLL] = cb_parms->corrected_gyro_readings[ROLL];
				//cb_parms->axis_errors[ROLL] = (pos_gain * encoder_error) - cb_parms->corrected_gyro_readings[ROLL];
        	} else {
        		// low-pass filter to do the upsampling between the 100Hz telemetry and 1kHz rate loop
				cb_parms->rate_cmd_inject_filtered[ROLL] = cb_parms->rate_cmd_inject_filtered[ROLL]	+ RATE_UPSAMPLING_ALPHA * (cb_parms->rate_cmd_inject[ROLL] - cb_parms->rate_cmd_inject_filtered[ROLL]);
				cb_parms->setpoints[ROLL] = cb_parms->rate_cmd_inject_filtered[ROLL];
				cb_parms->process_vars[ROLL] = cb_parms->corrected_gyro_readings[ROLL];
				//cb_parms->axis_errors[ROLL] = (int16)CLAMP_TO_BOUNDS(((int32)cb_parms->rate_cmd_inject_filtered[ROLL] - (int32)cb_parms->corrected_gyro_readings[ROLL]), INT16_MIN, INT16_MAX);
        	}

			// Set up the next rate loop pass to be the torque command output pass
			cb_parms->rate_loop_pass = TORQUE_OUT_PASS;
            break;

        case TORQUE_OUT_PASS:
            // Run PID rate loops

            // Compute the new motor torque commands
            torque_out[EL] = UpdatePID_Float(EL, cb_parms->setpoints[EL], cb_parms->process_vars[EL], torque_limit, 1.0f, 0.0f);
            torque_out[AZ] = UpdatePID_Float(AZ, cb_parms->setpoints[AZ], cb_parms->process_vars[AZ], torque_limit, cos_phi*cos_phi, torque_out[EL]*sin_phi);
            torque_out[ROLL] = UpdatePID_Float(ROLL, cb_parms->setpoints[ROLL], cb_parms->process_vars[ROLL], torque_limit, 1.0f, 0.0f);

            torque_out[ROLL] = update_filt2p(&(roll_torque_filt_params[0]), &(roll_torque_filt_state[0]), torque_out[ROLL]);
            torque_out[ROLL] = update_filt2p(&(roll_torque_filt_params[1]), &(roll_torque_filt_state[1]), torque_out[ROLL]);

            cb_parms->motor_torques[EL] = torque_out[EL] * TorqueSignMap[EL];
            cb_parms->motor_torques[AZ] = torque_out[AZ] * TorqueSignMap[AZ];
            cb_parms->motor_torques[ROLL] = torque_out[ROLL] * TorqueSignMap[ROLL];

            // Accumulate torque commands for telemetry (make sure to do it after saturation against limits)
			cb_parms->accumulated_torque_cmds[AZ] += cb_parms->motor_torques[AZ];
			cb_parms->accumulated_torque_cmds[EL] += cb_parms->motor_torques[EL];
			cb_parms->accumulated_torque_cmds[ROLL] += cb_parms->motor_torques[ROLL];
			cb_parms->num_torque_cmds_accumulated++;

            // Send out motor torques
            MDBSendTorques(cb_parms->motor_torques[AZ], cb_parms->motor_torques[ROLL]);

            // Also update our own torque (fake like we got a value over CAN)
            param_set[CAND_PID_TORQUE].param = cb_parms->motor_torques[EL];
            *param_set[CAND_PID_TORQUE].sema = TRUE;

            // Send encoder, gyro, and accelerometer telemetry at a decimated rate of 100Hz
            if (++telemetry_decimation_count >= TELEMETRY_DECIMATION_LIMIT) {
            	SendEncoderTelemetry(cb_parms->encoder_readings[AZ],
					cb_parms->encoder_readings[EL],
					cb_parms->encoder_readings[ROLL]);
                SendGyroTelemetry(cb_parms->integrated_raw_gyro_readings[AZ],
					cb_parms->integrated_raw_gyro_readings[EL],
					cb_parms->integrated_raw_gyro_readings[ROLL]);
                SendAccelTelemetry(cb_parms->integrated_raw_accel_readings[AZ],
					cb_parms->integrated_raw_accel_readings[EL],
					cb_parms->integrated_raw_accel_readings[ROLL]);

                // Zero out the gyro integrators for the next cycle
                cb_parms->integrated_raw_gyro_readings[AZ] = 0;
                cb_parms->integrated_raw_gyro_readings[EL] = 0;
                cb_parms->integrated_raw_gyro_readings[ROLL] = 0;

                // Zero out the accel integrators for the next cycle
                cb_parms->integrated_raw_accel_readings[AZ] = 0;
                cb_parms->integrated_raw_accel_readings[EL] = 0;
                cb_parms->integrated_raw_accel_readings[ROLL] = 0;

                telemetry_decimation_count = 0;
            }

            // Send torque command telemetry at a rate of 100Hz.
            // This is staggered with respect to the encoder, gyro, and accelerometer telemetry so that we don't saturate the CAN bus
            if (++torque_cmd_telemetry_decimation_count >= TELEMETRY_DECIMATION_LIMIT) {
            	SendTorqueCmdTelemetry(cb_parms->accumulated_torque_cmds[AZ] / cb_parms->num_torque_cmds_accumulated,
					cb_parms->accumulated_torque_cmds[EL] / cb_parms->num_torque_cmds_accumulated,
					cb_parms->accumulated_torque_cmds[ROLL] / cb_parms->num_torque_cmds_accumulated);

            	// Zero out the torque cmd accumulators for the next cycle
				cb_parms->accumulated_torque_cmds[AZ] = 0;
				cb_parms->accumulated_torque_cmds[EL] = 0;
				cb_parms->accumulated_torque_cmds[ROLL] = 0;
				cb_parms->num_torque_cmds_accumulated = 0;

				torque_cmd_telemetry_decimation_count = 0;
            }

            // We've completed one full rate loop iteration, so on the next pass go back to the beginning
            cb_parms->rate_loop_pass = READ_GYRO_PASS;
            break;
    }
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

static void SendGyroTelemetry(int32 az_gyro, int32 el_gyro, int32 rl_gyro)
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

    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_GYRO_AZ_TELEMETRY, gyro_az_readings, 4);
    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_GYRO_EL_TELEMETRY, gyro_el_readings, 4);
    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_GYRO_RL_TELEMETRY, gyro_rl_readings, 4);
}

static void SendAccelTelemetry(int32 az_accel, int32 el_accel, int32 rl_accel)
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

    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_ACCEL_AZ_TELEMETRY, accel_az_readings, 4);
    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_ACCEL_EL_TELEMETRY, accel_el_readings, 4);
    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_ACCEL_RL_TELEMETRY, accel_rl_readings, 4);
}
