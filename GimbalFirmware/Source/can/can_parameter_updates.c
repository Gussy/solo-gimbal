/*
 * can_parameter_updates.c
 *
 *  Created on: Feb 11, 2015
 *      Author: abamberger
 */

#include "PeripheralHeaderIncludes.h"
#include "PM_Sensorless.h"
#include "can/cand_BitFields.h"
#include "control/PID.h"
#include "hardware/device_init.h"
#include "gopro/gopro_interface.h"
#include "can/can_parameter_updates.h"

int16 rate_cmds_received[3];
Uint32 debug_output_decimation_count = 0;

void ProcessParamUpdates(ParamSet* param_set, ControlBoardParms* cb_parms, DebugData* debug_data, BalanceProcedureParms* balance_proc_parms)
{
    IntOrFloat float_converter;
    // Check for updated rate loop PID params
    if (*(param_set[CAND_PID_RATE_EL_P].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[EL].integralCumulative = 0.0;
        rate_pid_loop_float[EL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_EL_P].param;
        rate_pid_loop_float[EL].gainP = float_converter.float_val;
        *(param_set[CAND_PID_RATE_EL_P].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_EL_I].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[EL].integralCumulative = 0.0;
        rate_pid_loop_float[EL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_EL_I].param;
        rate_pid_loop_float[EL].gainI = float_converter.float_val;
        *(param_set[CAND_PID_RATE_EL_I].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_EL_D].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[EL].integralCumulative = 0.0;
        rate_pid_loop_float[EL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_EL_D].param;
        rate_pid_loop_float[EL].gainD = float_converter.float_val;
        *(param_set[CAND_PID_RATE_EL_D].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_EL_WINDUP].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[EL].integralCumulative = 0.0;
        rate_pid_loop_float[EL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_EL_WINDUP].param;
        rate_pid_loop_float[EL].integralMax = float_converter.float_val;
        rate_pid_loop_float[EL].integralMin = -float_converter.float_val;
        *(param_set[CAND_PID_RATE_EL_WINDUP].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_AZ_P].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[AZ].integralCumulative = 0.0;
        rate_pid_loop_float[AZ].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_AZ_P].param;
        rate_pid_loop_float[AZ].gainP = float_converter.float_val;
        *(param_set[CAND_PID_RATE_AZ_P].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_AZ_I].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[AZ].integralCumulative = 0.0;
        rate_pid_loop_float[AZ].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_AZ_I].param;
        rate_pid_loop_float[AZ].gainI = float_converter.float_val;
        *(param_set[CAND_PID_RATE_AZ_I].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_AZ_D].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[AZ].integralCumulative = 0.0;
        rate_pid_loop_float[AZ].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_AZ_D].param;
        rate_pid_loop_float[AZ].gainD = float_converter.float_val;
        *(param_set[CAND_PID_RATE_AZ_D].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_AZ_WINDUP].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[AZ].integralCumulative = 0.0;
        rate_pid_loop_float[AZ].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_AZ_WINDUP].param;
        rate_pid_loop_float[AZ].integralMax = float_converter.float_val;
        rate_pid_loop_float[AZ].integralMin = -float_converter.float_val;
        *(param_set[CAND_PID_RATE_AZ_WINDUP].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_RL_P].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[ROLL].integralCumulative = 0.0;
        rate_pid_loop_float[ROLL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_RL_P].param;
        rate_pid_loop_float[ROLL].gainP = float_converter.float_val;
        *(param_set[CAND_PID_RATE_RL_P].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_RL_I].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[ROLL].integralCumulative = 0.0;
        rate_pid_loop_float[ROLL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_RL_I].param;
        rate_pid_loop_float[ROLL].gainI = float_converter.float_val;
        *(param_set[CAND_PID_RATE_RL_I].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_RL_D].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[ROLL].integralCumulative = 0.0;
        rate_pid_loop_float[ROLL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_RL_D].param;
        rate_pid_loop_float[ROLL].gainD = float_converter.float_val;
        *(param_set[CAND_PID_RATE_RL_D].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_RATE_RL_WINDUP].sema) == TRUE) {
        // Dump the integrator and differentiator
        rate_pid_loop_float[ROLL].integralCumulative = 0.0;
        rate_pid_loop_float[ROLL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_RATE_RL_WINDUP].param;
        rate_pid_loop_float[ROLL].integralMax = float_converter.float_val;
        rate_pid_loop_float[ROLL].integralMin = -float_converter.float_val;
        *(param_set[CAND_PID_RATE_RL_WINDUP].sema) = FALSE;
    }

    // Check for updated position loop PID params
    if (*(param_set[CAND_PID_POS_EL_P].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[EL].integralCumulative = 0.0;
        pos_pid_loop_float[EL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_EL_P].param;
        pos_pid_loop_float[EL].gainP = float_converter.float_val;
        *(param_set[CAND_PID_POS_EL_P].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_EL_I].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[EL].integralCumulative = 0.0;
        pos_pid_loop_float[EL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_EL_I].param;
        pos_pid_loop_float[EL].gainI = float_converter.float_val;
        *(param_set[CAND_PID_POS_EL_I].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_EL_D].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[EL].integralCumulative = 0.0;
        pos_pid_loop_float[EL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_EL_D].param;
        pos_pid_loop_float[EL].gainD = float_converter.float_val;
        *(param_set[CAND_PID_POS_EL_D].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_EL_WINDUP].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[EL].integralCumulative = 0.0;
        pos_pid_loop_float[EL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_EL_WINDUP].param;
        pos_pid_loop_float[EL].integralMax = float_converter.float_val;
        pos_pid_loop_float[EL].integralMin = -float_converter.float_val;
        *(param_set[CAND_PID_POS_EL_WINDUP].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_AZ_P].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[AZ].integralCumulative = 0.0;
        pos_pid_loop_float[AZ].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_AZ_P].param;
        pos_pid_loop_float[AZ].gainP = float_converter.float_val;
        *(param_set[CAND_PID_POS_AZ_P].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_AZ_I].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[AZ].integralCumulative = 0.0;
        pos_pid_loop_float[AZ].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_AZ_I].param;
        pos_pid_loop_float[AZ].gainI = float_converter.float_val;
        *(param_set[CAND_PID_POS_AZ_I].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_AZ_D].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[AZ].integralCumulative = 0.0;
        pos_pid_loop_float[AZ].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_AZ_D].param;
        pos_pid_loop_float[AZ].gainD = float_converter.float_val;
        *(param_set[CAND_PID_POS_AZ_D].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_AZ_WINDUP].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[AZ].integralCumulative = 0.0;
        pos_pid_loop_float[AZ].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_AZ_WINDUP].param;
        pos_pid_loop_float[AZ].integralMax = float_converter.float_val;
        pos_pid_loop_float[AZ].integralMin = -float_converter.float_val;
        *(param_set[CAND_PID_POS_AZ_WINDUP].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_RL_P].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[ROLL].integralCumulative = 0.0;
        pos_pid_loop_float[ROLL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_RL_P].param;
        pos_pid_loop_float[ROLL].gainP = float_converter.float_val;
        *(param_set[CAND_PID_POS_RL_P].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_RL_I].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[ROLL].integralCumulative = 0.0;
        pos_pid_loop_float[ROLL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_RL_I].param;
        pos_pid_loop_float[ROLL].gainI = float_converter.float_val;
        *(param_set[CAND_PID_POS_RL_I].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_RL_D].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[ROLL].integralCumulative = 0.0;
        pos_pid_loop_float[ROLL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_RL_D].param;
        pos_pid_loop_float[ROLL].gainD = float_converter.float_val;
        *(param_set[CAND_PID_POS_RL_D].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_POS_RL_WINDUP].sema) == TRUE) {
        // Dump the integrator and differentiator
        pos_pid_loop_float[ROLL].integralCumulative = 0.0;
        pos_pid_loop_float[ROLL].errorPrevious = 0.0;
        // Load the new gain
        float_converter.uint32_val = param_set[CAND_PID_POS_RL_WINDUP].param;
        pos_pid_loop_float[ROLL].integralMax = float_converter.float_val;
        pos_pid_loop_float[ROLL].integralMin = -float_converter.float_val;
        *(param_set[CAND_PID_POS_RL_WINDUP].sema) = FALSE;
    }

    if ((*(param_set[CAND_PID_DEBUG_1].sema) == TRUE) || (*(param_set[CAND_PID_DEBUG_2].sema) == TRUE) || (*(param_set[CAND_PID_DEBUG_3].sema) == TRUE)) {
        if (*(param_set[CAND_PID_DEBUG_1].sema) == TRUE) {
            debug_data->debug_1 = param_set[CAND_PID_DEBUG_1].param;
            *(param_set[CAND_PID_DEBUG_1].sema) = FALSE;
        }

        if (*(param_set[CAND_PID_DEBUG_2].sema) == TRUE) {
            debug_data->debug_2 = param_set[CAND_PID_DEBUG_2].param;
            *(param_set[CAND_PID_DEBUG_2].sema) = FALSE;
        }

        if (*(param_set[CAND_PID_DEBUG_3].sema) == TRUE) {
            debug_data->debug_3 = param_set[CAND_PID_DEBUG_3].param;
            *(param_set[CAND_PID_DEBUG_3].sema) = FALSE;
        }

        // If any of the debug data changed, send the debug mavlink message
        if (debug_output_decimation_count++ > 9) {
            debug_output_decimation_count = 0;
            send_mavlink_debug_data(debug_data);
        }
    }

#ifdef ENABLE_BALANCE_PROCEDURE
    if (*(param_set[CAND_PID_BALANCE_AXIS].sema) == TRUE) {
        // Convert and set the new axis
        IntOrFloat float_converter;
        float_converter.uint32_val = param_set[CAND_PID_BALANCE_AXIS].param;
        GimbalAxis new_axis = (GimbalAxis)float_converter.float_val;
        balance_proc_parms->balance_axis = new_axis;

        // Also reset the direction, step, and time counters when we get a new axis, so we begin
        // at the start of the balance procedure for the new axis
        balance_proc_parms->balance_angle_counter = 0;
        balance_proc_parms->current_balance_angle_index = 0;
        balance_proc_parms->current_direction = 0;

        // Also update the target angle here so we immediately snap to the new position on the new axis
        cb_parms->angle_targets[balance_proc_parms->balance_axis] = balance_proc_parms->balance_angles[balance_proc_parms->balance_axis][balance_proc_parms->current_balance_angle_index];

        *(param_set[CAND_PID_BALANCE_AXIS].sema) = FALSE;
    }

    if (*(param_set[CAND_PID_BALANCE_STEP_DURATION].sema) == TRUE) {
        // Convert and update the new counter max
        IntOrFloat float_converter;
        float_converter.uint32_val = param_set[CAND_PID_BALANCE_STEP_DURATION].param;
        balance_proc_parms->balance_angle_counter_max = (int)(float_converter.float_val / 150.0); // The counter gets incremented every 150ms, and the parameter comes in as ms

        *(param_set[CAND_PID_BALANCE_STEP_DURATION].sema) = FALSE;
    }
#endif

    // There are several sets of parameters that only make sense if we're the elevation board,
    // such as rate commands, gyro offsets, and gopro commands
    if (GetBoardHWID() == EL) {
        // Check for new rate commands from the copter
        if ((*(param_set[CAND_PID_RATE_CMD_AZ].sema) == TRUE) || (*(param_set[CAND_PID_RATE_CMD_EL].sema) == TRUE) || (*(param_set[CAND_PID_RATE_CMD_RL].sema) == TRUE)) {
            if (*(param_set[CAND_PID_RATE_CMD_AZ].sema) == TRUE) {
                rate_cmds_received[AZ] = (int16)param_set[CAND_PID_RATE_CMD_AZ].param;
                *(param_set[CAND_PID_RATE_CMD_AZ].sema) = FALSE;
            }

            if (*(param_set[CAND_PID_RATE_CMD_EL].sema) == TRUE) {
                rate_cmds_received[EL] = (int16)param_set[CAND_PID_RATE_CMD_EL].param;
                *(param_set[CAND_PID_RATE_CMD_EL].sema) = FALSE;
            }

            if (*(param_set[CAND_PID_RATE_CMD_RL].sema) == TRUE) {
                rate_cmds_received[ROLL] = (int16)param_set[CAND_PID_RATE_CMD_RL].param;
                *(param_set[CAND_PID_RATE_CMD_RL].sema) = FALSE;
            }

            // If any of the rate commands have been updated, run the kinematics transform and update the transformed rate commands
            // (NOTE: in practice, all 3 rate commands should be updated at the same time, since the parameter updates come in the same message)
            do_gyro_correction(rate_cmds_received, cb_parms->encoder_readings, cb_parms->rate_cmd_inject);
        }

        // Check for new gyro offsets from the copter
        if (*(param_set[CAND_PID_GYRO_OFFSET_X_AXIS].sema) == TRUE) {
            cb_parms->gyro_offsets[X_AXIS] = (int16)param_set[CAND_PID_GYRO_OFFSET_X_AXIS].param;
            *(param_set[CAND_PID_GYRO_OFFSET_X_AXIS].sema) = FALSE;
        }

        if (*(param_set[CAND_PID_GYRO_OFFSET_Y_AXIS].sema) == TRUE) {
            cb_parms->gyro_offsets[Y_AXIS] = (int16)param_set[CAND_PID_GYRO_OFFSET_Y_AXIS].param;
            *(param_set[CAND_PID_GYRO_OFFSET_Y_AXIS].sema) = FALSE;
        }

        if (*(param_set[CAND_PID_GYRO_OFFSET_Z_AXIS].sema) == TRUE) {
            cb_parms->gyro_offsets[Z_AXIS] = (int16)param_set[CAND_PID_GYRO_OFFSET_Z_AXIS].param;
            *(param_set[CAND_PID_GYRO_OFFSET_Z_AXIS].sema) = FALSE;
        }

        // Check for any new GoPro commands
        if (*(param_set[CAND_PID_GP_CMD].sema) == TRUE) {
            // Extract the GoPro command and parameter from the CAN parameter
            GPCmd cmd;
            cmd.cmd[0] = (param_set[CAND_PID_GP_CMD].param >> 24) & 0x000000FF;
            cmd.cmd[1] = (param_set[CAND_PID_GP_CMD].param >> 16) & 0x000000FF;
            cmd.cmd_parm = (param_set[CAND_PID_GP_CMD].param >> 8) & 0x000000FF;
            gp_send_command(&cmd);

            *(param_set[CAND_PID_GP_CMD].sema) = FALSE;
        }
    }
}