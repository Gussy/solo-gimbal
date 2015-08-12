#include "PeripheralHeaderIncludes.h"
#include "PM_Sensorless.h"
#include "can/cand_BitFields.h"
#include "control/PID.h"
#include "hardware/device_init.h"
#include "gopro/gopro_interface.h"
#include "control/gyro_kinematics_correction.h"
#include "mavlink_interface/mavlink_gimbal_interface.h"
#include "can/can_parameter_updates.h"

int16 rate_cmds_received[3];
Uint32 debug_output_decimation_count = 0;

void init_param_set(ParamSet *param_set)
{
    // Initialize parameter set to be empty
    Uint8 i;
    for (i = 0; i < CAND_PID_LAST; i++) {
        param_set[i].param = 0;
        param_set[i].sema = FALSE;
    }
}

void ProcessParamUpdates(ControlBoardParms* cb_parms, DebugData* debug_data)
{
    if ((cb_parms->param_set[CAND_PID_DEBUG_1].sema == TRUE) || (cb_parms->param_set[CAND_PID_DEBUG_2].sema == TRUE) || (cb_parms->param_set[CAND_PID_DEBUG_3].sema == TRUE)) {
        if (cb_parms->param_set[CAND_PID_DEBUG_1].sema == TRUE) {
            debug_data->debug_1 = cb_parms->param_set[CAND_PID_DEBUG_1].param;
            cb_parms->param_set[CAND_PID_DEBUG_1].sema = FALSE;
        }

        if (cb_parms->param_set[CAND_PID_DEBUG_2].sema == TRUE) {
            debug_data->debug_2 = cb_parms->param_set[CAND_PID_DEBUG_2].param;
            cb_parms->param_set[CAND_PID_DEBUG_2].sema = FALSE;
        }

        if (cb_parms->param_set[CAND_PID_DEBUG_3].sema == TRUE) {
            debug_data->debug_3 = cb_parms->param_set[CAND_PID_DEBUG_3].param;
            cb_parms->param_set[CAND_PID_DEBUG_3].sema = FALSE;
        }

        // If any of the debug data changed, send the debug mavlink message
        if (debug_output_decimation_count++ > 9) {
            debug_output_decimation_count = 0;
            send_mavlink_debug_data(debug_data);
        }
    }

    // There are several sets of parameters that only make sense if we're the elevation board,
    // such as rate commands, gyro offsets, and gopro commands
    if (GetBoardHWID() == EL) {
        // Check for new rate commands from the copter
        if ((cb_parms->param_set[CAND_PID_RATE_CMD_AZ].sema == TRUE) || (cb_parms->param_set[CAND_PID_RATE_CMD_EL].sema == TRUE) || (cb_parms->param_set[CAND_PID_RATE_CMD_RL].sema == TRUE)) {
            if (cb_parms->param_set[CAND_PID_RATE_CMD_AZ].sema == TRUE) {
                rate_cmds_received[AZ] = (int16)cb_parms->param_set[CAND_PID_RATE_CMD_AZ].param;
                cb_parms->param_set[CAND_PID_RATE_CMD_AZ].sema = FALSE;
            }

            if (cb_parms->param_set[CAND_PID_RATE_CMD_EL].sema == TRUE) {
                rate_cmds_received[EL] = (int16)cb_parms->param_set[CAND_PID_RATE_CMD_EL].param;
                cb_parms->param_set[CAND_PID_RATE_CMD_EL].sema = FALSE;
            }

            if (cb_parms->param_set[CAND_PID_RATE_CMD_RL].sema == TRUE) {
                rate_cmds_received[ROLL] = (int16)cb_parms->param_set[CAND_PID_RATE_CMD_RL].param;
                cb_parms->param_set[CAND_PID_RATE_CMD_RL].sema = FALSE;
            }

            // If any of the rate commands have been updated, run the kinematics transform and update the transformed rate commands
            // (NOTE: in practice, all 3 rate commands should be updated at the same time, since the parameter updates come in the same message)
            transform_ang_vel_to_joint_rate(rate_cmds_received, cb_parms->rate_cmd_inject);
        }

        // Check for any new GoPro get requests
        if (cb_parms->param_set[CAND_PID_GOPRO_GET_REQUEST].sema == TRUE) {
            gp_get_request((Uint8)cb_parms->param_set[CAND_PID_GOPRO_GET_REQUEST].param, false);
            cb_parms->param_set[CAND_PID_GOPRO_GET_REQUEST].sema = FALSE;
        }

        // Check for any new GoPro set requests
        if (cb_parms->param_set[CAND_PID_GOPRO_SET_REQUEST].sema == TRUE) {
            // Extract the GoPro set request command id and value from the CAN parameter
            GPSetRequest set_request;
            set_request.cmd_id = (cb_parms->param_set[CAND_PID_GOPRO_SET_REQUEST].param >> 8) & 0x000000FF;
            set_request.value = (cb_parms->param_set[CAND_PID_GOPRO_SET_REQUEST].param >> 0) & 0x000000FF;
            gp_set_request(&set_request);
            cb_parms->param_set[CAND_PID_GOPRO_SET_REQUEST].sema = FALSE;
        }
    }
}
