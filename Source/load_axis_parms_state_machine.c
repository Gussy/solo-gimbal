/*
 * init_axis_parms_state_machine.c
 *
 *  Created on: Jan 9, 2015
 *      Author: abamberger
 */

#include "load_axis_parms_state_machine.h"
#include "cand.h"
#include "cand_BitFields.h"
#include "flash_params.h"
#include "PM_Sensorless-Settings.h"

IntOrFloat int_converter;

void LoadAxisParmsStateMachine(LoadAxisParmsStateInfo* load_parms_state_info)
{
    switch(load_parms_state_info->load_axis_parms_state) {
        // All rate params are sent to the EL axis, since this is where the rate loops run
        case LOAD_AXIS_PARMS_STATE_LOAD_RATE_P:
            switch (load_parms_state_info->current_load_axis) {
                case EL:
                    int_converter.float_val = flash_params.rate_pid_p[EL];
                    cand_tx_param(CAND_ID_EL, CAND_PID_RATE_EL_P, int_converter.uint32_val);
                    load_parms_state_info->current_load_axis = AZ;
                    break;

                case AZ:
                    int_converter.float_val = flash_params.rate_pid_p[AZ];
                    cand_tx_param(CAND_ID_EL, CAND_PID_RATE_AZ_P, int_converter.uint32_val);
                    load_parms_state_info->current_load_axis = ROLL;
                    break;

                case ROLL:
                    int_converter.float_val = flash_params.rate_pid_p[ROLL];
                    cand_tx_param(CAND_ID_EL, CAND_PID_RATE_RL_P, int_converter.uint32_val);
                    load_parms_state_info->current_load_axis = EL;
                    load_parms_state_info->load_axis_parms_state = LOAD_AXIS_PARMS_STATE_LOAD_RATE_I;
                    break;
            }
            break;

        case LOAD_AXIS_PARMS_STATE_LOAD_RATE_I:
            switch (load_parms_state_info->current_load_axis) {
                case EL:
                    int_converter.float_val = flash_params.rate_pid_i[EL];
                    cand_tx_param(CAND_ID_EL, CAND_PID_RATE_EL_I, int_converter.uint32_val);
                    load_parms_state_info->current_load_axis = AZ;
                    break;

                case AZ:
                    int_converter.float_val = flash_params.rate_pid_i[AZ];
                    cand_tx_param(CAND_ID_EL, CAND_PID_RATE_AZ_I, int_converter.uint32_val);
                    load_parms_state_info->current_load_axis = ROLL;
                    break;

                case ROLL:
                    int_converter.float_val = flash_params.rate_pid_i[ROLL];
                    cand_tx_param(CAND_ID_EL, CAND_PID_RATE_RL_I, int_converter.uint32_val);
                    load_parms_state_info->current_load_axis = EL;
                    load_parms_state_info->load_axis_parms_state = LOAD_AXIS_PARMS_STATE_LOAD_RATE_D;
                    break;
            }
            break;

        case LOAD_AXIS_PARMS_STATE_LOAD_RATE_D:
            switch (load_parms_state_info->current_load_axis) {
                case EL:
                    int_converter.float_val = flash_params.rate_pid_d[EL];
                    cand_tx_param(CAND_ID_EL, CAND_PID_RATE_EL_D, int_converter.uint32_val);
                    load_parms_state_info->current_load_axis = AZ;
                    break;

                case AZ:
                    int_converter.float_val = flash_params.rate_pid_d[AZ];
                    cand_tx_param(CAND_ID_EL, CAND_PID_RATE_AZ_D, int_converter.uint32_val);
                    load_parms_state_info->current_load_axis = ROLL;
                    break;

                case ROLL:
                    int_converter.float_val = flash_params.rate_pid_d[ROLL];
                    cand_tx_param(CAND_ID_EL, CAND_PID_RATE_RL_D, int_converter.uint32_val);
                    load_parms_state_info->current_load_axis = EL;
                    load_parms_state_info->load_axis_parms_state = LOAD_AXIS_PARMS_STATE_LOAD_RATE_WINDUP;
                    break;
            }
            break;

        case LOAD_AXIS_PARMS_STATE_LOAD_RATE_WINDUP:
            switch (load_parms_state_info->current_load_axis) {
                case EL:
                    int_converter.float_val = flash_params.rate_pid_windup[EL];
                    cand_tx_param(CAND_ID_EL, CAND_PID_RATE_EL_WINDUP, int_converter.uint32_val);
                    load_parms_state_info->current_load_axis = AZ;
                    break;

                case AZ:
                    int_converter.float_val = flash_params.rate_pid_windup[AZ];
                    cand_tx_param(CAND_ID_EL, CAND_PID_RATE_AZ_WINDUP, int_converter.uint32_val);
                    load_parms_state_info->current_load_axis = ROLL;
                    break;

                case ROLL:
                    int_converter.float_val = flash_params.rate_pid_windup[ROLL];
                    cand_tx_param(CAND_ID_EL, CAND_PID_RATE_RL_WINDUP, int_converter.uint32_val);
                    load_parms_state_info->axis_parms_load_complete = TRUE;
                    break;
            }
            break;
    }
}
