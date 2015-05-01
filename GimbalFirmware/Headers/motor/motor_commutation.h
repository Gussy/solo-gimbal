/*
 * motor_commutation.h
 *
 *  Created on: Feb 11, 2015
 *      Author: abamberger
 */

#ifndef MOTOR_COMMUTATION_H_
#define MOTOR_COMMUTATION_H_

#define CURRENT_LIMIT_HOMING 0.5
#define CURRENT_LIMIT 0.25 // ADB - Roughly 0.5A at +/- 1.75A full scale

void MotorCommutationLoop(ControlBoardParms* cb_parms,
        AxisParms* axis_parms,
        MotorDriveParms* md_parms,
        EncoderParms* encoder_parms,
        ParamSet* param_set,
        AveragePowerFilterParms* power_filter_parms,
        LoadAxisParmsStateInfo* load_ap_state_info);

#endif /* MOTOR_COMMUTATION_H_ */
