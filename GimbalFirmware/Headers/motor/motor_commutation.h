#ifndef MOTOR_COMMUTATION_H_
#define MOTOR_COMMUTATION_H_

void MotorCommutationLoop(ControlBoardParms* cb_parms,
        AxisParms* axis_parms,
        MotorDriveParms* md_parms,
        EncoderParms* encoder_parms,
        AveragePowerFilterParms* power_filter_parms,
        LoadAxisParmsStateInfo* load_ap_state_info);

#endif /* MOTOR_COMMUTATION_H_ */
