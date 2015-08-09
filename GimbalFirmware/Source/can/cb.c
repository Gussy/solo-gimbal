#include "can/cb.h"

#include "can/cand_BitFields.h"
#include "can/cand.h"
#include "PM_Sensorless.h"
#include "hardware/HWSpecific.h"
#include "mavlink_interface/gimbal_mavlink.h"
#include "hardware/device_init.h"
#include "hardware/led.h"

#include <string.h>

extern Uint16 DegreesC;	//in PM_Sensorless.c

void CBSendStatus( void )
{
	CAND_ParameterID pids[2];
	Uint32 vals[2];

	pids[0] = CAND_PID_BIT;
	vals[0] = 0;
	vals[0] |= (GpioDataRegs.GPADAT.bit.GPIO26) ? 0 : CAND_BIT_CH1_FAULT;
	vals[0] |= (GpioDataRegs.GPBDAT.bit.GPIO50) ? 0 : CAND_BIT_OTW;
	vals[0] |= (GetIndexTimeOut() < IndexTimeOutLimit) ? 0 : CAND_BIT_IDEXTMOUT;
	//vals[0] |= (GetIndexSyncFlag() > 0)?0:CAND_BIT_INDEXNF; // No index flag in this hw, figure out if we need something else
	vals[0] |= (GetEnableFlag()) ? 0 : CAND_BIT_NOT_ENABLED;
	vals[0] |= (GetAxisHomed() > 0) ? CAND_BIT_AXIS_HOMED : 0;
	vals[0] |= (GetAxisParmsLoaded()) ? CAND_BIT_AXIS_PARMS_RECVD : 0;

	pids[1] = CAND_PID_CORETEMP;
	vals[1] = DegreesC;

	cand_tx_multi_response(CAND_ID_ALL_AXES, pids, vals, 2);
}

void CBSendEncoder( Uint16 enc )
{
	cand_tx_response(CAND_ID_EL, CAND_PID_POSITION, enc); // EL axis is control board
}

void MDBSendTorques(int16 az, int16 roll)
{
    Uint32 combined[2];
    int id, packed_id;
    CAND_ParameterID pid = CAND_PID_TORQUE;

    for (id = 0, packed_id = 0; packed_id < (AXIS_CNT - 1); id++) {
        if (id == CAND_ID_AZ) {
            combined[packed_id++] = az;
        }

        if (id == CAND_ID_ROLL) {
            combined[packed_id++] = roll;
        }
    }

    cand_tx_multi_param(CAND_ID_ALL_AXES, &pid, combined, 1);
}

void SendDebug1ToAz(int16 debug_1, int16 debug_2, int16 debug_3)
{
    CAND_ParameterID pids[3];
    Uint32 params[3];

    pids[0] = CAND_PID_DEBUG_1;
    pids[1] = CAND_PID_DEBUG_2;
    pids[2] = CAND_PID_DEBUG_3;

    params[0] = debug_1;
    params[1] = debug_2;
    params[2] = debug_3;

    cand_tx_multi_param(CAND_ID_AZ, pids, params, 3);
}

void MDBRequestBIT(CAND_DestinationID did)
{
    CAND_SID sid;
    Uint8 payload = CAND_PID_BIT;

    sid.sidWord = 0;
    sid.all.m_id = CAND_MID_PARAMETER_QUERY;
    sid.param_query.d_id = did;
    sid.param_query.s_id = CAND_GetSenderID();
    sid.param_query.dir = CAND_DIR_QUERY;
    sid.param_query.repeat = 1;

    cand_tx(sid, &payload, 1);
}

void CANSendCalibrationProgress(Uint8 progress, GIMBAL_AXIS_CALIBRATION_STATUS calibration_status)
{
    Uint8 params[2];
    params[0] = progress;
    params[1] = calibration_status;

    switch (GetBoardHWID()) {
    case EL:
        cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_CALIBRATION_PROGRESS_EL, params, 2);
        break;

    case ROLL:
        cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_CALIBRATION_PROGRESS_RL, params, 2);
        break;
    }
}

void CANSendAxisCalibrationStatus(GIMBAL_AXIS_CALIBRATION_REQUIRED status)
{
    Uint8 params[2];
    params[0] = status;
    params[1] = GetBoardHWID();

    cand_tx_extended_param(CAND_ID_AZ, CAND_EPID_CALIBRATION_REQUIRED_STATUS, params, 2);
}

void CANUpdateBeaconState(LED_MODE mode, LED_RGBA color, Uint8 duration)
{
	Uint8 params[6];
	params[0] = mode;
	params[1] = color.red;
	params[2] = color.green;
	params[3] = color.blue;
	params[4] = color.alpha;
	params[5] = duration;

	cand_tx_extended_param(CAND_ID_EL, CAND_EPID_BEACON_CONTROL, params, 6);
}

void CANUpdateMaxTorque(int16 new_max_torque)
{
	Uint8 params[2];
	params[0] = ((((Uint16)new_max_torque) >> 8) & 0x00FF);
	params[1] = (((Uint16)new_max_torque) & 0x00FF);

	cand_tx_extended_param(CAND_ID_EL, CAND_EPID_MAX_TORQUE, params, 2);
}
