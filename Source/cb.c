#include "cb.h"

#include "cand_BitFields.h"
#include "cand.h"
#include "PM_Sensorless.h"
#include "HWSpecific.h"

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

	pids[1] = CAND_PID_CORETEMP;
	vals[1] = DegreesC;

	cand_tx_multi_response(pids, vals, 2);
}

void CBSendEncoder( Uint16 enc )
{
	cand_tx_response(CAND_ID_EL, CAND_PID_POSITION, enc); // EL axis is control board
}

void CBSendVoltage( float v )
{
	cand_tx_response( CAND_ID_EL, CAND_PID_VOLTAGE, (Uint8) (v*255)); // EL axis is control board
}

void MDBSendTorques(int16 az, int16 roll)
{
    Uint32 combined[2];
    int id, packed_id;

    for (id = 0, packed_id = 0; packed_id < (AXIS_CNT - 1); id++) {
        if (id == CAND_ID_AZ) {
            combined[packed_id++] = az;
        }

        if (id == CAND_ID_ROLL) {
            combined[packed_id++] = roll;
        }
    }

    CAND_ParameterID pid = CAND_PID_TORQUE;

    cand_tx_multi_param(CAND_ID_ALL_AXES, &pid, combined, 1);
}

void MDBRequestBIT(CAND_DestinationID did)
{
    CAND_SID sid;

    sid.sidWord = 0;
    sid.all.m_id = CAND_MID_PARAMETER_QUERY;
    sid.param_query.d_id = did;
    sid.param_query.s_id = CAND_GetSenderID();
    sid.param_query.dir = CAND_DIR_QUERY;
    sid.param_query.repeat = 1;

    Uint8 payload = CAND_PID_BIT;

    cand_tx(sid, &payload, 1);
}

void IFBSendVersionV2( DavinciVersion* v )
{
	static DavinciVersionState sw_version_state = VERSION_MAJOR;
	uint16_t sub_version;

	switch (sw_version_state) {
		case VERSION_MAJOR:
			sub_version = v->major;
			break;
		case VERSION_MINOR:
			sub_version = v->minor;
			break;
		case VERSION_REV:
			sub_version = v->rev;
			break;
		case VERSION_DIRTY:
			sub_version = v->dirty;
			break;
		case VERSION_BRANCH:
			sub_version = v->branch;
			break;
		case VERSION_DONE:
		default:
			sub_version = VERSION_RESYNC;
			break;
	}

	cand_tx_response(CAND_ID_AZ, CAND_PID_VERSION, sub_version); // AZ axis is interface board

	sw_version_state++;
	if (sw_version_state >= VERSION_DONE) {
		// sent resync byte, reset out state machine, requester will do the same
		sw_version_state = VERSION_MAJOR;
	}
}
