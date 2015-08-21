#include "PeripheralHeaderIncludes.h"
#include "can/cand.h"
#include "mavlink_interface/gimbal_mavlink.h"
#include "PM_Sensorless.h"
#include "hardware/led.h"

#define VERSION_RESYNC  0xff

void CBSendStatus( void );
void MDBSendTorques(int16 msg_id, int16 data1, int16 data2, int16 data3);
void MDBRequestBIT(CAND_DestinationID did);
void SendDebug1ToAz(int16 debug_1, int16 debug_2, int16 debug_3);
void CANSendCalibrationProgress(Uint8 progress, GIMBAL_AXIS_CALIBRATION_STATUS calibration_status);
void CANSendAxisCalibrationStatus(GIMBAL_AXIS_CALIBRATION_REQUIRED status);
void CANUpdateBeaconState(LED_MODE mode, LED_RGBA color, Uint8 duration);
