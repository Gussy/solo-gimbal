#ifndef CAN_H_
#define CAN_H_

#include "hardware/HWSpecific.h" // Include for GimbalAxis typedef
#include "F2806x_Device.h" // Include for Uint16 typedef

typedef Uint16 (* uint16fptr)();
extern  uint16fptr GetWordData;

void CAN_Init(GimbalAxis axis);
Uint16 CAN_GetWordData();
Uint32 CAN_GetLongData();
Uint16 CAN_SendWordData(Uint16 data);

#endif /* CAN_H_ */
