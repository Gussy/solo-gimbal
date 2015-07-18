#ifndef INIT_AXIS_PARMS_STATE_MACHINE_H_
#define INIT_AXIS_PARMS_STATE_MACHINE_H_

#include "PeripheralHeaderIncludes.h"
#include "PM_Sensorless.h"
#include "hardware/HWSpecific.h"
#include "can/cand_BitFields.h"

// The request retry period is in ticks of the main torque loop update rate (currently 10kHz)
#define REQUEST_RETRY_PERIOD 1000

typedef struct {
    int request_retry_counter;
    Uint16 axis_parms_load_complete;
    Uint16 axis_parms_checksum_verified;
    Uint16 current_load_offset;
    Uint16 current_request_load_offset;
    Uint16 total_words_to_load;
} LoadAxisParmsStateInfo;

void InitAxisParmsLoader(LoadAxisParmsStateInfo* load_parms_state_info);
void LoadAxisParmsStateMachine(LoadAxisParmsStateInfo* init_parms_state_info);

#endif /* INIT_AXIS_PARMS_STATE_MACHINE_H_ */
