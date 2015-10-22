#ifndef INIT_AXIS_PARMS_STATE_MACHINE_H_
#define INIT_AXIS_PARMS_STATE_MACHINE_H_

#include "PeripheralHeaderIncludes.h"
#include "PM_Sensorless.h"
#include "hardware/HWSpecific.h"
#include "can/cand_BitFields.h"

// The request retry period is in ticks of the main torque loop update rate (currently 10kHz)
#define REQUEST_RETRY_PERIOD 1000

typedef struct {
    bool load_complete;
    bool header_received;
    uint16_t current_key;
    uint16_t current_request_key;
    uint16_t total_keys_to_load;
    uint16_t request_retry_counter;
} LoadAxisParmsStateInfo;

void InitAxisParmsLoader(LoadAxisParmsStateInfo* load_parms_state_info);
void LoadAxisParmsStateMachine(LoadAxisParmsStateInfo* init_parms_state_info);

#endif /* INIT_AXIS_PARMS_STATE_MACHINE_H_ */
