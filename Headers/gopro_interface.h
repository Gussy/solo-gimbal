/*
 * gopro_interface.h
 *
 *  Created on: Jan 8, 2015
 *      Author: abamberger
 */

#ifndef GOPRO_INTERFACE_H_
#define GOPRO_INTERFACE_H_

#include "PeripheralHeaderIncludes.h"

#define GP_COMMAND_REQUEST_SIZE 4
#define GP_COMMAND_RESPONSE_SIZE 3
#define GP_STATE_MACHINE_PERIOD_MS 3
#define GP_PWRON_TIME_MS 120 // Spec says 100ms, but I'm making it a little longer here just in case, and so it's an even multiple of our state machine poll period

#define GP_PWRON_LOW() {GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;}
#define GP_PWRON_HIGH() {GpioDataRegs.GPASET.bit.GPIO22 = 1;}
#define GP_ASSERT_INTR() {GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;}
#define GP_DEASSERT_INTR() {GpioDataRegs.GPASET.bit.GPIO26 = 1;}

#define GP_VON (GpioDataRegs.GPADAT.bit.GPIO6)

typedef enum {
    GP_CONTROL_STATE_IDLE,
    GP_CONTROL_STATE_REQUEST_POWER_ON,
    GP_CONTROL_STATE_WAIT_POWER_ON,
    GP_CONTROL_STATE_REQUEST_CMD_SEND,
    GP_CONTROL_STATE_RECV_CMD_RESPONSE
} GPControlState;

typedef enum {
    GP_POWER_UNKNOWN,
    GP_POWER_ON,
    GP_POWER_OFF,
    GP_POWER_WAIT
} GPPowerStatus;

void init_gp_interface();
void gp_interface_state_machine();
GPPowerStatus gp_get_power_status();
int gp_request_power_on();
int gp_send_command(char cmd_name_1, char cmd_name_2, Uint8 cmd_parameter);

#endif /* GOPRO_INTERFACE_H_ */
