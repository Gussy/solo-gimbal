/*
 * gopro_interface.c
 *
 *  Created on: Jan 8, 2015
 *      Author: abamberger
 */


#include "gopro_interface.h"
#include "i2c.h"

GPControlState gp_control_state = GP_CONTROL_STATE_IDLE;
GPCmdResult last_cmd_result = GP_CMD_UNKNOWN;
Uint32 gp_power_on_counter = 0;
Uint8 request_cmd_buffer[GP_COMMAND_REQUEST_SIZE];
Uint32 wait_for_cmd_request_timeout = 0;

void init_gp_interface()
{
    init_i2c();
}

Uint16 gp_ready_for_cmd()
{
    return (gp_control_state == GP_CONTROL_STATE_IDLE) && !i2c_get_bb();
}

int gp_send_command(char cmd_name_1, char cmd_name_2, Uint8 cmd_parameter)
{
    if (gp_control_state == GP_CONTROL_STATE_IDLE) {
        request_cmd_buffer[0] = 0x3; // Length of 3 (2 command name bytes, 1 parameter byte), with upper bit set to 0 to indicate command originated from the gimbal (not the GoPro)
        request_cmd_buffer[1] = cmd_name_1;
        request_cmd_buffer[2] = cmd_name_2;
        request_cmd_buffer[3] = cmd_parameter;

        // Assert the GoPro interrupt line, letting it know we'd like it to read a command from us
        GP_ASSERT_INTR();

        gp_control_state = GP_CONTROL_STATE_REQUEST_CMD_SEND;

        return 0;
    } else {
        return -1;
    }
}

int gp_request_power_on()
{
    if (gp_control_state == GP_CONTROL_STATE_IDLE) {
        gp_control_state = GP_CONTROL_STATE_REQUEST_POWER_ON;
        return 0;
    } else {
        return -1;
    }
}

GPPowerStatus gp_get_power_status()
{
    // If we've either requested the power be turned on, or are waiting for the power on timeout to elapse, return
    // GP_POWER_WAIT so the user knows they should query the power status again at a later time
    // Otherwise, poll the gopro voltage on pin to get the current gopro power status
    if (gp_control_state == GP_CONTROL_STATE_REQUEST_POWER_ON || gp_control_state == GP_CONTROL_STATE_WAIT_POWER_ON) {
        return GP_POWER_WAIT;
    } else {
        if (GP_VON == 1) {
            return GP_POWER_ON;
        } else {
            return GP_POWER_OFF;
        }
    }
}

GPCmdResult gp_get_last_cmd_result()
{
    return last_cmd_result;
}

// It's expected that this function is repeatedly called every period as configured in the header (currently 3ms)
// for proper gopro interface operation
void gp_interface_state_machine()
{
    switch (gp_control_state) {
    case GP_CONTROL_STATE_IDLE:
        break;

    case GP_CONTROL_STATE_REQUEST_POWER_ON:
        GP_PWRON_LOW();
        gp_power_on_counter = 0;
        gp_control_state = GP_CONTROL_STATE_WAIT_POWER_ON;
        break;

    case GP_CONTROL_STATE_WAIT_POWER_ON:
        if (gp_power_on_counter++ > (GP_PWRON_TIME_MS / GP_STATE_MACHINE_PERIOD_MS)) {
            GP_PWRON_HIGH();
            gp_control_state = GP_CONTROL_STATE_IDLE;
        }
        break;

    case GP_CONTROL_STATE_REQUEST_CMD_SEND:
        // We wait here until we've been addressed by the GoPro, which means it's ready to read the command from us
        // TODO: Timeout back to idle if we haven't been addressed for 2 seconds
        if (wait_for_cmd_request_timeout++ > 667) { // Timeout is 2 seconds per HeroBus spec
            wait_for_cmd_request_timeout = 0;
            GP_DEASSERT_INTR();
            last_cmd_result = GP_CMD_TIMEOUT;
            gp_control_state = GP_CONTROL_STATE_IDLE;
        }

        if (i2c_get_aas()) {
            // Make sure we've been addressed as a slave transmitter.
            // If we've been addressed as a slave receiver, that means the GoPro is trying to send us a command,
            // and we need to give up on our command for now and receive the command from the GoPro
            if (i2c_get_sdir()) {
                // We were addressed as a slave transmitter, so we can de-assert the interrupt request and transmit the command
                GP_DEASSERT_INTR();
                i2c_send_data(request_cmd_buffer, GP_COMMAND_REQUEST_SIZE);
                gp_control_state = GP_CONTROL_STATE_RECV_CMD_RESPONSE;
            } else {
                // We were addressed as a slave receiver, so give up on our command and receive the incoming command
                // TODO: receive incoming command
            }
        }
        break;

    case GP_CONTROL_STATE_RECV_CMD_RESPONSE:
        last_cmd_result = GP_CMD_SUCCESSFUL;
        // TODO: Validate that the command was successful
        break;
    }
}
