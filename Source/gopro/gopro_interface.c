/*
 * gopro_interface.c
 *
 *  Created on: Jan 8, 2015
 *      Author: abamberger
 */


#include "gopro/gopro_interface.h"

volatile GPControlState gp_control_state = GP_CONTROL_STATE_IDLE;
GPCmdResult last_cmd_result = GP_CMD_UNKNOWN;
Uint32 gp_power_on_counter = 0;
Uint32 wait_for_cmd_request_timeout = 0;

void init_gp_interface()
{
    init_i2c(&addressed_as_slave_callback);
}

Uint16 gp_ready_for_cmd()
{
    return (gp_control_state == GP_CONTROL_STATE_IDLE) && !i2c_get_bb();
}

int gp_send_command(char cmd_name_1, char cmd_name_2, Uint8 cmd_parameter)
{
    if (gp_control_state == GP_CONTROL_STATE_IDLE) {
        Uint8 request_cmd_buffer[GP_COMMAND_REQUEST_SIZE];
        request_cmd_buffer[0] = 0x3; // Length of 3 (2 command name bytes, 1 parameter byte), with upper bit set to 0 to indicate command originated from the gimbal (not the GoPro)
        request_cmd_buffer[1] = cmd_name_1;
        request_cmd_buffer[2] = cmd_name_2;
        request_cmd_buffer[3] = cmd_parameter;

        // Send the command.  This won't actually send the command out of the I2C port, but will first put it in the transmit
        // ringbuffer, which will then preload the beginning of the command into the I2C transmit FIFO.  This will allow it to
        // to be transmitted as soon as the GoPro starts clocking it out
        i2c_send_data(request_cmd_buffer, GP_COMMAND_REQUEST_SIZE);

        // Assert the GoPro interrupt line, letting it know we'd like it to read a command from us
        GP_ASSERT_INTR();

        // Reset the timeout counter, and move to the wait for command send state
        wait_for_cmd_request_timeout = 0;
        gp_control_state = GP_CONTROL_STATE_WAIT_CMD_SEND;

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

        case GP_CONTROL_STATE_WAIT_CMD_SEND:
            // We wait here until we've been addressed by the GoPro, which means it's ready to read the command from us.
            // This will cause an interrupt that changes the state of the state machine, so the only way out of this state from
            // here is to time out.  We time out back to idle if we haven't been addressed for 2 seconds
            if (wait_for_cmd_request_timeout++ > 667) { // Timeout is 2 seconds per HeroBus spec
                wait_for_cmd_request_timeout = 0;
                GP_DEASSERT_INTR();
                last_cmd_result = GP_CMD_TIMEOUT;
                gp_control_state = GP_CONTROL_STATE_IDLE;
            }
            break;

        case GP_CONTROL_STATE_RECV_CMD_RESPONSE:
            last_cmd_result = GP_CMD_SUCCESSFUL;
            // TODO: Validate that the command was successful
            break;

        case GP_CONTROL_STATE_WAIT_READY_FOR_RESPONSE_SEND:
            // We need to wait for the GoPro to finish sending the command to assert the interrupt request line,
            // so we poll for a stop condition here
            if (i2c_get_scd()) {
                // If the stop condition has been asserted, the GoPro is done transmitting the command, so we read it here
                // and determine our response

                // First make sure the command is one we support.  Per the GoPro spec, we only have to respond to the "vs" command
                // to query the version of the protocol we support.  For any other command from the GoPro, return an error response
                if (i2c_get_available_chars() > 0) {
                    Uint8 receive_buffer[GP_COMMAND_RECEIVE_BUFFER_SIZE];
                    // Get the command length
                    receive_buffer[0] = i2c_get_next_char();
                    int i;
                    for (i = 0; i < receive_buffer[0]; i++) {
                        if (!(i2c_get_available_chars() > 0) || ((i + 1) >= GP_COMMAND_RECEIVE_BUFFER_SIZE)) {
                            // Make sure we don't corrupt the ringbuffer or overflow the receive buffer
                            break;
                        }
                        receive_buffer[i + 1] = i2c_get_next_char();
                    }

                    // Check that the command is "vs" (the only one we're required to respond to per the spec)
                    if ((receive_buffer[1] == 'v') && (receive_buffer[2] == 's')) {
                        // Preload the transmit buffer with the command response
                        Uint8 transmit_buffer[GP_COMMAND_RESPONSE_SIZE];
                        transmit_buffer[0] = 2; // Packet size, 1st byte is status byte, 2nd byte is protocol version
                        transmit_buffer[1] = 0; // 0 = Status success
                        transmit_buffer[2] = GP_PROTOCOL_VERSION; // Protocol version
                        i2c_send_data(transmit_buffer, GP_COMMAND_RESPONSE_SIZE);
                    } else {
                        // Preload the transmit buffer with an error response, since we don't support the command we
                        // were sent
                        Uint8 transmit_buffer[GP_COMMAND_RESPONSE_SIZE - 1];
                        transmit_buffer[0] = 1; // Packet size, only status byte
                        transmit_buffer[1] = 1; // 1 = Status failure
                        i2c_send_data(transmit_buffer, GP_COMMAND_RESPONSE_SIZE - 1);
                    }

                    // Assert the interrupt request line to indicate that we're ready to respond to the GoPro's command
                    GP_ASSERT_INTR();
                } else {
                    // If we didn't receive any characters from the GoPro, we don't assert the interrupt request line to indicate
                    // that we're ready to transmit a response.  This should eventually cause the GoPro to timeout with its pending
                    // command
                    gp_control_state = GP_CONTROL_STATE_IDLE;
                }

                // Clear the stop condition bit so we can poll it to determine when the GoPro has finished reading our response
                i2c_clr_scd();
            }
            break;

        case GP_CONTROL_STATE_WAIT_FOR_COMPLETE_RESPONSE_SEND:
            // We just have to wait in this state until we see a stop condition on the bus, indicating that the GoPro is
            // done reading our response
            if (i2c_get_scd()) {
                gp_control_state = GP_CONTROL_STATE_IDLE;
            }
            break;
    }
}

void addressed_as_slave_callback(I2CAIntSrc int_src)
{
    // Make sure that this is actually the interrupt we're looking for
    if (int_src == I2C_INT_SRC_ADDRESSED_AS_SLAVE) {
        if (i2c_get_sdir()) {
            // We were addressed as a slave transmitter, which means the GoPro is trying to read from us

            if (gp_control_state == GP_CONTROL_STATE_WAIT_CMD_SEND) {
                // If we're waiting to transmit a command, perform the required actions

                // Deassert the GoPro interrupt request line to indicate that we're ready to transmit the command
                GP_DEASSERT_INTR();

                // Transition to the receive command response state.  The command to send was already
                // preloaded into the ringbuffer before we asserted the interrupt request to the GoPro
                gp_control_state = GP_CONTROL_STATE_RECV_CMD_RESPONSE;
            } else if (gp_control_state == GP_CONTROL_STATE_WAIT_READY_FOR_RESPONSE_SEND) {
                // If we're waiting to respond to a GoPro command, de-assert the interrupt request line to indicate
                // to the GoPro that we're ready for it to read the response
                GP_DEASSERT_INTR();

                // We're done with the transaction now, so we just need to wait for the GoPro to finish reading the
                // response, and then we can go back to being idle
                gp_control_state = GP_CONTROL_STATE_WAIT_FOR_COMPLETE_RESPONSE_SEND;
            }
        } else {
            // We were addressed as a slave receiver, which means the GoPro is trying to write to us

            if (gp_control_state == GP_CONTROL_STATE_IDLE) {
                // If we were idle, change our state to indicate that we're waiting for the GoPro to read
                // data from us.  We need to wait until the GoPro has finished transmitting the command request
                // to assert the interrupt request line, so we check for the stop condition in the state machine
                gp_control_state = GP_CONTROL_STATE_WAIT_READY_FOR_RESPONSE_SEND;

                // Also clear the stop condition detected bit here so we can poll it to
                // determine when the GoPro is done transmitting the command
                i2c_clr_scd();

            }
        }
    }
}
