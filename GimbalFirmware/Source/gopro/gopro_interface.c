/*
 * gopro_interface.c
 *
 *  Created on: Jan 8, 2015
 *      Author: abamberger
 */


#include "f2806x_int8.h"
#include "gopro/gopro_interface.h"
#include "PeripheralHeaderIncludes.h"

#include <ctype.h>

static void gp_timeout(GPCmdResult reason);

volatile GPControlState gp_control_state = GP_CONTROL_STATE_IDLE;
Uint32 gp_power_on_counter = 0;
volatile Uint32 timeout_counter = 0;
Uint8 request_cmd_buffer[GP_COMMAND_REQUEST_SIZE];
Uint8 response_buffer[GP_COMMAND_RESPONSE_SIZE];
Uint8 receive_buffer[GP_COMMAND_RECEIVE_BUFFER_SIZE];
GPCmdResponse last_cmd_response = {0};
Uint8 new_response_available = FALSE;
Uint8 new_get_response_available = FALSE;
Uint8 new_set_response_available = FALSE;
Uint8 new_heartbeat_available = FALSE;
GPExpectingDataType next_reception_expected = GP_EXPECTING_COMMAND;
GPHeartbeatStatus heartbeat_status = GP_HEARTBEAT_DISCONNECTED;
Uint16 heartbeat_counter = 0;

void init_gp_interface()
{
    init_i2c(&addressed_as_slave_callback);
}

Uint16 gp_ready_for_cmd()
{
    return (gp_control_state == GP_CONTROL_STATE_IDLE) && !i2c_get_bb();
}

int gp_send_command(GPCmd* cmd)
{
    if (gp_control_state == GP_CONTROL_STATE_IDLE) {
        // Need to check if this is a command or a query.  Commands have a parameter, queries don't
        // Commands are 2 uppercase characters, queries are 2 lowercase characters
        if (isupper(cmd->cmd[0]) && isupper(cmd->cmd[1])) {
            // This is a command
            // Need to special case 'DL', 'DA', and 'FO' commands, since they don't have parameters
            if (((cmd->cmd[0] == 'D') && ((cmd->cmd[1] == 'L') || (cmd->cmd[1] == 'A'))) ||
                    ((cmd->cmd[0] == 'F') && (cmd->cmd[1] == 'O'))) {
                request_cmd_buffer[0] = 0x2; // Length of 2 (2 command name bytes), with upper bit clear to indicate command originated from the gimbal (not the GoPro)
                request_cmd_buffer[1] = cmd->cmd[0];
                request_cmd_buffer[2] = cmd->cmd[1];
            } else {
                request_cmd_buffer[0] = 0x3; // Length of 3 (2 command name bytes, 1 parameter byte), with upper bit clear to indicate command originated from the gimbal (not the GoPro)
                request_cmd_buffer[1] = cmd->cmd[0];
                request_cmd_buffer[2] = cmd->cmd[1];
                request_cmd_buffer[3] = cmd->cmd_parm;
            }
        } else {
            // This is a query
            request_cmd_buffer[0] = 0x2; // Length of 2 (2 query name bytes), with upper bit clear to indicate command originated from the gimbal (not the GoPro)
            request_cmd_buffer[1] = cmd->cmd[0];
            request_cmd_buffer[2] = cmd->cmd[1];
        }

        // Also load the command name bytes of the last response struct with the command name bytes for this command.  The next response should be a response to
        // this command, and this way a caller of gp_get_last_response can know what command the response goes with
        last_cmd_response.cmd[0] = cmd->cmd[0];
        last_cmd_response.cmd[1] = cmd->cmd[1];

        // Assert the GoPro interrupt line, letting it know we'd like it to read a command from us
        GP_ASSERT_INTR();

        // Reset the timeout counter, and transition to waiting for the GoPro to start reading the command from us
        timeout_counter = 0;
        gp_control_state = GP_CONTROL_STATE_WAIT_FOR_START_CMD_SEND;

        return 0;
    } else {
        return -1;
    }
}

Uint8 gp_get_new_heartbeat_available()
{
    return new_heartbeat_available;
}

Uint8 gp_get_new_get_response_available()
{
    // TODO: Implement
    return new_get_response_available;
}

Uint8 gp_get_new_set_response_available()
{
    // TODO: Implement
    return new_set_response_available;
}

GPHeartbeatStatus* gp_get_heartbeat_status()
{
    // TODO: Implement
    new_heartbeat_available = FALSE;
    //return &last_cmd_response;
}

GPGetResponse* gp_get_last_get_response()
{
    // TODO: Implement
    new_get_response_available = FALSE;
    //return &last_cmd_response;
}

GPSetResponse* gp_get_last_set_response()
{
    // TODO: Implement
    new_set_response_available = FALSE;
    //return &last_cmd_response;
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

int gp_request_power_off()
{
    if ((gp_get_power_status() == GP_POWER_ON) && gp_ready_for_cmd()) {
        GPCmd cmd;
        cmd.cmd[0] = 'P';
        cmd.cmd[1] = 'W';
        cmd.cmd_parm = 0x00;
        gp_send_command(&cmd);
        return 0;
    } else {
        return -1;
    }
}

int gp_get_request(Uint8* cmd_id)
{
    // TODO: Implement
    return -1;
}

int gp_set_request(GPSetRequest* request)
{
    // TODO: Implement
    return -1;
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

        case GP_CONTROL_STATE_WAIT_FOR_START_CMD_SEND:
            // We wait here until we've been addressed by the GoPro, which means it's ready to read the command from us.
            // This will cause an interrupt that changes the state of the state machine, so the only way out of this state from
            // here is to time out.  We time out back to idle if we haven't been addressed for 2 seconds
            if (timeout_counter++ > (GP_TIMEOUT_MS / GP_STATE_MACHINE_PERIOD_MS)) {
                gp_timeout(GP_CMD_SEND_CMD_START_TIMEOUT);
            }
            break;

        case GP_CONTROL_STATE_WAIT_FOR_COMPLETE_CMD_SEND:
            // Wait for either a stop condition, which means the GoPro has finished reading the command, or timeout if we don't
            // see a stop condition in enough time
            if (i2c_get_scd()) {
                // The GoPro has finished reading the command, so transition to waiting for it to transmit a response
                gp_control_state = GP_CONTROL_STATE_WAIT_FOR_CMD_RESPONSE;
            } else if (timeout_counter++ > (GP_TIMEOUT_MS / GP_STATE_MACHINE_PERIOD_MS)) {
                gp_timeout(GP_CMD_SEND_CMD_COMPLETE_TIMEOUT);
            }
            break;

        case GP_CONTROL_STATE_WAIT_FOR_CMD_RESPONSE:
            // We wait here until we've been addressed by the GoPro, which means it's about to transmit its response to us.
            // This will cause an interrupt that changes the state of the state machine, so the only way out of this state from
            // here is to time out.  We time out back to idle if we haven't been addressed for 2 seconds
            if (timeout_counter++ > (GP_TIMEOUT_MS / GP_STATE_MACHINE_PERIOD_MS)) {
                gp_timeout(GP_CMD_GET_RESPONSE_START_TIMEOUT);
            }
            break;

        case GP_CONTROL_STATE_WAIT_FOR_GP_DATA_COMPLETE:
            // Wait for the stop condition to be asserted, meaning the GoPro is done transmitting data
            if (i2c_get_scd()) {
                // We've received all of the data, retrieve it from the ringbuffer
                gp_control_state = GP_CONTROL_STATE_RETRIEVE_RECEIVED_DATA;
            } else if (timeout_counter++ > (GP_TIMEOUT_MS / GP_STATE_MACHINE_PERIOD_MS)) {
                // Timeout if we haven't received all of the GoPro's data in a certain amount of time
                if (next_reception_expected == GP_EXPECTING_COMMAND) {
                    gp_timeout(GP_CMD_GET_CMD_COMPLETE_TIMEOUT);
                } else {
                    gp_timeout(GP_CMD_GET_RESPONSE_COMPLETE_TIMEOUT);
                }
            }
            break;

        case GP_CONTROL_STATE_RETRIEVE_RECEIVED_DATA:
            if (i2c_get_available_chars() > 0) {
                Uint8 retrieve_error = FALSE;

                // Get the length of the received data
                receive_buffer[0] = i2c_get_next_char();

                // If we're expecting a command, we need to mask out the top bit of the length field
                // (as this is used to signal which direction the command was in, backpack to camera or camera to backpack,
                // not as part of the message length)
                if (next_reception_expected == GP_EXPECTING_COMMAND) {
                    receive_buffer[0] &= 0x7F;
                }

                int i;
                for (i = 0; i < receive_buffer[0]; i++) {
                    if ((i + 1) > GP_COMMAND_RECEIVE_BUFFER_SIZE) {
                        // Indicate the error
                        last_cmd_response.cmd_result = GP_CMD_RECEIVED_DATA_OVERFLOW;
                        new_response_available = TRUE;
                        retrieve_error = TRUE;

                        // Avoid overflowing the receive buffer
                        break;
                    } else if (!(i2c_get_available_chars() > 0)) {
                        // Indicate the error
                        last_cmd_response.cmd_result = GP_CMD_RECEIVED_DATA_UNDERFLOW;
                        new_response_available = TRUE;
                        retrieve_error = TRUE;

                        // Avoid corrupting the receive ringbuffer
                        break;
                    } else {
                        // Retrieve the next byte in the response
                        receive_buffer[i + 1] = i2c_get_next_char();
                    }
                }

                // If we had an error while retrieving the data, abort and go back to idle
                // Else, continue on to parsing the retrieved data
                if (retrieve_error) {
                    gp_control_state = GP_CONTROL_STATE_IDLE;
                } else {
                    // Parse the retrieved data differently depending on whether we're expecting a command or a response
                    if (next_reception_expected == GP_EXPECTING_COMMAND) {
                        gp_control_state = GP_CONTROL_STATE_PARSE_RECEIVED_CMD;
                    } else {
                        gp_control_state = GP_CONTROL_STATE_PARSE_RECEIVED_RESPONSE;
                    }
                }
            } else {
                // If there was no data in the receive buffer when we expect there to be, indicate the error
                last_cmd_response.cmd_result = GP_CMD_RECEIVED_DATA_UNDERFLOW;
                new_response_available = TRUE;
                gp_control_state = GP_CONTROL_STATE_IDLE;
            }
            break;

        case GP_CONTROL_STATE_PARSE_RECEIVED_CMD:
            // First make sure the command is one we support.  Per the GoPro spec, we only have to respond to the "vs" command
            // to query the version of the protocol we support.  For any other command from the GoPro, return an error response
            if ((receive_buffer[1] == 'v') && (receive_buffer[2] == 's')) {
                // Preload the response buffer with the command response.  This will be transmitted in the ISR
                response_buffer[0] = 2; // Packet size, 1st byte is status byte, 2nd byte is protocol version
                response_buffer[1] = GP_CMD_STATUS_SUCCESS;
                response_buffer[2] = GP_PROTOCOL_VERSION;
            } else {
                // Preload the response buffer with an error response, since we don't support the command we
                // were sent.  This will be transmitted in the ISR
                response_buffer[0] = 1; // Packet size, only status byte
                response_buffer[1] = GP_CMD_STATUS_FAILURE;
            }

            // Assert the interrupt request line to indicate that we're ready to respond to the GoPro's command
            GP_ASSERT_INTR();

            gp_control_state = GP_CONTROL_STATE_WAIT_READY_TO_SEND_RESPONSE;
            timeout_counter = 0;
            break;

        case GP_CONTROL_STATE_WAIT_READY_TO_SEND_RESPONSE:
            // We wait here until we've been addressed by the GoPro, which means it's about to read a response from us.
            // This will cause an interrupt that changes the state of the state machine, so the only way out of this state from
            // here is to time out.  We time out back to idle if we haven't been addressed for 2 seconds
            if (timeout_counter++ > (GP_TIMEOUT_MS / GP_STATE_MACHINE_PERIOD_MS)) {
                gp_timeout(GP_CMD_SEND_RESPONSE_START_TIMEOUT);
            }
            break;

        case GP_CONTROL_STATE_WAIT_TO_COMPLETE_RESPONSE_SEND:
            // We just have to wait in this state until we see a stop condition on the bus, indicating that the GoPro is
            // done reading our response, or timeout if the GoPro hasn't read our response in a certain amount of time
            if (i2c_get_scd()) {
                gp_control_state = GP_CONTROL_STATE_IDLE;
            } else if (timeout_counter++ > (GP_TIMEOUT_MS / GP_STATE_MACHINE_PERIOD_MS)) {
                gp_timeout(GP_CMD_SEND_RESPONSE_COMPLETE_TIMEOUT);
            }
            break;

        case GP_CONTROL_STATE_PARSE_RECEIVED_RESPONSE:
            // Load the last command response struct with the values from the actual response
            last_cmd_response.cmd_status = receive_buffer[1];
            last_cmd_response.cmd_response = receive_buffer[2];

            // The full command transmit has now completed successfully, so we can go back to idle
            last_cmd_response.cmd_result = GP_CMD_SUCCESSFUL;
            gp_control_state = GP_CONTROL_STATE_IDLE;

            // Indicate that there is a new response available
            new_response_available = TRUE;
            break;
    }

	// Periodically signal a MAVLINK_MSG_ID_GOPRO_HEARTBEAT message to be sent
	if (++heartbeat_counter >= (GP_MAVLINK_HEARTBEAT_INTERVAL / GP_STATE_MACHINE_PERIOD_MS))
	{
		new_heartbeat_available = TRUE;
		heartbeat_counter = 0;
	}
}

void addressed_as_slave_callback(I2CAIntSrc int_src)
{
    // Make sure that this is actually the interrupt we're looking for
    if (int_src == I2C_INT_SRC_ADDRESSED_AS_SLAVE) {
        if (i2c_get_sdir()) {
            // We were addressed as a slave transmitter, which means the GoPro is trying to read from us

            if (gp_control_state == GP_CONTROL_STATE_WAIT_FOR_START_CMD_SEND) {
                // If we're waiting to transmit a command, perform the required actions

                // Load the pending request into the I2C TX ringbuffer.  This will start the process of transferring the
                // data to the I2C peripheral to send out
                i2c_send_data(request_cmd_buffer, request_cmd_buffer[0] + 1); // Length of command is 1st byte in command buffer, add 1 for length field

                // Deassert the GoPro interrupt request line to indicate that we're ready to transmit the command
                GP_DEASSERT_INTR();

                // Clear the timeout counter in preparation of waiting for a response from the GoPro
                timeout_counter = 0;

                // Clear the stop condition detected bit so we can poll it to determine when the GoPro is finished reading the command
                i2c_clr_scd();

                // Wait for the GoPro to finish reading the command
                gp_control_state = GP_CONTROL_STATE_WAIT_FOR_COMPLETE_CMD_SEND;

            } else if (gp_control_state == GP_CONTROL_STATE_WAIT_READY_TO_SEND_RESPONSE) {
                // If we're waiting to respond to a GoPro command, first put the assembled response into the I2C transmit ringbuffer
                i2c_send_data(response_buffer, response_buffer[0] + 1); // Size of message is count in size field of message, +1 for size byte

                // De-assert the interrupt request line to indicate to the GoPro that we're ready for it to read the response
                GP_DEASSERT_INTR();

                // Clear the stop condition detected bit so we can poll it to determine when the GoPro is finished reading our response
                i2c_clr_scd();
                timeout_counter = 0;

                // We're done with the transaction now, so we just need to wait for the GoPro to finish reading the
                // response, and then we can go back to being idle
                gp_control_state = GP_CONTROL_STATE_WAIT_TO_COMPLETE_RESPONSE_SEND;
            }
        } else {
            // We were addressed as a slave receiver, which means the GoPro is trying to write to us

            if (gp_control_state == GP_CONTROL_STATE_IDLE) {
                // If we were idle, that means the GoPro is sending us a command.  Transition to waiting for the GoPro to finish transmitting the command
                next_reception_expected = GP_EXPECTING_COMMAND;
                gp_control_state = GP_CONTROL_STATE_WAIT_FOR_GP_DATA_COMPLETE;
                timeout_counter = 0;

                // Clear the stop condition detected bit so we can poll it to
                // determine when the GoPro is done transmitting the command
                i2c_clr_scd();

            } else if ((gp_control_state == GP_CONTROL_STATE_WAIT_FOR_COMPLETE_CMD_SEND) || (gp_control_state == GP_CONTROL_STATE_WAIT_FOR_CMD_RESPONSE)) {
                // We've been addressed by the GoPro to write its response back to us.  Now we just need
                // to wait for it to finish writing the response
                next_reception_expected = GP_EXPECTING_RESPONSE;
                gp_control_state = GP_CONTROL_STATE_WAIT_FOR_GP_DATA_COMPLETE;
                timeout_counter = 0;

                // Clear the stop condition detected bit so we can poll it to determine when
                // the GoPro is done transmitting the response
                i2c_clr_scd();

            } else if (gp_control_state == GP_CONTROL_STATE_WAIT_FOR_START_CMD_SEND) {
                // This means we have asked the GoPro to read a command from us, but before it has started to read the command,
                // it issues a command to us first.  Per the spec, we have to give up on our command request and service the GoPro's command

                // Indicate that the command we were trying to send has been preempted by the GoPro
                last_cmd_response.cmd_result = GP_CMD_PREEMPTED;

                // Indicate that a "new response" is available (what's available is the indication that the command was preempted)
                new_response_available = TRUE;

                // De-assert the interrupt line, since we're no longer trying to send a command to the GoPro
                GP_DEASSERT_INTR();

                // Clear the stop condition detected bit so we can poll it to determine when the GoPro is done transmitting the command
                i2c_clr_scd();

                // Transition to waiting for the GoPro to finish sending the command
                next_reception_expected = GP_EXPECTING_COMMAND;
                gp_control_state = GP_CONTROL_STATE_WAIT_FOR_GP_DATA_COMPLETE;
                timeout_counter = 0;
            }
        }
    }
}

static void gp_timeout(GPCmdResult reason)
{
    timeout_counter = 0; // Reset the timeout counter so it doesn't have an old value in it the next time we want to use it
    GP_DEASSERT_INTR(); // De-assert the interrupt request (even if it wasn't previously asserted, in idle the interrupt request should always be deasserted)
    last_cmd_response.cmd_result = reason;
    new_response_available = TRUE; // Indicate that a "new response" is available (what's available is the indication that we timed out)
    gp_control_state = GP_CONTROL_STATE_IDLE;
}
