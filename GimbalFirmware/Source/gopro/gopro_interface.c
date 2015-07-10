#include "f2806x_int8.h"
#include "gopro/gopro_interface.h"
#include "gopro_i2c.h"
#include "PeripheralHeaderIncludes.h"

// Include for GOPRO_COMMAND enum
#include "mavlink_interface/mavlink_gimbal_interface.h"

#include <ctype.h>

static void gp_timeout();
static bool gp_cmd_has_param(const GPCmd* c);

static void handle_rx_data();
static bool collect_i2c_data(uint16_t *buf, uint16_t maxlen, bool *from_camera);
static void gp_handle_command(const uint16_t *cmdbuf);
static void gp_handle_response(const uint16_t *respbuf);

volatile GPControlState gp_control_state = GP_CONTROL_STATE_IDLE;
Uint32 gp_power_on_counter = 0;
volatile Uint32 timeout_counter = 0;

Uint8 txbuf[GP_COMMAND_REQUEST_SIZE];
GPCmdResponse last_cmd_response = {0};

bool new_response_available = false;
bool new_heartbeat_available = false;

Uint16 heartbeat_counter = 0;
GPRequestType last_request_type = GP_REQUEST_NONE;
GOPRO_COMMAND last_request_cmd_id;
GPGetResponse last_get_response;
GPSetResponse last_set_response;
GPSetRequest last_set_request;
GPPowerStatus previous_power_status = GP_POWER_UNKNOWN;

bool gccb_version_queried = false;

typedef struct {
    bool waiting_for_i2c; // waiting for i2c either tx/rx
} gopro_t;

// global gopro instance
static gopro_t gp;


/*
 * backpack detect line management - lets the gopro know we're connected.
 * active low.
 */

static inline void gp_set_backpack_detect_active() {
    GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;
}

static inline void gp_set_backpack_detect_inactive(void) {
    GpioDataRegs.GPASET.bit.GPIO28 = 1;
}


void init_gp_interface()
{
    gp.waiting_for_i2c = false;
    gp_deassert_intr();

    gopro_i2c_init();

    gp_set_backpack_detect_active();
}

bool gp_ready_for_cmd()
{
    return (gp_control_state == GP_CONTROL_STATE_IDLE) && !i2c_get_bb();
}

bool gp_cmd_has_param(const GPCmd* c)
{
    /*
     * For the most part, commands have a parameter, queries never do.
     * Commands are 2 uppercase characters, queries are 2 lowercase characters
     */

    if (islower(c->cmd[0])) {
        return false;
    }

    // Need to special case 'DL', 'DA', and 'FO' commands, since they don't have parameters
    if ((c->cmd[0] == 'F') && (c->cmd[1] == 'O')) {
        return false;
    }

    if (c->cmd[0] == 'D' && (c->cmd[1] == 'L' || c->cmd[1] == 'A')) {
        return false;
    }

    return true;
}

bool gp_send_command(const GPCmd* cmd)
{
    if (gp_control_state != GP_CONTROL_STATE_IDLE) {
        return false;
    }

    txbuf[1] = cmd->cmd[0];
    txbuf[2] = cmd->cmd[1];

    // first byte is len, upper bit clear to indicate command originated from the gimbal (not the GoPro)
    if (gp_cmd_has_param(cmd)) {
        txbuf[0] = 0x3;
        txbuf[3] = cmd->cmd_parm;
    } else {
        txbuf[0] = 0x2;
    }

    // Clear the last command data
    last_cmd_response.value = 0x00;
    last_cmd_response.status = GP_CMD_STATUS_UNKNOWN;
    last_cmd_response.result = GP_CMD_UNKNOWN;

    // Also load the command name bytes of the last response struct with the command name bytes for this command.  The next response should be a response to
    // this command, and this way a caller of gp_get_last_response can know what command the response goes with
    last_cmd_response.cmd[0] = cmd->cmd[0];
    last_cmd_response.cmd[1] = cmd->cmd[1];

    // Assert the GoPro interrupt line, letting it know we'd like it to read a command from us
    gp_assert_intr();

    // Reset the timeout counter, and transition to waiting for the GoPro to start reading the command from us
    timeout_counter = 0;
    gp_control_state = GP_CONTROL_STATE_WAIT_FOR_START_CMD_SEND;

    return true;
}

bool gp_new_heartbeat_available()
{
    return new_heartbeat_available;
}

bool gp_new_get_response_available()
{
    if (last_request_type == GP_REQUEST_GET) {
        return new_response_available;
    }

    return false;
}

bool gp_new_set_response_available()
{
    if (last_request_type == GP_REQUEST_SET) {
        return new_response_available;
    }

    return false;
}

GPHeartbeatStatus gp_heartbeat_status()
{
	GPHeartbeatStatus heartbeat_status = GP_HEARTBEAT_DISCONNECTED;
	/*if (gp_get_power_status() == GP_POWER_ON
			&& gp_ready_for_cmd()
			&& last_request_type == GP_REQUEST_SET
			&& last_set_request.cmd_id == GOPRO_COMMAND_SHUTTER
			&& last_set_request.value == 1) {
			heartbeat_status = GP_HEARTBEAT_RECORDING;
	} else */if (gp_get_power_status() == GP_POWER_ON && gp_ready_for_cmd() && gccb_version_queried == 1) {
		heartbeat_status = GP_HEARTBEAT_CONNECTED;
	} else if (gp_get_power_status() != GP_POWER_ON && !i2c_get_bb()) {
		// If the power isn't 'on' but the I2C lines are still pulled high, it's likely an incompatible Hero 4 firmware
		heartbeat_status = GP_HEARTBEAT_INCOMPATIBLE;
	} else if (gp_get_power_status() == GP_POWER_ON && gccb_version_queried == 0) {
		heartbeat_status = GP_HEARTBEAT_INCOMPATIBLE;
	}
    new_heartbeat_available = false;
    return heartbeat_status;
}

GPGetResponse gp_last_get_response()
{
	last_get_response.cmd_id = last_request_cmd_id;
    if (last_cmd_response.status == GP_CMD_STATUS_SUCCESS && last_cmd_response.result == GP_CMD_SUCCESSFUL) {
        last_get_response.value = last_cmd_response.value;
	} else {
		last_get_response.value = 0xFF;
	}

	// Clear the last command response
    new_response_available = false;

    return last_get_response;
}

GPSetResponse gp_last_set_response()
{
	last_set_response.cmd_id = last_request_cmd_id;
    if (last_cmd_response.status == GP_CMD_STATUS_SUCCESS && last_cmd_response.result == GP_CMD_SUCCESSFUL) {
		last_set_response.result = GOPRO_SET_RESPONSE_RESULT_SUCCESS;
	} else {
		last_set_response.result = GOPRO_SET_RESPONSE_RESULT_FAILURE;
	}
    new_response_available = false;
    return last_set_response;
}

bool gp_request_power_on()
{
    if (gp_control_state == GP_CONTROL_STATE_IDLE) {
        gp_control_state = GP_CONTROL_STATE_REQUEST_POWER_ON;
        return true;
    }

    return false;
}

bool gp_request_power_off()
{
    if ((gp_get_power_status() == GP_POWER_ON) && gp_ready_for_cmd()) {
        GPCmd cmd;
        cmd.cmd[0] = 'P';
        cmd.cmd[1] = 'W';
        cmd.cmd_parm = 0x00;
        gp_send_command(&cmd);
        return true;
    }

    return false;
}

int gp_get_request(Uint8 cmd_id)
{
    /*
     * Called when a CAN msg has been received with a `gopro get request` msg type.
     *
     * Fire off the transaction for the requested cmd,
     * and assume that the CAN layer will pick up the response
     * via gp_get_last_get_response()
     */

	last_request_type = GP_REQUEST_GET;
	if ((gp_get_power_status() == GP_POWER_ON) && gp_ready_for_cmd()) {
		GPCmd cmd;

		switch(cmd_id) {
			case GOPRO_COMMAND_SHUTTER:
				cmd.cmd[0] = 's';
				cmd.cmd[1] = 'h';
			break;

			case GOPRO_COMMAND_CAPTURE_MODE:
				cmd.cmd[0] = 'c';
				cmd.cmd[1] = 'm';
			break;

			case GOPRO_COMMAND_MODEL:
				cmd.cmd[0] = 'c';
				cmd.cmd[1] = 'v';
			break;

			case GOPRO_COMMAND_BATTERY:
				cmd.cmd[0] = 'b';
				cmd.cmd[1] = 'l';
            break;

			default:
				// Unsupported Command ID
				last_request_cmd_id = (GOPRO_COMMAND)cmd_id;
                new_response_available = true;
				return -1;
		}

		last_request_cmd_id = (GOPRO_COMMAND)cmd_id;
		gp_send_command(&cmd);
		return 0;
	} else {
		last_request_cmd_id = (GOPRO_COMMAND)cmd_id;
        new_response_available = true;
		return -1;
	}
}

int gp_set_request(GPSetRequest* request)
{
    /*
     * Called when a CAN msg has been received with a 'gopro set request' msg type.
     *
     * Forward the requested set cmd,
     * and assume that the CAN layer will pick up the response
     * via gp_get_last_set_response()
     */

	last_request_type = GP_REQUEST_SET;

	// GoPro has to be powered on and ready, or the command has to be a power on command
	if ((gp_get_power_status() == GP_POWER_ON || (request->cmd_id == GOPRO_COMMAND_POWER && request->value == 0x01)) && gp_ready_for_cmd()) {
		GPCmd cmd;

		switch(request->cmd_id) {
			case GOPRO_COMMAND_POWER:
				if(request->value == 0x00 && gp_get_power_status() == GP_POWER_ON) {
					cmd.cmd[0] = 'P';
					cmd.cmd[1] = 'W';
					cmd.cmd_parm = 0x00;
					gp_send_command(&cmd);
				} else {
					gp_request_power_on();
				}
				break;

			case GOPRO_COMMAND_CAPTURE_MODE:
				cmd.cmd[0] = 'C';
				cmd.cmd[1] = 'M';
				cmd.cmd_parm = request->value;
				break;

			case GOPRO_COMMAND_SHUTTER:
				cmd.cmd[0] = 'S';
				cmd.cmd[1] = 'H';
				cmd.cmd_parm = request->value;
                break;

            case GOPRO_COMMAND_RESOLUTION:
                cmd.cmd[0] = 'V';
                cmd.cmd[1] = 'V';
                cmd.cmd_parm = request->value;
                break;

            case GOPRO_COMMAND_FRAME_RATE:
                cmd.cmd[0] = 'F';
                cmd.cmd[1] = 'S';
                cmd.cmd_parm = request->value;
                break;

            case GOPRO_COMMAND_FIELD_OF_VIEW:
                cmd.cmd[0] = 'F';
                cmd.cmd[1] = 'V';
                cmd.cmd_parm = request->value;
                break;

			default:
				// Unsupported Command ID
				last_request_cmd_id = (GOPRO_COMMAND)request->cmd_id;
                new_response_available = true;
				return -1;
		}

		last_set_request = *request;
		last_request_cmd_id = (GOPRO_COMMAND)request->cmd_id;
		gp_send_command(&cmd);
		return 0;
	} else {
		last_request_cmd_id = (GOPRO_COMMAND)request->cmd_id;
        new_response_available = true;
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

// It's expected that this function is repeatedly called every period as configured in the header (currently 3ms)
// for proper gopro interface operation
void gp_interface_state_machine()
{
    if (gp.waiting_for_i2c) {
        if (gopro_i2c_in_progress()) {
            if (timeout_counter++ > (GP_TIMEOUT_MS / GP_STATE_MACHINE_PERIOD_MS)) {
                gp_timeout();
            }
        } else {
            gp.waiting_for_i2c = false;

            if (gp_control_state == GP_CONTROL_STATE_WAIT_FOR_GP_DATA_COMPLETE) {
                // transaction was rx
                handle_rx_data();

            } else {
                // transaction was tx
                if (gp_control_state == GP_CONTROL_STATE_WAIT_FOR_COMPLETE_CMD_SEND) {
                    gp_control_state =  GP_CONTROL_STATE_WAIT_FOR_CMD_RESPONSE;
                } else {
                    // sent response, we're all done
                    gp_control_state = GP_CONTROL_STATE_IDLE;
                }
            }
        }
    }

    switch (gp_control_state) {
        case GP_CONTROL_STATE_REQUEST_POWER_ON:
            GP_PWRON_LOW();
            gp_power_on_counter = 0;
            gp_control_state = GP_CONTROL_STATE_WAIT_POWER_ON;
            break;

        case GP_CONTROL_STATE_WAIT_POWER_ON:
            if (gp_power_on_counter++ > (GP_PWRON_TIME_MS / GP_STATE_MACHINE_PERIOD_MS)) {
                GP_PWRON_HIGH();
                gp_control_state = GP_CONTROL_STATE_IDLE;
                last_cmd_response.result = GP_CMD_SUCCESSFUL;
                new_response_available = true;
            }
            break;
    }

	// Periodically signal a MAVLINK_MSG_ID_GOPRO_HEARTBEAT message to be sent
	if (++heartbeat_counter >= (GP_MAVLINK_HEARTBEAT_INTERVAL / GP_STATE_MACHINE_PERIOD_MS)) {
        new_heartbeat_available = true;
		heartbeat_counter = 0;
	}

	// Detect a change in power status to reset some flags when a GoPro is re-connected during operation
	GPPowerStatus new_power_status = gp_get_power_status();
	if(previous_power_status != new_power_status) {
		gccb_version_queried = 0;
	}
	previous_power_status = new_power_status;
}

void handle_rx_data()
{
    /*
     * Called when an i2c rx transaction has completed.
     *
     * Check if the data is formatted correctly and process accordingly.
     */

    bool from_camera;
    uint16_t rxbuf[GP_COMMAND_RECEIVE_BUFFER_SIZE];

    if (collect_i2c_data(rxbuf, GP_COMMAND_RECEIVE_BUFFER_SIZE, &from_camera)) {
        // Parse the retrieved data differently depending on whether it's a command or response
        if (from_camera) {
            gp_handle_command(rxbuf);
        } else {
            gp_handle_response(rxbuf);
        }
    } else {
        // error in data rx, return to idle
        new_response_available = true;
        gp_control_state = GP_CONTROL_STATE_IDLE;
    }
}

bool collect_i2c_data(uint16_t *buf, uint16_t maxlen, bool *from_camera)
{
    /*
     * Called when an i2c rx transaction has completed successfully,
     * to determine if the received data is formatted correctly.
     *
     * Drain all the data from the i2c ringbuf, and ensure
     * the advertised len matches what we actually recevied.
     */

    int len = i2c_get_available_chars();
    if (len <= 0) {
        return false;
    }

    // first byte is the length of the received data,
    // top bit signals the cmd originator, 1 == camera, 0 == backpack
    uint16_t b = i2c_get_next_char();
    *from_camera = b & (1 << 7);
    buf[0] = b & 0x7f;

    if (buf[0] != len - 1 || buf[0] > maxlen) {
        // under/overflow, drain i2c ringbuf
        // ... btw, why are we using a ringbuf, rather than a simple buffer per msg?
        while (i2c_get_available_chars() > 0) {
            (void)i2c_get_next_char();
        }
        return false;
    }

    int i;
    for (i = 0; i < buf[0]; ++i) {
        buf[i + 1] = i2c_get_next_char();
    }

    return true;
}

void gp_handle_command(const uint16_t *cmdbuf)
{
    /*
     * A validated command has been received from the camera.
     *
     * First make sure the command is one we support.  Per the GoPro spec,
     * we only have to respond to the "vs" command which queries
     * the version of the protocol we support.
     *
     * For any other command from the GoPro, return an error response
     */

    if ((cmdbuf[1] == 'v') && (cmdbuf[2] == 's')) {
        // Preload the response buffer with the command response.  This will be transmitted in the ISR
        txbuf[0] = 2; // Packet size, 1st byte is status byte, 2nd byte is protocol version
        txbuf[1] = GP_CMD_STATUS_SUCCESS;
        txbuf[2] = GP_PROTOCOL_VERSION;
        gccb_version_queried = 1;

    } else {
        // Preload the response buffer with an error response, since we don't support the command we
        // were sent.  This will be transmitted in the ISR
        txbuf[0] = 1; // Packet size, only status byte
        txbuf[1] = GP_CMD_STATUS_FAILURE;
    }

    // Assert the interrupt request line to indicate that we're ready to respond to the GoPro's command
    gp_assert_intr();

    gp_control_state = GP_CONTROL_STATE_WAIT_READY_TO_SEND_RESPONSE;
    timeout_counter = 0;
}

void gp_handle_response(const uint16_t *respbuf)
{
    /*
     * Process a response to one of our commands.
     */

    last_cmd_response.status = (GPCmdStatus)respbuf[1];

    // Special Handling of responses
    if (last_cmd_response.cmd[0] == 'c' && last_cmd_response.cmd[1] == 'v') {
        // Take third byte (CAMERA_MODEL) of the "camera model and firmware version" response
        last_cmd_response.value = respbuf[3];
    } else {
        last_cmd_response.value = respbuf[2];
    }

    // The full command transmit has now completed successfully, so we can go back to idle
    last_cmd_response.result = GP_CMD_SUCCESSFUL;
    gp_control_state = GP_CONTROL_STATE_IDLE;

    // Indicate that there is a new response available
    new_response_available = true;
}

void gp_write_eeprom()
{
	if(GP_VON != 1)
		return;

	// Disable the HeroBus port (GoPro should stop mastering the I2C bus)
    gp_set_backpack_detect_inactive();

	// Data to write into EEPROM
	uint8_t EEPROMData[GP_I2C_EEPROM_NUMBYTES] = {0x0E, 0x03, 0x01, 0x12, 0x0E, 0x03, 0x01, 0x12, 0x0E, 0x03, 0x01, 0x12, 0x0E, 0x03, 0x01, 0x12};

	// Init I2C peripheral
	I2caRegs.I2CMDR.all = 0x0000;
	I2caRegs.I2CSAR = 0x0050;					//Set slave address
	I2caRegs.I2CPSC.bit.IPSC = 6;				//Prescaler - need 7-12 Mhz on module clk

	// Setup I2C clock
	I2caRegs.I2CCLKL = 10;						// NOTE: must be non zero
	I2caRegs.I2CCLKH = 5;						// NOTE: must be non zero

	I2caRegs.I2CMDR.all = 0x0020;

	// Setup I2C FIFO
	I2caRegs.I2CFFTX.all = 0x6000;
	I2caRegs.I2CFFRX.all = 0x2040;

	// Reset the I2C bus
	I2caRegs.I2CMDR.bit.IRS = 1;
	I2caRegs.I2CSAR = 0x0050;

	// Wait for the I2C bus to become available
	while (I2caRegs.I2CSTR.bit.BB == 1);

	// Start, stop, no rm, reset i2c
	I2caRegs.I2CMDR.all = 0x6E20;

	uint8_t i;
	for(i = 0; i < GP_I2C_EEPROM_NUMBYTES; i++){
		// Setup I2C Master Write
		I2caRegs.I2CMDR.all = 0x6E20;
		I2caRegs.I2CCNT = 2;
		I2caRegs.I2CDXR = i;
		I2caRegs.I2CMDR.bit.STP = 1;
		while(I2caRegs.I2CSTR.bit.XRDY == 0){};

		// Write data byte and wait till it's shifted out
		I2caRegs.I2CDXR = EEPROMData[i];
		while(I2caRegs.I2CSTR.bit.XRDY == 0){};

		// Let the EEPROM write
		ADC_DELAY_US(5000);
	}

    // eeprom must be init'd before the gopro interface,
    // so wait for that to re-enable the backpack detect line.
    // otherwise we might miss traffic from the gopro before that gets initialized.
}

void gp_on_slave_address(bool addressed_as_tx)
{
    /*
     * Called in ISR context when the i2c device detects
     * that we've been successfully addressed by the camera.
     *
     *
     */

    timeout_counter = 0;
    gp.waiting_for_i2c = true;

    if (addressed_as_tx) {
        // send data as appropriate
        // length is 1st byte in command buffer, add 1 for length field

        gopro_i2c_send(txbuf, txbuf[0] + 1);

        // keep track of whether we're sending a cmd or response.
        // if sending cmd, need to know whether to wait for a response.
        if (gp_control_state == GP_CONTROL_STATE_WAIT_FOR_START_CMD_SEND) {
            gp_control_state =  GP_CONTROL_STATE_WAIT_FOR_COMPLETE_CMD_SEND;

        } else {
            gp_control_state = GP_CONTROL_STATE_WAIT_TO_COMPLETE_RESPONSE_SEND;
        }

    } else {
        // addressed as receiver.
        // in the general case, we wait for the stop condition to indicate a transaction is complete.

        if (gp_control_state == GP_CONTROL_STATE_WAIT_FOR_START_CMD_SEND) {
            // Special case - we have asked the GoPro to read a command from us, but before it has started to read the command,
            // it issues a command to us first.  Per the spec, we have to give up on our command request and service the GoPro's command

            // Indicate that the command we were trying to send has been preempted by the GoPro
            last_cmd_response.result = GP_CMD_PREEMPTED;
            // Indicate that a "new response" is available (what's available is the indication that the command was preempted)
            new_response_available = true;
        }

        // wait for the rx to complete
        gp_control_state = GP_CONTROL_STATE_WAIT_FOR_GP_DATA_COMPLETE;
    }
}

void gp_timeout()
{
    gopro_i2c_on_timeout();
    gp.waiting_for_i2c = false;

    timeout_counter = 0; // Reset the timeout counter so it doesn't have an old value in it the next time we want to use it
    gp_deassert_intr(); // De-assert the interrupt request (even if it wasn't previously asserted, in idle the interrupt request should always be deasserted)

    // Indicate that a "new response" is available (what's available is the indication that we timed out)
    //last_cmd_response.cmd_result = reason;
    //new_response_available = true;

    gp_control_state = GP_CONTROL_STATE_IDLE;
}
