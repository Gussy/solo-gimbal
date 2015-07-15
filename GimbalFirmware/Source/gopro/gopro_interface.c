#include "f2806x_int8.h"
#include "gopro_interface.h"
#include "gopro_i2c.h"
#include "gopro_hero_common.h"
#include "gopro_hero3p.h"
#include "gopro_hero4.h"
#include "PeripheralHeaderIncludes.h"

// Include for GOPRO_COMMAND enum
#include "mavlink_interface/mavlink_gimbal_interface.h"

#include <ctype.h>

static void gp_timeout();
static bool gp_cmd_has_param(const GPCmd* c);

static void handle_rx_data(uint16_t *buf, uint16_t len);
static void gp_handle_command(const uint16_t *cmdbuf);
static void gp_handle_response(const uint16_t *respbuf);

volatile GPControlState gp_control_state = GP_CONTROL_STATE_IDLE;
Uint32 gp_power_on_counter = 0;
volatile Uint32 timeout_counter = 0;

#define TX_BUF_SZ  128
#define RX_BUF_SZ  128

static uint16_t txbuf[TX_BUF_SZ];
static uint16_t rxbuf[RX_BUF_SZ];

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
uint16_t init_timeout_ms = 0;
bool init_timed_out = false;



typedef struct {
    bool waiting_for_i2c; // waiting for i2c either tx/rx

	bool response_pending;
    gp_response_t response;

	GPModel model;

    gp_h3p_t h3p;
    gp_h4_t h4;

} gopro_t;

// global gopro instance
static gopro_t gp;


void init_gp_interface()
{
    gp.waiting_for_i2c = false;
    gp.model = GP_MODEL_UNKNOWN;
    gp_deassert_intr();

    gp_h4_init(&gp.h4);

    gopro_i2c_init();

    gp_enable_hb_interface();
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
    /*
     * XXX: phase this out
     *      transition hero3+ specific portions to h3p module
     *      and then use gp_send_cmd() below
     */

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

bool gp_send_cmd(const uint16_t* cmd, uint16_t len)
{
    /*
     * Called by protocol modules (h4, h3p, etc)
     * to send a command.
     */

    if (gp_control_state != GP_CONTROL_STATE_IDLE) {
        return false;
    }

    int i;
    for (i = 0; i < len; ++i) {
        txbuf[i] = cmd[i];
    }

    // Assert the GoPro interrupt line, letting it know we'd like it to read a command from us
    gp_assert_intr();

    // Reset the timeout counter, and transition to waiting for the GoPro to start reading the command from us
    timeout_counter = 0;
    gp_control_state = GP_CONTROL_STATE_WAIT_FOR_START_CMD_SEND;

    return true;
}

void gp_set_response(const uint16_t *resp_bytes, uint16_t len, GPRequestType reqtype, GPCmdStatus status)
{
    /*
     * Called from a gopro protocol handler to indicate
     * that there is data available to be passed back up
     * through the CAN interface.
     */

    uint16_t i;
    for (i = 0; i < len; ++i) {
        gp.response.payload[i] = resp_bytes[i];
    }

    gp.response.reqtype = reqtype;
    gp.response.status = status;
    gp.response.len = len;
    gp.response_pending = true;
}

bool gp_new_heartbeat_available()
{
    return new_heartbeat_available;
}

bool gp_response_available()
{
    return gp.response_pending;
}

bool gp_get_response(gp_response_t ** rsp)
{
    /*
     * Retrieve the currently pending response.
     * Returns false if there is not a new response available.
     */

    if (!gp.response_pending) {
        return false;
    }

    *rsp = &gp.response;

    gp.response_pending = false;
    return true;
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
	} else if (gp_get_power_status() != GP_POWER_ON && !i2c_get_bb() && GP_VON) {
		// If the power isn't 'on' but the I2C lines are still pulled high, it's likely an incompatible Hero 4 firmware
		heartbeat_status = GP_HEARTBEAT_INCOMPATIBLE;
	} else if (gp_get_power_status() == GP_POWER_ON && gccb_version_queried == 0 && init_timed_out) {
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
        switch (gp.model) {
            case GP_MODEL_HERO3P:
                return gp_h3p_request_power_off();
            case GP_MODEL_HERO4:
                return gp_h4_request_power_off();
            case GP_MODEL_UNKNOWN:
                return false;
            default:
                return false;
        }
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
        switch (gp.model) {
            case GP_MODEL_HERO3P:
                return gp_h3p_get_request(cmd_id, &new_response_available, &last_request_cmd_id);
            case GP_MODEL_HERO4:
                return gp_h4_get_request(cmd_id, &last_request_cmd_id);
            case GP_MODEL_UNKNOWN:
                return -1;
            default:
                return -1;
        }
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
        switch (gp.model) {
            case GP_MODEL_HERO3P:
                return gp_h3p_set_request(request, &new_response_available, &last_set_request, &last_request_cmd_id);
            case GP_MODEL_HERO4:
                return gp_h4_set_request(request, &last_request_cmd_id);
            case GP_MODEL_UNKNOWN:
                return -1;
            default:
                return -1;
        }
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

void gp_enable_hb_interface()
{
    // Set BacPac detect low (active low)
    GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;
}

void gp_disable_hb_interface()
{
    // Set BacPac detect high (active low)
    GpioDataRegs.GPASET.bit.GPIO28 = 1;
}

void gp_enable_charging()
{
    // Set GoPro 5v enable low (active low)
    GpioDataRegs.GPACLEAR.bit.GPIO23 = 1;
}

void gp_disable_charging()
{
    // Set GoPro 5v enable high (active low)
    GpioDataRegs.GPASET.bit.GPIO23 = 1;
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
                handle_rx_data(rxbuf, i2c_get_rx_len());

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

    // Set 'init_timed_out' to true after GP_INIT_TIMEOUT_MS to avoid glitching
    // the heartbeat with an incompatible state while it's gccb version is being queried
    if(!gccb_version_queried && !init_timed_out && ++init_timeout_ms >= (GP_INIT_TIMEOUT_MS / GP_STATE_MACHINE_PERIOD_MS)) {
        init_timed_out = true;
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
		init_timeout_ms = 0;
		init_timed_out = false;
        gp.model = GP_MODEL_UNKNOWN;
	}
	previous_power_status = new_power_status;
}

void handle_rx_data(uint16_t *buf, uint16_t len)
{
    /*
     * Called when an i2c rx transaction has completed.
     *
     * Check if the data is formatted correctly and process accordingly.
     */

    // XXX: need better detection of h3+ vs h4 packets
    //      just favoring h4 for now
    if (gp_h4_rx_data_is_valid(buf, len)) {

        gp.model = GP_MODEL_HERO4; // TODO: reset gp.model when we are disconnected to GP_MODEL_UNKNOWN

        // XXX: avoid all this copying

        gp_h4_pkt_t pkt;
        gp_h4_pkt_t rsp;

        int i;
        for (i = 0; i < GP_COMMAND_RECEIVE_BUFFER_SIZE; ++i) {
            pkt.bytes[i] = buf[i];
        }

        if (gp_h4_handle_rx(&gp.h4, &pkt, &rsp)) {

            for (i = 0; i < rsp.rsp.len + 1; ++i) {
                txbuf[i] = rsp.bytes[i];
            }

            gp_assert_intr();

            gp_control_state = GP_CONTROL_STATE_WAIT_READY_TO_SEND_RESPONSE;
            timeout_counter = 0;
        }
        return;
    }

    bool from_camera;

    if (gp_h3p_rx_data_is_valid(rxbuf, len, &from_camera)) {

        gp.model = GP_MODEL_HERO3P;

        // Parse the retrieved data differently depending on whether it's a command or response
        if (from_camera) {
            gp_handle_command(buf);
        } else {
            gp_handle_response(buf);
        }
    } else {
        // error in data rx, return to idle
        new_response_available = true;
        gp_control_state = GP_CONTROL_STATE_IDLE;
    }
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

    switch (gp.model) {
        case GP_MODEL_HERO3P:
            gp_h3p_handle_command(cmdbuf, txbuf, &gccb_version_queried);
            break;
        case GP_MODEL_HERO4:
            gp_h4_handle_command(cmdbuf, txbuf);
            break;
        case GP_MODEL_UNKNOWN:
            return;
        default:
            return;
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

    switch (gp.model) {
        case GP_MODEL_HERO3P:
            gp_h3p_handle_response(respbuf, &last_cmd_response);
            break;
        case GP_MODEL_HERO4:
            gp_h4_handle_response(respbuf, &last_cmd_response);
            break;
        case GP_MODEL_UNKNOWN:
            return;
        default:
            return;
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
	gp_disable_hb_interface();

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

        i2c_begin_rx(rxbuf, RX_BUF_SZ);

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
