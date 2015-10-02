#include "f2806x_int8.h"
#include "gopro_interface.h"
#include "gopro_i2c.h"
#include "gopro_hero_common.h"
#include "gopro_hero3p.h"
#include "gopro_hero4.h"
#include "gopro_helpers.h"
#include "PeripheralHeaderIncludes.h"

// Include for GOPRO_COMMAND enum
#include "mavlink_interface/mavlink_gimbal_interface.h"

#include <ctype.h>


#define GP_INIT_TIMEOUT_MS 10000    // Time allowed for GoPro to complete handshake before it is deemed incompatible, could probably be reduced to something lower but it should be at least > 3000, if not more (accurate time TBD)

static void gp_reset();
static void gp_timeout();

static void gp_detect_camera_model(const uint16_t *buf, uint16_t len);
static void gp_on_txn_complete();
static bool handle_rx_data(uint16_t *buf, uint16_t len);
static bool gp_send_cmd(const uint16_t* cmd, uint16_t len);
static bool gp_ready_for_cmd();
static bool gp_request_capture_mode();
static bool gp_handshake_complete();
static bool gp_is_valid_capture_mode(Uint8 capture_mode);
static bool gp_is_recording();
static void gp_write_eeprom();


Uint32 gp_power_on_counter = 0;
volatile Uint32 timeout_counter = 0;

#define TX_BUF_SZ  128
#define RX_BUF_SZ  128

static uint16_t txbuf[TX_BUF_SZ];
static uint16_t rxbuf[RX_BUF_SZ];

typedef struct {
    bool waiting_for_i2c; // waiting for i2c either tx/rx

    volatile herobus_txn_phase_t hb_txn_phase;

    gp_transaction_t txn;

    GPPowerStatus power_status;

    uint16_t init_timeout_ms;
    GPModel model;
    GPCaptureMode capture_mode;
    GPCaptureMode pending_capture_mode;
    uint16_t capture_mode_polling_counter;
    bool recording;

    gp_h3p_t h3p;
    gp_h4_t h4;

} gopro_t;

// global gopro instance
static gopro_t gp;


void gp_init()
{
    gp_write_eeprom();
    gp_reset();
    gp_set_pwron_asserted_out(false);
    gp.power_status = GP_POWER_UNKNOWN;

    gopro_i2c_init();

    // bacpac detect is enabled once we know the camera is powered on
}

void gp_reset()
{
    gp.waiting_for_i2c = false;

    // txn is not initialized

    gp.init_timeout_ms = 0;
    gp.model = GP_MODEL_UNKNOWN;
    gp.capture_mode = GP_CAPTURE_MODE_UNKNOWN;
    gp.pending_capture_mode = GP_CAPTURE_MODE_UNKNOWN;
    gp.capture_mode_polling_counter = GP_CAPTURE_MODE_POLLING_INTERVAL;     // has to be >= (GP_CAPTURE_MODE_POLLING_INTERVAL / GP_STATE_MACHINE_PERIOD_MS) to trigger immediate request for camera mode
    gp.recording = false;

    gp.hb_txn_phase = HB_TXN_IDLE;

    gp_set_intr_asserted_out(false);

    gp_h3p_init(&gp.h3p);
    gp_h4_init(&gp.h4);
}

static bool init_timed_out()
{
    /*
     * init timeout ensures we wait long enough after a possible
     * camera connection before we declare it either `connected` or `incompatible`.
     */

    return gp.init_timeout_ms >= (GP_INIT_TIMEOUT_MS / GP_STATE_MACHINE_PERIOD_MS);
}

bool gp_ready_for_cmd()
{
    return (gp.hb_txn_phase == HB_TXN_IDLE) && !i2c_get_bb();
}

bool gp_send_cmd(const uint16_t* cmd, uint16_t len)
{
    /*
     * Called by protocol modules (h4, h3p, etc)
     * to send a command to the camera.
     */
    uint16_t i;

    if (gp.hb_txn_phase != HB_TXN_IDLE) {
        return false;
    }

    for (i = 0; i < len; ++i) {
        txbuf[i] = cmd[i];
    }

    // Assert the GoPro interrupt line, letting it know we'd like it to read a command from us
    gp_set_intr_asserted_out(true);

    // Reset the timeout counter, and transition to waiting for the GoPro to start reading the command from us
    timeout_counter = 0;
    gp.hb_txn_phase = HB_TXN_WAIT_FOR_CMD_START;

    return true;
}

GPRequestType gp_transaction_direction() {
    return gp.txn.reqtype;
}

void gp_set_transaction_result(const uint16_t *resp_bytes, uint16_t len, GPCmdStatus status)
{
    /*
     * Called from a gopro protocol handler to indicate
     * that there is data available to be passed back up
     * through the CAN interface.
     */

    uint16_t i;
    for (i = 0; i < len; ++i) {
        gp.txn.payload[i] = resp_bytes[i];
    }

    // XXX: probably want to:
    //      - unify the values of GPCmdStatus and GOPRO_SET_RESPONSE_RESULT, currently they're opposites
    //      - make GOPRO_SET_RESPONSE_RESULT more general (GOPRO_RESPONSE_RESULT or similar)
    gp.txn.status = (status == GP_CMD_STATUS_SUCCESS) ? GOPRO_SET_RESPONSE_RESULT_SUCCESS : GOPRO_SET_RESPONSE_RESULT_FAILURE;
    gp.txn.len = len;

    if (!gp.txn.is_internal) {
        if (gp.txn.reqtype == GP_REQUEST_GET) {
            // TODO: this does not handle the case of failed transactions - could reply with corrupted data?
            gp_send_mav_get_response(gp.txn.mav_cmd, gp.txn.payload[0]);
        } else {
            gp_send_mav_set_response(gp.txn.mav_cmd, gp.txn.status);
        }
    }
}

uint16_t gp_transaction_cmd()
{
    /*
     * Get the current transaction command,
     * can be handy for interpreting the corresponding response.
     */

    return gp.txn.mav_cmd;
}

bool gp_handshake_complete()
{
    switch (gp.model) {
    case GP_MODEL_HERO3P:
        return gp_h3p_handshake_complete(&gp.h3p);

    case GP_MODEL_HERO4:
        return gp_h4_handshake_complete(&gp.h4);

    default:
        return false;
    }
}

GPHeartbeatStatus gp_get_heartbeat_status()
{
    // A GoPro is connected, ready for action and had queried the gccb version
    if (gp_get_power_status() == GP_POWER_ON && gp_handshake_complete() && !init_timed_out()) {
	    // The connected state is overloaded with the recording state
        if (gp_is_recording()) {
            return GP_HEARTBEAT_RECORDING;
        } else {
            return GP_HEARTBEAT_CONNECTED;
        }
	}

	// A GoPro is not in a "connected" state, but we can see something is plugged in
	if (gp_get_power_status() != GP_POWER_ON && gp_get_von_asserted_in() && init_timed_out()) {
	    return GP_HEARTBEAT_INCOMPATIBLE;
	}

	// A GoPro is connected, but it's not talking to us, or we're not talking to it
	if (gp_get_power_status() == GP_POWER_ON && !gp_handshake_complete() && init_timed_out()) {
	    return GP_HEARTBEAT_INCOMPATIBLE;
	}

	// Either a GoPro is no connected, or there is no electrical way of detecting it
	return GP_HEARTBEAT_DISCONNECTED;
}

bool gp_request_power_on()
{
    /*
     * From the HERO3 docs:
     *
     * "PWRON is an active-low signal and is pulled-up internally on the camera to 3.0V.
     *  Drive this pin using an open-collector driver.
     *  Apply an active-low pulse of 100ms to turn on the camera"
     *
     * This does not appear to power on hero4 devices. TBD.
     */

    if (!gp_get_pwron_asserted_out()) {
        gp_set_pwron_asserted_out(true);
        gp_power_on_counter = 0;
        return true;
    }

    return false;
}

bool gp_request_capture_mode()
{
    if (gp_ready_for_cmd() && !gp_is_recording()) {
        gp_get_request(GOPRO_COMMAND_CAPTURE_MODE, false);                  // not internal since currently there is no other method to notify when the capture mode changes, besides addressing a request
        return true;
    }

    return false;
}

int gp_get_request(Uint8 cmd_id, bool txn_is_internal)
{
    /*
     * Called when a CAN msg has been received with a `gopro get request` msg type
     * or an internal transaction is performed.
     *
     * Otherwise, fire off the transaction for the requested cmd,
     * and assume that the CAN layer will pick up the response
     * via gp_get_last_get_response(), assuming txn_is_internal is false.
     *
     * If tx_is_internal is set to true, CAN layer will not pick up the response.
     *
     */

    if ((gp_get_power_status() != GP_POWER_ON) || !gp_ready_for_cmd()) {
        gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
        return -1;
    }

    gp.txn.reqtype = GP_REQUEST_GET;
    gp.txn.mav_cmd = (GOPRO_COMMAND)cmd_id;
    gp.txn.is_internal = txn_is_internal;

    switch (gp.model) {
    case GP_MODEL_HERO3P: {
        gp_h3p_pkt_t h3p;
        if (gp_h3p_produce_get_request(cmd_id, &h3p.cmd)) {
            gp_send_cmd(h3p.bytes, h3p.cmd.len + 1);
        }
    } break;

    case GP_MODEL_HERO4: {
        gp_h4_pkt_t h4p;
        if (gp_h4_produce_get_request(&gp.h4, cmd_id, &h4p)) {
            gp_send_cmd(h4p.bytes, h4p.cmd.len + 1);
        }
    } break;

    default:
        return -1;
    }

    return 0;
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

    if (!(gp_get_power_status() == GP_POWER_ON || (request->cmd_id == GOPRO_COMMAND_POWER && request->value == 0x01)) ||
        !gp_ready_for_cmd())
    {
        gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
        return -1;
    }

    gp.txn.reqtype = GP_REQUEST_SET;
    gp.txn.mav_cmd = (GOPRO_COMMAND)request->cmd_id;
    gp.txn.is_internal = false;     // parameterize if we ever need to send an internal 'SET' command

	// GoPro has to be powered on and ready, or the command has to be a power on command
    switch (gp.model) {
    case GP_MODEL_HERO3P: {
        gp_h3p_pkt_t h3p;
        if (gp_h3p_produce_set_request(request, &h3p.cmd)) {
            gp_send_cmd(h3p.bytes, h3p.cmd.len + 1);
        }
    } break;

    case GP_MODEL_HERO4: {
        gp_h4_pkt_t h4p;
        if (gp_h4_produce_set_request(&gp.h4, request, &h4p)) {
            gp_send_cmd(h4p.bytes, h4p.cmd.len + 1);
        }
    } break;

    default:
        return -1;
    }

    return 0;
}

GPPowerStatus gp_get_power_status()
{
    // If we've either requested the power be turned on, or are waiting for the power on timeout to elapse, return
    // GP_POWER_WAIT so the user knows they should query the power status again at a later time
    // Otherwise, poll the gopro voltage on pin to get the current gopro power status
    if (gp_get_pwron_asserted_out()) {
        return GP_POWER_WAIT;
    }

    if (gp_get_von_asserted_in()) {
        return GP_POWER_ON;
    } else {
        return GP_POWER_OFF;
    }
}

void gp_enable_charging()
{
    gp_set_charging_asserted_out(true);
}

void gp_disable_charging()
{
    gp_set_charging_asserted_out(false);
}

// It's expected that this function is repeatedly called every period as configured in the header (currently 3ms)
// for proper gopro interface operation
void gp_update()
{
    GPPowerStatus new_power_status;

    if (gp.waiting_for_i2c) {
        if (gopro_i2c_in_progress()) {
            if (timeout_counter++ > (GP_TIMEOUT_MS / GP_STATE_MACHINE_PERIOD_MS)) {
                gp_timeout();
            }
        } else {
            gp.waiting_for_i2c = false;

            if (gp.hb_txn_phase == HB_TXN_RXING) {
                // transaction was rx
                if (handle_rx_data(rxbuf, i2c_get_rx_len())) {
                    // if we have data to send, send it over I2C
                    gp_set_intr_asserted_out(true);

                    gp.hb_txn_phase = HB_TXN_WAIT_FOR_RSP_START;
                    timeout_counter = 0;
                } else {
                    gp_on_txn_complete();
                }

            } else {
                // transaction was tx
                if (gp.hb_txn_phase == HB_TXN_TXING_CMD) {
                    gp.hb_txn_phase =  HB_TXN_WAIT_FOR_GP_RSP;
                } else {
                    // sent response, we're all done
                    gp.hb_txn_phase = HB_TXN_IDLE;

                    gp_on_txn_complete();
                }
            }
        }
    }

    if (gp_get_pwron_asserted_out()) {
        if (gp_power_on_counter++ > (GP_PWRON_TIME_MS / GP_STATE_MACHINE_PERIOD_MS)) {
            gp_set_pwron_asserted_out(false);
        }
    }

    if (gp_get_power_status() == GP_POWER_ON) {
        // Set 'init_timed_out' to true after GP_INIT_TIMEOUT_MS to avoid glitching
        // the heartbeat with an incompatible state while it's gccb version is being queried
        if(!gp_handshake_complete() && !init_timed_out()) {
            gp.init_timeout_ms++;

            if (init_timed_out()) {
                // camera is incompatible,
                // try to avoid freezing it by disabling bacpac detect
                gp_set_bp_detect_asserted_out(false);
            }
        }

        // request an update on GoPro's current capture mode, infrequently to avoid freezing the GoPro
        if (gp.capture_mode_polling_counter++ >= (GP_CAPTURE_MODE_POLLING_INTERVAL / GP_STATE_MACHINE_PERIOD_MS)) {
            gp.capture_mode_polling_counter = 0;

            gp_request_capture_mode();
        }
    }

    // Detect a change in power status to reset some flags when a GoPro is re-connected during operation
    new_power_status = gp_get_power_status();
    if (gp.power_status != new_power_status) {
        gp_reset();
        gp.power_status = new_power_status;

        if (gp.power_status == GP_POWER_ON) {
            // camera is up and running, we can let it know we're here
            gp_set_bp_detect_asserted_out(true);
        } else {
            // keep bacpac detect disabled by default
            gp_set_bp_detect_asserted_out(false);
        }
    }
}

void gp_on_txn_complete()
{
    /*
     * Called after a HeroBus request/response transaction
     * has been completed.
     *
     */

    switch (gp.model) {
    case GP_MODEL_HERO3P:
        // do nothing
        break;
    case GP_MODEL_HERO4: {
        gp_h4_pkt_t p;
        if (gp_h4_on_txn_complete(&gp.h4, &p)) {
            gp_send_cmd(p.bytes, p.cmd.len + 1);
        }
    } break;

    case GP_MODEL_UNKNOWN:
    default:
        break;
    }
}

void gp_detect_camera_model(const uint16_t *buf, uint16_t len)
{
    /*
     * Called when we've received an i2c packet but we don't yet
     * know which kind of camera we're talking to.
     *
     * Check if any of our protocol handlers recognize it
     */

    if (gp_h4_recognize_packet(buf, len)) {
        gp.model = GP_MODEL_HERO4;
    } else if (gp_h3p_recognize_packet(buf, len)) {
        gp.model = GP_MODEL_HERO3P;
    }
}

bool handle_rx_data(uint16_t *buf, uint16_t len)
{
    /*
     * Called when an i2c rx transaction has completed.
     *
     * Check if the data is formatted correctly and process accordingly.
     */

    if (gp.model == GP_MODEL_UNKNOWN) {
        gp_detect_camera_model(buf, len);
    }

    // base case, we have received a response and we're back to idle,
    // handlers can override if they have a response to send
    gp.hb_txn_phase = HB_TXN_IDLE;

    switch (gp.model) {
    case GP_MODEL_HERO3P:{
        bool from_camera;
        if (gp_h3p_rx_data_is_valid(buf, len, &from_camera)) {
            if (gp_h3p_handle_rx(&gp.h3p, buf, from_camera, txbuf)) {
                return true;
            }
        }
        break;
    }
    case GP_MODEL_HERO4:
        if (gp_h4_rx_data_is_valid(buf, len)) {
            // XXX: avoid all this copying

            gp_h4_pkt_t pkt;
            gp_h4_pkt_t rsp;

            uint16_t i;
            for (i = 0; i < GP_COMMAND_RECEIVE_BUFFER_SIZE; ++i) {
                pkt.bytes[i] = buf[i];
            }

            if (gp_h4_handle_rx(&gp.h4, &pkt, &rsp)) {

                for (i = 0; i < rsp.rsp.len + 1; ++i) {
                    txbuf[i] = rsp.bytes[i];
                }

                return true;
            }
        }
        break;

    default:
        // error in data rx, return to idle
        gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
        gp.hb_txn_phase = HB_TXN_IDLE;
        break;
    }

    return false;
}

static void gp_write_eeprom()
{
    // This function writes to the 24LC00 EEPROM which the GoPro reads from
    // as specified in the hero bus datasheet
    // Data to write into EEPROM
    uint8_t EEPROMData[GP_I2C_EEPROM_NUMBYTES] = {0x0E, 0x03, 0x01, 0x12, 0x0E, 0x03, 0x01, 0x12, 0x0E, 0x03, 0x01, 0x12, 0x0E, 0x03, 0x01, 0x12};
    uint8_t i;

    // if gopro is on, don't write EEPROM because this may crash the gopro
    if (gp_get_von_asserted_in()) {
		return;
    }

	// Disable the HeroBus port (GoPro should stop mastering the I2C bus)
	gp_set_bp_detect_asserted_out(false);

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
     */

    gp_set_intr_asserted_out(false);

    timeout_counter = 0;
    gp.waiting_for_i2c = true;

    if (addressed_as_tx) {
        // send data as appropriate
        // length is 1st byte in command buffer, add 1 for length field

        gopro_i2c_send(txbuf, txbuf[0] + 1);

        // keep track of whether we're sending a cmd or response.
        // if sending cmd, need to know whether to wait for a response.
        if (gp.hb_txn_phase == HB_TXN_WAIT_FOR_CMD_START) {
            gp.hb_txn_phase =  HB_TXN_TXING_CMD;
        } else {
            gp.hb_txn_phase = HB_TXN_TXING_RSP;
        }

    } else {
        // addressed as receiver.
        // in the general case, we wait for the stop condition to indicate a transaction is complete.

        i2c_begin_rx(rxbuf, RX_BUF_SZ);

        if (gp.hb_txn_phase == HB_TXN_WAIT_FOR_CMD_START) {
            // Special case - we have asked the GoPro to read a command from us, but before it has started to read the command,
            // it issues a command to us first.  Per the spec, we have to give up on our command request and service the GoPro's command

            // Indicate that the command we were trying to send has been preempted by the GoPro
            // Indicate that a "new response" is available (what's available is the indication that the command was preempted)
            gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
//            last_cmd_response.result = GP_CMD_PREEMPTED;
        }

        // wait for the rx to complete
        gp.hb_txn_phase = HB_TXN_RXING;
    }
}

void gp_timeout()
{
    gopro_i2c_on_timeout();
    gp.waiting_for_i2c = false;

    timeout_counter = 0; // Reset the timeout counter so it doesn't have an old value in it the next time we want to use it
    gp_set_intr_asserted_out(false); // De-assert the interrupt request (even if it wasn't previously asserted, in idle the interrupt request should always be deasserted)

    // Indicate that a "new response" is available (what's available is the indication that we timed out)
    //last_cmd_response.cmd_result = reason;
    //new_response_available = true;

    gp.hb_txn_phase = HB_TXN_IDLE;
}

GPCaptureMode gp_capture_mode()
{
    return gp.capture_mode;
}

bool gp_is_valid_capture_mode(Uint8 capture_mode) {
    return (capture_mode == GP_CAPTURE_MODE_VIDEO || capture_mode == GP_CAPTURE_MODE_PHOTO || capture_mode == GP_CAPTURE_MODE_BURST);
}

void gp_latch_pending_capture_mode()
{
    if (gp.pending_capture_mode != GP_CAPTURE_MODE_UNKNOWN) {
        gp_set_capture_mode(gp.pending_capture_mode);
    }
}

bool gp_pend_capture_mode(Uint8 capture_mode)
{
    /*
     * Called when a 'SET capture mode' cmd, which contains the new capture
     * mode, is received from the CAN/MAVLink layer. The new capture mode is
     * then pended until we have confirmation that the I2C 'SET capture mode'
     * transaction has completed successfully, at which point the new
     * capture mode state will be set.
     *
     * The process described above is implemented because the response to the
     * 'SET' I2C cmd does not contain information about the new capture mode.
     *
     */

    if (gp_is_valid_capture_mode(capture_mode)) {
        gp.pending_capture_mode = (GPCaptureMode)capture_mode;
        return true;
    }

    return false;
}

bool gp_set_capture_mode(Uint8 capture_mode)
{
    if (gp_is_valid_capture_mode(capture_mode)) {
        gp.capture_mode = (GPCaptureMode)capture_mode;
        gp.pending_capture_mode = GP_CAPTURE_MODE_UNKNOWN;
        return true;
    }

    return false;
}

void gp_set_recording_state(bool recording_state)
{
    gp.recording = recording_state;
}

bool gp_is_recording()
{
    return gp.recording;
}

