#include "f2806x_int8.h"
#include "gopro_interface.h"
#include "gopro_i2c.h"
#include "gopro_hero_common.h"
#include "gopro_hero3p.h"
#include "gopro_hero4.h"
#include "gopro_helpers.h"
#include "PeripheralHeaderIncludes.h"

#include <ctype.h>


#define GP_INIT_TIMEOUT_MS 10000    // Time allowed for GoPro to complete handshake before it is deemed incompatible, could probably be reduced to something lower but it should be at least > 3000, if not more (accurate time TBD)

// empirically discovered, we must delay at least GP_INTR_DELAY_US
// after an i2c cmd is received before asserting INTR, otherwise
// the gopro appears to miss the edge and fail to respond.
#define GP_INTR_DELAY_US    500

static void gp_timeout();

static void gp_detect_camera_model(const uint8_t *buf, uint16_t len);
static void gp_on_txn_complete();
static bool handle_rx_data(uint8_t *buf, uint16_t len);
static bool gp_begin_cmd_send(uint16_t len);
static bool gp_ready_for_cmd();
static bool gp_poll_camera_state();
static bool gp_handshake_complete();
static GOPRO_HEARTBEAT_STATUS gp_get_heartbeat_status();
static bool gp_is_valid_capture_mode(uint8_t mode);
static bool gp_is_recording();
static void gp_write_eeprom();

// Data to write into EEPROM
static const uint8_t EEPROMData[GP_I2C_EEPROM_NUMBYTES] = {0x0E, 0x03, 0x01, 0x12, 0x0E, 0x03, 0x01, 0x12, 0x0E, 0x03, 0x01, 0x12, 0x0E, 0x03, 0x01, 0x12};

// buffers for i2c transactions
struct pktbuf {
    union {
        gp_h3p_pkt_t h3p;
        gp_h4_pkt_t h4p;
        uint8_t bytes[MAX(sizeof(gp_h3p_pkt_t), sizeof(gp_h4_pkt_t))];
    };

    uint16_t len;
};

static struct pktbuf txbuf;
static struct pktbuf rxbuf;

// state associated with current i2c transaction
struct i2c_txn_t {
    bool in_progress;       // tx/rx is ongoing
    bool direction_is_tx;   // direction of transaction - rx or tx
};

typedef struct {
    bool enabled;

    struct i2c_txn_t i2c_txn;

    volatile herobus_txn_phase_t hb_txn_phase;

    gp_transaction_t txn;

    GPPowerStatus power_status;

    uint16_t handshake_timeout_count;
    uint16_t intr_timeout_count;        // how long has INTR been asserted without seeing a START
    bool intr_retry_pulse_in_progress;  // if our INTR request was ignored, are we in the middle of a retry pulse?
    bool intr_pending;                  // is an intr assertion pending?
    uint32_t i2c_stop_timestamp_us;     // last time we saw an i2c stop condition. related to GP_INTR_DELAY_US
    GPModel model;
    GOPRO_CAPTURE_MODE capture_mode;
    GOPRO_CAPTURE_MODE pending_capture_mode;
    uint16_t camera_poll_counter;       // periodically poll the camera for state changes, it doesn't send events
    bool recording;

    uint32_t gp_power_on_counter;
    volatile uint32_t timeout_counter;

    gp_h3p_t h3p;
    gp_h4_t h4;

} gopro_t;

// global gopro instance
static gopro_t gp = {
    .enabled = false,
    .i2c_txn.in_progress = false,

    // txn is not initialized

    .handshake_timeout_count = 0,
    .intr_timeout_count = 0,
    .intr_retry_pulse_in_progress = false,
    .i2c_stop_timestamp_us = 0,
    .intr_pending = false,
    .model = GP_MODEL_UNKNOWN,
    .power_status = GP_POWER_UNKNOWN,
    .capture_mode = GOPRO_CAPTURE_MODE_UNKNOWN,
    .pending_capture_mode = GOPRO_CAPTURE_MODE_UNKNOWN,
    .camera_poll_counter = GP_CAPTURE_MODE_POLLING_INTERVAL,     // has to be >= (GP_CAPTURE_MODE_POLLING_INTERVAL / GP_STATE_MACHINE_PERIOD_MS) to trigger immediate request for camera mode
    .recording = false,

    .gp_power_on_counter = 0,
    .timeout_counter = 0,

    .txn.response.mav.status = GP_CMD_STATUS_INCOMPLETE,
    .hb_txn_phase = HB_TXN_IDLE
};


void gp_init()
{
    gp.enabled = true;

    gp_write_eeprom();
    gp_reset();
    gp_set_pwron_asserted_out(false);

    // bacpac detect is enabled once we know the camera is powered on
}

void gp_disable(void)
{
    gp.enabled = false;
}

bool gp_enabled()
{
    return gp.enabled;
}


void gp_reset()
{
    gp.i2c_txn.in_progress = false;

    // txn is not initialized

    gp.handshake_timeout_count = 0;
    gp.intr_timeout_count = 0;
    gp.intr_retry_pulse_in_progress = false;
    gp.i2c_stop_timestamp_us = 0;
    gp.intr_pending = false;
    gp.model = GP_MODEL_UNKNOWN;
    gp.power_status = GP_POWER_UNKNOWN;
    gp.capture_mode = GOPRO_CAPTURE_MODE_UNKNOWN;
    gp.pending_capture_mode = GOPRO_CAPTURE_MODE_UNKNOWN;
    gp.camera_poll_counter = GP_CAPTURE_MODE_POLLING_INTERVAL;     // has to be >= (GP_CAPTURE_MODE_POLLING_INTERVAL / GP_STATE_MACHINE_PERIOD_MS) to trigger immediate request for camera mode
    gp.recording = false;

    gp.txn.response.mav.status = GP_CMD_STATUS_INCOMPLETE;
    gp.hb_txn_phase = HB_TXN_IDLE;

    gp_set_intr_asserted_out(false);
    gp_set_bp_detect_asserted_out(false);

    gp_h3p_init(&gp.h3p);
    gp_h4_init(&gp.h4);

    gopro_i2c_init();   // resets the i2c device
}

static void gp_pend_intr_assertion()
{
    /*
     * If it's been GP_INTR_DELAY_US since the last stop condition,
     * assert immediately, otherwise set the pending flag such that
     * intr is eventually asserted in gp_fast_update()
     *
     * See comments on GP_INTR_DELAY_US above.
     */

    if (micros() - gp.i2c_stop_timestamp_us > GP_INTR_DELAY_US) {
        gp_set_intr_asserted_out(true);
    } else {
        gp.intr_pending = true;
    }
}

static bool init_timed_out()
{
    /*
     * init timeout ensures we wait long enough after a possible
     * camera connection before we declare it either `connected` or `incompatible`.
     */

    return gp.handshake_timeout_count >= (GP_INIT_TIMEOUT_MS / GP_STATE_MACHINE_PERIOD_MS);
}

bool gp_ready_for_cmd()
{
    return (gp.hb_txn_phase == HB_TXN_IDLE) && !i2c_get_bb();
}

bool gp_begin_cmd_send(uint16_t len)
{
    /*
     * Begin transmitting the contents of txbuf.
     *
     * txbuf must already be filled, just the length is set here.
     *
     * Assert INTR, we'll begin writing bytes once we get addressed
     * by the camera.
     */

    if (gp.hb_txn_phase != HB_TXN_IDLE) {
        return false;
    }

    txbuf.len = len;

    // Assert the GoPro interrupt line, letting it know we'd like it to read a command from us
    gp_pend_intr_assertion();

    // Reset the timeout counter, and transition to waiting for the GoPro to start reading the command from us
    gp.timeout_counter = 0;
    gp.hb_txn_phase = HB_TXN_WAIT_FOR_CMD_START;

    return true;
}

GPRequestType gp_transaction_direction() {
    return gp.txn.reqtype;
}

void gp_set_transaction_result(const uint8_t *resp_bytes, uint16_t len, GPCmdStatus status)
{
    /*
     * Called from a gopro protocol handler to indicate
     * that there is data available to be passed back up
     * through the CAN interface.
     */

    memcpy(gp.txn.response.mav.value, resp_bytes, len);

    // not strictly necessary, but set remaining payload bytes to 0
    const size_t valsz = sizeof(gp.txn.response.mav.value);
    if (len < valsz) {
        memset(&gp.txn.response.mav.value[len], 0, valsz - len);
    }

    gp.txn.len = len;
    gp.txn.response.mav.status = (status == GP_CMD_STATUS_SUCCESS) ? GOPRO_REQUEST_SUCCESS : GOPRO_REQUEST_FAILED;

    // result transmitted via gp_send_mav_response()
}

static void gp_send_mav_response()
{
    /*
     * Called frequently on the main thread.
     * Send the results of a mavlink-bound transaction that has completed, if available.
     *
     * A mavlink response can only become available as the result of
     * a successfully completed i2c transaction, or an i2c timeout.
     */

    i2c_disable_scd_isr();  // critical section

    if (gp.txn.response.mav.status != GP_CMD_STATUS_INCOMPLETE) {
        if (!gp.txn.is_internal) {
            gp_send_mav_can_response(&gp.txn);
        }
        gp.txn.response.mav.status = GP_CMD_STATUS_INCOMPLETE;
    }

    i2c_enable_scd_isr();  // end critical section
}

uint16_t gp_transaction_cmd()
{
    /*
     * Get the current transaction command,
     * can be handy for interpreting the corresponding response.
     */

    return gp.txn.response.mav.cmd_id;
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

void gp_get_heartbeat(gp_can_mav_heartbeat_t *hb)
{
    /*
     * Populate the given heartbeat struct with
     * the current state.
     */

    hb->mav.hb_status = gp_get_heartbeat_status();
    hb->mav.capture_mode = gp_capture_mode();
    hb->mav.flags = 0;
    if (gp_is_recording()) {
        hb->mav.flags |= GOPRO_FLAG_RECORDING;
    }
}

GOPRO_HEARTBEAT_STATUS gp_get_heartbeat_status()
{
    // The HeroBus interface is not enabled
    if(!gp.enabled) {
        return GOPRO_HEARTBEAT_STATUS_DISCONNECTED;
    }

    // A GoPro is connected, ready for action and had queried the gccb version
    if (gp_get_power_status() == GP_POWER_ON && gp_handshake_complete() && !init_timed_out()) {
        return GOPRO_HEARTBEAT_STATUS_CONNECTED;
	}

	// A GoPro is not in a "connected" state, but we can see something is plugged in
	if (gp_get_power_status() != GP_POWER_ON && gp_get_von_asserted_in() && init_timed_out()) {
        return GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE;
	}

	// A GoPro is connected, but it's not talking to us, or we're not talking to it
	if (gp_get_power_status() == GP_POWER_ON && !gp_handshake_complete() && init_timed_out()) {
        return GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE;
	}

	// Either a GoPro is no connected, or there is no electrical way of detecting it
    return GOPRO_HEARTBEAT_STATUS_DISCONNECTED;
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
        gp.gp_power_on_counter = 0;
        return true;
    }

    return false;
}

bool gp_poll_camera_state()
{
    /*
     * Called from the update ticker.
     *
     * If a user manually changes the camera mode (or any other setting),
     * the camera does not notify us, so we periodically poll for that state.
     *
     * On hero 3, we can grab all state in a single msg,
     * on hero 4 we just get camera mode for now.
     */

    if (gp_ready_for_cmd() && !gp_is_recording()) {
        gp_can_mav_get_req_t req;
        if (gp.model == GP_MODEL_HERO3P) {
            req.mav.cmd_id = GP_H3P_COMMAND_ENTIRE_CAM_STATUS;
        } else {
            req.mav.cmd_id = GOPRO_COMMAND_CAPTURE_MODE;
        }
        gp_get_request(&req, true); // internal txn for our own consumption
        return true;
    }

    return false;
}

int gp_get_request(const gp_can_mav_get_req_t *req, bool txn_is_internal)
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
    gp.txn.response.mav.cmd_id = (GOPRO_COMMAND)req->mav.cmd_id;
    gp.txn.is_internal = txn_is_internal;

    switch (gp.model) {
    case GP_MODEL_HERO3P:
        if (gp_h3p_produce_get_request(req->mav.cmd_id, &txbuf.h3p.cmd)) {
            gp_begin_cmd_send(txbuf.h3p.cmd.len + 1);
        }
        break;

    case GP_MODEL_HERO4:
        if (gp_h4_produce_get_request(&gp.h4, req->mav.cmd_id, &txbuf.h4p)) {
            gp_begin_cmd_send(txbuf.h4p.cmd.len + 1);
        }
        break;

    default:
        return -1;
    }

    return 0;
}

int gp_set_request(const gp_can_mav_set_req_t* req)
{
    /*
     * Called when a CAN msg has been received with a 'gopro set request' msg type.
     *
     * Forward the requested set cmd,
     * and assume that the CAN layer will pick up the response
     * via gp_get_last_set_response()
     */

    if (!(gp_get_power_status() == GP_POWER_ON || (req->mav.cmd_id == GOPRO_COMMAND_POWER && req->mav.value[0] == 0x01)) ||
        !gp_ready_for_cmd())
    {
        gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
        return -1;
    }

    gp.txn.reqtype = GP_REQUEST_SET;
    gp.txn.response.mav.cmd_id = (GOPRO_COMMAND)req->mav.cmd_id;
    gp.txn.is_internal = false;     // parameterize if we ever need to send an internal 'SET' command

	// GoPro has to be powered on and ready, or the command has to be a power on command
    switch (gp.model) {
    case GP_MODEL_HERO3P:
        if (gp_h3p_produce_set_request(&gp.h3p, req, &txbuf.h3p.cmd)) {
            gp_begin_cmd_send(txbuf.h3p.cmd.len + 1);
        }
        break;

    case GP_MODEL_HERO4:
        if (gp_h4_produce_set_request(&gp.h4, req, &txbuf.h4p)) {
            gp_begin_cmd_send(txbuf.h4p.cmd.len + 1);
        }
        break;

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

static void gp_check_intr_timeout()
{
    /*
     * GoPro datasheet says that if INTR is asserted for 2 seconds
     * and it doesn't respond to us, just try again :/
     *
     * Trying again consists of deasserting INTR and reasserting
     * after a short pulse duration.
     *
     * What happens if we're ignored again? Just keep trying.
     *
     * Called from within the state machine ticker.
     */

    const unsigned INTR_TIMEOUT_MILLIS = 2000;
    const unsigned INTR_RETRY_PULSE_MILLIS = 1;

    // pulsing to retry?
    if (gp.intr_retry_pulse_in_progress) {
        if (++gp.intr_timeout_count >= (INTR_RETRY_PULSE_MILLIS/GP_STATE_MACHINE_PERIOD_MS)) {
            gp_pend_intr_assertion();
            gp.intr_retry_pulse_in_progress = false;
            gp.intr_timeout_count = 0;
        }
        return;
    }

    // check for timeout while asserted
    if (gp_get_intr_asserted_out()) {
        if (++gp.intr_timeout_count >= (INTR_TIMEOUT_MILLIS/GP_STATE_MACHINE_PERIOD_MS)) {
            gp_set_intr_asserted_out(false);
            gp.intr_retry_pulse_in_progress = true;
            gp.intr_timeout_count = 0;
        }
    } else {
        // we rely on INTR being deasserted once we're addressed via i2c
        gp.intr_timeout_count = 0;
    }

}

void gp_fast_update()
{
    /*
     * Called directly from the main loop (ie, not via the scheduler).
     */

    if(!gp.enabled) {
        return;
    }

    gp_send_mav_response();

    // handle pending intr requests
    if (gp.intr_pending && (micros() - gp.i2c_stop_timestamp_us > GP_INTR_DELAY_US)) {
        gp_set_intr_asserted_out(true);
        gp.intr_pending = false;
    }
}

// It's expected that this function is repeatedly called every period as configured in the header (currently 3ms)
// for proper gopro interface operation
void gp_update()
{
    GPPowerStatus new_power_status;

    if(!gp.enabled) {
        return;
    }

    gp_check_intr_timeout();

    if (gp.i2c_txn.in_progress) {
        if (gp.timeout_counter++ > (GP_TIMEOUT_MS / GP_STATE_MACHINE_PERIOD_MS)) {
            gp_timeout();
        }
    }

    if (gp_get_pwron_asserted_out()) {
        if (gp.gp_power_on_counter++ > (GP_PWRON_TIME_MS / GP_STATE_MACHINE_PERIOD_MS)) {
            gp_set_pwron_asserted_out(false);
        }
    }

    if (gp_get_power_status() == GP_POWER_ON) {
        // Set 'init_timed_out' to true after GP_INIT_TIMEOUT_MS to avoid glitching
        // the heartbeat with an incompatible state while it's gccb version is being queried
        if(!gp_handshake_complete() && !init_timed_out()) {
            gp.handshake_timeout_count++;

            if (init_timed_out()) {
                // camera is incompatible,
                // try to avoid freezing it by disabling bacpac detect
                gp_set_bp_detect_asserted_out(false);
            }
        }

        // request an update on GoPro's current capture mode, infrequently to avoid freezing the GoPro
        if (gp.camera_poll_counter++ >= (GP_CAPTURE_MODE_POLLING_INTERVAL / GP_STATE_MACHINE_PERIOD_MS)) {
            gp.camera_poll_counter = 0;

            gp_poll_camera_state();
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
        if (gp_h3p_on_transaction_complete(&gp.h3p, &txbuf.h3p)) {
            gp_begin_cmd_send(txbuf.h3p.cmd.len + 1);
        }
        break;

    case GP_MODEL_HERO4:
        if (gp_h4_on_txn_complete(&gp.h4, &txbuf.h4p)) {
            gp_begin_cmd_send(txbuf.h4p.cmd.len + 1);
        }
        break;

    default:
        break;
    }
}

void gp_detect_camera_model(const uint8_t *buf, uint16_t len)
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

bool handle_rx_data(uint8_t *buf, uint16_t len)
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
    case GP_MODEL_HERO3P: {
        bool from_camera;
        if (gp_h3p_rx_data_is_valid(buf, len, &from_camera)) {
            if (gp_h3p_handle_rx(&gp.h3p, &rxbuf.h3p, from_camera, &txbuf.h3p.rsp)) {
                txbuf.len = txbuf.h3p.rsp.len + 1;
                return true;
            }
        }
        break;
    }
    case GP_MODEL_HERO4:
        if (gp_h4_rx_data_is_valid(buf, len)) {

            txbuf.h4p.rsp.len = 0;

            if (gp_h4_handle_rx(&gp.h4, &rxbuf.h4p, &txbuf.h4p) == GP_H4_ERR_OK) {
                if (txbuf.h4p.rsp.len > 0) {
                    txbuf.len = txbuf.h4p.rsp.len + 1;
                    return true;
                }
            } else {
                // if we get a error response during handshake, reset and start over
                // otherwise, report the transaction failure via mavlink
                if (!gp_h4_handshake_complete(&gp.h4)) {
                    gp_reset();
                } else {
                    gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
                    gp.hb_txn_phase = HB_TXN_IDLE;
                }
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

    gp.timeout_counter = 0;
    gp.i2c_txn.in_progress = true;
    gp.i2c_txn.direction_is_tx = addressed_as_tx;

    if (addressed_as_tx) {
        // send data as appropriate
        // length is 1st byte in command buffer, add 1 for length field

        switch (gp.hb_txn_phase) {
        case HB_TXN_WAIT_FOR_CMD_START:
            gopro_i2c_send(txbuf.bytes, txbuf.len);
            gp.hb_txn_phase =  HB_TXN_TXING_CMD;
            break;

        case HB_TXN_WAIT_FOR_RSP_START:
            gopro_i2c_send(txbuf.bytes, txbuf.len);
            gp.hb_txn_phase =  HB_TXN_TXING_RSP;
            break;

        default:
            // error, return to idle
            gp.hb_txn_phase = HB_TXN_IDLE;
            // NAK i2c
            break;
        }

    } else {
        // addressed as receiver.
        switch (gp.hb_txn_phase) {
        case HB_TXN_WAIT_FOR_CMD_START:
            // Special case - we have asked the GoPro to read a command from us, but before it has started to read the command,
            // it issues a command to us first.  Per the spec, we have to give up on our command request and service the GoPro's command

            // Indicate that the command we were trying to send has been preempted by the GoPro
            // Indicate that a "new response" is available (what's available is the indication that the command was preempted)
            gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
            // fall through, and start rxing anyway

        case HB_TXN_IDLE:
        case HB_TXN_WAIT_FOR_GP_RSP:
            i2c_begin_rx(rxbuf.bytes, sizeof rxbuf.bytes);
            gp.hb_txn_phase = HB_TXN_RXING;
            break;

        default:
            // error, return to idle
            gp.hb_txn_phase = HB_TXN_IDLE;
            // NAK i2c
            break;
        }
    }
}

void gp_on_i2c_stop_condition()
{
    /*
     * Called in ISR context when the i2c device detects a STOP condition.
     *
     * Ensure we're in a reasonable state, and continue processing
     * this herobus transaction.
     */

    gp.i2c_stop_timestamp_us = micros();

    if (!gp.i2c_txn.in_progress) {
        // error, unexpected completion event
        return;
    }

    gp.i2c_txn.in_progress = false;
    gp.timeout_counter = 0;

    if (gp.i2c_txn.direction_is_tx) {
        switch (gp.hb_txn_phase) {
        case HB_TXN_TXING_CMD:
            // finished sending cmd, gopro should now respond to us
            gp.hb_txn_phase = HB_TXN_WAIT_FOR_GP_RSP;
            break;

        case HB_TXN_TXING_RSP:
            // sent response, we're all done
            gp.hb_txn_phase = HB_TXN_IDLE;
            gp_on_txn_complete();
            break;

        default:
            // error, return to idle
            gp.hb_txn_phase = HB_TXN_IDLE;
            break;
        }
    } else {
        // finished rxing
        switch (gp.hb_txn_phase) {
        case HB_TXN_RXING:
            rxbuf.len = i2c_get_rx_len();
            if (handle_rx_data(rxbuf.bytes, rxbuf.len)) {
                // if we have data to send, send it over I2C
                gp_pend_intr_assertion();
                gp.hb_txn_phase = HB_TXN_WAIT_FOR_RSP_START;
            } else {
                gp.hb_txn_phase = HB_TXN_IDLE;
                gp_on_txn_complete();
            }
            break;

        default:
            // error, return to idle
            gp.hb_txn_phase = HB_TXN_IDLE;
            break;
        }
    }
}

void gp_timeout()
{
    gopro_i2c_init();   // resets the i2c device

    gp.i2c_txn.in_progress = false;

    gp.timeout_counter = 0; // Reset the timeout counter so it doesn't have an old value in it the next time we want to use it
    gp_set_intr_asserted_out(false); // De-assert the interrupt request (even if it wasn't previously asserted, in idle the interrupt request should always be deasserted)

    i2c_disable_scd_isr();  // critical section
    // default case is for transaction to be completed in ISR ctx,
    // timeout gets processed on main thread, so need to ensure
    // exclusive access to the txn result
    gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
    i2c_enable_scd_isr();  // critical section

    gp.hb_txn_phase = HB_TXN_IDLE;
}

GOPRO_CAPTURE_MODE gp_capture_mode()
{
    return gp.capture_mode;
}

bool gp_is_valid_capture_mode(uint8_t mode) {
    switch (mode) {
    case GOPRO_CAPTURE_MODE_VIDEO:
    case GOPRO_CAPTURE_MODE_PHOTO:
    case GOPRO_CAPTURE_MODE_BURST:
    case GOPRO_CAPTURE_MODE_TIME_LAPSE:
    case GOPRO_CAPTURE_MODE_MULTI_SHOT:
    case GOPRO_CAPTURE_MODE_PLAYBACK:
    case GOPRO_CAPTURE_MODE_SETUP:
        return true;

    default:
        return false;
    }
}

void gp_latch_pending_capture_mode()
{
    if (gp.pending_capture_mode != GOPRO_CAPTURE_MODE_UNKNOWN) {
        gp_set_capture_mode(gp.pending_capture_mode);
    }
}

bool gp_pend_capture_mode(uint8_t capture_mode)
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
        gp.pending_capture_mode = (GOPRO_CAPTURE_MODE)capture_mode;
        return true;
    }

    return false;
}

bool gp_set_capture_mode(uint8_t capture_mode)
{
    if (gp_is_valid_capture_mode(capture_mode)) {
        gp.capture_mode = (GOPRO_CAPTURE_MODE)capture_mode;
        gp.pending_capture_mode = GOPRO_CAPTURE_MODE_UNKNOWN;
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

