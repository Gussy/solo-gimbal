#ifndef GOPRO_INTERFACE_H_
#define GOPRO_INTERFACE_H_

#include "hardware/i2c.h"
#include "gopro/gopro_hero_common.h"
#include "PM_Sensorless-Settings.h"

#include <stdbool.h>

#define GP_COMMAND_REQUEST_SIZE 4
#define GP_COMMAND_RESPONSE_SIZE 3
#define GP_COMMAND_RECEIVE_BUFFER_SIZE 40
#define GP_STATE_MACHINE_PERIOD_MS 3
#define GP_PWRON_TIME_MS 120 // Spec says 100ms, but I'm making it a little longer here just in case, and so it's an even multiple of our state machine poll period
#define GP_TIMEOUT_MS 2000 // If at any point we're waiting in the state machine (except at idle) for longer than this timeout, return to idle.  This timeout is 2s per HeroBus spec
#define GP_PROTOCOL_VERSION 0x00
#define GP_MAVLINK_HEARTBEAT_INTERVAL 1000
#define GP_CAPTURE_MODE_POLLING_INTERVAL 1000
#define GP_I2C_EEPROM_NUMBYTES 16


typedef enum {
    HB_TXN_IDLE,
    HB_TXN_WAIT_FOR_CMD_START,      // we've asserted INTR, will respond with our cmd
    HB_TXN_TXING_CMD,               // transmitting cmd via i2c
    HB_TXN_WAIT_FOR_GP_RSP,         // we've transmitted a cmd, waiting for response from gopro
    HB_TXN_WAIT_FOR_RSP_START,      // we've asserted INTR, will respond with our rsp
    HB_TXN_TXING_RSP,               // transmitting response via i2c
    HB_TXN_RXING,                   // receiving bytes via i2c, contents determine cmd vs rsp
} herobus_txn_phase_t;

typedef enum {
    GP_POWER_UNKNOWN,
    GP_POWER_ON,
    GP_POWER_OFF,
    GP_POWER_WAIT
} GPPowerStatus;

typedef enum {
    GP_HEARTBEAT_DISCONNECTED = 0,
    GP_HEARTBEAT_INCOMPATIBLE = 1,
    GP_HEARTBEAT_CONNECTED = 2,
    GP_HEARTBEAT_RECORDING = 3
} GPHeartbeatStatus;

typedef enum{
    GP_MODEL_HERO3P,
    GP_MODEL_HERO4,
    GP_MODEL_UNKNOWN
} GPModel;

typedef enum{
    GP_CAPTURE_MODE_VIDEO,  // would like to start at 0x00 since this is consistent with spec but could be troublesome if mode enums begin to differ in the future
    GP_CAPTURE_MODE_PHOTO,
    GP_CAPTURE_MODE_BURST,
    // GP_MODE_TIME_LAPSE // TODO: do we want to make these GP-specific? maybe do it at the i2c layer and keep this as the superset? GPMode in general?
    GP_CAPTURE_MODE_UNKNOWN //= 99
} GPCaptureMode;

// XXX: what should this actually be?
#define GP_MAX_RESP_LEN     32

// represents a command/response transaction
typedef struct {
    GPRequestType reqtype;      // what kind of transaction is this
    uint16_t mav_cmd;           // which msg was set/gotten (see GOPRO_COMMAND)
    uint16_t status;            // was this set/get operation successful (see ...)
    uint16_t payload[GP_MAX_RESP_LEN];  // response payload
    uint16_t len;                       // response payload len
    bool is_internal;           // determines if a response over MAVLink is warranted
} gp_transaction_t;

// public interface
void gp_init();
void gp_send_mav_response();
void gp_update();
int gp_get_request(Uint8 cmd_id, bool txn_is_internal);
int gp_set_request(GPSetRequest* request);
GPHeartbeatStatus gp_get_heartbeat_status();
void gp_enable_charging();
void gp_disable_charging();

// "private" functions, called from gopro_hero4/gopro_hero3/gopro_i2c
// TODO detangle/remove from this header
bool gp_request_power_on();
void gp_on_i2c_stop_condition();
void gp_on_slave_address(bool addressed_as_tx);
uint16_t gp_transaction_cmd();
GPRequestType gp_transaction_direction();
void gp_set_transaction_result(const uint16_t *resp_bytes, uint16_t len, GPCmdStatus status);
GPCaptureMode gp_capture_mode();
void gp_latch_pending_capture_mode();
bool gp_pend_capture_mode(Uint8 capture_mode);
bool gp_set_capture_mode(Uint8 capture_mode);
void gp_set_recording_state(bool recording_state);
GPPowerStatus gp_get_power_status();


#endif /* GOPRO_INTERFACE_H_ */
