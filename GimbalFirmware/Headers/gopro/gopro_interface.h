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
#define GP_I2C_EEPROM_NUMBYTES 16

#define GP_PWRON_LOW() {GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;}
#define GP_PWRON_HIGH() {GpioDataRegs.GPASET.bit.GPIO22 = 1;}

#define GP_VON (GpioDataRegs.GPADAT.bit.GPIO6)

typedef enum {
    GP_CONTROL_STATE_IDLE,
    GP_CONTROL_STATE_REQUEST_POWER_ON,
    GP_CONTROL_STATE_WAIT_POWER_ON,
    GP_CONTROL_STATE_WAIT_FOR_START_CMD_SEND,
    GP_CONTROL_STATE_WAIT_FOR_COMPLETE_CMD_SEND,
    GP_CONTROL_STATE_WAIT_FOR_CMD_RESPONSE,
    GP_CONTROL_STATE_WAIT_FOR_GP_DATA_COMPLETE,
    GP_CONTROL_STATE_RETRIEVE_RECEIVED_DATA,
    GP_CONTROL_STATE_PARSE_RECEIVED_CMD,
    GP_CONTROL_STATE_PARSE_RECEIVED_RESPONSE,
    GP_CONTROL_STATE_WAIT_READY_TO_SEND_RESPONSE,
    GP_CONTROL_STATE_WAIT_TO_COMPLETE_RESPONSE_SEND
} GPControlState;

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

// XXX: what should this actually be?
#define GP_MAX_RESP_LEN     32

// represents a command/response transaction
typedef struct {
    GPRequestType reqtype;      // what kind of transaction is this
    uint16_t mav_cmd;           // which msg was set/gotten (see GOPRO_COMMAND)
    uint16_t status;            // was this set/get operation successful (see ...)
    uint16_t payload[GP_MAX_RESP_LEN];  // response payload
    uint16_t len;                       // response payload len
} gp_transaction_t;

void init_gp_interface();
void gp_interface_state_machine();
GPPowerStatus gp_get_power_status();
bool gp_request_power_on();
bool gp_request_power_off();
bool gp_send_command(const GPCmd* cmd);
bool gp_send_cmd(const uint16_t* cmd, uint16_t len);
bool gp_ready_for_cmd();
void gp_write_eeprom();

void gp_on_slave_address(bool addressed_as_tx);

uint16_t gp_transaction_cmd();
void gp_set_transaction_result(const uint16_t *resp_bytes, uint16_t len, GPCmdStatus status);
bool gp_transaction_result_available();
bool gp_get_completed_transaction(gp_transaction_t **rsp);

inline void gp_assert_intr() {
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;
}

inline void gp_deassert_intr(void) {
    GpioDataRegs.GPASET.bit.GPIO26 = 1;
}

bool gp_new_heartbeat_available();

int gp_get_request(Uint8 cmd_id);
int gp_set_request(GPSetRequest* request);

GPHeartbeatStatus gp_heartbeat_status();

void gp_enable_hb_interface();
void gp_disable_hb_interface();
void gp_enable_charging();
void gp_disable_charging();

#endif /* GOPRO_INTERFACE_H_ */
