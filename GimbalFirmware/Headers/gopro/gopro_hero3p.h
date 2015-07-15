#ifndef _GOPRO_HERO3P_H
#define _GOPRO_HERO3P_H

#include <stdint.h>
#include <stdbool.h>
#include "gopro_hero_common.h"
#include "PeripheralHeaderIncludes.h"

// Include for GOPRO_COMMAND enum
#include "mavlink_interface/mavlink_gimbal_interface.h"

#define GP_H3P_MAX_PACKET        256
#define GP_H3P_MAX_PAYLOAD       (GP_H3P_MAX_PACKET - 5)

// Hero 3 related state
typedef struct {
    // TODO
} gp_h3p_t;

// Hero 3+ packet formats

// Request (CMD) General Format
typedef struct {
    uint16_t len;     // size of this packet (not including 'len'), also contains requester id (Camera or Bac-Pac) in most significant bit
    uint16_t cmd1;    // first letter of 2-letter cmd
    uint16_t cmd2;    // second letter of 2-letter cmd
    uint16_t payload[GP_H3P_MAX_PAYLOAD - 3]; // known as 'params' in documentation // TODO: keep this consistent with hero4 naming or call params?
} gp_h3p_cmd_t;

// Response (RSP) General Format
typedef struct {
    uint16_t len;       // size of this packet (not including 'len')
    uint16_t status;    // successful/unsuccessful
    uint16_t payload[GP_H3P_MAX_PAYLOAD - 2];
} gp_h3p_rsp_t;

typedef union {

    gp_h3p_cmd_t cmd;
    gp_h3p_rsp_t rsp;

    uint16_t bytes[GP_H3P_MAX_PACKET];

} gp_h3p_pkt_t;

bool gp_h3p_request_power_off();
bool gp_h3p_cmd_has_param(const GPCmd* c);
int gp_h3p_get_request(Uint8 cmd_id, bool *new_response_available, GOPRO_COMMAND *last_request_cmd_id, GPCmdResponse *last_cmd_response);
int gp_h3p_set_request(GPSetRequest* request, bool *new_response_available, GPSetRequest* last_set_request, GOPRO_COMMAND *last_request_cmd_id, GPCmdResponse *last_cmd_response);
bool gp_h3p_handle_command(const uint16_t *cmdbuf, uint16_t *txbuf, bool *gccb_version_queried);
bool gp_h3p_handle_response(const uint16_t *respbuf, GPCmdResponse *last_cmd_response);
bool gp_h3p_rx_data_is_valid(uint16_t *buf, uint16_t len, bool *from_camera);
bool gp_h3p_send_command(const GPCmd* cmd, GPCmdResponse *last_cmd_response);

#endif // _GOPRO_HERO3P_H
