#ifndef _GOPRO_HERO3P_H
#define _GOPRO_HERO3P_H

#include <stdint.h>
#include <stdbool.h>
#include "gopro_hero_common.h"
#include "PeripheralHeaderIncludes.h"

// Include for GOPRO_COMMAND enum
#include "mavlink_interface/mavlink_gimbal_interface.h"

// Hero 3 related state
typedef struct {
    // TODO
} gp_h3p_t;

// Hero 3+ packet formats

// Request (CMD) General Format
typedef struct {
    // TODO
} gp_h3p_cmd_t;

// Response (RSP) General Format
typedef struct {
    // TODO
} gp_h3p_rsp_t;

bool gp_h3p_request_power_off();
bool gp_h3p_cmd_has_param(const GPCmd* c);
int gp_h3p_get_request(Uint8 cmd_id, bool *new_response_available, GOPRO_COMMAND *last_request_cmd_id);
int gp_h3p_set_request(GPSetRequest* request, bool *new_response_available, GPSetRequest* last_set_request, GOPRO_COMMAND *last_request_cmd_id);
bool gp_h3p_handle_command(const uint16_t *cmdbuf, uint16_t *txbuf, bool *gccb_version_queried);
bool gp_h3p_handle_response(const uint16_t *respbuf, GPCmdResponse *last_cmd_response);

bool gp_h3p_rx_data_is_valid(uint16_t *buf, uint16_t len, bool *from_camera);

#endif // _GOPRO_HERO3P_H
