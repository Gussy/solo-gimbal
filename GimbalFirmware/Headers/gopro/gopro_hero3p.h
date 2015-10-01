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
    bool gccb_version_queried;
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

void gp_h3p_init(gp_h3p_t *h3p);
bool gp_h3p_handshake_complete(const gp_h3p_t *h3p);
bool gp_h3p_recognize_packet(const uint16_t *buf, uint16_t len);

bool gp_h3p_produce_get_request(uint8_t cmd_id, gp_h3p_cmd_t *c);
bool gp_h3p_produce_set_request(const GPSetRequest* request, gp_h3p_cmd_t *c);

bool gp_h3p_handle_rx(gp_h3p_t *h3p, uint16_t *buf, bool from_camera, uint16_t *txbuf);

bool gp_h3p_rx_data_is_valid(const uint16_t *buf, uint16_t len, bool *from_camera);
void gp_h3p_produce_command(const GPCmd* cmd, gp_h3p_cmd_t *c);

#endif // _GOPRO_HERO3P_H
