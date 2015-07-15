#ifndef _GOPRO_HERO4_H
#define _GOPRO_HERO4_H

#include <stdint.h>
#include <stdbool.h>
#include "gopro_hero_common.h"

// Include for GOPRO_COMMAND enum
#include "mavlink_interface/mavlink_gimbal_interface.h" // TODO: remove this after update to new response format

#define GP_H4_MAX_PACKET        256
#define GP_H4_MAX_PAYLOAD       (GP_H4_MAX_PACKET - 5)
#define GP_H4_HDR_LEN           4
#define GP_H4_PROTO_NUM_BYTES   3

// Hero 4 related state
typedef struct {
    uint16_t camera_proto_version[GP_H4_PROTO_NUM_BYTES];   // ZZ_REQ_SEND_PROTO_VERSION
    uint16_t camera_fw_version[16];                         // non-null-terminated string
} gp_h4_t;

// hero 4 packet formats

// Request (CMD) General Format
typedef struct {
    uint16_t len;   // size of this packet (not including 'len')
    uint16_t l1;    // first letter of 2-letter cmd
    uint16_t l2;    // second letter of 2-letter cmd
    uint16_t tid;   // transaction id (for future use, keep as 0)
    uint16_t tcb;   // Transaction Control Block bit mask (see GP_H4_TCB_VALS)
    uint16_t payload[GP_H4_MAX_PAYLOAD - 5];
} gp_h4_cmd_t;

// Response (RSP) General Format
typedef struct {
    uint16_t len;       // size of this packet (not including 'len')
    uint16_t ack;       // NACK will occur if format of packet is unrecognized
    uint16_t reserved;  // keep as 0
    uint16_t tid;       // transaction id (keep as 0)
    uint16_t tcb;       // Transaction Control Block bit mask (see GP_H4_TCB_VALS)
    uint16_t payload[GP_H4_MAX_PACKET - 5];
} gp_h4_rsp_t;

typedef struct {
    uint16_t len;       // size of this packet (not including 'len')
    uint16_t l1;        // first letter of 2-letter cmd
    uint16_t l2;        // second letter of 2-letter cmd
    uint16_t tid;       // transaction id (keep as 0)
    uint16_t tcb;       // Transaction Control Block bit mask (see GP_H4_TCB_VALS)
    uint16_t chan_id;   // used as a session id for all subsequent YY requests
    uint16_t api_group;
    uint16_t api_id;
    uint16_t datalen1;
    uint16_t datalen2;
    uint16_t payload[GP_H4_MAX_PACKET - 10];
} gp_h4_yy_cmd_t;

typedef struct {
    uint16_t len;       // size of this packet (not including 'len')
    uint16_t l1;        // first letter of 2-letter cmd
    uint16_t l2;        // second letter of 2-letter cmd
    uint16_t tid;       // transaction id (keep as 0)
    uint16_t tcb;       // Transaction Control Block bit mask (see GP_H4_TCB_VALS)
    uint16_t chan_id;   // used as a session id for all subsequent YY requests
    uint16_t api_group;
    uint16_t api_id;
    uint16_t err_code;  // zero == success, non-zero == error
    uint16_t datalen1;
    uint16_t datalen2;
    uint16_t payload[GP_H4_MAX_PACKET - 11];
} gp_h4_yy_rsp_t;

typedef union {

    gp_h4_cmd_t cmd;
    gp_h4_rsp_t rsp;
    gp_h4_yy_cmd_t yy_cmd;
    gp_h4_yy_rsp_t yy_rsp;

    uint16_t bytes[GP_H4_MAX_PACKET];

} gp_h4_pkt_t;

bool gp_h4_rx_data_is_valid(const uint16_t *buf, uint16_t len);
bool gp_h4_handle_rx(gp_h4_t *h4, const gp_h4_pkt_t *p, gp_h4_pkt_t *rsp);

bool gp_h4_request_power_off();
bool gp_h4_cmd_has_param(const GPCmd* c);
int gp_h4_get_request(Uint8 cmd_id, GOPRO_COMMAND *last_request_cmd_id);
int gp_h4_set_request(GPSetRequest* request, GOPRO_COMMAND *last_request_cmd_id);
bool gp_h4_handle_command(const uint16_t *cmdbuf, uint16_t *txbuf);
bool gp_h4_handle_response(const uint16_t *respbuf, GPCmdResponse *last_cmd_response);

#endif // _GOPRO_HERO4_H
