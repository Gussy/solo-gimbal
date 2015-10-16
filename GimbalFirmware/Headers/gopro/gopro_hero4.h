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
    uint16_t channel_id;
    uint16_t handshake_step;                                // GP_H4_HANDSHAKE_STEPS
    bool pending_recording_state;   // latched based on response from camera
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

#define GP_H4_YY_CMD_HEADER_SIZE 10
#define GP_H4_YY_CMD_MAX_PAYLOAD (GP_H4_MAX_PACKET - GP_H4_YY_CMD_HEADER_SIZE) // TODO: necessary? maybe combine with GP_H4_MAX_PAYLOAD somehow?
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
    uint16_t payload[GP_H4_YY_CMD_MAX_PAYLOAD];
} gp_h4_yy_cmd_t;

#define GP_H4_YY_RSP_HEADER_SIZE 11
#define GP_H4_YY_RSP_MAX_PAYLOAD (GP_H4_MAX_PACKET - GP_H4_YY_RSP_HEADER_SIZE) // TODO: necessary? maybe combine with GP_H4_MAX_PAYLOAD somehow?
typedef struct {
    uint16_t len;       // size of this packet (not including 'len')
    uint16_t ack;       // NACK will occur if format of packet is unrecognized
    uint16_t reserved;  // keep as 0
    uint16_t tid;       // transaction id (keep as 0)
    uint16_t tcb;       // Transaction Control Block bit mask (see GP_H4_TCB_VALS)
    uint16_t chan_id;   // used as a session id for all subsequent YY requests
    uint16_t api_group;
    uint16_t api_id;
    uint16_t err_code;  // zero == success, non-zero == error
    uint16_t datalen1;
    uint16_t datalen2;
    uint16_t payload[GP_H4_YY_RSP_MAX_PAYLOAD];
} gp_h4_yy_rsp_t;

typedef union {

    gp_h4_cmd_t cmd;
    gp_h4_rsp_t rsp;
    gp_h4_yy_cmd_t yy_cmd;
    gp_h4_yy_rsp_t yy_rsp;

    uint16_t bytes[GP_H4_MAX_PACKET];

} gp_h4_pkt_t;

typedef enum {
    GP_H4_POWER_OFF_NORMAL,
    GP_H4_POWER_OFF_FORCED
} GPH4Power;

typedef enum {
    GP_H4_ERR_OK,
    GP_H4_ERR_NACK,         // ack byte set to nack
    GP_H4_ERR_YY_ERR_BYTE,  // err_code was non-zero
    GP_H4_ERR_RSP_BAD_TCB,  // a tcb value other than TCB_RSP_FINAL_FRAME
} gp_h4_err_t;

void gp_h4_init(gp_h4_t *h4);
bool gp_h4_handshake_complete(const gp_h4_t *h4);
bool gp_h4_finish_handshake(gp_h4_t *h4, gp_h4_pkt_t *p);
bool gp_h4_on_txn_complete(gp_h4_t *h4, gp_h4_pkt_t *p);
bool gp_h4_recognize_packet(const uint16_t *buf, uint16_t len);

bool gp_h4_rx_data_is_valid(const uint16_t *buf, uint16_t len);
gp_h4_err_t gp_h4_handle_rx(gp_h4_t *h4, const gp_h4_pkt_t *p, gp_h4_pkt_t *rsp);

bool gp_h4_produce_get_request(gp_h4_t *h4, Uint8 cmd_id, gp_h4_pkt_t *p);
bool gp_h4_produce_set_request(gp_h4_t *h4, const GPSetRequest* request, gp_h4_pkt_t *p);

#endif // _GOPRO_HERO4_H
