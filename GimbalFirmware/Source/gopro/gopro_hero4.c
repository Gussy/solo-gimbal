#include "gopro_hero4.h"
#include "hardware/i2c.h"
#include "gopro_interface.h"

#include <string.h>

// Include for GOPRO_COMMAND enum
#include "mavlink_interface/mavlink_gimbal_interface.h"

enum GP_H4_ACK_VALS {
    H4_ACK  = 0,
    H4_NACK = 1,
};

enum GP_H4_TCB_VALS {
    TCB_CMD_SINGLE_PKT  = 0x0,
    TCB_RSP_FINAL_FRAME = 0x10,
};

enum GP_ZZ_FMT {
    ZZ_I2C_BUS_SPEED    = 1,    // 0 == standard
    ZZ_RDY              = 2,    // 1 == ready
    ZZ_FW_VERSION       = 3,    // fmt: "HD4.0x.MM.mm.rr"
    ZZ_PROTO_VERSION    = 4,    // fmt: MM, mm, rr (3 bytes)
};

enum GP_H4_HANDSHAKE_STEPS {
    GP_H4_HANDSHAKE_NONE,
    GP_H4_HANDSHAKE_READY,              // have received `ZZ ready` cmd from camera
    GP_H4_HANDSHAKE_HB_PROTO_VERSION,   // have receive `ZZ HeroBus Protocol Version` from camera
    GP_H4_HANDSHAKE_CHANNEL_OPEN,       // have opened the comms channel with the camera
};

static void gp_h4_handle_cmd(gp_h4_t *h4, const gp_h4_pkt_t *c, gp_h4_pkt_t *rsp);
static void gp_h4_handle_rsp(gp_h4_t *h4, const gp_h4_pkt_t *p);
static bool gp_h4_handle_handshake(gp_h4_t *h4, const gp_h4_cmd_t *c, gp_h4_rsp_t *r);
static void gp_h4_send_yy_cmd(gp_h4_t *h4, uint16_t api_group, uint16_t api_id, const uint16_t *b, uint16_t len);

void gp_h4_init(gp_h4_t *h4)
{
    unsigned i;
    for (i = 0; i < GP_H4_PROTO_NUM_BYTES; ++i) {
        h4->camera_proto_version[i] = 0xff;
    }

    h4->channel_id = 0; // defaults to 0
    h4->handshake_step = GP_H4_HANDSHAKE_NONE;
}

bool gp_h4_handshake_complete(const gp_h4_t *h4)
{
    return h4->handshake_step == GP_H4_HANDSHAKE_CHANNEL_OPEN;
}

void gp_h4_finish_handshake(gp_h4_t *h4)
{
    /*
     * Called when:
     * - we know the camera model is hero4
     * - a response write has just completed
     *
     * The first 2 steps in the handshake sequence are initiated
     * by the camera, but after we've responded to those we must
     * send the 'Get Channel ID/Open Channel' to retrieve a channel
     * ID that can be used in all subsequent communication.
     */

    if (h4->handshake_step == GP_H4_HANDSHAKE_HB_PROTO_VERSION) {
        // 'Get Channel ID/Open Channel' is api 0/1
        gp_h4_send_yy_cmd(h4, 0, 1, NULL, 0);
    }
}

bool gp_h4_recognize_packet(const uint16_t *buf, uint16_t len)
{
    /*
     * Called when we don't yet know what kind of camera we're talking to.
     *
     * We only expect to see handshake packets in this state,
     * so make sure it's long enough and is a ZZ packet.
     */

    return gp_h4_rx_data_is_valid(buf, len) && len >= 6 && buf[1] == 'Z' && buf[2] == 'Z';
}

bool gp_h4_rx_data_is_valid(const uint16_t *buf, uint16_t len)
{
    /*
     * Called when an i2c rx transaction has completed successfully,
     * to determine if the received data is formatted correctly.
     */

    // first byte is the length of the received data,
    // ensure it matches the received size
    if (buf[0] != len - 1) {
        return false;
    }

    return true;
}

bool gp_h4_handle_rx(gp_h4_t *h4, const gp_h4_pkt_t *p, gp_h4_pkt_t *rsp)
{
    /*
     * Handle a newly received packet.
     *
     * Return true if we generate response data that
     * needs to be written back out.
     */

    switch (p->cmd.tcb) {
    case TCB_CMD_SINGLE_PKT:
        gp_h4_handle_cmd(h4, p, rsp);
        return true;

    case TCB_RSP_FINAL_FRAME:
        gp_h4_handle_rsp(h4, p);
        break;
    }

    return false;
}

static void yy_set_cmd_len(gp_h4_yy_cmd_t *c, uint16_t len)
{
    /*
     * 'len' specifies payload length,
     * but c->len must be total packet size minus 1.
     *
     * somewhat insane, but the gopro datasheet says,
     * "For example, Byte 10 with value '1' and Byte 11 with value '2'
     * would indicate that there are 12 byte of response data bytes"
     */

    c->len = len + 9;
    c->datalen1 = len / 10;
    c->datalen2 = len % 10;
}

static uint16_t yy_rsp_len(const gp_h4_yy_rsp_t *r)
{
    /*
     * See datalen handling notes above.
     */

    return (r->datalen1 * 10) + r->datalen2;
}

void gp_h4_send_yy_cmd(gp_h4_t *h4, uint16_t api_group, uint16_t api_id, const uint16_t *b, uint16_t len)
{
    /*
     * assemble a YY cmd for transmission.
     *
     * 'len' specifies payload length.
     */

    gp_h4_pkt_t p;
    gp_h4_yy_cmd_t *c = &p.yy_cmd;

    yy_set_cmd_len(c, len);
    c->l1 = 'Y';
    c->l2 = 'Y';
    c->chan_id = h4->channel_id;
    c->tid = 0;
    c->tcb = TCB_CMD_SINGLE_PKT;
    c->api_group = api_group;
    c->api_id = api_id;

    int i;
    for (i = 0; i < len; ++i) {
        c->payload[i] = b[i];
    }

    gp_send_cmd(p.bytes, p.cmd.len + 1);
}

static bool is_zz(const gp_h4_pkt_t* c)
{
    return c->cmd.l1 == 'Z' && c->cmd.l2 == 'Z';
}

void gp_h4_handle_cmd(gp_h4_t *h4, const gp_h4_pkt_t* c, gp_h4_pkt_t *rsp)
{
    /*
     * A command from the camera has arrived.
     * Handle it, and write the response into 'rsp'.
     */

    if (is_zz(c)) {
        if (gp_h4_handle_handshake(h4, &c->cmd, &rsp->rsp)) {
            // XXX: implement me
        } else {
            // XXX: signal that we saw invalid handshake data?
        }
    }
}

void gp_h4_handle_rsp(gp_h4_t *h4, const gp_h4_pkt_t* p)
{
    const gp_h4_yy_rsp_t * rsp = &p->yy_rsp;
    uint16_t len = yy_rsp_len(rsp);

    if (rsp->tcb != TCB_RSP_FINAL_FRAME || rsp->ack != H4_ACK) {
        gp_set_transaction_result(rsp->payload, len, GP_CMD_STATUS_FAILURE);
        return;
    }

    // handle any packets that shouldn't be forwarded via mavlink
    if (rsp->api_group == 0 && rsp->api_id == 1 && len == 1) {
        // 'Get Channel ID/Open Channel' is api 0/1
        h4->channel_id = rsp->payload[0];
        h4->handshake_step = GP_H4_HANDSHAKE_CHANNEL_OPEN;
        return;
    }

    gp_set_transaction_result(rsp->payload, len, GP_CMD_STATUS_SUCCESS);
}

bool gp_h4_handle_handshake(gp_h4_t *h4, const gp_h4_cmd_t *c, gp_h4_rsp_t *r)
{
    /*
     * Called when a handshake request has been received from the camera.
     * Echo the payload to complete the handshake.
     */

    unsigned i;

    switch (c->payload[0]) {
    case ZZ_RDY:
        h4->handshake_step = GP_H4_HANDSHAKE_READY;
        break;

    case ZZ_I2C_BUS_SPEED:
        // nothing special to do
        break;

    case ZZ_FW_VERSION:
        for (i = 0; i < c->len - GP_H4_HDR_LEN - 1; ++i) {
            h4->camera_fw_version[i] = c->payload[i+1];
        }
        break;

    case ZZ_PROTO_VERSION:
        for (i = 0; i < GP_H4_PROTO_NUM_BYTES; ++i) {
            h4->camera_proto_version[i] = c->payload[i+1];
        }
        h4->handshake_step = GP_H4_HANDSHAKE_HB_PROTO_VERSION;
        break;

    default:
        return false;
    }

    r->len = c->len;
    r->ack = H4_ACK;
    r->reserved = 0;
    r->tid = 0;
    // all ZZ responses indicate that they are the final (and only) frame
    r->tcb = TCB_RSP_FINAL_FRAME;
    memcpy(r->payload, c->payload, (c->len - GP_H4_HDR_LEN) * sizeof(uint16_t));

    return true;
}

#if 1
// TODO: maybe use this one in gp_h4_request_power_off
typedef enum{
    GP_H4_POWER_OFF_NORMAL,
    GP_H4_POWER_OFF_FORCED
} GPH4Power;
#endif

bool gp_h4_request_power_off(gp_h4_t *h4)
{
    uint16_t api_group = 8;
    uint16_t api_id = 2;
    uint16_t b[1] = {GP_H4_POWER_OFF_NORMAL}; // 0x00 - Normal, 0x01 - Forced
    uint16_t len = 1;
    gp_h4_send_yy_cmd(h4, api_group, api_id, b, len);
    return true;
}

int gp_h4_forward_get_request(gp_h4_t *h4, Uint8 cmd_id)
{
    /*
     * A GET request has been received via the CAN interface,
     * forward it to the camera.
     */

    uint16_t api_group = 0;
    uint16_t api_id = 0;
    uint16_t b[1] = {0}; // should always be null when treating get_requests, alt: b[GP_H4_YY_CMD_MAX_PAYLOAD]
    uint16_t len = 0;

    switch (cmd_id) {
        case GOPRO_COMMAND_CAPTURE_MODE:
            api_group = 1;
            api_id    = 0;
            break;

        case GOPRO_COMMAND_BATTERY:
            api_group = 8;
            api_id    = 0;
            break;

        case GOPRO_COMMAND_RESOLUTION:
        case GOPRO_COMMAND_FRAME_RATE:
        case GOPRO_COMMAND_FIELD_OF_VIEW:
            api_group = 2;
            api_id    = 2;
            break;

        case GOPRO_COMMAND_MODEL:
        case GOPRO_COMMAND_SHUTTER:
        default:
            // Unsupported Command ID
            gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
            return -1;
    }

    gp_h4_send_yy_cmd(h4, api_group, api_id, b, len);
    return 0;
}

int gp_h4_forward_set_request(gp_h4_t *h4, const GPSetRequest* request)
{
    /*
     * A SET request has been received via the CAN interface,
     * forward it to the camera.
     */

    uint16_t api_group = 0;
    uint16_t api_id = 0;
    uint16_t b[GP_H4_YY_CMD_MAX_PAYLOAD];
    uint16_t len = 0;

    switch (request->cmd_id) {
        case GOPRO_COMMAND_POWER:
            if(request->value == 0x00 && gp_get_power_status() == GP_POWER_ON) {
                api_group = 8;  // TODO: alternatively use gp_h4_request_power_off() and pass in power-off type
                api_id = 2;
                b[0] = request->value;
                len = 1;
            } else {
                // do nothing, can't request power on
            }
            break;

        case GOPRO_COMMAND_CAPTURE_MODE:
            api_group = 1;
            api_id = 1;
            b[0] = request->value;
            len = 1;
            break;

        case GOPRO_COMMAND_SHUTTER:
            // Start video recording
            api_group = 2;
            api_id = 0x1b;
            b[0] = request->value;
            len = 1;
            break;

        case GOPRO_COMMAND_RESOLUTION: // TODO: how do we know the other values? new MAVLink message type might be needed here
            b[0] = request->value;  // resolution
            b[1] = 0;               // fps
            b[2] = 0;               // fov
        case GOPRO_COMMAND_FIELD_OF_VIEW:
            b[0] = 0;
            b[1] = request->value;
            b[2] = 0;
        case GOPRO_COMMAND_FRAME_RATE:
            b[0] = 0;
            b[1] = 0;
            b[2] = request->value;

            /* TODO set to default while we figure this out */
            // TODO: current defaults: 1080p (enum:9) @ 30 FPS (enum:8), wide (enum:0)
            b[0] = 9;
            b[1] = 8;
            b[2] = 0;

            len = 3;
            api_group = 2;
            api_id = 3;
            break;

        default:
            // Unsupported Command ID
            gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
            return -1;
    }

    gp_h4_send_yy_cmd(h4, api_group, api_id, b, len);
    return 0;
}
