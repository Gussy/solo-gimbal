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


static void gp_h4_handle_cmd(gp_h4_t *h4, const gp_h4_pkt_t *c, gp_h4_pkt_t *rsp);
static void gp_h4_handle_rsp(gp_h4_t *h4, const gp_h4_pkt_t *p);
static bool gp_h4_handle_handshake(gp_h4_t *h4, const gp_h4_cmd_t *c, gp_h4_rsp_t *r);

void gp_h4_init(gp_h4_t *h4)
{
    unsigned i;
    for (i = 0; i < GP_H4_PROTO_NUM_BYTES; ++i) {
        h4->camera_proto_version[i] = 0xff;
    }

    h4->channel_id = 0; // defaults to 0
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

static bool is_yy(const gp_h4_pkt_t* c)
{
    return c->cmd.l1 == 'Y' && c->cmd.l2 == 'Y';
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
    if (!is_yy(p)) {
        // XXX: report unexpected response?
        return;
    }

    const gp_h4_yy_rsp_t * rsp = &p->yy_rsp;

    if (rsp->tcb != TCB_RSP_FINAL_FRAME || rsp->ack != H4_ACK) {
        // XXX: report failure
        return;
    }

    uint16_t len = yy_rsp_len(rsp);

    // handle any packets that shouldn't be forwarded via mavlink
    if (rsp->api_group == 0 && rsp->api_id == 1 && len == 1) {
        // 'Get Channel ID/Open Channel' is api 0/1
        h4->channel_id = rsp->payload[0];
        return;
    }

    // SET requests always have 0 len, GET requests always have non-zero len
    GPRequestType rt = (len == 0) ? GP_REQUEST_SET : GP_REQUEST_GET;
    gp_set_response(rsp->payload, len, rt, GP_CMD_STATUS_SUCCESS);
}

bool gp_h4_handle_handshake(gp_h4_t *h4, const gp_h4_cmd_t *c, gp_h4_rsp_t *r)
{
    /*
     * Called when a handshake request has been received from the camera.
     * Echo the payload to complete the handshake.
     */

    unsigned i;

    switch (c->payload[0]) {
    case ZZ_I2C_BUS_SPEED:
    case ZZ_RDY:
        // nothing special to do in these cases
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

bool gp_h4_request_power_off()
{
    // TODO
    return true;
}

bool gp_h4_cmd_has_param(const GPCmd* c)
{
    // TODO

    return true;
}

int gp_h4_get_request(Uint8 cmd_id, GOPRO_COMMAND *last_request_cmd_id) // TODO: name is a bit awkward, think about refactoring (gp_h4_handle_get_request?), same with set request
{
    // TODO

    return 0;
}

int gp_h4_set_request(GPSetRequest* request, GOPRO_COMMAND *last_request_cmd_id) // TODO: see TODO above
{
    // TODO
    return 0;
}

bool gp_h4_handle_command(const uint16_t *cmdbuf, uint16_t *txbuf)
{
    // TODO
    return true;
}

bool gp_h4_handle_response(const uint16_t *respbuf, GPCmdResponse *last_cmd_response)
{
    // TODO
    return true;
}
