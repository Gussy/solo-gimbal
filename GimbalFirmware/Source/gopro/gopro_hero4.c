#include "gopro_hero4.h"
#include "gopro_interface.h"
#include "hardware/i2c.h"

#include <string.h>

// Include for GOPRO_COMMAND enum
#include "mavlink_interface/mavlink_gimbal_interface.h"

enum GP_H4_ZZ_ACK_VALS {
    ZZ_ACK  = 0,
    ZZ_NACK = 1,
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
    // XXX: implement me
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
    r->ack = ZZ_ACK;
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
