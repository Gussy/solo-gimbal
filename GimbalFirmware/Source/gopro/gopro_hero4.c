#include "gopro_hero4.h"
#include "hardware/i2c.h"
#include "gopro_interface.h"

#include <string.h>

// Include for GOPRO_COMMAND enum
#include "mavlink_interface/mavlink_gimbal_interface.h"

enum GP_H4_ACK_VALS {
    H4_ACK  = 0,
    H4_NACK = 1
};

enum GP_H4_TCB_VALS {
    TCB_CMD_SINGLE_PKT  = 0x0,
    TCB_RSP_FINAL_FRAME = 0x10
};

enum GP_ZZ_FMT {
    ZZ_I2C_BUS_SPEED    = 1,    // 0 == standard
    ZZ_RDY              = 2,    // 1 == ready
    ZZ_FW_VERSION       = 3,    // fmt: "HD4.0x.MM.mm.rr"
    ZZ_PROTO_VERSION    = 4     // fmt: MM, mm, rr (3 bytes)
};

enum GP_H4_HANDSHAKE_STEPS {
    GP_H4_HANDSHAKE_NONE,
    GP_H4_HANDSHAKE_READY,              // have received `ZZ ready` cmd from camera
    GP_H4_HANDSHAKE_HB_PROTO_VERSION,   // have receive `ZZ HeroBus Protocol Version` from camera
    GP_H4_HANDSHAKE_CHANNEL_OPEN        // have opened the comms channel with the camera
};

static void gp_h4_handle_cmd(gp_h4_t *h4, const gp_h4_pkt_t *c, gp_h4_pkt_t *rsp);
static void gp_h4_handle_rsp(gp_h4_t *h4, const gp_h4_pkt_t *p);
static bool gp_h4_handle_handshake(gp_h4_t *h4, const gp_h4_cmd_t *c, gp_h4_rsp_t *r);
static void gp_h4_finalize_yy_cmd(gp_h4_t *h4, uint16_t payloadlen, gp_h4_yy_cmd_t *c);

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

bool gp_h4_finish_handshake(gp_h4_t *h4, gp_h4_pkt_t *p)
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
        gp_h4_yy_cmd_t *yy = &p->yy_cmd;
        yy->api_group = 0;
        yy->api_id = 1;
        gp_h4_finalize_yy_cmd(h4, 0, yy);
        return true;
    }

    return false;
}

bool gp_h4_on_txn_complete(gp_h4_t *h4, gp_h4_pkt_t *p)
{
    // must kick off final step of handshake sequence on hero4
    if (!gp_h4_handshake_complete(h4)) {
        return gp_h4_finish_handshake(h4, p);
    }

    return false;
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

void gp_h4_finalize_yy_cmd(gp_h4_t *h4, uint16_t payloadlen, gp_h4_yy_cmd_t *c)
{
    /*
     * assemble a YY cmd for transmission.
     * expects that api_group and api_id are populated elsewhere.
     *
     * 'len' specifies payload length.
     */

    yy_set_cmd_len(c, payloadlen);
    c->l1 = 'Y';
    c->l2 = 'Y';
    c->chan_id = h4->channel_id;
    c->tid = 0;
    c->tcb = TCB_CMD_SINGLE_PKT;
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

    if (rsp->tcb != TCB_RSP_FINAL_FRAME || rsp->ack != H4_ACK || rsp->err_code) {
        gp_set_transaction_result(rsp->payload, len, GP_CMD_STATUS_FAILURE);
        return;
    }

    // handle any packets that shouldn't be forwarded via mavlink
    if (rsp->api_group == 0 && rsp->api_id == 1 && len == 1) {
        // 'Get Channel ID/Open Channel' is api 0/1
        h4->channel_id = rsp->payload[0];
        h4->handshake_step = GP_H4_HANDSHAKE_CHANNEL_OPEN;
        return;         // TODO: update with new bool for internal transactions
    }

    if (gp_transaction_cmd() == GOPRO_COMMAND_CAPTURE_MODE) {
        if (gp_transaction_direction() == GP_REQUEST_GET) {
            if (len >= 1) {
                gp_set_capture_mode(rsp->payload[0]);                          // Set capture mode state with capture mode received from GoPro
            }
        } else if (gp_transaction_direction() == GP_REQUEST_SET) {
            gp_latch_pending_capture_mode();                                  // Set request acknowledged, update capture mode state with pending capture mode received via MAVLink/CAN
        }
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

bool gp_h4_produce_get_request(gp_h4_t *h4, Uint8 cmd_id, gp_h4_pkt_t *p)
{
    /*
     * A GET request has been received via the CAN interface,
     * forward it to the camera.
     */

    gp_h4_yy_cmd_t *yy = &p->yy_cmd;

    uint16_t payloadlen = 0;

    switch (cmd_id) {
        case GOPRO_COMMAND_CAPTURE_MODE:
            yy->api_group = 1;
            yy->api_id    = 0;
            break;

        case GOPRO_COMMAND_BATTERY:
            yy->api_group = 8;
            yy->api_id    = 0;
            break;

        case GOPRO_COMMAND_RESOLUTION:
        case GOPRO_COMMAND_FRAME_RATE:
        case GOPRO_COMMAND_FIELD_OF_VIEW:
            yy->api_group = 2;
            yy->api_id    = 2;
            break;

        case GOPRO_COMMAND_MODEL:
        case GOPRO_COMMAND_SHUTTER:
        default:
            // Unsupported Command ID
            gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
            return false;
    }

    gp_h4_finalize_yy_cmd(h4, payloadlen, yy);
    return true;
}

bool gp_h4_produce_set_request(gp_h4_t *h4, const GPSetRequest* request, gp_h4_pkt_t *p)
{
    /*
     * A SET request has been received via the CAN interface,
     * forward it to the camera.
     */

    gp_h4_yy_cmd_t *yy = &p->yy_cmd;

    uint16_t payloadlen = 0;

    switch (request->cmd_id) {
        case GOPRO_COMMAND_POWER:
            if(request->value == 0x00 && gp_get_power_status() == GP_POWER_ON) {
                yy->api_group = 8;
                yy->api_id = 2;
                yy->payload[0] = request->value;
                payloadlen = 1;
            } else {
                // no supported command to power on.
                // have tried gp_request_power_on(), which is implemented based on hero3 docs,
                // but does not appear to power up the camera.
                gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
                return false;
            }
            break;

        case GOPRO_COMMAND_CAPTURE_MODE:
            yy->api_group = 1;
            yy->api_id = 1;
            // TODO: verify this is a valid GPH4Power value
            yy->payload[0] = request->value;
            payloadlen = 1;

            gp_pend_capture_mode(request->value);
            break;

        case GOPRO_COMMAND_SHUTTER:

            switch (gp_capture_mode()) {
            case GP_CAPTURE_MODE_VIDEO:
                yy->api_group = 2;
                switch (request->value) {
                case GP_RECORDING_START:
                    yy->api_id = 0x1b;
                    gp_set_recording_state(true); // TODO: settings this after the command has received a successful response would be more robust
                    break;
                case GP_RECORDING_STOP:
                    yy->api_id = 0x1c;
                    gp_set_recording_state(false);
                    break;
                default:
                    gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
                    return false;
                }
                break;

            case GP_CAPTURE_MODE_PHOTO:
                yy->api_group = 3;
                switch (request->value) {
                case GP_RECORDING_START:
                    yy->api_id = 0x17;
                    //gp_set_recording_state(true);     // no need since we don't have a way to find out when recording is finished
                    break;
                case GP_RECORDING_STOP:
                    yy->api_id = 0x18;
                    //gp_set_recording_state(false);
                    break;
                default:
                    gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
                    return false;
                }
                break;

            case GP_CAPTURE_MODE_BURST:
                yy->api_group = 4;
                switch (request->value) {
                case GP_RECORDING_START:
                    yy->api_id = 0x1b;
                    //gp_set_recording_state(true);      // no need since we don't have a way to find out when recording is finished
                    break;
                case GP_RECORDING_STOP:
                    yy->api_id = 0x1c;
                    //gp_set_recording_state(false);
                    break;
                default:
                    gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
                    return false;
                }
                break;

            default:
                // unknown capture mode
                gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
                return false;
            }

            payloadlen = 0;
            break;

        case GOPRO_COMMAND_RESOLUTION: // TODO: how do we know the other values? new MAVLink message type might be needed here
            yy->payload[0] = request->value;  // resolution
            yy->payload[1] = 0;               // fps
            yy->payload[2] = 0;               // fov
        case GOPRO_COMMAND_FIELD_OF_VIEW:
            yy->payload[0] = 0;
            yy->payload[1] = request->value;
            yy->payload[2] = 0;
        case GOPRO_COMMAND_FRAME_RATE:
            yy->payload[0] = 0;
            yy->payload[1] = 0;
            yy->payload[2] = request->value;

            /* TODO set to default while we figure this out */
            // TODO: current defaults: 1080p (enum:9) @ 30 FPS (enum:8), wide (enum:0)
            yy->payload[0] = 9;
            yy->payload[1] = 8;
            yy->payload[2] = 0;

            payloadlen = 3;
            yy->api_group = 2;
            yy->api_id = 3;
            break;

        default:
            // Unsupported Command ID
            gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
            return false;
    }

    gp_h4_finalize_yy_cmd(h4, payloadlen, yy);
    return true;
}
