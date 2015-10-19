#include "gopro_hero4.h"
#include "gopro_hero4_defs.h"
#include "hardware/i2c.h"
#include "gopro_interface.h"

#include <string.h>

// Include for GOPRO_COMMAND enum
#include "mavlink_interface/mavlink_gimbal_interface.h"

static void gp_h4_handle_cmd(gp_h4_t *h4, const gp_h4_pkt_t *c, gp_h4_pkt_t *rsp);
static gp_h4_err_t gp_h4_handle_rsp(gp_h4_t *h4, const gp_h4_pkt_t *p);
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
    h4->pending_recording_state = false;
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
        gp_h4_yy_cmd_t *yy = &p->yy_cmd;
        yy->api_group = API_GRP_GEN_CMDS;
        yy->api_id = API_ID_OPEN_CHAN;
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

    // all hero4 packets have at least 5 bytes:
    // len, ack, reserved, tid, tcb
    if (len < 5) {
        return false;
    }

    // first byte is the length of the received data,
    // ensure it matches the received size
    if (buf[0] != len - 1) {
        return false;
    }

    return true;
}

gp_h4_err_t gp_h4_handle_rx(gp_h4_t *h4, const gp_h4_pkt_t *p, gp_h4_pkt_t *rsp)
{
    /*
     * Handle a newly received packet.
     *
     * Return true if we generate response data that
     * needs to be written back out.
     */

    gp_h4_err_t err = GP_H4_ERR_OK;

    switch (p->cmd.tcb) {
    case TCB_CMD_SINGLE_PKT:
        gp_h4_handle_cmd(h4, p, rsp);
        break;

    case TCB_RSP_FINAL_FRAME:
        err = gp_h4_handle_rsp(h4, p);
        break;

    default:
        err = GP_H4_ERR_RSP_BAD_TCB;
        break;
    }

    return err;
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

static gp_h4_err_t gp_h4_get_rsp_err(const gp_h4_yy_rsp_t *rsp)
{
    /*
     * Check rsp for error condition.
     */

    if (rsp->tcb != TCB_RSP_FINAL_FRAME) {
        return GP_H4_ERR_RSP_BAD_TCB;
    }

    if (rsp->ack != H4_ACK) {
        return GP_H4_ERR_NACK;
    }

    if (rsp->err_code != 0) {
        return GP_H4_ERR_YY_ERR_BYTE;
    }

    return GP_H4_ERR_OK;
}

gp_h4_err_t gp_h4_handle_rsp(gp_h4_t *h4, const gp_h4_pkt_t* p)
{
    const gp_h4_yy_rsp_t * rsp = &p->yy_rsp;
    uint16_t len = yy_rsp_len(rsp);

    gp_h4_err_t err = gp_h4_get_rsp_err(rsp);
    if (err != GP_H4_ERR_OK) {
        gp_set_transaction_result(rsp->payload, len, GP_CMD_STATUS_FAILURE);
        return err;
    }

    // handle any packets that shouldn't be forwarded via mavlink
    if (rsp->api_group == API_GRP_GEN_CMDS && rsp->api_id == API_ID_OPEN_CHAN && len == 1) {
        h4->channel_id = rsp->payload[0];
        h4->handshake_step = GP_H4_HANDSHAKE_CHANNEL_OPEN;
        return err; // TODO: update with new bool for internal transactions
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

    if (rsp->api_group == API_GRP_MODE_VID &&
       (rsp->api_id == API_ID_TRIGGER_VID_START || rsp->api_id == API_ID_TRIGGER_VID_STOP))
    {
        gp_set_recording_state(h4->pending_recording_state);
    }

    gp_set_transaction_result(rsp->payload, len, GP_CMD_STATUS_SUCCESS);
    return err;
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
        yy->api_group = API_GRP_MODE_CAM;
        yy->api_id    = API_ID_GET_CAM_MAIN_MODE;
        break;

    case GOPRO_COMMAND_BATTERY:
        yy->api_group = API_GRP_CAM_SETTINGS;
        yy->api_id    = API_ID_GET_BATTERY_LVL;
        break;

    case GOPRO_COMMAND_RESOLUTION:
    case GOPRO_COMMAND_FRAME_RATE:
    case GOPRO_COMMAND_FIELD_OF_VIEW:
        yy->api_group = API_GRP_MODE_VID;
        yy->api_id    = API_ID_GET_RES_FR_FOV;
        break;

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
                yy->api_group = API_GRP_CAM_SETTINGS;
                yy->api_id = API_ID_POWER_OFF;
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
            yy->api_group = API_GRP_MODE_CAM;
            yy->api_id = API_ID_SET_CAM_MAIN_MODE;
            // TODO: verify this is a valid GPH4Power value
            yy->payload[0] = request->value;
            payloadlen = 1;

            gp_pend_capture_mode(request->value);
            break;

        case GOPRO_COMMAND_SHUTTER:
            switch (gp_capture_mode()) {
            case GP_CAPTURE_MODE_VIDEO:
                yy->api_group = API_GRP_MODE_VID;
                switch (request->value) {
                case GP_RECORDING_START:
                    yy->api_id = API_ID_TRIGGER_VID_START;
                    h4->pending_recording_state = true;
                    break;

                case GP_RECORDING_STOP:
                    yy->api_id = API_ID_TRIGGER_VID_STOP;
                    h4->pending_recording_state = false;
                    break;

                default:
                    gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
                    return false;
                }
                break;

            case GP_CAPTURE_MODE_PHOTO:
                yy->api_group = API_GRP_MODE_PHOTO;
                switch (request->value) {
                case GP_RECORDING_START:
                    yy->api_id = API_ID_TRIGGER_PHOTO_START;
                    break;

                case GP_RECORDING_STOP:
                    yy->api_id = API_ID_TRIGGER_PHOTO_STOP;
                    break;

                default:
                    gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
                    return false;
                }
                break;

            case GP_CAPTURE_MODE_BURST:
                yy->api_group = API_GRP_MODE_MULTISHOT;
                switch (request->value) {
                case GP_RECORDING_START:
                    yy->api_id = API_ID_TRIGGER_MULTI_START;
                    break;

                case GP_RECORDING_STOP:
                    yy->api_id = API_ID_TRIGGER_MULTI_STOP;
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
            yy->api_group = API_GRP_MODE_VID;
            yy->api_id = API_ID_SET_RES_FR_FOV;
            break;

        default:
            // Unsupported Command ID
            gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
            return false;
    }

    gp_h4_finalize_yy_cmd(h4, payloadlen, yy);
    return true;
}
