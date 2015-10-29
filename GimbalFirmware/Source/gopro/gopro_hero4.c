#include "gopro_hero4.h"
#include "gopro_hero4_defs.h"
#include "gopro_interface.h"
#include "gopro_mav_converters.h"
#include "gopro_helpers.h"
#include "helpers/gmtime.h"
#include "mavlink_interface/gimbal_mavlink.h"

#include <string.h>

// track our state during multi msg commands (like GOPRO_COMMAND_VIDEO_SETTINGS)
enum H4_MULTIMSG_STATE {
    H4_MULTIMSG_NONE,           // not performing a multi msg command
    H4_MULTIMSG_TV_MODE,        // ntsc/pal sent
    H4_MULTIMSG_VID_SETTINGS,   // resolution/frame rate/fov sent
    H4_MULTIMSG_FINAL = H4_MULTIMSG_VID_SETTINGS // last msg in the sequence
};

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

    h4->multi_msg_cmd.state = H4_MULTIMSG_NONE;

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

static bool gp_h4_produce_set_video_settings(gp_h4_t *h4, gp_h4_yy_cmd_t *yy)
{
    /*
     * Helper to pack yy with a video settings set command.
     */

    bool ok;

    yy->payload[0] = mav_to_h4_resolution(h4->multi_msg_cmd.payload[0], &ok);
    if (!ok) {
        return false;
    }

    yy->payload[1] = mav_to_h4_framerate(h4->multi_msg_cmd.payload[1], &ok);
    if (!ok) {
        return false;
    }

    yy->payload[2] = h4->multi_msg_cmd.payload[2];

    yy->api_group = API_GRP_MODE_VID;
    yy->api_id = API_ID_SET_RES_FR_FOV;
    gp_h4_finalize_yy_cmd(h4, 3, yy);
    return true;
}

bool gp_h4_on_txn_complete(gp_h4_t *h4, gp_h4_pkt_t *p)
{
    // must kick off final step of handshake sequence on hero4
    if (!gp_h4_handshake_complete(h4)) {
        return gp_h4_finish_handshake(h4, p);
    }

    switch (h4->multi_msg_cmd.state) {
    case H4_MULTIMSG_TV_MODE:
        // vid settings is next
        if (gp_transaction_direction() == GP_REQUEST_SET) {
            if (gp_h4_produce_set_video_settings(h4, &p->yy_cmd)) {
                h4->multi_msg_cmd.state = H4_MULTIMSG_VID_SETTINGS;
                return true;
            }
            gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
        } else {
            gp_h4_yy_cmd_t *yy = &p->yy_cmd;
            yy->api_group = API_GRP_MODE_VID;
            yy->api_id = API_ID_GET_RES_FR_FOV;
            gp_h4_finalize_yy_cmd(h4, 0, yy);
            h4->multi_msg_cmd.state = H4_MULTIMSG_VID_SETTINGS;
            return true;
        }
    }

    return false;
}

bool gp_h4_recognize_packet(const uint8_t *buf, uint16_t len)
{
    /*
     * Called when we don't yet know what kind of camera we're talking to.
     *
     * We only expect to see handshake packets in this state,
     * so make sure it's long enough and is a ZZ packet.
     */

    return gp_h4_rx_data_is_valid(buf, len) && len >= 6 && buf[1] == 'Z' && buf[2] == 'Z';
}

bool gp_h4_rx_data_is_valid(const uint8_t *buf, uint16_t len)
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
        return err;
    }

    uint8_t mav_rsp_len = 0;
    gp_can_mav_get_rsp_t mav_rsp;   // collect mavlink-translated payload vals

    // capture mode
    if (rsp->api_group == API_GRP_MODE_CAM) {
        if (rsp->api_id == API_ID_GET_CAM_MAIN_MODE) {
            bool ok;
            uint8_t mode = h4_to_mav_cap_mode(rsp->payload[0], &ok);
            if (!ok) { mode = GOPRO_CAPTURE_MODE_UNKNOWN; }

            gp_set_capture_mode(mode);

            mav_rsp.mav.value[0] = mode;
            mav_rsp_len = 1;
        } else if (rsp->api_id == API_ID_SET_CAM_MAIN_MODE) {
            gp_latch_pending_capture_mode();
        }
    }
    // battery level
    else if (rsp->api_group == API_GRP_CAM_SETTINGS && rsp->api_id == API_ID_GET_BATTERY_LVL) {
        if (len == 1) {
            mav_rsp.mav.value[0] = rsp->payload[0];
            mav_rsp_len = 1;
        }
    }
    // get camera time
    else if (rsp->api_group == API_GRP_PLAYBACK_MODE && rsp->api_id == API_ID_GET_CAM_TIME) {

        struct tm ti;
        ti.tm_year = ((rsp->payload[0] << 8) | rsp->payload[1]) - 1900;
        ti.tm_mon = rsp->payload[2] - 1;
        ti.tm_mday = rsp->payload[3];
        ti.tm_hour = rsp->payload[4];
        ti.tm_min = rsp->payload[5];
        ti.tm_sec = rsp->payload[6];

        gp_time_to_mav(&mav_rsp, &ti);
        mav_rsp_len = 4;
    }
    // trigger shutter
    else if (rsp->api_group == API_GRP_MODE_VID &&
       (rsp->api_id == API_ID_TRIGGER_VID_START || rsp->api_id == API_ID_TRIGGER_VID_STOP))
    {
        gp_set_recording_state(h4->pending_recording_state);
    }
    // tv mode
    else if (rsp->api_group == API_GRP_PLAYBACK_MODE) {
        // if this is being sent as part of a multi msg,
        // ensure we don't complete the transaction until the final msg is completed.
        if (rsp->api_id == API_ID_SET_NTSC_PAL) {
            return err;
        }

        if (rsp->api_id == API_ID_GET_NTSC_PAL) {
            if (rsp->payload[0] == H4_TV_PAL) {
                h4->multi_msg_cmd.payload[3] |= GOPRO_VIDEO_SETTINGS_TV_MODE;
            }
            return err;
        }
    }
    // vid settings
    else if (rsp->api_group == API_GRP_MODE_VID) {
        if (rsp->api_id == API_ID_SET_RES_FR_FOV) {
            h4->multi_msg_cmd.state  = H4_MULTIMSG_NONE;

        } else if (rsp->api_id == API_ID_GET_RES_FR_FOV) {
            bool ok;
            mav_rsp.mav.value[0] = h4_to_mav_resolution(rsp->payload[0], &ok);
            mav_rsp.mav.value[1] = h4_to_mav_framerate(rsp->payload[1], &ok);
            mav_rsp.mav.value[2] = rsp->payload[2]; // field of view does not require conversion
            mav_rsp.mav.value[3] = h4->multi_msg_cmd.payload[3];
            mav_rsp_len = 4;
            h4->multi_msg_cmd.state  = H4_MULTIMSG_NONE;
        }
    }

    gp_set_transaction_result(mav_rsp.mav.value, mav_rsp_len, GP_CMD_STATUS_SUCCESS);
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
    memcpy(r->payload, c->payload, (c->len - GP_H4_HDR_LEN));

    return true;
}

bool gp_h4_produce_get_request(gp_h4_t *h4, uint8_t cmd_id, gp_h4_pkt_t *p)
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

    case GOPRO_COMMAND_TIME:
        yy->api_group = API_GRP_PLAYBACK_MODE;
        yy->api_id = API_ID_GET_CAM_TIME;
        break;

    case GOPRO_COMMAND_VIDEO_SETTINGS:
        yy->api_group = API_GRP_PLAYBACK_MODE;
        yy->api_id = API_ID_GET_NTSC_PAL;

        h4->multi_msg_cmd.payload[3] = 0;  // init flags to 0
        h4->multi_msg_cmd.state = H4_MULTIMSG_TV_MODE;
        break;

    default:
        // Unsupported Command ID
        gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
        return false;
    }

    gp_h4_finalize_yy_cmd(h4, payloadlen, yy);
    return true;
}

bool gp_h4_produce_set_request(gp_h4_t *h4, const gp_can_mav_set_req_t* request, gp_h4_pkt_t *p)
{
    /*
     * A SET request has been received via the CAN interface,
     * forward it to the camera.
     */

    gp_h4_yy_cmd_t *yy = &p->yy_cmd;

    uint16_t payloadlen = 0;

    switch (request->mav.cmd_id) {
        case GOPRO_COMMAND_POWER:
            if(request->mav.value[0] == 0x00 && gp_get_power_status() == GP_POWER_ON) {
                yy->api_group = API_GRP_CAM_SETTINGS;
                yy->api_id = API_ID_POWER_OFF;
                yy->payload[0] = request->mav.value[0];
                payloadlen = 1;
            } else {
                // no supported command to power on.
                // have tried gp_request_power_on(), which is implemented based on hero3 docs,
                // but does not appear to power up the camera.
                gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
                return false;
            }
            break;

        case GOPRO_COMMAND_CAPTURE_MODE: {
            yy->api_group = API_GRP_MODE_CAM;
            yy->api_id = API_ID_SET_CAM_MAIN_MODE;
            bool ok;
            uint8_t mode = mav_to_h4_cap_mode(request->mav.value[0], &ok);
            if (ok) {
                yy->payload[0] = mode;
                payloadlen = 1;
                gp_pend_capture_mode(mode);
            }else {
                gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
                return false;
            }
        } break;

        case GOPRO_COMMAND_SHUTTER:
            switch (gp_capture_mode()) {
            case GOPRO_CAPTURE_MODE_VIDEO:
                yy->api_group = API_GRP_MODE_VID;
                if (request->mav.value[0]) {
                    yy->api_id = API_ID_TRIGGER_VID_START;
                    h4->pending_recording_state = true;
                } else {
                    yy->api_id = API_ID_TRIGGER_VID_STOP;
                    h4->pending_recording_state = false;
                }
                break;

            case GOPRO_CAPTURE_MODE_PHOTO:
                yy->api_group = API_GRP_MODE_PHOTO;
                if (request->mav.value[0]) {
                    yy->api_id = API_ID_TRIGGER_PHOTO_START;
                } else {
                    yy->api_id = API_ID_TRIGGER_PHOTO_STOP;
                }
                break;

            case GOPRO_CAPTURE_MODE_BURST:
                yy->api_group = API_GRP_MODE_MULTISHOT;
                if (request->mav.value[0]) {
                    yy->api_id = API_ID_TRIGGER_MULTI_START;
                } else {
                    yy->api_id = API_ID_TRIGGER_MULTI_STOP;
                }
                break;

            default:
                // unknown capture mode
                gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
                return false;
            }

            payloadlen = 0;
            break;

        case GOPRO_COMMAND_TIME: {
            yy->api_group = API_GRP_PLAYBACK_MODE;
            yy->api_id = API_ID_SET_CAM_TIME;
            payloadlen = 7;

            struct tm utc;
            time_t t = gp_time_from_mav(request);

            if (gmtime_r(&t, &utc) == NULL) {
                gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
                return false;
            }

            uint16_t year = utc.tm_year + 1900;
            yy->payload[0] = (year >> 8) & 0xff;
            yy->payload[1] = year & 0xff;
            yy->payload[2] = utc.tm_mon + 1;    // not specified, but appears to be 1-based
            yy->payload[3] = utc.tm_mday;
            yy->payload[4] = utc.tm_hour;
            yy->payload[5] = utc.tm_min;
            yy->payload[6] = utc.tm_sec;
        } break;

        case GOPRO_COMMAND_VIDEO_SETTINGS:
            // video settings is a multi msg command, first msg is tv mode
            // store the payload so we can continue sending subsequent messages in
            memcpy(h4->multi_msg_cmd.payload, request->mav.value, sizeof request->mav.value);
            h4->multi_msg_cmd.state = H4_MULTIMSG_TV_MODE;

            yy->api_group = API_GRP_PLAYBACK_MODE;
            yy->api_id = API_ID_SET_NTSC_PAL;
            if (h4->multi_msg_cmd.payload[3] & GOPRO_VIDEO_SETTINGS_TV_MODE) {
                yy->payload[0] = H4_TV_PAL;
            } else {
                yy->payload[0] = H4_TV_NTSC;
            }
            payloadlen = 1;
            break;

        default:
            // Unsupported Command ID
            gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
            return false;
    }

    gp_h4_finalize_yy_cmd(h4, payloadlen, yy);
    return true;
}
