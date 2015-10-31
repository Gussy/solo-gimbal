#include "gopro_hero3p.h"
#include "gopro_hero3p_defs.h"
#include "gopro_hero_common.h"
#include "gopro_interface.h"
#include "gopro_mav_converters.h"
#include "gopro_helpers.h"
#include "helpers/macros.h"
#include "helpers/gmtime.h"

#include <ctype.h>

#include "mavlink_interface/gimbal_mavlink.h"

// index of the start of photo info in the 'entire camera status' response
#define SE_RSP_PHOTO_INFO_IDX   20

// number of bytes required for id in a gp_h3p_cmd_t
#define CMD_ID_NUM_BYTES        2

// track our state during multi msg commands (like GOPRO_COMMAND_VIDEO_SETTINGS)
enum H3P_MULTIMSG_STATE {
    H3_MULTIMSG_NONE,           // not performing a multi msg command
    H3_MULTIMSG_TV_MODE,        // ntsc/pal sent
    H3_MULTIMSG_RESOLUTION,     // resolution sent
    H3_MULTIMSG_FRAME_RATE,     // frame rate sent
    H3_MULTIMSG_FOV,            // field of view sent
    H3_MULTIMSG_FINAL = H3_MULTIMSG_FOV // last msg in the sequence
};

static void gp_h3p_set_transaction_result(gp_h3p_t *h3p, const uint8_t *resp_bytes, uint16_t len, GPCmdStatus status);
static void gp_h3p_handle_command(gp_h3p_t *h3p, const gp_h3p_cmd_t *cmd, gp_h3p_rsp_t *rsp);
static void gp_h3p_handle_response(gp_h3p_t *h3p, const gp_h3p_rsp_t *rsp);
static bool gp_h3p_handle_video_settings_rsp(gp_h3p_t *h3p, const gp_h3p_rsp_t *rsp);
static void gp_h3p_sanitize_buf_len(uint8_t *buf);

void gp_h3p_init(gp_h3p_t *h3p)
{
    // ensure our little bit of cheating is OK,
    // verify GP_H3P_COMMAND_ENTIRE_CAM_STATUS is not used in mavlink interface
    STATIC_ASSERT(GP_H3P_COMMAND_ENTIRE_CAM_STATUS >= GOPRO_COMMAND_ENUM_END);

    h3p->multi_msg_cmd.state = H3_MULTIMSG_NONE;
    h3p->gccb_version_queried = false;
    h3p->pending_recording_state = false;
    h3p->sd_card_inserted = false;
}

bool gp_h3p_handshake_complete(const gp_h3p_t *h3p)
{
    return h3p->gccb_version_queried;
}

static void cmd_init(gp_h3p_cmd_t *c, const char *cmd_id)
{
    /*
     * Set the cmd and initial length of this cmd.
     * cmd_id must be 2 bytes, but we do not verify here.
     * c->len specifies packet length minus the length byte,
     * so we init to a payload len of 0.
     * add payload bytes with cmd_add_byte()
     */

    c->len = CMD_ID_NUM_BYTES;
    c->cmd1 = cmd_id[0];
    c->cmd2 = cmd_id[1];
}

static void cmd_add_byte(gp_h3p_cmd_t *c, uint8_t b)
{
    /*
     * Add a byte to the payload of c
     * Expects c to have been init'd with cmd_init(),
     * such that c->len is valid.
     */

    c->payload[c->len - CMD_ID_NUM_BYTES] = b;
    c->len++;
}

bool gp_h3p_recognize_packet(const uint8_t *buf, uint16_t len)
{
    /*
     * Called when we don't yet know what kind of camera we're talking to.
     *
     * We only expect to see a version command in this state,
     * so just check for that.
     */

    bool dontcare;
    if (gp_h3p_rx_data_is_valid(buf, len, &dontcare)) {
        return buf[1] == 'v' && buf[2] == 's';
    }

    return false;
}

void gp_h3p_set_transaction_result(gp_h3p_t *h3p, const uint8_t *resp_bytes, uint16_t len, GPCmdStatus status)
{
    /*
     * wrapper around gp_set_transaction_result() to ensure
     * we always clear our multi msg state, in case we error
     * out before the entire command completes successfully.
     */

    h3p->multi_msg_cmd.state = H3_MULTIMSG_NONE;
    gp_set_transaction_result(resp_bytes, len, status);
}

bool gp_h3p_on_transaction_complete(gp_h3p_t *h3p, gp_h3p_pkt_t *p)
{
    /*
     * Called when an i2c transaction with the camera has completed.
     * Use this to drive the next step of any ongoing multi msg commands.
     */

    gp_h3p_cmd_t *c = &p->cmd;

    switch (h3p->multi_msg_cmd.state) {
    case H3_MULTIMSG_TV_MODE:
        // next step is resolution
        if (gp_transaction_direction() == GP_REQUEST_GET) {
            cmd_init(c, "vv");
            h3p->multi_msg_cmd.state = H3_MULTIMSG_RESOLUTION;
        } else {
            bool ok;
            uint8_t res = mav_to_h3p_res(h3p->multi_msg_cmd.payload[0], &ok);
            if (ok) {
                cmd_init(c, "VV");
                cmd_add_byte(c, res);
                h3p->multi_msg_cmd.state = H3_MULTIMSG_RESOLUTION;
            } else {
                gp_h3p_set_transaction_result(h3p, NULL, 0, GP_CMD_STATUS_FAILURE);
                return false;
            }
        }
        break;

    case H3_MULTIMSG_RESOLUTION:
        // next step is frame rate
        if (gp_transaction_direction() == GP_REQUEST_GET) {
            cmd_init(c, "fs");
            h3p->multi_msg_cmd.state = H3_MULTIMSG_FRAME_RATE;
        } else {
            bool ok;
            uint8_t rate = mav_to_h3p_framerate(h3p->multi_msg_cmd.payload[1], &ok);
            if (ok) {
                cmd_init(c, "FS");
                cmd_add_byte(c, rate);
                h3p->multi_msg_cmd.state = H3_MULTIMSG_FRAME_RATE;
            } else {
                gp_h3p_set_transaction_result(h3p, NULL, 0, GP_CMD_STATUS_FAILURE);
                return false;
            }
        }
        break;

    case H3_MULTIMSG_FRAME_RATE:
        // next step is field of view
        if (gp_transaction_direction() == GP_REQUEST_GET) {
            cmd_init(c, "fv");
        } else {
            cmd_init(c, "FV");
            // fov values do not require mavlink<->herobus conversion
            cmd_add_byte(c, h3p->multi_msg_cmd.payload[2]);
        }
        h3p->multi_msg_cmd.state = H3_MULTIMSG_FOV;
        break;

    default:
        return false;
    }

    return true;
}

bool gp_h3p_produce_get_request(gp_h3p_t *h3p, uint8_t cmd_id, gp_h3p_cmd_t *c)
{
    /*
     * Convert the GetRequest from the CAN layer into a herobus command.
     * Return true if we produced data to send, false otherwise.
     */

    switch (cmd_id) {
        case GOPRO_COMMAND_SHUTTER:
            cmd_init(c, "sh");
            // TODO: not sure if this command should be called directly, since don't want to be sending commands to GoPro while recording (spec document)
            break;

        case GOPRO_COMMAND_CAPTURE_MODE:
            cmd_init(c, "cm");
            break;

        case GOPRO_COMMAND_MODEL:
            cmd_init(c, "cv");
            break;

        case GOPRO_COMMAND_BATTERY:
            cmd_init(c, "bl");
            break;

        case GP_H3P_COMMAND_ENTIRE_CAM_STATUS:
            cmd_init(c, "se");
            break;

        case GOPRO_COMMAND_TIME:
            cmd_init(c, "tm");
            break;

        case GOPRO_COMMAND_VIDEO_SETTINGS:
            // video settings is a multi msg command, first msg is tv mode
            cmd_init(c, "vm");
            h3p->multi_msg_cmd.payload[3] = 0;  // init flags to 0
            h3p->multi_msg_cmd.state = H3_MULTIMSG_TV_MODE;
            break;

        case GOPRO_COMMAND_LOW_LIGHT:
            cmd_init(c, "lw");
            break;

        case GOPRO_COMMAND_PHOTO_RESOLUTION:
            cmd_init(c, "pr");
            break;

        case GOPRO_COMMAND_PROTUNE:
            cmd_init(c, "pt");
            break;

        default:
            // Unsupported Command ID
            gp_h3p_set_transaction_result(h3p, NULL, 0, GP_CMD_STATUS_FAILURE);
            return false;
    }

    return true;
}

bool gp_h3p_produce_set_request(gp_h3p_t *h3p, const gp_can_mav_set_req_t* request, gp_h3p_cmd_t *c)
{
    /*
     * Convert the SetRequest from the CAN layer into a herobus command.
     * Return true if we produced data to send, false otherwise.
     */

    switch (request->mav.cmd_id) {
        case GOPRO_COMMAND_POWER:
            if(request->mav.value[0] == 0x00 && gp_get_power_status() == GP_POWER_ON) {
                cmd_init(c, "PW");
                cmd_add_byte(c, 0x00);
            } else {
                // gp_request_power_on() does not require a herobus transaction,
                // so mark it complete immediately
                gp_request_power_on();
                gp_h3p_set_transaction_result(h3p, NULL, 0, GP_CMD_STATUS_SUCCESS);
                return false;
            }
            break;

        case GOPRO_COMMAND_CAPTURE_MODE: {
            bool ok;
            uint8_t mode = mav_to_h3p_cap_mode(request->mav.value[0], &ok);
            if (ok) {
                cmd_init(c, "CM");
                cmd_add_byte(c, mode);
                gp_pend_capture_mode(mode);
            } else {
                gp_h3p_set_transaction_result(h3p, NULL, 0, GP_CMD_STATUS_FAILURE);
                return false;
            }
        } break;

        case GOPRO_COMMAND_SHUTTER:
            if (!h3p->sd_card_inserted) {
                gp_h3p_set_transaction_result(h3p, NULL, 0, GP_CMD_STATUS_FAILURE);
                return false;
            }

            cmd_init(c, "SH");
            cmd_add_byte(c, request->mav.value[0]);

            // don't change recording state for non-video capture modes since we don't have a way to find out when recording is finished by GoPro
            if (gp_capture_mode() == GOPRO_CAPTURE_MODE_VIDEO) {
                h3p->pending_recording_state = (request->mav.value[0] != 0);
            }
            break;

        case GOPRO_COMMAND_TIME: {
            struct tm utc;
            time_t t = gp_time_from_mav(request);

            if (gmtime_r(&t, &utc) == NULL) {
                gp_h3p_set_transaction_result(h3p, NULL, 0, GP_CMD_STATUS_FAILURE);
                return false;
            }

            cmd_init(c, "TM");
            cmd_add_byte(c, utc.tm_year - 100);  // year since 2000, rather than 1900
            cmd_add_byte(c, utc.tm_mon + 1);     // (Jan = 0x01)
            cmd_add_byte(c, utc.tm_mday);
            cmd_add_byte(c, utc.tm_hour);
            cmd_add_byte(c, utc.tm_min);
            cmd_add_byte(c, utc.tm_sec);
        } break;

        case GOPRO_COMMAND_VIDEO_SETTINGS: {
            // video settings is a multi msg command, first msg is tv mode
            // store the payload so we can continue sending subsequent messages in
            memcpy(h3p->multi_msg_cmd.payload, request->mav.value, sizeof request->mav.value);
            h3p->multi_msg_cmd.state = H3_MULTIMSG_TV_MODE;

            cmd_init(c, "VM");
            if (h3p->multi_msg_cmd.payload[3] & GOPRO_VIDEO_SETTINGS_TV_MODE) {
                cmd_add_byte(c, H3P_TV_PAL);
            } else {
                cmd_add_byte(c, H3P_TV_NTSC);
            }
        } break;

        case GOPRO_COMMAND_LOW_LIGHT:
            cmd_init(c, "LW");
            cmd_add_byte(c, request->mav.value[0] ? 1 : 0);
            break;

        case GOPRO_COMMAND_PHOTO_RESOLUTION: {
            bool ok;
            uint8_t res = mav_to_h3p_photo_res(request->mav.value[0], &ok);
            if (ok) {
                cmd_init(c, "PR");
                cmd_add_byte(c, res);
            }
        } break;

        case GOPRO_COMMAND_PROTUNE:
            cmd_init(c, "PT");
            cmd_add_byte(c, request->mav.value[0] ? 1 : 0);
            break;

        default:
            // Unsupported Command ID
            gp_h3p_set_transaction_result(h3p, NULL, 0, GP_CMD_STATUS_FAILURE);
            return false;
    }

    return true;
}

bool gp_h3p_handle_rx(gp_h3p_t *h3p, gp_h3p_pkt_t *p, bool from_camera, gp_h3p_rsp_t *rsp)
{
    /*
     * Handle incoming i2c data from the camera.
     *
     * Return true if we generated a response that
     * should be written back to the camera.
     */

    gp_h3p_sanitize_buf_len(p->bytes);

    if (from_camera) {
        gp_h3p_handle_command(h3p, &p->cmd, rsp);
        return true;
    }

    gp_h3p_handle_response(h3p, &p->rsp);
    return false;
}

void gp_h3p_handle_command(gp_h3p_t *h3p, const gp_h3p_cmd_t *cmd, gp_h3p_rsp_t *rsp)
{
    /*
     * A validated command has been received from the camera.
     * Write our response into 'rsp'.
     *
     * First make sure the command is one we support.  Per the GoPro spec,
     * we only have to respond to the "vs" command which queries
     * the version of the protocol we support.
     *
     * For any other command from the GoPro, return an error response
     */

    if ((cmd->cmd1 == 'v') && (cmd->cmd2 == 's')) {
        // Preload the response buffer with the command response.  This will be transmitted in the ISR
        rsp->len = 2;
        rsp->status = GP_CMD_STATUS_SUCCESS;
        rsp->payload[0] = GP_PROTOCOL_VERSION;
        h3p->gccb_version_queried = true;
        return;
    }

    // Preload the response buffer with an error response, since we don't support the command we
    // were sent.  This will be transmitted in the ISR
    rsp->len = 1;
    rsp->status = GP_CMD_STATUS_FAILURE;
}

void gp_h3p_handle_response(gp_h3p_t *h3p, const gp_h3p_rsp_t *rsp)
{
    /*
     * Process a response to one of our commands.
     *
     * Payload values being passed back via mavlink
     * must be converted from HeroBus values to mavlink values.
     */

    if (rsp->status != GP_CMD_STATUS_SUCCESS) {
        gp_h3p_set_transaction_result(h3p, NULL, 0, GP_CMD_STATUS_FAILURE);
        return;
    }

    uint8_t mav_rsp_len = 0;
    gp_can_mav_get_rsp_t mav_rsp;   // collect mavlink-translated payload vals

    // Special Handling of responses
    switch (gp_transaction_cmd()) {
    case GOPRO_COMMAND_BATTERY:
        mav_rsp.mav.value[0] = rsp->payload[0];
        mav_rsp_len = 1;
        break;

    case GOPRO_COMMAND_CAPTURE_MODE:
        if (gp_transaction_direction() == GP_REQUEST_GET) {
            bool ok;
            uint8_t mode = h3p_to_mav_cap_mode(rsp->payload[0], &ok);
            if (!ok) { mode = GOPRO_CAPTURE_MODE_UNKNOWN; }

            gp_set_capture_mode(mode);

            mav_rsp.mav.value[0] = mode;
            mav_rsp_len = 1;

        } else if (gp_transaction_direction() == GP_REQUEST_SET) {
            gp_latch_pending_capture_mode();        // Set request acknowledged, update capture mode state with pending capture mode received via MAVLink/CAN
        }

    case GOPRO_COMMAND_SHUTTER:
        /*
         * This alone is not enough to verify the recording state is correct,
         * as the camera responds with success status even if the SD card is not inserted.
         * Need to check for SD card presence before sending shutter trigger cmd.
         */
        if (gp_capture_mode() == GOPRO_CAPTURE_MODE_VIDEO) {
            gp_set_recording_state(h3p->pending_recording_state);
        }
        break;

    case GP_H3P_COMMAND_ENTIRE_CAM_STATUS: {
        /*
         * Try to detect whether the SD card is inserted, so we can respond
         * reasonably to shutter commands, as described above.
         *
         * Test whether the 2 fields of photo info are empty.
         */
        bool ok;
        uint8_t mode = h3p_to_mav_cap_mode(rsp->payload[0], &ok);
        gp_set_capture_mode(ok ? mode : GOPRO_CAPTURE_MODE_UNKNOWN);

        const uint8_t empty[] = { 0xff, 0xff, 0xff, 0xff };
        h3p->sd_card_inserted = memcmp(&rsp->payload[SE_RSP_PHOTO_INFO_IDX], empty, sizeof empty) != 0;
    } break;

    case GOPRO_COMMAND_TIME:
        if (gp_transaction_direction() == GP_REQUEST_GET) {
            struct tm ti;

            ti.tm_year = rsp->payload[0] + 2000 - 1900; // years since 2000
            ti.tm_mon = rsp->payload[1] - 1;            // (Jan = 0x01)
            ti.tm_mday = rsp->payload[2];
            ti.tm_hour = rsp->payload[3];
            ti.tm_min = rsp->payload[4];
            ti.tm_sec = rsp->payload[5];

            gp_time_to_mav(&mav_rsp, &ti);
            mav_rsp_len = 4;
        }
        break;

    case GOPRO_COMMAND_VIDEO_SETTINGS:
        if (!gp_h3p_handle_video_settings_rsp(h3p, rsp)) {
            return;
        }

        if (gp_transaction_direction() == GP_REQUEST_GET) {
            memcpy(mav_rsp.mav.value, h3p->multi_msg_cmd.payload, sizeof mav_rsp.mav.value);
            mav_rsp_len = 4;
        }
        break;

    case GOPRO_COMMAND_LOW_LIGHT:
        if (gp_transaction_direction() == GP_REQUEST_GET) {
            mav_rsp.mav.value[0] = rsp->payload[0];
            mav_rsp_len = 1;
        }
        break;

    case GOPRO_COMMAND_PHOTO_RESOLUTION:
        if (gp_transaction_direction() == GP_REQUEST_GET) {
            bool ok;
            uint8_t res = h3p_to_mav_photo_res(rsp->payload[0], &ok);
            if (ok) {
                mav_rsp.mav.value[0] = res;
                mav_rsp_len = 1;
            }
        }
        break;

    case GOPRO_COMMAND_PROTUNE:
        if (gp_transaction_direction() == GP_REQUEST_GET) {
            mav_rsp.mav.value[0] = (rsp->payload[0] == 1) ? 1 : 0;
            mav_rsp_len = 1;
        }
    }

    gp_h3p_set_transaction_result(h3p, mav_rsp.mav.value, mav_rsp_len, GP_CMD_STATUS_SUCCESS);
}

bool gp_h3p_handle_video_settings_rsp(gp_h3p_t *h3p, const gp_h3p_rsp_t *rsp)
{
    /*
     * Helper to handle video settings responses, only called if rsp->status is successful.
     *
     * Don't mark the command as complete until we get a response to the final msg.
     *
     * Return whether video settings command is complete.
     */

    if (gp_transaction_direction() == GP_REQUEST_GET) {
        // get responses, store payloads in h3p->multi_msg_cmd.payload
        bool ok;
        uint8_t val;
        switch (h3p->multi_msg_cmd.state) {
        case H3_MULTIMSG_TV_MODE:
            if (rsp->payload[0] == H3P_TV_PAL) {
                h3p->multi_msg_cmd.payload[3] |= GOPRO_VIDEO_SETTINGS_TV_MODE;
            }
            break;

        case H3_MULTIMSG_RESOLUTION:
            val = h3p_to_mav_res(rsp->payload[0], &ok);
            if (ok) {
                h3p->multi_msg_cmd.payload[0] = val;
            }
            break;

        case H3_MULTIMSG_FRAME_RATE:
            val = h3p_to_mav_framerate(rsp->payload[0], &ok);
            if (ok) {
                h3p->multi_msg_cmd.payload[1] = val;
            }
            break;

        case H3_MULTIMSG_FOV:
            // field of view doesn't require mavlink translation
            h3p->multi_msg_cmd.payload[2] = rsp->payload[0];

            h3p->multi_msg_cmd.state = H3_MULTIMSG_NONE;
            break;
        }
    } else {
        if (h3p->multi_msg_cmd.state == H3_MULTIMSG_FINAL) {
            h3p->multi_msg_cmd.state = H3_MULTIMSG_NONE;
        }
    }

    return (h3p->multi_msg_cmd.state == H3_MULTIMSG_NONE);
}

bool gp_h3p_rx_data_is_valid(const uint8_t *buf, uint16_t len, bool *from_camera)
{
    /*
     * Called when an i2c rx transaction has completed successfully,
     * to determine if the received data is formatted correctly.
     *
     * Ensure the advertised len matches what we actually recevied.
     */

    uint16_t buf_len;

    // must include at least len & status, in the case of a response
    if (len < 2) {
        return false;
    }

    // first byte is the length of the received data,
    // top bit signals the cmd originator, 1 == camera, 0 == backpack
    *from_camera = buf[0] & (1 << 7);
    buf_len = buf[0] & 0x7f;

    if (buf_len != len - 1) {
        return false;
    }

    return true;
}

void gp_h3p_sanitize_buf_len(uint8_t *buf) { // TODO: inline?
    buf[0] &= 0x7f;     // remove most significant bit representing sender id (camera or BacPac)
}
