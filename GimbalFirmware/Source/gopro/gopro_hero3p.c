#include "gopro_hero3p.h"
#include "gopro_hero3p_defs.h"
#include "gopro_hero_common.h"
#include "gopro_interface.h"
#include "gopro_mav_converters.h"
#include "helpers/macros.h"

#include <ctype.h>

#include "mavlink_interface/gimbal_mavlink.h"

// index of the start of photo info in the 'entire camera status' response
#define SE_RSP_PHOTO_INFO_IDX   20

static void gp_h3p_handle_command(gp_h3p_t *h3p, const gp_h3p_cmd_t *cmd, gp_h3p_rsp_t *rsp);
static void gp_h3p_handle_response(gp_h3p_t *h3p, const gp_h3p_rsp_t *rsp);
static void gp_h3p_sanitize_buf_len(uint8_t *buf);
static void gp_h3p_finalize_command(gp_h3p_cmd_t *c, uint8_t payloadlen);

void gp_h3p_init(gp_h3p_t *h3p)
{
    // ensure our little bit of cheating is OK,
    // verify GP_H3P_COMMAND_ENTIRE_CAM_STATUS is not used in mavlink interface
    STATIC_ASSERT(GP_H3P_COMMAND_ENTIRE_CAM_STATUS >= GOPRO_COMMAND_ENUM_END);

    h3p->gccb_version_queried = false;
    h3p->pending_recording_state = false;
    h3p->sd_card_inserted = false;
}

bool gp_h3p_handshake_complete(const gp_h3p_t *h3p)
{
    return h3p->gccb_version_queried;
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

bool gp_h3p_produce_get_request(uint8_t cmd_id, gp_h3p_cmd_t *c)
{
    /*
     * Convert the GetRequest from the CAN layer into a herobus command.
     * Return true if we produced data to send, false otherwise.
     */

    switch (cmd_id) {
        case GOPRO_COMMAND_SHUTTER:
            c->cmd1 = 's';
            c->cmd2 = 'h';
            // TODO: not sure if this command should be called directly, since don't want to be sending commands to GoPro while recording (spec document)
            break;

        case GOPRO_COMMAND_CAPTURE_MODE:
            c->cmd1 = 'c';
            c->cmd2 = 'm';
            break;

        case GOPRO_COMMAND_MODEL:
            c->cmd1 = 'c';
            c->cmd2 = 'v';
            break;

        case GOPRO_COMMAND_BATTERY:
            c->cmd1 = 'b';
            c->cmd2 = 'l';
            break;

        case GP_H3P_COMMAND_ENTIRE_CAM_STATUS:
            c->cmd1 = 's';
            c->cmd2 = 'e';
            break;

        default:
            // Unsupported Command ID
            gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
            return false;
    }

    gp_h3p_finalize_command(c, 0);
    return true;
}

bool gp_h3p_produce_set_request(gp_h3p_t *h3p, const gp_can_mav_set_req_t* request, gp_h3p_cmd_t *c)
{
    /*
     * Convert the SetRequest from the CAN layer into a herobus command.
     * Return true if we produced data to send, false otherwise.
     */

    uint8_t payloadlen;

    switch (request->mav.cmd_id) {
        case GOPRO_COMMAND_POWER:
            if(request->mav.value[0] == 0x00 && gp_get_power_status() == GP_POWER_ON) {
                c->cmd1 = 'P';
                c->cmd2 = 'W';
                payloadlen = 1;
                c->payload[0] = 0x00;
            } else {
                // gp_request_power_on() does not require a herobus transaction,
                // so mark it complete immediately
                gp_request_power_on();
                gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_SUCCESS);
                return false;
            }
            break;

        case GOPRO_COMMAND_CAPTURE_MODE: {
            c->cmd1 = 'C';
            c->cmd2 = 'M';
            bool ok;
            uint8_t mode = mav_to_h3p_cap_mode(request->mav.value[0], &ok);
            if (ok) {
                payloadlen = 1;
                c->payload[0] = mode;
                gp_pend_capture_mode(mode);
            } else {
                gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
                return false;
            }
        } break;

        case GOPRO_COMMAND_SHUTTER:
            if (!h3p->sd_card_inserted) {
                gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
                return false;
            }

            c->cmd1 = 'S';
            c->cmd2 = 'H';
            payloadlen = 1;
            c->payload[0] = request->mav.value[0];

            // don't change recording state for non-video capture modes since we don't have a way to find out when recording is finished by GoPro
            if (gp_capture_mode() == GOPRO_CAPTURE_MODE_VIDEO) {
                h3p->pending_recording_state = (request->mav.value[0] != 0);
            }
            break;

        case GOPRO_COMMAND_TIME: {
            c->cmd1 = 'T';
            c->cmd2 = 'M';
            payloadlen = 6;

            struct tm utc;
            time_t t = gp_time_from_mav(request);

            if (gmtime_r(&t, &utc) == NULL) {
                gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
                return false;
            }

            c->payload[0] = utc.tm_year - 100;  // year since 2000, rather than 1900
            c->payload[1] = utc.tm_mon + 1;     // (Jan = 0x01)
            c->payload[2] = utc.tm_mday;
            c->payload[3] = utc.tm_hour;
            c->payload[4] = utc.tm_min;
            c->payload[5] = utc.tm_sec;
        } break;

        default:
            // Unsupported Command ID
            gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
            return false;
    }

    gp_h3p_finalize_command(c, payloadlen);
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
        if (rsp->status == GP_CMD_STATUS_SUCCESS && gp_capture_mode() == GOPRO_CAPTURE_MODE_VIDEO) {
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
    }

    gp_set_transaction_result(mav_rsp.mav.value, mav_rsp_len, (GPCmdStatus)rsp->status);
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

void gp_h3p_finalize_command(gp_h3p_cmd_t *c, uint8_t payloadlen)
{
    /*
     * length field specifies length of packet, not including length byte.
     * always have at least 2 bytes for the cmd id.
     *
     * leave upper bit of length byte clear to indicate command originated
     * from the gimbal (not the GoPro)
     */

    c->len = 2 + payloadlen;
}

void gp_h3p_sanitize_buf_len(uint8_t *buf) { // TODO: inline?
    buf[0] &= 0x7f;     // remove most significant bit representing sender id (camera or BacPac)
}
