#include "gopro_hero3p.h"
#include "gopro_hero_common.h"
#include "gopro_interface.h"

#include <ctype.h>

// Include for GOPRO_COMMAND enum
#include "mavlink_interface/mavlink_gimbal_interface.h"

static void gp_h3p_handle_command(gp_h3p_t *h3p, const gp_h3p_cmd_t *cmd, gp_h3p_rsp_t *rsp);
static void gp_h3p_handle_response(gp_h3p_t *h3p, const gp_h3p_rsp_t *rsp);
static void gp_h3p_sanitize_buf_len(uint16_t *buf);
static void gp_h3p_finalize_command(gp_h3p_cmd_t *c);

void gp_h3p_init(gp_h3p_t *h3p)
{
    h3p->gccb_version_queried = false;
    h3p->pending_recording_state = false;
}

bool gp_h3p_handshake_complete(const gp_h3p_t *h3p)
{
    return h3p->gccb_version_queried;
}

bool gp_h3p_recognize_packet(const uint16_t *buf, uint16_t len)
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

static bool gp_h3p_cmd_has_param(const gp_h3p_cmd_t *c)
{
    /*
     * For the most part, commands have a parameter, queries never do.
     * Commands are 2 uppercase characters, queries are 2 lowercase characters
     */

    if (islower(c->cmd1)) {
        return false;
    }

    // Need to special case 'DL', 'DA', and 'FO' commands, since they don't have parameters
    if ((c->cmd1 == 'F') && (c->cmd2 == 'O')) {
        return false;
    }

    if (c->cmd1 == 'D' && (c->cmd2 == 'L' || c->cmd2 == 'A')) {
        return false;
    }

    return true;
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

        case GOPRO_COMMAND_RESOLUTION:
            c->cmd1 = 'v';
            c->cmd2 = 'v';
            break;

        case GOPRO_COMMAND_FRAME_RATE:
            c->cmd1 = 'f';
            c->cmd2 = 's';
            break;

        case GOPRO_COMMAND_FIELD_OF_VIEW:
            c->cmd1 = 'f';
            c->cmd2 = 'v';
            break;

        default:
            // Unsupported Command ID
            gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
            return false;
    }

    gp_h3p_finalize_command(c);
    return true;
}

bool gp_h3p_produce_set_request(gp_h3p_t *h3p, const GPSetRequest* request, gp_h3p_cmd_t *c)
{
    /*
     * Convert the SetRequest from the CAN layer into a herobus command.
     * Return true if we produced data to send, false otherwise.
     */

    switch (request->cmd_id) {
        case GOPRO_COMMAND_POWER:
            if(request->value == 0x00 && gp_get_power_status() == GP_POWER_ON) {
                c->cmd1 = 'P';
                c->cmd2 = 'W';
                c->payload[0] = 0x00;
            } else {
                // gp_request_power_on() does not require a herobus transaction,
                // so mark it complete immediately
                gp_request_power_on();
                gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_SUCCESS);
                return false;
            }
            break;

        case GOPRO_COMMAND_CAPTURE_MODE:
            c->cmd1 = 'C';
            c->cmd2 = 'M';
            c->payload[0] = request->value;
            gp_pend_capture_mode(request->value);
            break;

        case GOPRO_COMMAND_SHUTTER:
            c->cmd1 = 'S';
            c->cmd2 = 'H';
            c->payload[0] = request->value;

            // don't change recording state for non-video capture modes since we don't have a way to find out when recording is finished by GoPro
            if (gp_capture_mode() == GP_CAPTURE_MODE_VIDEO) {
                switch (request->value) {
                case GP_RECORDING_START:
                case GP_RECORDING_STOP:
                    h3p->pending_recording_state = (request->value == GP_RECORDING_START);
                    break;

                default:
                    gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
                    return false;
                }
            }
            break;

        /* Unsupported commands */
        case GOPRO_COMMAND_RESOLUTION:
            c->cmd1 = 'V';
            c->cmd2 = 'V';
            c->payload[0] = request->value;
            break;

        case GOPRO_COMMAND_FRAME_RATE:
            c->cmd1 = 'F';
            c->cmd2 = 'S';
            c->payload[0] = request->value;
            break;

        case GOPRO_COMMAND_FIELD_OF_VIEW:
            c->cmd1 = 'F';
            c->cmd2 = 'V';
            c->payload[0] = request->value;
            break;
        /* End of unsupported commands */

        default:
            // Unsupported Command ID
            gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
            return false;
    }

    gp_h3p_finalize_command(c);
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
     */

    // Special Handling of responses
    switch (gp_transaction_cmd()) {
    case GOPRO_COMMAND_CAPTURE_MODE:
        if (gp_transaction_direction() == GP_REQUEST_GET) {
            gp_set_capture_mode(rsp->payload[0]);   // Set capture mode state with capture mode received from GoPro
        } else if (gp_transaction_direction() == GP_REQUEST_SET) {
            gp_latch_pending_capture_mode();        // Set request acknowledged, update capture mode state with pending capture mode received via MAVLink/CAN
        }

    case GOPRO_COMMAND_SHUTTER:
        /*
         * This alone is not enough to verify the recording state is correct,
         * as the camera responds with success status even if the SD card is not inserted.
         * Need to check for SD card presence before sending shutter trigger cmd.
         */
        if (rsp->status == GP_CMD_STATUS_SUCCESS && gp_capture_mode() == GP_CAPTURE_MODE_VIDEO) {
            gp_set_recording_state(h3p->pending_recording_state);
        }
        break;
    }

    gp_set_transaction_result(&rsp->payload[0], 1, (GPCmdStatus)rsp->status);
}


bool gp_h3p_rx_data_is_valid(const uint16_t *buf, uint16_t len, bool *from_camera)
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

void gp_h3p_finalize_command(gp_h3p_cmd_t *c)
{
    // first byte is len, upper bit clear to indicate command originated from the gimbal (not the GoPro)
    if (gp_h3p_cmd_has_param(c)) {
        c->len = 0x3;
    } else {
        c->len = 0x2;
    }
}

void gp_h3p_sanitize_buf_len(uint16_t *buf) { // TODO: inline?
    buf[0] &= 0x7f;     // remove most significant bit representing sender id (camera or BacPac)
}

