#include "gopro_hero3p.h"
#include "gopro_hero_common.h"
#include "gopro_interface.h"

#include <ctype.h>

// Include for GOPRO_COMMAND enum
#include "mavlink_interface/mavlink_gimbal_interface.h" // TODO: get rid of this after replacing GOPRO_COMMAND

static void gp_h3p_handle_command(gp_h3p_t *h3p, const uint16_t *cmdbuf, uint16_t *txbuf);
static void gp_h3p_handle_response(const uint16_t *respbuf);
static void gp_h3p_sanitize_buf_len(uint16_t *buf);

void gp_h3p_init(gp_h3p_t *h3p)
{
    h3p->gccb_version_queried = false;
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

bool gp_h3p_request_power_off()
{
    GPCmd cmd;
    cmd.cmd[0] = 'P';
    cmd.cmd[1] = 'W';
    cmd.cmd_parm = 0x00;
    gp_h3p_send_command(&cmd);
    return true;
}

static bool gp_h3p_cmd_has_param(const GPCmd* c)
{
    /*
     * For the most part, commands have a parameter, queries never do.
     * Commands are 2 uppercase characters, queries are 2 lowercase characters
     */

    if (islower(c->cmd[0])) {
        return false;
    }

    // Need to special case 'DL', 'DA', and 'FO' commands, since they don't have parameters
    if ((c->cmd[0] == 'F') && (c->cmd[1] == 'O')) {
        return false;
    }

    if (c->cmd[0] == 'D' && (c->cmd[1] == 'L' || c->cmd[1] == 'A')) {
        return false;
    }

    return true;
}

// TODO: return type int change to bool? to match other functions
int gp_h3p_forward_get_request(Uint8 cmd_id)
{
    GPCmd cmd;

    switch (cmd_id) {
        case GOPRO_COMMAND_SHUTTER:
            cmd.cmd[0] = 's';
            cmd.cmd[1] = 'h';
        break;

        case GOPRO_COMMAND_CAPTURE_MODE:
            cmd.cmd[0] = 'c';
            cmd.cmd[1] = 'm';
        break;

        case GOPRO_COMMAND_MODEL:
            cmd.cmd[0] = 'c';
            cmd.cmd[1] = 'v';
        break;

        case GOPRO_COMMAND_BATTERY:
            cmd.cmd[0] = 'b';
            cmd.cmd[1] = 'l';
        break;

        default:
            // Unsupported Command ID
            gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
            return -1;
    }

    gp_h3p_send_command(&cmd);
    return 0;
}

int gp_h3p_forward_set_request(const GPSetRequest* request)
{
    GPCmd cmd;

    switch (request->cmd_id) {
        case GOPRO_COMMAND_POWER:
            if(request->value == 0x00 && gp_get_power_status() == GP_POWER_ON) {
                gp_h3p_request_power_off();
            } else {
                gp_request_power_on();
            }
            break;

        case GOPRO_COMMAND_CAPTURE_MODE:
            cmd.cmd[0] = 'C';
            cmd.cmd[1] = 'M';
            cmd.cmd_parm = request->value;
            break;

        case GOPRO_COMMAND_SHUTTER:
            cmd.cmd[0] = 'S';
            cmd.cmd[1] = 'H';
            cmd.cmd_parm = request->value;
            break;

        /* Unsupported commands */
        case GOPRO_COMMAND_RESOLUTION:
            cmd.cmd[0] = 'V';
            cmd.cmd[1] = 'V';
            cmd.cmd_parm = request->value;
            break;

        case GOPRO_COMMAND_FRAME_RATE:
            cmd.cmd[0] = 'F';
            cmd.cmd[1] = 'S';
            cmd.cmd_parm = request->value;
            break;

        case GOPRO_COMMAND_FIELD_OF_VIEW:
            cmd.cmd[0] = 'F';
            cmd.cmd[1] = 'V';
            cmd.cmd_parm = request->value;
            break;
        /* End of unsupported commands */

        default:
            // Unsupported Command ID
            gp_set_transaction_result(NULL, 0, GP_CMD_STATUS_FAILURE);
            return -1;
    }

    gp_h3p_send_command(&cmd);
    return 0;
}

bool gp_h3p_handle_rx(gp_h3p_t *h3p, uint16_t *buf, uint16_t len, bool from_camera, uint16_t *txbuf)
{
    /*
     * Handle incoming i2c data from the camera.
     *
     * Return true if we generated a response that
     * should be written back to the camera.
     */

    gp_h3p_sanitize_buf_len(buf);

    if (from_camera) {
        gp_h3p_handle_command(h3p, buf, txbuf);
        return true;
    }

    gp_h3p_handle_response(buf);
    return false;
}

void gp_h3p_handle_command(gp_h3p_t *h3p, const uint16_t *cmdbuf, uint16_t *txbuf)
{
    /*
     * A validated command has been received from the camera.
     *
     * First make sure the command is one we support.  Per the GoPro spec,
     * we only have to respond to the "vs" command which queries
     * the version of the protocol we support.
     *
     * For any other command from the GoPro, return an error response
     */

    if ((cmdbuf[1] == 'v') && (cmdbuf[2] == 's')) {
        // Preload the response buffer with the command response.  This will be transmitted in the ISR
        txbuf[0] = 2; // Packet size, 1st byte is status byte, 2nd byte is protocol version
        txbuf[1] = GP_CMD_STATUS_SUCCESS;
        txbuf[2] = GP_PROTOCOL_VERSION;
        h3p->gccb_version_queried = true;
        return;
    }

    // Preload the response buffer with an error response, since we don't support the command we
    // were sent.  This will be transmitted in the ISR
    txbuf[0] = 1; // Packet size, only status byte
    txbuf[1] = GP_CMD_STATUS_FAILURE;
}

void gp_h3p_handle_response(const uint16_t *respbuf)
{
    /*
     * Process a response to one of our commands.
     */

    GPCmdStatus status = (GPCmdStatus)respbuf[1];

    // Special Handling of responses
    if (gp_transaction_cmd() == GOPRO_COMMAND_MODEL) {
        // Take third byte (CAMERA_MODEL) of the "camera model and firmware version" response
        gp_set_transaction_result(&respbuf[3], 1, status);

    } else {
        gp_set_transaction_result(&respbuf[2], 1, status);
    }
}


bool gp_h3p_rx_data_is_valid(const uint16_t *buf, uint16_t len, bool *from_camera)
{
    /*
     * Called when an i2c rx transaction has completed successfully,
     * to determine if the received data is formatted correctly.
     *
     * Drain all the data from the i2c ringbuf, and ensure
     * the advertised len matches what we actually recevied.
     */

    // first byte is the length of the received data,
    // top bit signals the cmd originator, 1 == camera, 0 == backpack
    *from_camera = buf[0] & (1 << 7);
    uint16_t buf_len = buf[0] & 0x7f;

    if (buf_len != len - 1) {
        return false;
    }

    return true;
}

bool gp_h3p_send_command(const GPCmd* cmd)
{
    gp_h3p_pkt_t p;
    gp_h3p_cmd_t *c = &p.cmd;

    c->cmd1 = cmd->cmd[0];
    c->cmd2 = cmd->cmd[1];

    // first byte is len, upper bit clear to indicate command originated from the gimbal (not the GoPro)
    if (gp_h3p_cmd_has_param(cmd)) {
        c->len = 0x3;
        c->payload[0] = cmd->cmd_parm;
    } else {
        c->len = 0x2;
    }

    gp_send_cmd(p.bytes, p.cmd.len + 1);
    return true;
}

void gp_h3p_sanitize_buf_len(uint16_t *buf) { // TODO: inline?
    buf[0] &= 0x7f;     // remove most significant bit representing sender id (camera or BacPac)
}
