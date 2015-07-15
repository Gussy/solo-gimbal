#include "gopro_hero3p.h"
#include "gopro_hero_common.h"
#include "gopro_interface.h"

#include <ctype.h>

// Include for GOPRO_COMMAND enum
#include "mavlink_interface/mavlink_gimbal_interface.h" // TODO: get rid of this after replacing GOPRO_COMMAND

void gp_h3p_init(gp_h3p_t *h3p)
{
    h3p->gccb_version_queried = false;
}

bool gp_h3p_handshake_complete(const gp_h3p_t *h3p)
{
    return h3p->gccb_version_queried;
}

bool gp_h3p_request_power_off(GPCmdResponse *last_cmd_response)
{
    GPCmd cmd;
    cmd.cmd[0] = 'P';
    cmd.cmd[1] = 'W';
    cmd.cmd_parm = 0x00;
    gp_h3p_send_command(&cmd, last_cmd_response);
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
int gp_h3p_get_request(Uint8 cmd_id, bool *new_response_available, GPCmdResponse *last_cmd_response) // TODO: name is a bit awkward, think about refactoring (gp_h3p_handle_get_request?), same with set request
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
            *new_response_available = true;
            return -1;
    }

    gp_h3p_send_command(&cmd, last_cmd_response);
    return 0;
}

int gp_h3p_set_request(GPSetRequest* request, bool *new_response_available, GPSetRequest* last_set_request, GPCmdResponse *last_cmd_response)
{
    GPCmd cmd;

    switch (request->cmd_id) {
        case GOPRO_COMMAND_POWER:
            if(request->value == 0x00 && gp_get_power_status() == GP_POWER_ON) {
                gp_h3p_request_power_off(last_cmd_response);
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
            *new_response_available = true;
            return -1;
    }

    *last_set_request = *request;
    gp_h3p_send_command(&cmd, last_cmd_response);
    return 0;
}

bool gp_h3p_handle_command(gp_h3p_t *h3p, const uint16_t *cmdbuf, uint16_t *txbuf)
{
    if ((cmdbuf[1] == 'v') && (cmdbuf[2] == 's')) {
        // Preload the response buffer with the command response.  This will be transmitted in the ISR
        txbuf[0] = 2; // Packet size, 1st byte is status byte, 2nd byte is protocol version
        txbuf[1] = GP_CMD_STATUS_SUCCESS;
        txbuf[2] = GP_PROTOCOL_VERSION;
        h3p->gccb_version_queried = true;
        return true;

    } else {
        // Preload the response buffer with an error response, since we don't support the command we
        // were sent.  This will be transmitted in the ISR
        txbuf[0] = 1; // Packet size, only status byte
        txbuf[1] = GP_CMD_STATUS_FAILURE;
        return false;
    }
}

bool gp_h3p_handle_response(const uint16_t *respbuf, GPCmdResponse *last_cmd_response)
{
    // Special Handling of responses
    if (last_cmd_response->cmd[0] == 'c' && last_cmd_response->cmd[1] == 'v') {
        // Take third byte (CAMERA_MODEL) of the "camera model and firmware version" response
        last_cmd_response->value = respbuf[3];
        return true;
    } else {
        last_cmd_response->value = respbuf[2];
        return false;
    }
}


bool gp_h3p_rx_data_is_valid(uint16_t *buf, uint16_t len, bool *from_camera)
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
    buf[0] &= 0x7f;

    if (buf[0] != len - 1) {
        return false;
    }

    return true;
}

bool gp_h3p_send_command(const GPCmd* cmd, GPCmdResponse *last_cmd_response)
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

#if 1
    // TODO: phase out this section

    // Clear the last command data
    last_cmd_response->value = 0x00;
    last_cmd_response->status = GP_CMD_STATUS_UNKNOWN;
    last_cmd_response->result = GP_CMD_UNKNOWN;

    // Also load the command name bytes of the last response struct with the command name bytes for this command.  The next response should be a response to
    // this command, and this way a caller of gp_get_last_response can know what command the response goes with
    last_cmd_response->cmd[0] = cmd->cmd[0];
    last_cmd_response->cmd[1] = cmd->cmd[1];
#endif

    gp_send_cmd(p.bytes, p.cmd.len + 1);

#if 0
    // Assert the GoPro interrupt line, letting it know we'd like it to read a command from us
    gp_assert_intr();

    // Reset the timeout counter, and transition to waiting for the GoPro to start reading the command from us
    timeout_counter = 0;
    gp_control_state = GP_CONTROL_STATE_WAIT_FOR_START_CMD_SEND;
#endif

    return true;
}
