#include "hardware/device_init.h"
#include "uart.h"

#include "mavlink_bootloader.h"
#include "mavlink_interface.h"
#include "protocol_c2000.h"
#include "mavlink.h"



void MAVLINK_send_heartbeat()
{
	Uint16 length;
	mavlink_message_t heartbeat_msg;
	mavlink_msg_heartbeat_pack(MAVLINK_SYSTEM_ID,
								MAV_COMP_ID_GIMBAL,
								&heartbeat_msg,
								MAV_TYPE_GIMBAL,
								MAV_AUTOPILOT_INVALID,
								MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
								0, /* MAV_MODE_GIMBAL_UNINITIALIZED */
								0  /* MAV_STATE_UNINIT */
								);

	length = mavlink_msg_to_send_buffer(buffer, &heartbeat_msg);
	send_serial_port(buffer, length);
}

void send_mavlink_statustext(char* message)
{
	Uint16 length;
    mavlink_message_t status_msg;
    mavlink_msg_statustext_pack(MAVLINK_SYSTEM_ID,
								MAV_COMP_ID_GIMBAL,
								&status_msg,
								MAV_SEVERITY_DEBUG,
								message);

    length = mavlink_msg_to_send_buffer(buffer, &status_msg);
    send_serial_port(buffer, length);
}
