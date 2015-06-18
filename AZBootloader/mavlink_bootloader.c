
//#define MAVLINK_COMM_NUM_BUFFERS 1
#define MAVLINK_EXTERNAL_RX_BUFFER
#define MAVLINK_EXTERNAL_RX_STATUS

#include "protocol_c2000.h"

#include "mavlink.h"
#include "memory_map.h"

#pragma    DATA_SECTION(m_mavlink_buffer,"DMARAML5");
#pragma    DATA_SECTION(m_mavlink_status,"DMARAML5");

mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];
mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];


#define BOOTLOADER_VERSION_MAJOR	0x02
#define BOOTLOADER_VERSION_MINOR	0x06
#define BOOTLOADER_VERSION			((BOOTLOADER_VERSION_MAJOR << 8) | BOOTLOADER_VERSION_MINOR)

#define MAVLINK_SYSTEM_ID			1

// Max payload of MAVLINK_MSG_ID_ENCAPSULATED_DATA message is 253 Bytes
// Must be an even number or the uint8_t[] to uint16_t[] conversion will fail
#define ENCAPSULATED_DATA_LENGTH 	252


static Uint16 Example_CsmUnlock()
{
    volatile Uint16 temp;
	Uint16 PRG_key0 = 0xFFFF;        //   CSM Key values

    // Load the key registers with the current password
    // These are defined in Example_Flash2806x_CsmKeys.asm

    EALLOW;
    CsmRegs.KEY0 = PRG_key0;
    CsmRegs.KEY1 = PRG_key0;
    CsmRegs.KEY2 = PRG_key0;
    CsmRegs.KEY3 = PRG_key0;
    CsmRegs.KEY4 = PRG_key0;
    CsmRegs.KEY5 = PRG_key0;
    CsmRegs.KEY6 = PRG_key0;
    CsmRegs.KEY7 = PRG_key0;
    EDIS;

    // Perform a dummy read of the password locations
    // if they match the key values, the CSM will unlock

    temp = CsmPwl.PSWD0;
    temp = CsmPwl.PSWD1;
    temp = CsmPwl.PSWD2;
    temp = CsmPwl.PSWD3;
    temp = CsmPwl.PSWD4;
    temp = CsmPwl.PSWD5;
    temp = CsmPwl.PSWD6;
    temp = CsmPwl.PSWD7;

    // If the CSM unlocked, return succes, otherwise return
    // failure.
    if((CsmRegs.CSMSCR.all & 0x0001) == 0) return STATUS_SUCCESS;
    else return STATUS_FAIL_CSM_LOCKED;

}

/* all these may be overwritten when loading the image */
#pragma    DATA_SECTION(msg,"DMARAML5");
#pragma    DATA_SECTION(inmsg,"DMARAML5");
#pragma    DATA_SECTION(status,"DMARAML5");
#pragma    DATA_SECTION(buffer,"DMARAML5");
mavlink_message_t msg, inmsg;
mavlink_status_t status;
#define BUFFER_LENGTH	300
unsigned char buffer[BUFFER_LENGTH];

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

Uint32 MAVLINK_Flash()
{
	int getting_messages = 0;
	Uint16 seq = 0, length;
	FLASH_ST FlashStatus = {0};
	Uint16  *Flash_ptr = (Uint16 *)APP_START;     // Pointer to a location in flash
    Uint16 blink_counter = 0;
    int idx1 = 0;
    Uint16 idx2 = 0;

	memset(&msg, 0, sizeof(msg));
	memset(&status, 0, sizeof(status));
	memset(&m_mavlink_buffer, 0, sizeof(m_mavlink_buffer[0]));
	memset(&m_mavlink_status, 0, sizeof(m_mavlink_status[0]));
	setup_serial_port();

	EALLOW;
	Flash_CPUScaleFactor = SCALE_FACTOR;
	EDIS;

	// wait for an image to arrive over mavlink serial
	while(1) {
		// send a mavlink heartbeat if the bootload sequence hasn't started yet
		//if(seq == 0)
			MAVLINK_send_heartbeat();

		// send mavlink message to request data stream
		mavlink_msg_data_transmission_handshake_pack(
			MAVLINK_SYSTEM_ID,
			MAV_COMP_ID_GIMBAL,
			&msg,
			MAVLINK_TYPE_UINT16_T,
			0 /* size */,
			seq /* width */,
			BOOTLOADER_VERSION /*uint16_t height*/,
			0 /*uint16_t packets*/,
			ENCAPSULATED_DATA_LENGTH /*uint8_t payload*/,
			0 /*uint8_t jpg_quality*/
		);

		length = mavlink_msg_to_send_buffer(buffer, &msg);
		send_serial_port(buffer, length);
		getting_messages = 1;

		while(getting_messages) {
			int read_size = 0;
			// read the serial port, and if I get no messages, timeout
			memset(buffer, 0, sizeof(buffer[0])*BUFFER_LENGTH);
			if((read_size = read_serial_port(buffer, BUFFER_LENGTH)) == 0) {
				getting_messages = 0;
				memset(&msg, 0, sizeof(msg));
				memset(&status, 0, sizeof(status));
				memset(&m_mavlink_buffer, 0, sizeof(m_mavlink_buffer[0]));
				memset(&m_mavlink_status, 0, sizeof(m_mavlink_status[0]));
			} else {
				getting_messages = 1;
				for(idx1 = 0; idx1 < read_size; idx1++) {
					if(mavlink_parse_char(MAVLINK_COMM_0, buffer[idx1]&0xFF, &inmsg, &status)) {
						if(inmsg.msgid == MAVLINK_MSG_ID_ENCAPSULATED_DATA) {
							if(inmsg.len > 2 && seq == mavlink_msg_encapsulated_data_get_seqnr(&inmsg)) {
								mavlink_msg_encapsulated_data_get_data(&inmsg, buffer);
								getting_messages = 0;

								// Convert the uint8_t[] into a uint16_t[]
								// 16bit words are sent as LSB byte then MSB byte
								for (idx2 = 0; idx2 < ENCAPSULATED_DATA_LENGTH/2; idx2++) {
									buffer[idx2] = (buffer[idx2*2+1]<<8)|
												   (buffer[idx2*2+0]);
								}

								// Fast toggle the LED
								STATUS_LED_TOGGLE();
								if (Flash_ptr <= (Uint16*)APP_END) {
									// Unlock and erase flash with first packet
									if(seq == 0) {
										Example_CsmUnlock();

										/* don't erase SECTOR A */
										Flash_Erase(SECTORB, &FlashStatus);
										STATUS_LED_TOGGLE();
										Flash_Erase(SECTORC, &FlashStatus);
										STATUS_LED_TOGGLE();
										Flash_Erase(SECTORD, &FlashStatus);
										STATUS_LED_TOGGLE();
										Flash_Erase(SECTORE, &FlashStatus);
										STATUS_LED_TOGGLE();
										Flash_Erase(SECTORF, &FlashStatus);
										STATUS_LED_TOGGLE();
										Flash_Erase(SECTORG, &FlashStatus);
										STATUS_LED_TOGGLE();
										/* don't erase SECTOR H */

										// write data to flash
										Flash_Program(Flash_ptr, (Uint16 *)buffer, ENCAPSULATED_DATA_LENGTH/2, &FlashStatus);
										Flash_ptr += ENCAPSULATED_DATA_LENGTH/2;
										seq++;
									} else {
										// write data to flash
										Flash_Program(Flash_ptr, (Uint16 *)buffer, ENCAPSULATED_DATA_LENGTH/2, &FlashStatus);
										Flash_ptr += ENCAPSULATED_DATA_LENGTH/2;
										seq++;
									}
								}
							}
						} else if(inmsg.msgid == MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE) {
							// send mavlink message to request data stream
							mavlink_msg_data_transmission_handshake_pack(
								MAVLINK_SYSTEM_ID,
								MAV_COMP_ID_GIMBAL,
								&msg,
								MAVLINK_TYPE_UINT16_T,
								0 /* size */,
								UINT16_MAX /* width */,
								BOOTLOADER_VERSION /*uint16_t height*/,
								0 /*uint16_t packets*/,
								ENCAPSULATED_DATA_LENGTH /*uint8_t payload*/,
								0 /*uint8_t jpg_quality*/
							);

							length = mavlink_msg_to_send_buffer(buffer, &msg);
							send_serial_port(buffer, length);

							/* must be done */
							WatchDogEnable();

							// Don't reset immediately, otherwise the MAVLink message above won't be flushed.

							// This should never be reached.
							while(1);
						}
					}
				}
			}
		}

		// If we're here, we've timed out and are about to send another mavlink request for a boot image
		// Toggle the LED in here to show that we're doing something
        if (++blink_counter >= 2) {
            STATUS_LED_TOGGLE();
            blink_counter = 0;
        }
	}

	// reset?
	return 0;
}

