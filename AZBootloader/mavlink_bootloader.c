#include "memory_map.h"
#include "uart.h"
#include "hardware/led.h"
#include "hardware/device_init.h"
#include "hardware/HWSpecific.h"
#include "hardware/watchdog.h"
#include "flash/flash_helpers.h"

#define	FLASH_F2806x 1

#include "Flash2806x_API_Library.h"
#include "F2806x_SysCtrl.h"

#define MAVLINK_EXTERNAL_RX_BUFFER
#define MAVLINK_EXTERNAL_RX_STATUS

#include "mavlink.h"
#include "mavlink_interface.h"
#include "mavlink_bootloader.h"
#include "protocol_c2000.h"

#pragma    DATA_SECTION(m_mavlink_buffer,"DMARAML5");
#pragma    DATA_SECTION(m_mavlink_status,"DMARAML5");

mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];
mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];

/* all these may be overwritten when loading the image */
#pragma    DATA_SECTION(msg,"DMARAML5");
#pragma    DATA_SECTION(inmsg,"DMARAML5");
#pragma    DATA_SECTION(status,"DMARAML5");
#pragma    DATA_SECTION(buffer,"DMARAML5");
mavlink_message_t msg, inmsg;
mavlink_status_t status;
unsigned char buffer[BUFFER_LENGTH];

static void prepare_flash(void);
static void mavlink_data_handshake(Uint16 seq);
static void clear_buffers(void);
static void reset_to_app(void);

void MAVLINK_Flash()
{
	int getting_messages = 0;
	Uint16 seq = 0;
	FLASH_ST FlashStatus = {0};
	Uint16  *Flash_ptr = (Uint16 *)APP_START;     // Pointer to a location in flash
    Uint16 blink_counter = 0;
    int idx1 = 0;
    Uint16 idx2 = 0;

	clear_buffers();
	setup_serial_port();

	EALLOW;
	Flash_CPUScaleFactor = SCALE_FACTOR;
	EDIS;

	// wait for an image to arrive over mavlink serial
	while(1) {
		// send a mavlink heartbeat
	    MAVLINK_send_heartbeat();

		// send mavlink message to request data stream
		mavlink_data_handshake(seq);

		getting_messages = 1;
		while(getting_messages) {
			int read_size = 0;
			// read the serial port, and if I get no messages, timeout
			memset(buffer, 0, sizeof(buffer));
			if((read_size = read_serial_port(buffer, BUFFER_LENGTH)) == 0) {
				getting_messages = 0;
				clear_buffers();
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
								led_status_toggle();
								if (Flash_ptr <= (Uint16*)APP_END) {
									// Unlock and erase flash with first packet
									if(seq == 0) {
										prepare_flash();
									}

                                    // write data to flash
                                    Flash_Program(Flash_ptr, (Uint16 *)buffer, ENCAPSULATED_DATA_LENGTH/2, &FlashStatus);
                                    Flash_ptr += ENCAPSULATED_DATA_LENGTH/2;
                                    seq++;
								}
							}
						} else if(inmsg.msgid == MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE) {
						    reset_to_app(); // This never returns
						}
					}
				}
			}
		}

		// If we're here, we've timed out and are about to send another mavlink request for a boot image
		// Toggle the LED in here to show that we're doing something
        if (++blink_counter >= 2) {
            led_status_toggle();
            blink_counter = 0;
        }
	}
}

static void clear_buffers(void) {
    memset(&msg, 0, sizeof(msg));
    memset(&status, 0, sizeof(status));
    memset(&m_mavlink_buffer, 0, sizeof(m_mavlink_buffer));
    memset(&m_mavlink_status, 0, sizeof(m_mavlink_status));
}

static void prepare_flash(void) {
    flash_csm_unlock();

    FLASH_ST FlashStatus = {0};

    /* don't erase SECTOR A */
    Flash_Erase(SECTORB, &FlashStatus);
    led_status_toggle();
    Flash_Erase(SECTORC, &FlashStatus);
    led_status_toggle();
    Flash_Erase(SECTORD, &FlashStatus);
    led_status_toggle();
    Flash_Erase(SECTORE, &FlashStatus);
    led_status_toggle();
    Flash_Erase(SECTORF, &FlashStatus);
    led_status_toggle();
    Flash_Erase(SECTORG, &FlashStatus);
    led_status_toggle();
    /* don't erase SECTOR H */
}

static void mavlink_data_handshake(Uint16 seq) {
    // send mavlink message to request data stream
    mavlink_msg_data_transmission_handshake_pack(
    MAVLINK_SYSTEM_ID, MAV_COMP_ID_GIMBAL, &msg, MAVLINK_TYPE_UINT16_T,
            0 /* size */,
            seq /* width */,
            BOOTLOADER_VERSION /*uint16_t height*/,
            0 /*uint16_t packets*/,
            ENCAPSULATED_DATA_LENGTH /*uint8_t payload*/,
            0 /*uint8_t jpg_quality*/
    );
    Uint16 length;
    length = mavlink_msg_to_send_buffer(buffer, &msg);
    send_serial_port(buffer, length);
}

static void reset_to_app(void) {
    mavlink_data_handshake(UINT16_MAX);

    /* must be done */
    watchdog_enable();

    // Don't reset immediately, otherwise the MAVLink message above won't be flushed.

    // This should never be reached.
    while(1);
}
