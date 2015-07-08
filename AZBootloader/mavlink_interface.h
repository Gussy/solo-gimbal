#ifndef MAVLINK_INTERFACE_H_
#define MAVLINK_INTERFACE_H_

#define MAVLINK_SYSTEM_ID			1
#define BUFFER_LENGTH				300

// Max payload of MAVLINK_MSG_ID_ENCAPSULATED_DATA message is 253 Bytes
// Must be an even number or the uint8_t[] to uint16_t[] conversion will fail
#define ENCAPSULATED_DATA_LENGTH 	252


void MAVLINK_send_heartbeat();
void send_mavlink_statustext(char* message);

#endif /* MAVLINK_INTERFACE_H_ */
