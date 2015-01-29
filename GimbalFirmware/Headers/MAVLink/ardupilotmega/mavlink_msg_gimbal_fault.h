// MESSAGE GIMBAL_FAULT PACKING

#if MAVLINK_C2000
#include "protocol_c2000.h"
#endif

#define MAVLINK_MSG_ID_GIMBAL_FAULT 195

typedef struct __mavlink_gimbal_fault_t
{
 uint8_t fault_axis; ///< Axis reporting the fault, see GIMBAL_AXIS enumeration
 uint8_t fault_type; ///< Type of fault being reported, see GIMBAL_FAULT enumeration
} mavlink_gimbal_fault_t;

#define MAVLINK_MSG_ID_GIMBAL_FAULT_LEN 2
#define MAVLINK_MSG_ID_195_LEN 2

#define MAVLINK_MSG_ID_GIMBAL_FAULT_CRC 48
#define MAVLINK_MSG_ID_195_CRC 48



#define MAVLINK_MESSAGE_INFO_GIMBAL_FAULT { \
	"GIMBAL_FAULT", \
	2, \
	{  { "fault_axis", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gimbal_fault_t, fault_axis) }, \
         { "fault_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gimbal_fault_t, fault_type) }, \
         } \
}


/**
 * @brief Pack a gimbal_fault message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param fault_axis Axis reporting the fault, see GIMBAL_AXIS enumeration
 * @param fault_type Type of fault being reported, see GIMBAL_FAULT enumeration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_fault_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t fault_axis, uint8_t fault_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_FAULT_LEN];
	_mav_put_uint8_t(buf, 0, fault_axis);
	_mav_put_uint8_t(buf, 1, fault_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_FAULT_LEN);
#elif MAVLINK_C2000
		mav_put_uint8_t_c2000(&(msg->payload64[0]), 0, fault_axis);
		mav_put_uint8_t_c2000(&(msg->payload64[0]), 1, fault_type);
	
	
#else
	mavlink_gimbal_fault_t packet;
	packet.fault_axis = fault_axis;
	packet.fault_type = fault_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_FAULT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GIMBAL_FAULT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_FAULT_LEN, MAVLINK_MSG_ID_GIMBAL_FAULT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_FAULT_LEN);
#endif
}

/**
 * @brief Pack a gimbal_fault message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param fault_axis Axis reporting the fault, see GIMBAL_AXIS enumeration
 * @param fault_type Type of fault being reported, see GIMBAL_FAULT enumeration
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_fault_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t fault_axis,uint8_t fault_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_FAULT_LEN];
	_mav_put_uint8_t(buf, 0, fault_axis);
	_mav_put_uint8_t(buf, 1, fault_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_FAULT_LEN);
#else
	mavlink_gimbal_fault_t packet;
	packet.fault_axis = fault_axis;
	packet.fault_type = fault_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_FAULT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GIMBAL_FAULT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_FAULT_LEN, MAVLINK_MSG_ID_GIMBAL_FAULT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_FAULT_LEN);
#endif
}

/**
 * @brief Encode a gimbal_fault struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_fault C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_fault_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_fault_t* gimbal_fault)
{
	return mavlink_msg_gimbal_fault_pack(system_id, component_id, msg, gimbal_fault->fault_axis, gimbal_fault->fault_type);
}

/**
 * @brief Encode a gimbal_fault struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_fault C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_fault_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_fault_t* gimbal_fault)
{
	return mavlink_msg_gimbal_fault_pack_chan(system_id, component_id, chan, msg, gimbal_fault->fault_axis, gimbal_fault->fault_type);
}

/**
 * @brief Send a gimbal_fault message
 * @param chan MAVLink channel to send the message
 *
 * @param fault_axis Axis reporting the fault, see GIMBAL_AXIS enumeration
 * @param fault_type Type of fault being reported, see GIMBAL_FAULT enumeration
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_fault_send(mavlink_channel_t chan, uint8_t fault_axis, uint8_t fault_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_FAULT_LEN];
	_mav_put_uint8_t(buf, 0, fault_axis);
	_mav_put_uint8_t(buf, 1, fault_type);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_FAULT, buf, MAVLINK_MSG_ID_GIMBAL_FAULT_LEN, MAVLINK_MSG_ID_GIMBAL_FAULT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_FAULT, buf, MAVLINK_MSG_ID_GIMBAL_FAULT_LEN);
#endif
#else
	mavlink_gimbal_fault_t packet;
	packet.fault_axis = fault_axis;
	packet.fault_type = fault_type;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_FAULT, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_FAULT_LEN, MAVLINK_MSG_ID_GIMBAL_FAULT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_FAULT, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_FAULT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_FAULT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_fault_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t fault_axis, uint8_t fault_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, fault_axis);
	_mav_put_uint8_t(buf, 1, fault_type);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_FAULT, buf, MAVLINK_MSG_ID_GIMBAL_FAULT_LEN, MAVLINK_MSG_ID_GIMBAL_FAULT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_FAULT, buf, MAVLINK_MSG_ID_GIMBAL_FAULT_LEN);
#endif
#else
	mavlink_gimbal_fault_t *packet = (mavlink_gimbal_fault_t *)msgbuf;
	packet->fault_axis = fault_axis;
	packet->fault_type = fault_type;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_FAULT, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_FAULT_LEN, MAVLINK_MSG_ID_GIMBAL_FAULT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_FAULT, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_FAULT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE GIMBAL_FAULT UNPACKING


/**
 * @brief Get field fault_axis from gimbal_fault message
 *
 * @return Axis reporting the fault, see GIMBAL_AXIS enumeration
 */
static inline uint8_t mavlink_msg_gimbal_fault_get_fault_axis(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_uint8_t(msg,  0);
#else
	return mav_get_uint8_t_c2000(&(msg->payload64[0]),  0);
#endif
}

/**
 * @brief Get field fault_type from gimbal_fault message
 *
 * @return Type of fault being reported, see GIMBAL_FAULT enumeration
 */
static inline uint8_t mavlink_msg_gimbal_fault_get_fault_type(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_uint8_t(msg,  1);
#else
	return mav_get_uint8_t_c2000(&(msg->payload64[0]),  1);
#endif
}

/**
 * @brief Decode a gimbal_fault message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_fault C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_fault_decode(const mavlink_message_t* msg, mavlink_gimbal_fault_t* gimbal_fault)
{
#if MAVLINK_NEED_BYTE_SWAP || MAVLINK_C2000
	gimbal_fault->fault_axis = mavlink_msg_gimbal_fault_get_fault_axis(msg);
	gimbal_fault->fault_type = mavlink_msg_gimbal_fault_get_fault_type(msg);
#else
	memcpy(gimbal_fault, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GIMBAL_FAULT_LEN);
#endif
}
