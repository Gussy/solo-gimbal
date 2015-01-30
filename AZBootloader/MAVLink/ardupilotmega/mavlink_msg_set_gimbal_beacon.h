// MESSAGE SET_GIMBAL_BEACON PACKING

#if MAVLINK_C2000
#include "protocol_c2000.h"
#endif

#define MAVLINK_MSG_ID_SET_GIMBAL_BEACON 196

typedef struct __mavlink_set_gimbal_beacon_t
{
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t beacon_mode; ///< State for beacon to indicate, see GIMBAL_BEACON_STATE enumeration
 uint8_t beacon_brightness; ///< Beacon brightness, from 0-100%
} mavlink_set_gimbal_beacon_t;

#define MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN 4
#define MAVLINK_MSG_ID_196_LEN 4

#define MAVLINK_MSG_ID_SET_GIMBAL_BEACON_CRC 15
#define MAVLINK_MSG_ID_196_CRC 15



#define MAVLINK_MESSAGE_INFO_SET_GIMBAL_BEACON { \
	"SET_GIMBAL_BEACON", \
	4, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_set_gimbal_beacon_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_set_gimbal_beacon_t, target_component) }, \
         { "beacon_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_set_gimbal_beacon_t, beacon_mode) }, \
         { "beacon_brightness", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_set_gimbal_beacon_t, beacon_brightness) }, \
         } \
}


/**
 * @brief Pack a set_gimbal_beacon message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param beacon_mode State for beacon to indicate, see GIMBAL_BEACON_STATE enumeration
 * @param beacon_brightness Beacon brightness, from 0-100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_gimbal_beacon_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint8_t beacon_mode, uint8_t beacon_brightness)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, beacon_mode);
	_mav_put_uint8_t(buf, 3, beacon_brightness);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN);
#elif MAVLINK_C2000
		mav_put_uint8_t_c2000(&(msg->payload64[0]), 0, target_system);
		mav_put_uint8_t_c2000(&(msg->payload64[0]), 1, target_component);
		mav_put_uint8_t_c2000(&(msg->payload64[0]), 2, beacon_mode);
		mav_put_uint8_t_c2000(&(msg->payload64[0]), 3, beacon_brightness);
	
	
#else
	mavlink_set_gimbal_beacon_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.beacon_mode = beacon_mode;
	packet.beacon_brightness = beacon_brightness;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_GIMBAL_BEACON;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN);
#endif
}

/**
 * @brief Pack a set_gimbal_beacon message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param beacon_mode State for beacon to indicate, see GIMBAL_BEACON_STATE enumeration
 * @param beacon_brightness Beacon brightness, from 0-100%
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_set_gimbal_beacon_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint8_t beacon_mode,uint8_t beacon_brightness)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, beacon_mode);
	_mav_put_uint8_t(buf, 3, beacon_brightness);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN);
#else
	mavlink_set_gimbal_beacon_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.beacon_mode = beacon_mode;
	packet.beacon_brightness = beacon_brightness;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SET_GIMBAL_BEACON;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN);
#endif
}

/**
 * @brief Encode a set_gimbal_beacon struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param set_gimbal_beacon C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_gimbal_beacon_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_set_gimbal_beacon_t* set_gimbal_beacon)
{
	return mavlink_msg_set_gimbal_beacon_pack(system_id, component_id, msg, set_gimbal_beacon->target_system, set_gimbal_beacon->target_component, set_gimbal_beacon->beacon_mode, set_gimbal_beacon->beacon_brightness);
}

/**
 * @brief Encode a set_gimbal_beacon struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param set_gimbal_beacon C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_set_gimbal_beacon_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_set_gimbal_beacon_t* set_gimbal_beacon)
{
	return mavlink_msg_set_gimbal_beacon_pack_chan(system_id, component_id, chan, msg, set_gimbal_beacon->target_system, set_gimbal_beacon->target_component, set_gimbal_beacon->beacon_mode, set_gimbal_beacon->beacon_brightness);
}

/**
 * @brief Send a set_gimbal_beacon message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param beacon_mode State for beacon to indicate, see GIMBAL_BEACON_STATE enumeration
 * @param beacon_brightness Beacon brightness, from 0-100%
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_set_gimbal_beacon_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t beacon_mode, uint8_t beacon_brightness)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, beacon_mode);
	_mav_put_uint8_t(buf, 3, beacon_brightness);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_GIMBAL_BEACON, buf, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_GIMBAL_BEACON, buf, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN);
#endif
#else
	mavlink_set_gimbal_beacon_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.beacon_mode = beacon_mode;
	packet.beacon_brightness = beacon_brightness;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_GIMBAL_BEACON, (const char *)&packet, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_GIMBAL_BEACON, (const char *)&packet, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_set_gimbal_beacon_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t beacon_mode, uint8_t beacon_brightness)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, beacon_mode);
	_mav_put_uint8_t(buf, 3, beacon_brightness);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_GIMBAL_BEACON, buf, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_GIMBAL_BEACON, buf, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN);
#endif
#else
	mavlink_set_gimbal_beacon_t *packet = (mavlink_set_gimbal_beacon_t *)msgbuf;
	packet->target_system = target_system;
	packet->target_component = target_component;
	packet->beacon_mode = beacon_mode;
	packet->beacon_brightness = beacon_brightness;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_GIMBAL_BEACON, (const char *)packet, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SET_GIMBAL_BEACON, (const char *)packet, MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SET_GIMBAL_BEACON UNPACKING


/**
 * @brief Get field target_system from set_gimbal_beacon message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_set_gimbal_beacon_get_target_system(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_uint8_t(msg,  0);
#else
	return mav_get_uint8_t_c2000(&(msg->payload64[0]),  0);
#endif
}

/**
 * @brief Get field target_component from set_gimbal_beacon message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_set_gimbal_beacon_get_target_component(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_uint8_t(msg,  1);
#else
	return mav_get_uint8_t_c2000(&(msg->payload64[0]),  1);
#endif
}

/**
 * @brief Get field beacon_mode from set_gimbal_beacon message
 *
 * @return State for beacon to indicate, see GIMBAL_BEACON_STATE enumeration
 */
static inline uint8_t mavlink_msg_set_gimbal_beacon_get_beacon_mode(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_uint8_t(msg,  2);
#else
	return mav_get_uint8_t_c2000(&(msg->payload64[0]),  2);
#endif
}

/**
 * @brief Get field beacon_brightness from set_gimbal_beacon message
 *
 * @return Beacon brightness, from 0-100%
 */
static inline uint8_t mavlink_msg_set_gimbal_beacon_get_beacon_brightness(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_uint8_t(msg,  3);
#else
	return mav_get_uint8_t_c2000(&(msg->payload64[0]),  3);
#endif
}

/**
 * @brief Decode a set_gimbal_beacon message into a struct
 *
 * @param msg The message to decode
 * @param set_gimbal_beacon C-struct to decode the message contents into
 */
static inline void mavlink_msg_set_gimbal_beacon_decode(const mavlink_message_t* msg, mavlink_set_gimbal_beacon_t* set_gimbal_beacon)
{
#if MAVLINK_NEED_BYTE_SWAP || MAVLINK_C2000
	set_gimbal_beacon->target_system = mavlink_msg_set_gimbal_beacon_get_target_system(msg);
	set_gimbal_beacon->target_component = mavlink_msg_set_gimbal_beacon_get_target_component(msg);
	set_gimbal_beacon->beacon_mode = mavlink_msg_set_gimbal_beacon_get_beacon_mode(msg);
	set_gimbal_beacon->beacon_brightness = mavlink_msg_set_gimbal_beacon_get_beacon_brightness(msg);
#else
	memcpy(set_gimbal_beacon, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SET_GIMBAL_BEACON_LEN);
#endif
}
