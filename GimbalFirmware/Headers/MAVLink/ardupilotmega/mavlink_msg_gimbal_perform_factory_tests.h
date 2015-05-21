// MESSAGE GIMBAL_PERFORM_FACTORY_TESTS PACKING

#if MAVLINK_C2000
#include "protocol_c2000.h"
#endif

#define MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS 209

typedef struct __mavlink_gimbal_perform_factory_tests_t
{
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t test_id; ///< ID of test to perform
 uint8_t test_arg; ///< Arbitrary argument to be passed to test
} mavlink_gimbal_perform_factory_tests_t;

#define MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN 4
#define MAVLINK_MSG_ID_209_LEN 4

#define MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_CRC 49
#define MAVLINK_MSG_ID_209_CRC 49



#define MAVLINK_MESSAGE_INFO_GIMBAL_PERFORM_FACTORY_TESTS { \
	"GIMBAL_PERFORM_FACTORY_TESTS", \
	4, \
	{  { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gimbal_perform_factory_tests_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_gimbal_perform_factory_tests_t, target_component) }, \
         { "test_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_gimbal_perform_factory_tests_t, test_id) }, \
         { "test_arg", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_gimbal_perform_factory_tests_t, test_arg) }, \
         } \
}


/**
 * @brief Pack a gimbal_perform_factory_tests message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param test_id ID of test to perform
 * @param test_arg Arbitrary argument to be passed to test
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_perform_factory_tests_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, uint8_t test_id, uint8_t test_arg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, test_id);
	_mav_put_uint8_t(buf, 3, test_arg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN);
#elif MAVLINK_C2000
		mav_put_uint8_t_c2000(&(msg->payload64[0]), 0, target_system);
		mav_put_uint8_t_c2000(&(msg->payload64[0]), 1, target_component);
		mav_put_uint8_t_c2000(&(msg->payload64[0]), 2, test_id);
		mav_put_uint8_t_c2000(&(msg->payload64[0]), 3, test_arg);
	
	
#else
	mavlink_gimbal_perform_factory_tests_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.test_id = test_id;
	packet.test_arg = test_arg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN);
#endif
}

/**
 * @brief Pack a gimbal_perform_factory_tests message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param test_id ID of test to perform
 * @param test_arg Arbitrary argument to be passed to test
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gimbal_perform_factory_tests_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,uint8_t test_id,uint8_t test_arg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, test_id);
	_mav_put_uint8_t(buf, 3, test_arg);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN);
#else
	mavlink_gimbal_perform_factory_tests_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.test_id = test_id;
	packet.test_arg = test_arg;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN);
#endif
}

/**
 * @brief Encode a gimbal_perform_factory_tests struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_perform_factory_tests C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_perform_factory_tests_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gimbal_perform_factory_tests_t* gimbal_perform_factory_tests)
{
	return mavlink_msg_gimbal_perform_factory_tests_pack(system_id, component_id, msg, gimbal_perform_factory_tests->target_system, gimbal_perform_factory_tests->target_component, gimbal_perform_factory_tests->test_id, gimbal_perform_factory_tests->test_arg);
}

/**
 * @brief Encode a gimbal_perform_factory_tests struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gimbal_perform_factory_tests C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gimbal_perform_factory_tests_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gimbal_perform_factory_tests_t* gimbal_perform_factory_tests)
{
	return mavlink_msg_gimbal_perform_factory_tests_pack_chan(system_id, component_id, chan, msg, gimbal_perform_factory_tests->target_system, gimbal_perform_factory_tests->target_component, gimbal_perform_factory_tests->test_id, gimbal_perform_factory_tests->test_arg);
}

/**
 * @brief Send a gimbal_perform_factory_tests message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param test_id ID of test to perform
 * @param test_arg Arbitrary argument to be passed to test
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gimbal_perform_factory_tests_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint8_t test_id, uint8_t test_arg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN];
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, test_id);
	_mav_put_uint8_t(buf, 3, test_arg);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS, buf, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS, buf, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN);
#endif
#else
	mavlink_gimbal_perform_factory_tests_t packet;
	packet.target_system = target_system;
	packet.target_component = target_component;
	packet.test_id = test_id;
	packet.test_arg = test_arg;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS, (const char *)&packet, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gimbal_perform_factory_tests_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t target_system, uint8_t target_component, uint8_t test_id, uint8_t test_arg)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, target_system);
	_mav_put_uint8_t(buf, 1, target_component);
	_mav_put_uint8_t(buf, 2, test_id);
	_mav_put_uint8_t(buf, 3, test_arg);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS, buf, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS, buf, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN);
#endif
#else
	mavlink_gimbal_perform_factory_tests_t *packet = (mavlink_gimbal_perform_factory_tests_t *)msgbuf;
	packet->target_system = target_system;
	packet->target_component = target_component;
	packet->test_id = test_id;
	packet->test_arg = test_arg;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS, (const char *)packet, MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE GIMBAL_PERFORM_FACTORY_TESTS UNPACKING


/**
 * @brief Get field target_system from gimbal_perform_factory_tests message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_gimbal_perform_factory_tests_get_target_system(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_uint8_t(msg,  0);
#else
	return mav_get_uint8_t_c2000(&(msg->payload64[0]),  0);
#endif
}

/**
 * @brief Get field target_component from gimbal_perform_factory_tests message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_gimbal_perform_factory_tests_get_target_component(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_uint8_t(msg,  1);
#else
	return mav_get_uint8_t_c2000(&(msg->payload64[0]),  1);
#endif
}

/**
 * @brief Get field test_id from gimbal_perform_factory_tests message
 *
 * @return ID of test to perform
 */
static inline uint8_t mavlink_msg_gimbal_perform_factory_tests_get_test_id(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_uint8_t(msg,  2);
#else
	return mav_get_uint8_t_c2000(&(msg->payload64[0]),  2);
#endif
}

/**
 * @brief Get field test_arg from gimbal_perform_factory_tests message
 *
 * @return Arbitrary argument to be passed to test
 */
static inline uint8_t mavlink_msg_gimbal_perform_factory_tests_get_test_arg(const mavlink_message_t* msg)
{
#if !MAVLINK_C2000
	return _MAV_RETURN_uint8_t(msg,  3);
#else
	return mav_get_uint8_t_c2000(&(msg->payload64[0]),  3);
#endif
}

/**
 * @brief Decode a gimbal_perform_factory_tests message into a struct
 *
 * @param msg The message to decode
 * @param gimbal_perform_factory_tests C-struct to decode the message contents into
 */
static inline void mavlink_msg_gimbal_perform_factory_tests_decode(const mavlink_message_t* msg, mavlink_gimbal_perform_factory_tests_t* gimbal_perform_factory_tests)
{
#if MAVLINK_NEED_BYTE_SWAP || MAVLINK_C2000
	gimbal_perform_factory_tests->target_system = mavlink_msg_gimbal_perform_factory_tests_get_target_system(msg);
	gimbal_perform_factory_tests->target_component = mavlink_msg_gimbal_perform_factory_tests_get_target_component(msg);
	gimbal_perform_factory_tests->test_id = mavlink_msg_gimbal_perform_factory_tests_get_test_id(msg);
	gimbal_perform_factory_tests->test_arg = mavlink_msg_gimbal_perform_factory_tests_get_test_arg(msg);
#else
	memcpy(gimbal_perform_factory_tests, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GIMBAL_PERFORM_FACTORY_TESTS_LEN);
#endif
}
