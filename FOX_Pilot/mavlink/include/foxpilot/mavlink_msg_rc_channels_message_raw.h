// MESSAGE RC_CHANNELS_MESSAGE_RAW PACKING

#define MAVLINK_MSG_ID_RC_CHANNELS_MESSAGE_RAW 71

typedef struct __mavlink_rc_channels_message_raw_t 
{
	uint8_t target_system; ///< System ID
	uint8_t target_component; ///< Component ID
	float chan1_raw; ///< RC channel 1 value
	float chan2_raw; ///< RC channel 2 value
	float chan3_raw; ///< RC channel 3 value
	float chan4_raw; ///< RC channel 4 value
	float chan5_raw; ///< RC channel 5 value
	float chan6_raw; ///< RC channel 6 value
	float chan7_raw; ///< RC channel 7 value
	float chan8_raw; ///< RC channel 8 value

} mavlink_rc_channels_message_raw_t;



/**
 * @brief Pack a rc_channels_message_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param chan1_raw RC channel 1 value
 * @param chan2_raw RC channel 2 value
 * @param chan3_raw RC channel 3 value
 * @param chan4_raw RC channel 4 value
 * @param chan5_raw RC channel 5 value
 * @param chan6_raw RC channel 6 value
 * @param chan7_raw RC channel 7 value
 * @param chan8_raw RC channel 8 value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_message_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, float chan1_raw, float chan2_raw, float chan3_raw, float chan4_raw, float chan5_raw, float chan6_raw, float chan7_raw, float chan8_raw)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_MESSAGE_RAW;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_float_by_index(chan1_raw, i, msg->payload); // RC channel 1 value
	i += put_float_by_index(chan2_raw, i, msg->payload); // RC channel 2 value
	i += put_float_by_index(chan3_raw, i, msg->payload); // RC channel 3 value
	i += put_float_by_index(chan4_raw, i, msg->payload); // RC channel 4 value
	i += put_float_by_index(chan5_raw, i, msg->payload); // RC channel 5 value
	i += put_float_by_index(chan6_raw, i, msg->payload); // RC channel 6 value
	i += put_float_by_index(chan7_raw, i, msg->payload); // RC channel 7 value
	i += put_float_by_index(chan8_raw, i, msg->payload); // RC channel 8 value

	return mavlink_finalize_message(msg, system_id, component_id, i);
}

/**
 * @brief Pack a rc_channels_message_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param chan1_raw RC channel 1 value
 * @param chan2_raw RC channel 2 value
 * @param chan3_raw RC channel 3 value
 * @param chan4_raw RC channel 4 value
 * @param chan5_raw RC channel 5 value
 * @param chan6_raw RC channel 6 value
 * @param chan7_raw RC channel 7 value
 * @param chan8_raw RC channel 8 value
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rc_channels_message_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, uint8_t target_system, uint8_t target_component, float chan1_raw, float chan2_raw, float chan3_raw, float chan4_raw, float chan5_raw, float chan6_raw, float chan7_raw, float chan8_raw)
{
	uint16_t i = 0;
	msg->msgid = MAVLINK_MSG_ID_RC_CHANNELS_MESSAGE_RAW;

	i += put_uint8_t_by_index(target_system, i, msg->payload); // System ID
	i += put_uint8_t_by_index(target_component, i, msg->payload); // Component ID
	i += put_float_by_index(chan1_raw, i, msg->payload); // RC channel 1 value
	i += put_float_by_index(chan2_raw, i, msg->payload); // RC channel 2 value
	i += put_float_by_index(chan3_raw, i, msg->payload); // RC channel 3 value
	i += put_float_by_index(chan4_raw, i, msg->payload); // RC channel 4 value
	i += put_float_by_index(chan5_raw, i, msg->payload); // RC channel 5 value
	i += put_float_by_index(chan6_raw, i, msg->payload); // RC channel 6 value
	i += put_float_by_index(chan7_raw, i, msg->payload); // RC channel 7 value
	i += put_float_by_index(chan8_raw, i, msg->payload); // RC channel 8 value

	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, i);
}

/**
 * @brief Encode a rc_channels_message_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rc_channels_message_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rc_channels_message_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rc_channels_message_raw_t* rc_channels_message_raw)
{
	return mavlink_msg_rc_channels_message_raw_pack(system_id, component_id, msg, rc_channels_message_raw->target_system, rc_channels_message_raw->target_component, rc_channels_message_raw->chan1_raw, rc_channels_message_raw->chan2_raw, rc_channels_message_raw->chan3_raw, rc_channels_message_raw->chan4_raw, rc_channels_message_raw->chan5_raw, rc_channels_message_raw->chan6_raw, rc_channels_message_raw->chan7_raw, rc_channels_message_raw->chan8_raw);
}

/**
 * @brief Send a rc_channels_message_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param chan1_raw RC channel 1 value
 * @param chan2_raw RC channel 2 value
 * @param chan3_raw RC channel 3 value
 * @param chan4_raw RC channel 4 value
 * @param chan5_raw RC channel 5 value
 * @param chan6_raw RC channel 6 value
 * @param chan7_raw RC channel 7 value
 * @param chan8_raw RC channel 8 value
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rc_channels_message_raw_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, float chan1_raw, float chan2_raw, float chan3_raw, float chan4_raw, float chan5_raw, float chan6_raw, float chan7_raw, float chan8_raw)
{
	mavlink_message_t msg;
	mavlink_msg_rc_channels_message_raw_pack_chan(mavlink_system.sysid, mavlink_system.compid, chan, &msg, target_system, target_component, chan1_raw, chan2_raw, chan3_raw, chan4_raw, chan5_raw, chan6_raw, chan7_raw, chan8_raw);
	mavlink_send_uart(chan, &msg);
}

#endif
// MESSAGE RC_CHANNELS_MESSAGE_RAW UNPACKING

/**
 * @brief Get field target_system from rc_channels_message_raw message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_rc_channels_message_raw_get_target_system(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload)[0];
}

/**
 * @brief Get field target_component from rc_channels_message_raw message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_rc_channels_message_raw_get_target_component(const mavlink_message_t* msg)
{
	return (uint8_t)(msg->payload+sizeof(uint8_t))[0];
}

/**
 * @brief Get field chan1_raw from rc_channels_message_raw message
 *
 * @return RC channel 1 value
 */
static inline float mavlink_msg_rc_channels_message_raw_get_chan1_raw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t))[3];
	return (float)r.f;
}

/**
 * @brief Get field chan2_raw from rc_channels_message_raw message
 *
 * @return RC channel 2 value
 */
static inline float mavlink_msg_rc_channels_message_raw_get_chan2_raw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field chan3_raw from rc_channels_message_raw message
 *
 * @return RC channel 3 value
 */
static inline float mavlink_msg_rc_channels_message_raw_get_chan3_raw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field chan4_raw from rc_channels_message_raw message
 *
 * @return RC channel 4 value
 */
static inline float mavlink_msg_rc_channels_message_raw_get_chan4_raw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field chan5_raw from rc_channels_message_raw message
 *
 * @return RC channel 5 value
 */
static inline float mavlink_msg_rc_channels_message_raw_get_chan5_raw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field chan6_raw from rc_channels_message_raw message
 *
 * @return RC channel 6 value
 */
static inline float mavlink_msg_rc_channels_message_raw_get_chan6_raw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field chan7_raw from rc_channels_message_raw message
 *
 * @return RC channel 7 value
 */
static inline float mavlink_msg_rc_channels_message_raw_get_chan7_raw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Get field chan8_raw from rc_channels_message_raw message
 *
 * @return RC channel 8 value
 */
static inline float mavlink_msg_rc_channels_message_raw_get_chan8_raw(const mavlink_message_t* msg)
{
	generic_32bit r;
	r.b[3] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[0];
	r.b[2] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[1];
	r.b[1] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[2];
	r.b[0] = (msg->payload+sizeof(uint8_t)+sizeof(uint8_t)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float)+sizeof(float))[3];
	return (float)r.f;
}

/**
 * @brief Decode a rc_channels_message_raw message into a struct
 *
 * @param msg The message to decode
 * @param rc_channels_message_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_rc_channels_message_raw_decode(const mavlink_message_t* msg, mavlink_rc_channels_message_raw_t* rc_channels_message_raw)
{
	rc_channels_message_raw->target_system = mavlink_msg_rc_channels_message_raw_get_target_system(msg);
	rc_channels_message_raw->target_component = mavlink_msg_rc_channels_message_raw_get_target_component(msg);
	rc_channels_message_raw->chan1_raw = mavlink_msg_rc_channels_message_raw_get_chan1_raw(msg);
	rc_channels_message_raw->chan2_raw = mavlink_msg_rc_channels_message_raw_get_chan2_raw(msg);
	rc_channels_message_raw->chan3_raw = mavlink_msg_rc_channels_message_raw_get_chan3_raw(msg);
	rc_channels_message_raw->chan4_raw = mavlink_msg_rc_channels_message_raw_get_chan4_raw(msg);
	rc_channels_message_raw->chan5_raw = mavlink_msg_rc_channels_message_raw_get_chan5_raw(msg);
	rc_channels_message_raw->chan6_raw = mavlink_msg_rc_channels_message_raw_get_chan6_raw(msg);
	rc_channels_message_raw->chan7_raw = mavlink_msg_rc_channels_message_raw_get_chan7_raw(msg);
	rc_channels_message_raw->chan8_raw = mavlink_msg_rc_channels_message_raw_get_chan8_raw(msg);
}
