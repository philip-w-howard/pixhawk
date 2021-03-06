// MESSAGE PID_TUNING PACKING

#define MAVLINK_MSG_ID_PID_TUNING 194

typedef struct __mavlink_pid_tuning_t
{
 uint8_t axis; ///< axis
 float desired; ///< desired rate (degrees/s)
 float achieved; ///< achieved rate (degrees/s)
 float FF; ///< FF component
 float P; ///< P component
 float I; ///< I component
 float D; ///< D component
} mavlink_pid_tuning_t;

#define MAVLINK_MSG_ID_PID_TUNING_LEN 25
#define MAVLINK_MSG_ID_194_LEN 25

#define MAVLINK_MSG_ID_PID_TUNING_CRC 182
#define MAVLINK_MSG_ID_194_CRC 182



#define MAVLINK_MESSAGE_INFO_PID_TUNING { \
	"PID_TUNING", \
	7, \
	{  { "axis", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_pid_tuning_t, axis) }, \
         { "desired", NULL, MAVLINK_TYPE_FLOAT, 0, 1, offsetof(mavlink_pid_tuning_t, desired) }, \
         { "achieved", NULL, MAVLINK_TYPE_FLOAT, 0, 5, offsetof(mavlink_pid_tuning_t, achieved) }, \
         { "FF", NULL, MAVLINK_TYPE_FLOAT, 0, 9, offsetof(mavlink_pid_tuning_t, FF) }, \
         { "P", NULL, MAVLINK_TYPE_FLOAT, 0, 13, offsetof(mavlink_pid_tuning_t, P) }, \
         { "I", NULL, MAVLINK_TYPE_FLOAT, 0, 17, offsetof(mavlink_pid_tuning_t, I) }, \
         { "D", NULL, MAVLINK_TYPE_FLOAT, 0, 21, offsetof(mavlink_pid_tuning_t, D) }, \
         } \
}


/**
 * @brief Pack a pid_tuning message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param axis axis
 * @param desired desired rate (degrees/s)
 * @param achieved achieved rate (degrees/s)
 * @param FF FF component
 * @param P P component
 * @param I I component
 * @param D D component
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pid_tuning_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t axis, float desired, float achieved, float FF, float P, float I, float D)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PID_TUNING_LEN];
	_mav_put_uint8_t(buf, 0, axis);
	_mav_put_float(buf, 1, desired);
	_mav_put_float(buf, 5, achieved);
	_mav_put_float(buf, 9, FF);
	_mav_put_float(buf, 13, P);
	_mav_put_float(buf, 17, I);
	_mav_put_float(buf, 21, D);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PID_TUNING_LEN);
#else
	mavlink_pid_tuning_t packet;
	packet.axis = axis;
	packet.desired = desired;
	packet.achieved = achieved;
	packet.FF = FF;
	packet.P = P;
	packet.I = I;
	packet.D = D;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PID_TUNING_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PID_TUNING;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PID_TUNING_LEN, MAVLINK_MSG_ID_PID_TUNING_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PID_TUNING_LEN);
#endif
}

/**
 * @brief Pack a pid_tuning message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param axis axis
 * @param desired desired rate (degrees/s)
 * @param achieved achieved rate (degrees/s)
 * @param FF FF component
 * @param P P component
 * @param I I component
 * @param D D component
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pid_tuning_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t axis,float desired,float achieved,float FF,float P,float I,float D)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PID_TUNING_LEN];
	_mav_put_uint8_t(buf, 0, axis);
	_mav_put_float(buf, 1, desired);
	_mav_put_float(buf, 5, achieved);
	_mav_put_float(buf, 9, FF);
	_mav_put_float(buf, 13, P);
	_mav_put_float(buf, 17, I);
	_mav_put_float(buf, 21, D);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PID_TUNING_LEN);
#else
	mavlink_pid_tuning_t packet;
	packet.axis = axis;
	packet.desired = desired;
	packet.achieved = achieved;
	packet.FF = FF;
	packet.P = P;
	packet.I = I;
	packet.D = D;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PID_TUNING_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PID_TUNING;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PID_TUNING_LEN, MAVLINK_MSG_ID_PID_TUNING_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PID_TUNING_LEN);
#endif
}

/**
 * @brief Encode a pid_tuning struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pid_tuning C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pid_tuning_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pid_tuning_t* pid_tuning)
{
	return mavlink_msg_pid_tuning_pack(system_id, component_id, msg, pid_tuning->axis, pid_tuning->desired, pid_tuning->achieved, pid_tuning->FF, pid_tuning->P, pid_tuning->I, pid_tuning->D);
}

/**
 * @brief Encode a pid_tuning struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pid_tuning C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pid_tuning_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_pid_tuning_t* pid_tuning)
{
	return mavlink_msg_pid_tuning_pack_chan(system_id, component_id, chan, msg, pid_tuning->axis, pid_tuning->desired, pid_tuning->achieved, pid_tuning->FF, pid_tuning->P, pid_tuning->I, pid_tuning->D);
}

/**
 * @brief Send a pid_tuning message
 * @param chan MAVLink channel to send the message
 *
 * @param axis axis
 * @param desired desired rate (degrees/s)
 * @param achieved achieved rate (degrees/s)
 * @param FF FF component
 * @param P P component
 * @param I I component
 * @param D D component
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pid_tuning_send(mavlink_channel_t chan, uint8_t axis, float desired, float achieved, float FF, float P, float I, float D)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PID_TUNING_LEN];
	_mav_put_uint8_t(buf, 0, axis);
	_mav_put_float(buf, 1, desired);
	_mav_put_float(buf, 5, achieved);
	_mav_put_float(buf, 9, FF);
	_mav_put_float(buf, 13, P);
	_mav_put_float(buf, 17, I);
	_mav_put_float(buf, 21, D);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_TUNING, buf, MAVLINK_MSG_ID_PID_TUNING_LEN, MAVLINK_MSG_ID_PID_TUNING_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_TUNING, buf, MAVLINK_MSG_ID_PID_TUNING_LEN);
#endif
#else
	mavlink_pid_tuning_t packet;
	packet.axis = axis;
	packet.desired = desired;
	packet.achieved = achieved;
	packet.FF = FF;
	packet.P = P;
	packet.I = I;
	packet.D = D;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_TUNING, (const char *)&packet, MAVLINK_MSG_ID_PID_TUNING_LEN, MAVLINK_MSG_ID_PID_TUNING_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_TUNING, (const char *)&packet, MAVLINK_MSG_ID_PID_TUNING_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_PID_TUNING_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_pid_tuning_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t axis, float desired, float achieved, float FF, float P, float I, float D)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint8_t(buf, 0, axis);
	_mav_put_float(buf, 1, desired);
	_mav_put_float(buf, 5, achieved);
	_mav_put_float(buf, 9, FF);
	_mav_put_float(buf, 13, P);
	_mav_put_float(buf, 17, I);
	_mav_put_float(buf, 21, D);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_TUNING, buf, MAVLINK_MSG_ID_PID_TUNING_LEN, MAVLINK_MSG_ID_PID_TUNING_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_TUNING, buf, MAVLINK_MSG_ID_PID_TUNING_LEN);
#endif
#else
	mavlink_pid_tuning_t *packet = (mavlink_pid_tuning_t *)msgbuf;
	packet->axis = axis;
	packet->desired = desired;
	packet->achieved = achieved;
	packet->FF = FF;
	packet->P = P;
	packet->I = I;
	packet->D = D;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_TUNING, (const char *)packet, MAVLINK_MSG_ID_PID_TUNING_LEN, MAVLINK_MSG_ID_PID_TUNING_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PID_TUNING, (const char *)packet, MAVLINK_MSG_ID_PID_TUNING_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE PID_TUNING UNPACKING


/**
 * @brief Get field axis from pid_tuning message
 *
 * @return axis
 */
static inline uint8_t mavlink_msg_pid_tuning_get_axis(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field desired from pid_tuning message
 *
 * @return desired rate (degrees/s)
 */
static inline float mavlink_msg_pid_tuning_get_desired(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  1);
}

/**
 * @brief Get field achieved from pid_tuning message
 *
 * @return achieved rate (degrees/s)
 */
static inline float mavlink_msg_pid_tuning_get_achieved(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  5);
}

/**
 * @brief Get field FF from pid_tuning message
 *
 * @return FF component
 */
static inline float mavlink_msg_pid_tuning_get_FF(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  9);
}

/**
 * @brief Get field P from pid_tuning message
 *
 * @return P component
 */
static inline float mavlink_msg_pid_tuning_get_P(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  13);
}

/**
 * @brief Get field I from pid_tuning message
 *
 * @return I component
 */
static inline float mavlink_msg_pid_tuning_get_I(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  17);
}

/**
 * @brief Get field D from pid_tuning message
 *
 * @return D component
 */
static inline float mavlink_msg_pid_tuning_get_D(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  21);
}

/**
 * @brief Decode a pid_tuning message into a struct
 *
 * @param msg The message to decode
 * @param pid_tuning C-struct to decode the message contents into
 */
static inline void mavlink_msg_pid_tuning_decode(const mavlink_message_t* msg, mavlink_pid_tuning_t* pid_tuning)
{
#if MAVLINK_NEED_BYTE_SWAP
	pid_tuning->axis = mavlink_msg_pid_tuning_get_axis(msg);
	pid_tuning->desired = mavlink_msg_pid_tuning_get_desired(msg);
	pid_tuning->achieved = mavlink_msg_pid_tuning_get_achieved(msg);
	pid_tuning->FF = mavlink_msg_pid_tuning_get_FF(msg);
	pid_tuning->P = mavlink_msg_pid_tuning_get_P(msg);
	pid_tuning->I = mavlink_msg_pid_tuning_get_I(msg);
	pid_tuning->D = mavlink_msg_pid_tuning_get_D(msg);
#else
	memcpy(pid_tuning, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_PID_TUNING_LEN);
#endif
}
