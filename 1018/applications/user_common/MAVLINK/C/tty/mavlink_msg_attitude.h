#pragma once
// MESSAGE ATTITUDE PACKING

#define MAVLINK_MSG_ID_ATTITUDE 4


typedef struct __mavlink_attitude_t {
 float roll; /*< [rad] */
 float pitch; /*< [rad] */
 float yaw; /*< [rad] */
 float roll_speed; /*< [rad/s] */
 float pitch_speed; /*< [rad/s] */
 float yaw_speed; /*< [rad/s] */
} mavlink_attitude_t;

#define MAVLINK_MSG_ID_ATTITUDE_LEN 24
#define MAVLINK_MSG_ID_ATTITUDE_MIN_LEN 24
#define MAVLINK_MSG_ID_4_LEN 24
#define MAVLINK_MSG_ID_4_MIN_LEN 24

#define MAVLINK_MSG_ID_ATTITUDE_CRC 185
#define MAVLINK_MSG_ID_4_CRC 185



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ATTITUDE { \
    4, \
    "ATTITUDE", \
    6, \
    {  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_attitude_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_attitude_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_attitude_t, yaw) }, \
         { "roll_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_attitude_t, roll_speed) }, \
         { "pitch_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_attitude_t, pitch_speed) }, \
         { "yaw_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_attitude_t, yaw_speed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ATTITUDE { \
    "ATTITUDE", \
    6, \
    {  { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_attitude_t, roll) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_attitude_t, pitch) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_attitude_t, yaw) }, \
         { "roll_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_attitude_t, roll_speed) }, \
         { "pitch_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_attitude_t, pitch_speed) }, \
         { "yaw_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_attitude_t, yaw_speed) }, \
         } \
}
#endif

/**
 * @brief Pack a attitude message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll [rad] 
 * @param pitch [rad] 
 * @param yaw [rad] 
 * @param roll_speed [rad/s] 
 * @param pitch_speed [rad/s] 
 * @param yaw_speed [rad/s] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float roll, float pitch, float yaw, float roll_speed, float pitch_speed, float yaw_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, roll_speed);
    _mav_put_float(buf, 16, pitch_speed);
    _mav_put_float(buf, 20, yaw_speed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_LEN);
#else
    mavlink_attitude_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.roll_speed = roll_speed;
    packet.pitch_speed = pitch_speed;
    packet.yaw_speed = yaw_speed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
}

/**
 * @brief Pack a attitude message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll [rad] 
 * @param pitch [rad] 
 * @param yaw [rad] 
 * @param roll_speed [rad/s] 
 * @param pitch_speed [rad/s] 
 * @param yaw_speed [rad/s] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_attitude_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float roll,float pitch,float yaw,float roll_speed,float pitch_speed,float yaw_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, roll_speed);
    _mav_put_float(buf, 16, pitch_speed);
    _mav_put_float(buf, 20, yaw_speed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ATTITUDE_LEN);
#else
    mavlink_attitude_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.roll_speed = roll_speed;
    packet.pitch_speed = pitch_speed;
    packet.yaw_speed = yaw_speed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ATTITUDE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ATTITUDE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
}

/**
 * @brief Encode a attitude struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_attitude_t* attitude)
{
    return mavlink_msg_attitude_pack(system_id, component_id, msg, attitude->roll, attitude->pitch, attitude->yaw, attitude->roll_speed, attitude->pitch_speed, attitude->yaw_speed);
}

/**
 * @brief Encode a attitude struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param attitude C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_attitude_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_attitude_t* attitude)
{
    return mavlink_msg_attitude_pack_chan(system_id, component_id, chan, msg, attitude->roll, attitude->pitch, attitude->yaw, attitude->roll_speed, attitude->pitch_speed, attitude->yaw_speed);
}

/**
 * @brief Send a attitude message
 * @param chan MAVLink channel to send the message
 *
 * @param roll [rad] 
 * @param pitch [rad] 
 * @param yaw [rad] 
 * @param roll_speed [rad/s] 
 * @param pitch_speed [rad/s] 
 * @param yaw_speed [rad/s] 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_attitude_send(mavlink_channel_t chan, float roll, float pitch, float yaw, float roll_speed, float pitch_speed, float yaw_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ATTITUDE_LEN];
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, roll_speed);
    _mav_put_float(buf, 16, pitch_speed);
    _mav_put_float(buf, 20, yaw_speed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, buf, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
#else
    mavlink_attitude_t packet;
    packet.roll = roll;
    packet.pitch = pitch;
    packet.yaw = yaw;
    packet.roll_speed = roll_speed;
    packet.pitch_speed = pitch_speed;
    packet.yaw_speed = yaw_speed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, (const char *)&packet, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
#endif
}

/**
 * @brief Send a attitude message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_attitude_send_struct(mavlink_channel_t chan, const mavlink_attitude_t* attitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_attitude_send(chan, attitude->roll, attitude->pitch, attitude->yaw, attitude->roll_speed, attitude->pitch_speed, attitude->yaw_speed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, (const char *)attitude, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
#endif
}

#if MAVLINK_MSG_ID_ATTITUDE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_attitude_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float roll, float pitch, float yaw, float roll_speed, float pitch_speed, float yaw_speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, roll);
    _mav_put_float(buf, 4, pitch);
    _mav_put_float(buf, 8, yaw);
    _mav_put_float(buf, 12, roll_speed);
    _mav_put_float(buf, 16, pitch_speed);
    _mav_put_float(buf, 20, yaw_speed);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, buf, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
#else
    mavlink_attitude_t *packet = (mavlink_attitude_t *)msgbuf;
    packet->roll = roll;
    packet->pitch = pitch;
    packet->yaw = yaw;
    packet->roll_speed = roll_speed;
    packet->pitch_speed = pitch_speed;
    packet->yaw_speed = yaw_speed;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ATTITUDE, (const char *)packet, MAVLINK_MSG_ID_ATTITUDE_MIN_LEN, MAVLINK_MSG_ID_ATTITUDE_LEN, MAVLINK_MSG_ID_ATTITUDE_CRC);
#endif
}
#endif

#endif

// MESSAGE ATTITUDE UNPACKING


/**
 * @brief Get field roll from attitude message
 *
 * @return [rad] 
 */
static inline float mavlink_msg_attitude_get_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field pitch from attitude message
 *
 * @return [rad] 
 */
static inline float mavlink_msg_attitude_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field yaw from attitude message
 *
 * @return [rad] 
 */
static inline float mavlink_msg_attitude_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field roll_speed from attitude message
 *
 * @return [rad/s] 
 */
static inline float mavlink_msg_attitude_get_roll_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field pitch_speed from attitude message
 *
 * @return [rad/s] 
 */
static inline float mavlink_msg_attitude_get_pitch_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field yaw_speed from attitude message
 *
 * @return [rad/s] 
 */
static inline float mavlink_msg_attitude_get_yaw_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a attitude message into a struct
 *
 * @param msg The message to decode
 * @param attitude C-struct to decode the message contents into
 */
static inline void mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    attitude->roll = mavlink_msg_attitude_get_roll(msg);
    attitude->pitch = mavlink_msg_attitude_get_pitch(msg);
    attitude->yaw = mavlink_msg_attitude_get_yaw(msg);
    attitude->roll_speed = mavlink_msg_attitude_get_roll_speed(msg);
    attitude->pitch_speed = mavlink_msg_attitude_get_pitch_speed(msg);
    attitude->yaw_speed = mavlink_msg_attitude_get_yaw_speed(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ATTITUDE_LEN? msg->len : MAVLINK_MSG_ID_ATTITUDE_LEN;
        memset(attitude, 0, MAVLINK_MSG_ID_ATTITUDE_LEN);
    memcpy(attitude, _MAV_PAYLOAD(msg), len);
#endif
}
