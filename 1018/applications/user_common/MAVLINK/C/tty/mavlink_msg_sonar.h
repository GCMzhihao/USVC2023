#pragma once
// MESSAGE SONAR PACKING

#define MAVLINK_MSG_ID_SONAR 3


typedef struct __mavlink_sonar_t {
 float distance; /*< [m] 测量距离*/
 float distance_alt; /*< [m] 高度*/
} mavlink_sonar_t;

#define MAVLINK_MSG_ID_SONAR_LEN 8
#define MAVLINK_MSG_ID_SONAR_MIN_LEN 8
#define MAVLINK_MSG_ID_3_LEN 8
#define MAVLINK_MSG_ID_3_MIN_LEN 8

#define MAVLINK_MSG_ID_SONAR_CRC 58
#define MAVLINK_MSG_ID_3_CRC 58



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SONAR { \
    3, \
    "SONAR", \
    2, \
    {  { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sonar_t, distance) }, \
         { "distance_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sonar_t, distance_alt) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SONAR { \
    "SONAR", \
    2, \
    {  { "distance", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sonar_t, distance) }, \
         { "distance_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sonar_t, distance_alt) }, \
         } \
}
#endif

/**
 * @brief Pack a sonar message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param distance [m] 测量距离
 * @param distance_alt [m] 高度
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sonar_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float distance, float distance_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SONAR_LEN];
    _mav_put_float(buf, 0, distance);
    _mav_put_float(buf, 4, distance_alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SONAR_LEN);
#else
    mavlink_sonar_t packet;
    packet.distance = distance;
    packet.distance_alt = distance_alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SONAR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SONAR;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SONAR_MIN_LEN, MAVLINK_MSG_ID_SONAR_LEN, MAVLINK_MSG_ID_SONAR_CRC);
}

/**
 * @brief Pack a sonar message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param distance [m] 测量距离
 * @param distance_alt [m] 高度
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sonar_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float distance,float distance_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SONAR_LEN];
    _mav_put_float(buf, 0, distance);
    _mav_put_float(buf, 4, distance_alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SONAR_LEN);
#else
    mavlink_sonar_t packet;
    packet.distance = distance;
    packet.distance_alt = distance_alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SONAR_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SONAR;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SONAR_MIN_LEN, MAVLINK_MSG_ID_SONAR_LEN, MAVLINK_MSG_ID_SONAR_CRC);
}

/**
 * @brief Encode a sonar struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sonar C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sonar_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sonar_t* sonar)
{
    return mavlink_msg_sonar_pack(system_id, component_id, msg, sonar->distance, sonar->distance_alt);
}

/**
 * @brief Encode a sonar struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sonar C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sonar_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sonar_t* sonar)
{
    return mavlink_msg_sonar_pack_chan(system_id, component_id, chan, msg, sonar->distance, sonar->distance_alt);
}

/**
 * @brief Send a sonar message
 * @param chan MAVLink channel to send the message
 *
 * @param distance [m] 测量距离
 * @param distance_alt [m] 高度
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sonar_send(mavlink_channel_t chan, float distance, float distance_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SONAR_LEN];
    _mav_put_float(buf, 0, distance);
    _mav_put_float(buf, 4, distance_alt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SONAR, buf, MAVLINK_MSG_ID_SONAR_MIN_LEN, MAVLINK_MSG_ID_SONAR_LEN, MAVLINK_MSG_ID_SONAR_CRC);
#else
    mavlink_sonar_t packet;
    packet.distance = distance;
    packet.distance_alt = distance_alt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SONAR, (const char *)&packet, MAVLINK_MSG_ID_SONAR_MIN_LEN, MAVLINK_MSG_ID_SONAR_LEN, MAVLINK_MSG_ID_SONAR_CRC);
#endif
}

/**
 * @brief Send a sonar message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sonar_send_struct(mavlink_channel_t chan, const mavlink_sonar_t* sonar)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sonar_send(chan, sonar->distance, sonar->distance_alt);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SONAR, (const char *)sonar, MAVLINK_MSG_ID_SONAR_MIN_LEN, MAVLINK_MSG_ID_SONAR_LEN, MAVLINK_MSG_ID_SONAR_CRC);
#endif
}

#if MAVLINK_MSG_ID_SONAR_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sonar_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float distance, float distance_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, distance);
    _mav_put_float(buf, 4, distance_alt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SONAR, buf, MAVLINK_MSG_ID_SONAR_MIN_LEN, MAVLINK_MSG_ID_SONAR_LEN, MAVLINK_MSG_ID_SONAR_CRC);
#else
    mavlink_sonar_t *packet = (mavlink_sonar_t *)msgbuf;
    packet->distance = distance;
    packet->distance_alt = distance_alt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SONAR, (const char *)packet, MAVLINK_MSG_ID_SONAR_MIN_LEN, MAVLINK_MSG_ID_SONAR_LEN, MAVLINK_MSG_ID_SONAR_CRC);
#endif
}
#endif

#endif

// MESSAGE SONAR UNPACKING


/**
 * @brief Get field distance from sonar message
 *
 * @return [m] 测量距离
 */
static inline float mavlink_msg_sonar_get_distance(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field distance_alt from sonar message
 *
 * @return [m] 高度
 */
static inline float mavlink_msg_sonar_get_distance_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a sonar message into a struct
 *
 * @param msg The message to decode
 * @param sonar C-struct to decode the message contents into
 */
static inline void mavlink_msg_sonar_decode(const mavlink_message_t* msg, mavlink_sonar_t* sonar)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sonar->distance = mavlink_msg_sonar_get_distance(msg);
    sonar->distance_alt = mavlink_msg_sonar_get_distance_alt(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SONAR_LEN? msg->len : MAVLINK_MSG_ID_SONAR_LEN;
        memset(sonar, 0, MAVLINK_MSG_ID_SONAR_LEN);
    memcpy(sonar, _MAV_PAYLOAD(msg), len);
#endif
}
