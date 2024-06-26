#pragma once
// MESSAGE PARAM_WRITE PACKING

#define MAVLINK_MSG_ID_PARAM_WRITE 3


typedef struct __mavlink_param_write_t {
 float value; /*<  */
 uint8_t param_id; /*<  参数类型，详见参数定义*/
} mavlink_param_write_t;

#define MAVLINK_MSG_ID_PARAM_WRITE_LEN 5
#define MAVLINK_MSG_ID_PARAM_WRITE_MIN_LEN 5
#define MAVLINK_MSG_ID_3_LEN 5
#define MAVLINK_MSG_ID_3_MIN_LEN 5

#define MAVLINK_MSG_ID_PARAM_WRITE_CRC 11
#define MAVLINK_MSG_ID_3_CRC 11



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PARAM_WRITE { \
    3, \
    "PARAM_WRITE", \
    2, \
    {  { "param_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_param_write_t, param_id) }, \
         { "value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_param_write_t, value) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PARAM_WRITE { \
    "PARAM_WRITE", \
    2, \
    {  { "param_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_param_write_t, param_id) }, \
         { "value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_param_write_t, value) }, \
         } \
}
#endif

/**
 * @brief Pack a param_write message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param param_id  参数类型，详见参数定义
 * @param value  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_write_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t param_id, float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAM_WRITE_LEN];
    _mav_put_float(buf, 0, value);
    _mav_put_uint8_t(buf, 4, param_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAM_WRITE_LEN);
#else
    mavlink_param_write_t packet;
    packet.value = value;
    packet.param_id = param_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAM_WRITE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAM_WRITE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PARAM_WRITE_MIN_LEN, MAVLINK_MSG_ID_PARAM_WRITE_LEN, MAVLINK_MSG_ID_PARAM_WRITE_CRC);
}

/**
 * @brief Pack a param_write message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param param_id  参数类型，详见参数定义
 * @param value  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_write_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t param_id,float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAM_WRITE_LEN];
    _mav_put_float(buf, 0, value);
    _mav_put_uint8_t(buf, 4, param_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAM_WRITE_LEN);
#else
    mavlink_param_write_t packet;
    packet.value = value;
    packet.param_id = param_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAM_WRITE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAM_WRITE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PARAM_WRITE_MIN_LEN, MAVLINK_MSG_ID_PARAM_WRITE_LEN, MAVLINK_MSG_ID_PARAM_WRITE_CRC);
}

/**
 * @brief Encode a param_write struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param param_write C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_write_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_param_write_t* param_write)
{
    return mavlink_msg_param_write_pack(system_id, component_id, msg, param_write->param_id, param_write->value);
}

/**
 * @brief Encode a param_write struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param param_write C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_write_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_param_write_t* param_write)
{
    return mavlink_msg_param_write_pack_chan(system_id, component_id, chan, msg, param_write->param_id, param_write->value);
}

/**
 * @brief Send a param_write message
 * @param chan MAVLink channel to send the message
 *
 * @param param_id  参数类型，详见参数定义
 * @param value  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_param_write_send(mavlink_channel_t chan, uint8_t param_id, float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAM_WRITE_LEN];
    _mav_put_float(buf, 0, value);
    _mav_put_uint8_t(buf, 4, param_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_WRITE, buf, MAVLINK_MSG_ID_PARAM_WRITE_MIN_LEN, MAVLINK_MSG_ID_PARAM_WRITE_LEN, MAVLINK_MSG_ID_PARAM_WRITE_CRC);
#else
    mavlink_param_write_t packet;
    packet.value = value;
    packet.param_id = param_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_WRITE, (const char *)&packet, MAVLINK_MSG_ID_PARAM_WRITE_MIN_LEN, MAVLINK_MSG_ID_PARAM_WRITE_LEN, MAVLINK_MSG_ID_PARAM_WRITE_CRC);
#endif
}

/**
 * @brief Send a param_write message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_param_write_send_struct(mavlink_channel_t chan, const mavlink_param_write_t* param_write)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_param_write_send(chan, param_write->param_id, param_write->value);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_WRITE, (const char *)param_write, MAVLINK_MSG_ID_PARAM_WRITE_MIN_LEN, MAVLINK_MSG_ID_PARAM_WRITE_LEN, MAVLINK_MSG_ID_PARAM_WRITE_CRC);
#endif
}

#if MAVLINK_MSG_ID_PARAM_WRITE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_param_write_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t param_id, float value)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, value);
    _mav_put_uint8_t(buf, 4, param_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_WRITE, buf, MAVLINK_MSG_ID_PARAM_WRITE_MIN_LEN, MAVLINK_MSG_ID_PARAM_WRITE_LEN, MAVLINK_MSG_ID_PARAM_WRITE_CRC);
#else
    mavlink_param_write_t *packet = (mavlink_param_write_t *)msgbuf;
    packet->value = value;
    packet->param_id = param_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_WRITE, (const char *)packet, MAVLINK_MSG_ID_PARAM_WRITE_MIN_LEN, MAVLINK_MSG_ID_PARAM_WRITE_LEN, MAVLINK_MSG_ID_PARAM_WRITE_CRC);
#endif
}
#endif

#endif

// MESSAGE PARAM_WRITE UNPACKING


/**
 * @brief Get field param_id from param_write message
 *
 * @return  参数类型，详见参数定义
 */
static inline uint8_t mavlink_msg_param_write_get_param_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field value from param_write message
 *
 * @return  
 */
static inline float mavlink_msg_param_write_get_value(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a param_write message into a struct
 *
 * @param msg The message to decode
 * @param param_write C-struct to decode the message contents into
 */
static inline void mavlink_msg_param_write_decode(const mavlink_message_t* msg, mavlink_param_write_t* param_write)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    param_write->value = mavlink_msg_param_write_get_value(msg);
    param_write->param_id = mavlink_msg_param_write_get_param_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PARAM_WRITE_LEN? msg->len : MAVLINK_MSG_ID_PARAM_WRITE_LEN;
        memset(param_write, 0, MAVLINK_MSG_ID_PARAM_WRITE_LEN);
    memcpy(param_write, _MAV_PAYLOAD(msg), len);
#endif
}
