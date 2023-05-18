#pragma once
// MESSAGE PARAM_READ_REQUEST PACKING

#define MAVLINK_MSG_ID_PARAM_READ_REQUEST 7


typedef struct __mavlink_param_read_request_t {
 uint8_t SYS_TYPE; /*<  */
 uint8_t DEV_ID; /*<  */
 uint8_t param_id; /*<  参数类型，详见参数定义*/
} mavlink_param_read_request_t;

#define MAVLINK_MSG_ID_PARAM_READ_REQUEST_LEN 3
#define MAVLINK_MSG_ID_PARAM_READ_REQUEST_MIN_LEN 3
#define MAVLINK_MSG_ID_7_LEN 3
#define MAVLINK_MSG_ID_7_MIN_LEN 3

#define MAVLINK_MSG_ID_PARAM_READ_REQUEST_CRC 97
#define MAVLINK_MSG_ID_7_CRC 97



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PARAM_READ_REQUEST { \
    7, \
    "PARAM_READ_REQUEST", \
    3, \
    {  { "SYS_TYPE", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_param_read_request_t, SYS_TYPE) }, \
         { "DEV_ID", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_param_read_request_t, DEV_ID) }, \
         { "param_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_param_read_request_t, param_id) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PARAM_READ_REQUEST { \
    "PARAM_READ_REQUEST", \
    3, \
    {  { "SYS_TYPE", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_param_read_request_t, SYS_TYPE) }, \
         { "DEV_ID", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_param_read_request_t, DEV_ID) }, \
         { "param_id", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_param_read_request_t, param_id) }, \
         } \
}
#endif

/**
 * @brief Pack a param_read_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param SYS_TYPE  
 * @param DEV_ID  
 * @param param_id  参数类型，详见参数定义
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_read_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t SYS_TYPE, uint8_t DEV_ID, uint8_t param_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAM_READ_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, SYS_TYPE);
    _mav_put_uint8_t(buf, 1, DEV_ID);
    _mav_put_uint8_t(buf, 2, param_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAM_READ_REQUEST_LEN);
#else
    mavlink_param_read_request_t packet;
    packet.SYS_TYPE = SYS_TYPE;
    packet.DEV_ID = DEV_ID;
    packet.param_id = param_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAM_READ_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAM_READ_REQUEST;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PARAM_READ_REQUEST_MIN_LEN, MAVLINK_MSG_ID_PARAM_READ_REQUEST_LEN, MAVLINK_MSG_ID_PARAM_READ_REQUEST_CRC);
}

/**
 * @brief Pack a param_read_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param SYS_TYPE  
 * @param DEV_ID  
 * @param param_id  参数类型，详见参数定义
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_read_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t SYS_TYPE,uint8_t DEV_ID,uint8_t param_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAM_READ_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, SYS_TYPE);
    _mav_put_uint8_t(buf, 1, DEV_ID);
    _mav_put_uint8_t(buf, 2, param_id);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAM_READ_REQUEST_LEN);
#else
    mavlink_param_read_request_t packet;
    packet.SYS_TYPE = SYS_TYPE;
    packet.DEV_ID = DEV_ID;
    packet.param_id = param_id;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAM_READ_REQUEST_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PARAM_READ_REQUEST;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PARAM_READ_REQUEST_MIN_LEN, MAVLINK_MSG_ID_PARAM_READ_REQUEST_LEN, MAVLINK_MSG_ID_PARAM_READ_REQUEST_CRC);
}

/**
 * @brief Encode a param_read_request struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param param_read_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_read_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_param_read_request_t* param_read_request)
{
    return mavlink_msg_param_read_request_pack(system_id, component_id, msg, param_read_request->SYS_TYPE, param_read_request->DEV_ID, param_read_request->param_id);
}

/**
 * @brief Encode a param_read_request struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param param_read_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_read_request_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_param_read_request_t* param_read_request)
{
    return mavlink_msg_param_read_request_pack_chan(system_id, component_id, chan, msg, param_read_request->SYS_TYPE, param_read_request->DEV_ID, param_read_request->param_id);
}

/**
 * @brief Send a param_read_request message
 * @param chan MAVLink channel to send the message
 *
 * @param SYS_TYPE  
 * @param DEV_ID  
 * @param param_id  参数类型，详见参数定义
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_param_read_request_send(mavlink_channel_t chan, uint8_t SYS_TYPE, uint8_t DEV_ID, uint8_t param_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PARAM_READ_REQUEST_LEN];
    _mav_put_uint8_t(buf, 0, SYS_TYPE);
    _mav_put_uint8_t(buf, 1, DEV_ID);
    _mav_put_uint8_t(buf, 2, param_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_READ_REQUEST, buf, MAVLINK_MSG_ID_PARAM_READ_REQUEST_MIN_LEN, MAVLINK_MSG_ID_PARAM_READ_REQUEST_LEN, MAVLINK_MSG_ID_PARAM_READ_REQUEST_CRC);
#else
    mavlink_param_read_request_t packet;
    packet.SYS_TYPE = SYS_TYPE;
    packet.DEV_ID = DEV_ID;
    packet.param_id = param_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_READ_REQUEST, (const char *)&packet, MAVLINK_MSG_ID_PARAM_READ_REQUEST_MIN_LEN, MAVLINK_MSG_ID_PARAM_READ_REQUEST_LEN, MAVLINK_MSG_ID_PARAM_READ_REQUEST_CRC);
#endif
}

/**
 * @brief Send a param_read_request message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_param_read_request_send_struct(mavlink_channel_t chan, const mavlink_param_read_request_t* param_read_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_param_read_request_send(chan, param_read_request->SYS_TYPE, param_read_request->DEV_ID, param_read_request->param_id);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_READ_REQUEST, (const char *)param_read_request, MAVLINK_MSG_ID_PARAM_READ_REQUEST_MIN_LEN, MAVLINK_MSG_ID_PARAM_READ_REQUEST_LEN, MAVLINK_MSG_ID_PARAM_READ_REQUEST_CRC);
#endif
}

#if MAVLINK_MSG_ID_PARAM_READ_REQUEST_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_param_read_request_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t SYS_TYPE, uint8_t DEV_ID, uint8_t param_id)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, SYS_TYPE);
    _mav_put_uint8_t(buf, 1, DEV_ID);
    _mav_put_uint8_t(buf, 2, param_id);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_READ_REQUEST, buf, MAVLINK_MSG_ID_PARAM_READ_REQUEST_MIN_LEN, MAVLINK_MSG_ID_PARAM_READ_REQUEST_LEN, MAVLINK_MSG_ID_PARAM_READ_REQUEST_CRC);
#else
    mavlink_param_read_request_t *packet = (mavlink_param_read_request_t *)msgbuf;
    packet->SYS_TYPE = SYS_TYPE;
    packet->DEV_ID = DEV_ID;
    packet->param_id = param_id;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_READ_REQUEST, (const char *)packet, MAVLINK_MSG_ID_PARAM_READ_REQUEST_MIN_LEN, MAVLINK_MSG_ID_PARAM_READ_REQUEST_LEN, MAVLINK_MSG_ID_PARAM_READ_REQUEST_CRC);
#endif
}
#endif

#endif

// MESSAGE PARAM_READ_REQUEST UNPACKING


/**
 * @brief Get field SYS_TYPE from param_read_request message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_param_read_request_get_SYS_TYPE(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field DEV_ID from param_read_request message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_param_read_request_get_DEV_ID(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field param_id from param_read_request message
 *
 * @return  参数类型，详见参数定义
 */
static inline uint8_t mavlink_msg_param_read_request_get_param_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a param_read_request message into a struct
 *
 * @param msg The message to decode
 * @param param_read_request C-struct to decode the message contents into
 */
static inline void mavlink_msg_param_read_request_decode(const mavlink_message_t* msg, mavlink_param_read_request_t* param_read_request)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    param_read_request->SYS_TYPE = mavlink_msg_param_read_request_get_SYS_TYPE(msg);
    param_read_request->DEV_ID = mavlink_msg_param_read_request_get_DEV_ID(msg);
    param_read_request->param_id = mavlink_msg_param_read_request_get_param_id(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PARAM_READ_REQUEST_LEN? msg->len : MAVLINK_MSG_ID_PARAM_READ_REQUEST_LEN;
        memset(param_read_request, 0, MAVLINK_MSG_ID_PARAM_READ_REQUEST_LEN);
    memcpy(param_read_request, _MAV_PAYLOAD(msg), len);
#endif
}
