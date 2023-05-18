#pragma once
// MESSAGE MOTION_SET PACKING

#define MAVLINK_MSG_ID_MOTION_SET 16


typedef struct __mavlink_motion_set_t {
 float X_Speed; /*<  */
 float Y_Speed; /*<  */
 float Z_Speed; /*<  */
 uint8_t SYS_TYPE; /*<  */
 uint8_t DEV_ID; /*<  */
} mavlink_motion_set_t;

#define MAVLINK_MSG_ID_MOTION_SET_LEN 14
#define MAVLINK_MSG_ID_MOTION_SET_MIN_LEN 14
#define MAVLINK_MSG_ID_16_LEN 14
#define MAVLINK_MSG_ID_16_MIN_LEN 14

#define MAVLINK_MSG_ID_MOTION_SET_CRC 131
#define MAVLINK_MSG_ID_16_CRC 131



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MOTION_SET { \
    16, \
    "MOTION_SET", \
    5, \
    {  { "SYS_TYPE", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_motion_set_t, SYS_TYPE) }, \
         { "DEV_ID", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_motion_set_t, DEV_ID) }, \
         { "X_Speed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_motion_set_t, X_Speed) }, \
         { "Y_Speed", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_motion_set_t, Y_Speed) }, \
         { "Z_Speed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_motion_set_t, Z_Speed) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MOTION_SET { \
    "MOTION_SET", \
    5, \
    {  { "SYS_TYPE", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_motion_set_t, SYS_TYPE) }, \
         { "DEV_ID", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_motion_set_t, DEV_ID) }, \
         { "X_Speed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_motion_set_t, X_Speed) }, \
         { "Y_Speed", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_motion_set_t, Y_Speed) }, \
         { "Z_Speed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_motion_set_t, Z_Speed) }, \
         } \
}
#endif

/**
 * @brief Pack a motion_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param SYS_TYPE  
 * @param DEV_ID  
 * @param X_Speed  
 * @param Y_Speed  
 * @param Z_Speed  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motion_set_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t SYS_TYPE, uint8_t DEV_ID, float X_Speed, float Y_Speed, float Z_Speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTION_SET_LEN];
    _mav_put_float(buf, 0, X_Speed);
    _mav_put_float(buf, 4, Y_Speed);
    _mav_put_float(buf, 8, Z_Speed);
    _mav_put_uint8_t(buf, 12, SYS_TYPE);
    _mav_put_uint8_t(buf, 13, DEV_ID);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTION_SET_LEN);
#else
    mavlink_motion_set_t packet;
    packet.X_Speed = X_Speed;
    packet.Y_Speed = Y_Speed;
    packet.Z_Speed = Z_Speed;
    packet.SYS_TYPE = SYS_TYPE;
    packet.DEV_ID = DEV_ID;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTION_SET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTION_SET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MOTION_SET_MIN_LEN, MAVLINK_MSG_ID_MOTION_SET_LEN, MAVLINK_MSG_ID_MOTION_SET_CRC);
}

/**
 * @brief Pack a motion_set message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param SYS_TYPE  
 * @param DEV_ID  
 * @param X_Speed  
 * @param Y_Speed  
 * @param Z_Speed  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_motion_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t SYS_TYPE,uint8_t DEV_ID,float X_Speed,float Y_Speed,float Z_Speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTION_SET_LEN];
    _mav_put_float(buf, 0, X_Speed);
    _mav_put_float(buf, 4, Y_Speed);
    _mav_put_float(buf, 8, Z_Speed);
    _mav_put_uint8_t(buf, 12, SYS_TYPE);
    _mav_put_uint8_t(buf, 13, DEV_ID);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MOTION_SET_LEN);
#else
    mavlink_motion_set_t packet;
    packet.X_Speed = X_Speed;
    packet.Y_Speed = Y_Speed;
    packet.Z_Speed = Z_Speed;
    packet.SYS_TYPE = SYS_TYPE;
    packet.DEV_ID = DEV_ID;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MOTION_SET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MOTION_SET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MOTION_SET_MIN_LEN, MAVLINK_MSG_ID_MOTION_SET_LEN, MAVLINK_MSG_ID_MOTION_SET_CRC);
}

/**
 * @brief Encode a motion_set struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param motion_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motion_set_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_motion_set_t* motion_set)
{
    return mavlink_msg_motion_set_pack(system_id, component_id, msg, motion_set->SYS_TYPE, motion_set->DEV_ID, motion_set->X_Speed, motion_set->Y_Speed, motion_set->Z_Speed);
}

/**
 * @brief Encode a motion_set struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param motion_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_motion_set_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_motion_set_t* motion_set)
{
    return mavlink_msg_motion_set_pack_chan(system_id, component_id, chan, msg, motion_set->SYS_TYPE, motion_set->DEV_ID, motion_set->X_Speed, motion_set->Y_Speed, motion_set->Z_Speed);
}

/**
 * @brief Send a motion_set message
 * @param chan MAVLink channel to send the message
 *
 * @param SYS_TYPE  
 * @param DEV_ID  
 * @param X_Speed  
 * @param Y_Speed  
 * @param Z_Speed  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_motion_set_send(mavlink_channel_t chan, uint8_t SYS_TYPE, uint8_t DEV_ID, float X_Speed, float Y_Speed, float Z_Speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MOTION_SET_LEN];
    _mav_put_float(buf, 0, X_Speed);
    _mav_put_float(buf, 4, Y_Speed);
    _mav_put_float(buf, 8, Z_Speed);
    _mav_put_uint8_t(buf, 12, SYS_TYPE);
    _mav_put_uint8_t(buf, 13, DEV_ID);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTION_SET, buf, MAVLINK_MSG_ID_MOTION_SET_MIN_LEN, MAVLINK_MSG_ID_MOTION_SET_LEN, MAVLINK_MSG_ID_MOTION_SET_CRC);
#else
    mavlink_motion_set_t packet;
    packet.X_Speed = X_Speed;
    packet.Y_Speed = Y_Speed;
    packet.Z_Speed = Z_Speed;
    packet.SYS_TYPE = SYS_TYPE;
    packet.DEV_ID = DEV_ID;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTION_SET, (const char *)&packet, MAVLINK_MSG_ID_MOTION_SET_MIN_LEN, MAVLINK_MSG_ID_MOTION_SET_LEN, MAVLINK_MSG_ID_MOTION_SET_CRC);
#endif
}

/**
 * @brief Send a motion_set message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_motion_set_send_struct(mavlink_channel_t chan, const mavlink_motion_set_t* motion_set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_motion_set_send(chan, motion_set->SYS_TYPE, motion_set->DEV_ID, motion_set->X_Speed, motion_set->Y_Speed, motion_set->Z_Speed);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTION_SET, (const char *)motion_set, MAVLINK_MSG_ID_MOTION_SET_MIN_LEN, MAVLINK_MSG_ID_MOTION_SET_LEN, MAVLINK_MSG_ID_MOTION_SET_CRC);
#endif
}

#if MAVLINK_MSG_ID_MOTION_SET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_motion_set_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t SYS_TYPE, uint8_t DEV_ID, float X_Speed, float Y_Speed, float Z_Speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, X_Speed);
    _mav_put_float(buf, 4, Y_Speed);
    _mav_put_float(buf, 8, Z_Speed);
    _mav_put_uint8_t(buf, 12, SYS_TYPE);
    _mav_put_uint8_t(buf, 13, DEV_ID);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTION_SET, buf, MAVLINK_MSG_ID_MOTION_SET_MIN_LEN, MAVLINK_MSG_ID_MOTION_SET_LEN, MAVLINK_MSG_ID_MOTION_SET_CRC);
#else
    mavlink_motion_set_t *packet = (mavlink_motion_set_t *)msgbuf;
    packet->X_Speed = X_Speed;
    packet->Y_Speed = Y_Speed;
    packet->Z_Speed = Z_Speed;
    packet->SYS_TYPE = SYS_TYPE;
    packet->DEV_ID = DEV_ID;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MOTION_SET, (const char *)packet, MAVLINK_MSG_ID_MOTION_SET_MIN_LEN, MAVLINK_MSG_ID_MOTION_SET_LEN, MAVLINK_MSG_ID_MOTION_SET_CRC);
#endif
}
#endif

#endif

// MESSAGE MOTION_SET UNPACKING


/**
 * @brief Get field SYS_TYPE from motion_set message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_motion_set_get_SYS_TYPE(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field DEV_ID from motion_set message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_motion_set_get_DEV_ID(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field X_Speed from motion_set message
 *
 * @return  
 */
static inline float mavlink_msg_motion_set_get_X_Speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field Y_Speed from motion_set message
 *
 * @return  
 */
static inline float mavlink_msg_motion_set_get_Y_Speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field Z_Speed from motion_set message
 *
 * @return  
 */
static inline float mavlink_msg_motion_set_get_Z_Speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a motion_set message into a struct
 *
 * @param msg The message to decode
 * @param motion_set C-struct to decode the message contents into
 */
static inline void mavlink_msg_motion_set_decode(const mavlink_message_t* msg, mavlink_motion_set_t* motion_set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    motion_set->X_Speed = mavlink_msg_motion_set_get_X_Speed(msg);
    motion_set->Y_Speed = mavlink_msg_motion_set_get_Y_Speed(msg);
    motion_set->Z_Speed = mavlink_msg_motion_set_get_Z_Speed(msg);
    motion_set->SYS_TYPE = mavlink_msg_motion_set_get_SYS_TYPE(msg);
    motion_set->DEV_ID = mavlink_msg_motion_set_get_DEV_ID(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MOTION_SET_LEN? msg->len : MAVLINK_MSG_ID_MOTION_SET_LEN;
        memset(motion_set, 0, MAVLINK_MSG_ID_MOTION_SET_LEN);
    memcpy(motion_set, _MAV_PAYLOAD(msg), len);
#endif
}
