#pragma once
// MESSAGE USV_SET PACKING

#define MAVLINK_MSG_ID_USV_SET 15


typedef struct __mavlink_usv_set_t {
 int16_t MotorSet; /*<  */
 int16_t RudderSet; /*<  */
 uint8_t SYS_TYPE; /*<  */
 uint8_t DEV_ID; /*<  */
} mavlink_usv_set_t;

#define MAVLINK_MSG_ID_USV_SET_LEN 6
#define MAVLINK_MSG_ID_USV_SET_MIN_LEN 6
#define MAVLINK_MSG_ID_15_LEN 6
#define MAVLINK_MSG_ID_15_MIN_LEN 6

#define MAVLINK_MSG_ID_USV_SET_CRC 247
#define MAVLINK_MSG_ID_15_CRC 247



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_USV_SET { \
    15, \
    "USV_SET", \
    4, \
    {  { "SYS_TYPE", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_usv_set_t, SYS_TYPE) }, \
         { "DEV_ID", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_usv_set_t, DEV_ID) }, \
         { "MotorSet", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_usv_set_t, MotorSet) }, \
         { "RudderSet", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_usv_set_t, RudderSet) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_USV_SET { \
    "USV_SET", \
    4, \
    {  { "SYS_TYPE", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_usv_set_t, SYS_TYPE) }, \
         { "DEV_ID", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_usv_set_t, DEV_ID) }, \
         { "MotorSet", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_usv_set_t, MotorSet) }, \
         { "RudderSet", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_usv_set_t, RudderSet) }, \
         } \
}
#endif

/**
 * @brief Pack a usv_set message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param SYS_TYPE  
 * @param DEV_ID  
 * @param MotorSet  
 * @param RudderSet  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_usv_set_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t SYS_TYPE, uint8_t DEV_ID, int16_t MotorSet, int16_t RudderSet)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_USV_SET_LEN];
    _mav_put_int16_t(buf, 0, MotorSet);
    _mav_put_int16_t(buf, 2, RudderSet);
    _mav_put_uint8_t(buf, 4, SYS_TYPE);
    _mav_put_uint8_t(buf, 5, DEV_ID);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_USV_SET_LEN);
#else
    mavlink_usv_set_t packet;
    packet.MotorSet = MotorSet;
    packet.RudderSet = RudderSet;
    packet.SYS_TYPE = SYS_TYPE;
    packet.DEV_ID = DEV_ID;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_USV_SET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_USV_SET;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_USV_SET_MIN_LEN, MAVLINK_MSG_ID_USV_SET_LEN, MAVLINK_MSG_ID_USV_SET_CRC);
}

/**
 * @brief Pack a usv_set message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param SYS_TYPE  
 * @param DEV_ID  
 * @param MotorSet  
 * @param RudderSet  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_usv_set_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t SYS_TYPE,uint8_t DEV_ID,int16_t MotorSet,int16_t RudderSet)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_USV_SET_LEN];
    _mav_put_int16_t(buf, 0, MotorSet);
    _mav_put_int16_t(buf, 2, RudderSet);
    _mav_put_uint8_t(buf, 4, SYS_TYPE);
    _mav_put_uint8_t(buf, 5, DEV_ID);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_USV_SET_LEN);
#else
    mavlink_usv_set_t packet;
    packet.MotorSet = MotorSet;
    packet.RudderSet = RudderSet;
    packet.SYS_TYPE = SYS_TYPE;
    packet.DEV_ID = DEV_ID;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_USV_SET_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_USV_SET;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_USV_SET_MIN_LEN, MAVLINK_MSG_ID_USV_SET_LEN, MAVLINK_MSG_ID_USV_SET_CRC);
}

/**
 * @brief Encode a usv_set struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param usv_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_usv_set_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_usv_set_t* usv_set)
{
    return mavlink_msg_usv_set_pack(system_id, component_id, msg, usv_set->SYS_TYPE, usv_set->DEV_ID, usv_set->MotorSet, usv_set->RudderSet);
}

/**
 * @brief Encode a usv_set struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param usv_set C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_usv_set_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_usv_set_t* usv_set)
{
    return mavlink_msg_usv_set_pack_chan(system_id, component_id, chan, msg, usv_set->SYS_TYPE, usv_set->DEV_ID, usv_set->MotorSet, usv_set->RudderSet);
}

/**
 * @brief Send a usv_set message
 * @param chan MAVLink channel to send the message
 *
 * @param SYS_TYPE  
 * @param DEV_ID  
 * @param MotorSet  
 * @param RudderSet  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_usv_set_send(mavlink_channel_t chan, uint8_t SYS_TYPE, uint8_t DEV_ID, int16_t MotorSet, int16_t RudderSet)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_USV_SET_LEN];
    _mav_put_int16_t(buf, 0, MotorSet);
    _mav_put_int16_t(buf, 2, RudderSet);
    _mav_put_uint8_t(buf, 4, SYS_TYPE);
    _mav_put_uint8_t(buf, 5, DEV_ID);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_USV_SET, buf, MAVLINK_MSG_ID_USV_SET_MIN_LEN, MAVLINK_MSG_ID_USV_SET_LEN, MAVLINK_MSG_ID_USV_SET_CRC);
#else
    mavlink_usv_set_t packet;
    packet.MotorSet = MotorSet;
    packet.RudderSet = RudderSet;
    packet.SYS_TYPE = SYS_TYPE;
    packet.DEV_ID = DEV_ID;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_USV_SET, (const char *)&packet, MAVLINK_MSG_ID_USV_SET_MIN_LEN, MAVLINK_MSG_ID_USV_SET_LEN, MAVLINK_MSG_ID_USV_SET_CRC);
#endif
}

/**
 * @brief Send a usv_set message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_usv_set_send_struct(mavlink_channel_t chan, const mavlink_usv_set_t* usv_set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_usv_set_send(chan, usv_set->SYS_TYPE, usv_set->DEV_ID, usv_set->MotorSet, usv_set->RudderSet);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_USV_SET, (const char *)usv_set, MAVLINK_MSG_ID_USV_SET_MIN_LEN, MAVLINK_MSG_ID_USV_SET_LEN, MAVLINK_MSG_ID_USV_SET_CRC);
#endif
}

#if MAVLINK_MSG_ID_USV_SET_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_usv_set_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t SYS_TYPE, uint8_t DEV_ID, int16_t MotorSet, int16_t RudderSet)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, MotorSet);
    _mav_put_int16_t(buf, 2, RudderSet);
    _mav_put_uint8_t(buf, 4, SYS_TYPE);
    _mav_put_uint8_t(buf, 5, DEV_ID);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_USV_SET, buf, MAVLINK_MSG_ID_USV_SET_MIN_LEN, MAVLINK_MSG_ID_USV_SET_LEN, MAVLINK_MSG_ID_USV_SET_CRC);
#else
    mavlink_usv_set_t *packet = (mavlink_usv_set_t *)msgbuf;
    packet->MotorSet = MotorSet;
    packet->RudderSet = RudderSet;
    packet->SYS_TYPE = SYS_TYPE;
    packet->DEV_ID = DEV_ID;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_USV_SET, (const char *)packet, MAVLINK_MSG_ID_USV_SET_MIN_LEN, MAVLINK_MSG_ID_USV_SET_LEN, MAVLINK_MSG_ID_USV_SET_CRC);
#endif
}
#endif

#endif

// MESSAGE USV_SET UNPACKING


/**
 * @brief Get field SYS_TYPE from usv_set message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_usv_set_get_SYS_TYPE(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field DEV_ID from usv_set message
 *
 * @return  
 */
static inline uint8_t mavlink_msg_usv_set_get_DEV_ID(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field MotorSet from usv_set message
 *
 * @return  
 */
static inline int16_t mavlink_msg_usv_set_get_MotorSet(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field RudderSet from usv_set message
 *
 * @return  
 */
static inline int16_t mavlink_msg_usv_set_get_RudderSet(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Decode a usv_set message into a struct
 *
 * @param msg The message to decode
 * @param usv_set C-struct to decode the message contents into
 */
static inline void mavlink_msg_usv_set_decode(const mavlink_message_t* msg, mavlink_usv_set_t* usv_set)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    usv_set->MotorSet = mavlink_msg_usv_set_get_MotorSet(msg);
    usv_set->RudderSet = mavlink_msg_usv_set_get_RudderSet(msg);
    usv_set->SYS_TYPE = mavlink_msg_usv_set_get_SYS_TYPE(msg);
    usv_set->DEV_ID = mavlink_msg_usv_set_get_DEV_ID(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_USV_SET_LEN? msg->len : MAVLINK_MSG_ID_USV_SET_LEN;
        memset(usv_set, 0, MAVLINK_MSG_ID_USV_SET_LEN);
    memcpy(usv_set, _MAV_PAYLOAD(msg), len);
#endif
}
