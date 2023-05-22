#pragma once
// MESSAGE USV_STATE PACKING

#define MAVLINK_MSG_ID_USV_STATE 7


typedef struct __mavlink_usv_state_t {
 float x; /*<  */
 float y; /*<  */
 float speed; /*<  */
 float heading; /*<  */
 float battery_voltage; /*<  */
} mavlink_usv_state_t;

#define MAVLINK_MSG_ID_USV_STATE_LEN 20
#define MAVLINK_MSG_ID_USV_STATE_MIN_LEN 20
#define MAVLINK_MSG_ID_7_LEN 20
#define MAVLINK_MSG_ID_7_MIN_LEN 20

#define MAVLINK_MSG_ID_USV_STATE_CRC 126
#define MAVLINK_MSG_ID_7_CRC 126



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_USV_STATE { \
    7, \
    "USV_STATE", \
    5, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_usv_state_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_usv_state_t, y) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_usv_state_t, speed) }, \
         { "heading", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_usv_state_t, heading) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_usv_state_t, battery_voltage) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_USV_STATE { \
    "USV_STATE", \
    5, \
    {  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_usv_state_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_usv_state_t, y) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_usv_state_t, speed) }, \
         { "heading", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_usv_state_t, heading) }, \
         { "battery_voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_usv_state_t, battery_voltage) }, \
         } \
}
#endif

/**
 * @brief Pack a usv_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param x  
 * @param y  
 * @param speed  
 * @param heading  
 * @param battery_voltage  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_usv_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float x, float y, float speed, float heading, float battery_voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_USV_STATE_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, speed);
    _mav_put_float(buf, 12, heading);
    _mav_put_float(buf, 16, battery_voltage);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_USV_STATE_LEN);
#else
    mavlink_usv_state_t packet;
    packet.x = x;
    packet.y = y;
    packet.speed = speed;
    packet.heading = heading;
    packet.battery_voltage = battery_voltage;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_USV_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_USV_STATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_USV_STATE_MIN_LEN, MAVLINK_MSG_ID_USV_STATE_LEN, MAVLINK_MSG_ID_USV_STATE_CRC);
}

/**
 * @brief Pack a usv_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x  
 * @param y  
 * @param speed  
 * @param heading  
 * @param battery_voltage  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_usv_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float x,float y,float speed,float heading,float battery_voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_USV_STATE_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, speed);
    _mav_put_float(buf, 12, heading);
    _mav_put_float(buf, 16, battery_voltage);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_USV_STATE_LEN);
#else
    mavlink_usv_state_t packet;
    packet.x = x;
    packet.y = y;
    packet.speed = speed;
    packet.heading = heading;
    packet.battery_voltage = battery_voltage;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_USV_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_USV_STATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_USV_STATE_MIN_LEN, MAVLINK_MSG_ID_USV_STATE_LEN, MAVLINK_MSG_ID_USV_STATE_CRC);
}

/**
 * @brief Encode a usv_state struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param usv_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_usv_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_usv_state_t* usv_state)
{
    return mavlink_msg_usv_state_pack(system_id, component_id, msg, usv_state->x, usv_state->y, usv_state->speed, usv_state->heading, usv_state->battery_voltage);
}

/**
 * @brief Encode a usv_state struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param usv_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_usv_state_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_usv_state_t* usv_state)
{
    return mavlink_msg_usv_state_pack_chan(system_id, component_id, chan, msg, usv_state->x, usv_state->y, usv_state->speed, usv_state->heading, usv_state->battery_voltage);
}

/**
 * @brief Send a usv_state message
 * @param chan MAVLink channel to send the message
 *
 * @param x  
 * @param y  
 * @param speed  
 * @param heading  
 * @param battery_voltage  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_usv_state_send(mavlink_channel_t chan, float x, float y, float speed, float heading, float battery_voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_USV_STATE_LEN];
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, speed);
    _mav_put_float(buf, 12, heading);
    _mav_put_float(buf, 16, battery_voltage);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_USV_STATE, buf, MAVLINK_MSG_ID_USV_STATE_MIN_LEN, MAVLINK_MSG_ID_USV_STATE_LEN, MAVLINK_MSG_ID_USV_STATE_CRC);
#else
    mavlink_usv_state_t packet;
    packet.x = x;
    packet.y = y;
    packet.speed = speed;
    packet.heading = heading;
    packet.battery_voltage = battery_voltage;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_USV_STATE, (const char *)&packet, MAVLINK_MSG_ID_USV_STATE_MIN_LEN, MAVLINK_MSG_ID_USV_STATE_LEN, MAVLINK_MSG_ID_USV_STATE_CRC);
#endif
}

/**
 * @brief Send a usv_state message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_usv_state_send_struct(mavlink_channel_t chan, const mavlink_usv_state_t* usv_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_usv_state_send(chan, usv_state->x, usv_state->y, usv_state->speed, usv_state->heading, usv_state->battery_voltage);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_USV_STATE, (const char *)usv_state, MAVLINK_MSG_ID_USV_STATE_MIN_LEN, MAVLINK_MSG_ID_USV_STATE_LEN, MAVLINK_MSG_ID_USV_STATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_USV_STATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_usv_state_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float x, float y, float speed, float heading, float battery_voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, x);
    _mav_put_float(buf, 4, y);
    _mav_put_float(buf, 8, speed);
    _mav_put_float(buf, 12, heading);
    _mav_put_float(buf, 16, battery_voltage);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_USV_STATE, buf, MAVLINK_MSG_ID_USV_STATE_MIN_LEN, MAVLINK_MSG_ID_USV_STATE_LEN, MAVLINK_MSG_ID_USV_STATE_CRC);
#else
    mavlink_usv_state_t *packet = (mavlink_usv_state_t *)msgbuf;
    packet->x = x;
    packet->y = y;
    packet->speed = speed;
    packet->heading = heading;
    packet->battery_voltage = battery_voltage;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_USV_STATE, (const char *)packet, MAVLINK_MSG_ID_USV_STATE_MIN_LEN, MAVLINK_MSG_ID_USV_STATE_LEN, MAVLINK_MSG_ID_USV_STATE_CRC);
#endif
}
#endif

#endif

// MESSAGE USV_STATE UNPACKING


/**
 * @brief Get field x from usv_state message
 *
 * @return  
 */
static inline float mavlink_msg_usv_state_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from usv_state message
 *
 * @return  
 */
static inline float mavlink_msg_usv_state_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field speed from usv_state message
 *
 * @return  
 */
static inline float mavlink_msg_usv_state_get_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field heading from usv_state message
 *
 * @return  
 */
static inline float mavlink_msg_usv_state_get_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field battery_voltage from usv_state message
 *
 * @return  
 */
static inline float mavlink_msg_usv_state_get_battery_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a usv_state message into a struct
 *
 * @param msg The message to decode
 * @param usv_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_usv_state_decode(const mavlink_message_t* msg, mavlink_usv_state_t* usv_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    usv_state->x = mavlink_msg_usv_state_get_x(msg);
    usv_state->y = mavlink_msg_usv_state_get_y(msg);
    usv_state->speed = mavlink_msg_usv_state_get_speed(msg);
    usv_state->heading = mavlink_msg_usv_state_get_heading(msg);
    usv_state->battery_voltage = mavlink_msg_usv_state_get_battery_voltage(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_USV_STATE_LEN? msg->len : MAVLINK_MSG_ID_USV_STATE_LEN;
        memset(usv_state, 0, MAVLINK_MSG_ID_USV_STATE_LEN);
    memcpy(usv_state, _MAV_PAYLOAD(msg), len);
#endif
}
