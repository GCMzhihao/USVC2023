#pragma once
// MESSAGE PRESSURE PACKING

#define MAVLINK_MSG_ID_PRESSURE 2


typedef struct __mavlink_pressure_t {
 float abs_pressure; /*< [hPa] 绝对压强*/
 float temperature; /*< [hPa] 温度*/
 float ground_pressure; /*< [hPa] 地面压强*/
 float pressure_alt; /*< [m] 高度*/
} mavlink_pressure_t;

#define MAVLINK_MSG_ID_PRESSURE_LEN 16
#define MAVLINK_MSG_ID_PRESSURE_MIN_LEN 16
#define MAVLINK_MSG_ID_2_LEN 16
#define MAVLINK_MSG_ID_2_MIN_LEN 16

#define MAVLINK_MSG_ID_PRESSURE_CRC 217
#define MAVLINK_MSG_ID_2_CRC 217



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PRESSURE { \
    2, \
    "PRESSURE", \
    4, \
    {  { "abs_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_pressure_t, abs_pressure) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_pressure_t, temperature) }, \
         { "ground_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_pressure_t, ground_pressure) }, \
         { "pressure_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_pressure_t, pressure_alt) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PRESSURE { \
    "PRESSURE", \
    4, \
    {  { "abs_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_pressure_t, abs_pressure) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_pressure_t, temperature) }, \
         { "ground_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_pressure_t, ground_pressure) }, \
         { "pressure_alt", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_pressure_t, pressure_alt) }, \
         } \
}
#endif

/**
 * @brief Pack a pressure message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param abs_pressure [hPa] 绝对压强
 * @param temperature [hPa] 温度
 * @param ground_pressure [hPa] 地面压强
 * @param pressure_alt [m] 高度
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pressure_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float abs_pressure, float temperature, float ground_pressure, float pressure_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PRESSURE_LEN];
    _mav_put_float(buf, 0, abs_pressure);
    _mav_put_float(buf, 4, temperature);
    _mav_put_float(buf, 8, ground_pressure);
    _mav_put_float(buf, 12, pressure_alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PRESSURE_LEN);
#else
    mavlink_pressure_t packet;
    packet.abs_pressure = abs_pressure;
    packet.temperature = temperature;
    packet.ground_pressure = ground_pressure;
    packet.pressure_alt = pressure_alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PRESSURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PRESSURE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_LEN, MAVLINK_MSG_ID_PRESSURE_CRC);
}

/**
 * @brief Pack a pressure message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param abs_pressure [hPa] 绝对压强
 * @param temperature [hPa] 温度
 * @param ground_pressure [hPa] 地面压强
 * @param pressure_alt [m] 高度
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pressure_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float abs_pressure,float temperature,float ground_pressure,float pressure_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PRESSURE_LEN];
    _mav_put_float(buf, 0, abs_pressure);
    _mav_put_float(buf, 4, temperature);
    _mav_put_float(buf, 8, ground_pressure);
    _mav_put_float(buf, 12, pressure_alt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PRESSURE_LEN);
#else
    mavlink_pressure_t packet;
    packet.abs_pressure = abs_pressure;
    packet.temperature = temperature;
    packet.ground_pressure = ground_pressure;
    packet.pressure_alt = pressure_alt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PRESSURE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PRESSURE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_LEN, MAVLINK_MSG_ID_PRESSURE_CRC);
}

/**
 * @brief Encode a pressure struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pressure C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pressure_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pressure_t* pressure)
{
    return mavlink_msg_pressure_pack(system_id, component_id, msg, pressure->abs_pressure, pressure->temperature, pressure->ground_pressure, pressure->pressure_alt);
}

/**
 * @brief Encode a pressure struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pressure C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pressure_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_pressure_t* pressure)
{
    return mavlink_msg_pressure_pack_chan(system_id, component_id, chan, msg, pressure->abs_pressure, pressure->temperature, pressure->ground_pressure, pressure->pressure_alt);
}

/**
 * @brief Send a pressure message
 * @param chan MAVLink channel to send the message
 *
 * @param abs_pressure [hPa] 绝对压强
 * @param temperature [hPa] 温度
 * @param ground_pressure [hPa] 地面压强
 * @param pressure_alt [m] 高度
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pressure_send(mavlink_channel_t chan, float abs_pressure, float temperature, float ground_pressure, float pressure_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PRESSURE_LEN];
    _mav_put_float(buf, 0, abs_pressure);
    _mav_put_float(buf, 4, temperature);
    _mav_put_float(buf, 8, ground_pressure);
    _mav_put_float(buf, 12, pressure_alt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRESSURE, buf, MAVLINK_MSG_ID_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_LEN, MAVLINK_MSG_ID_PRESSURE_CRC);
#else
    mavlink_pressure_t packet;
    packet.abs_pressure = abs_pressure;
    packet.temperature = temperature;
    packet.ground_pressure = ground_pressure;
    packet.pressure_alt = pressure_alt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRESSURE, (const char *)&packet, MAVLINK_MSG_ID_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_LEN, MAVLINK_MSG_ID_PRESSURE_CRC);
#endif
}

/**
 * @brief Send a pressure message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_pressure_send_struct(mavlink_channel_t chan, const mavlink_pressure_t* pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_pressure_send(chan, pressure->abs_pressure, pressure->temperature, pressure->ground_pressure, pressure->pressure_alt);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRESSURE, (const char *)pressure, MAVLINK_MSG_ID_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_LEN, MAVLINK_MSG_ID_PRESSURE_CRC);
#endif
}

#if MAVLINK_MSG_ID_PRESSURE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_pressure_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float abs_pressure, float temperature, float ground_pressure, float pressure_alt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, abs_pressure);
    _mav_put_float(buf, 4, temperature);
    _mav_put_float(buf, 8, ground_pressure);
    _mav_put_float(buf, 12, pressure_alt);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRESSURE, buf, MAVLINK_MSG_ID_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_LEN, MAVLINK_MSG_ID_PRESSURE_CRC);
#else
    mavlink_pressure_t *packet = (mavlink_pressure_t *)msgbuf;
    packet->abs_pressure = abs_pressure;
    packet->temperature = temperature;
    packet->ground_pressure = ground_pressure;
    packet->pressure_alt = pressure_alt;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRESSURE, (const char *)packet, MAVLINK_MSG_ID_PRESSURE_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_LEN, MAVLINK_MSG_ID_PRESSURE_CRC);
#endif
}
#endif

#endif

// MESSAGE PRESSURE UNPACKING


/**
 * @brief Get field abs_pressure from pressure message
 *
 * @return [hPa] 绝对压强
 */
static inline float mavlink_msg_pressure_get_abs_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field temperature from pressure message
 *
 * @return [hPa] 温度
 */
static inline float mavlink_msg_pressure_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field ground_pressure from pressure message
 *
 * @return [hPa] 地面压强
 */
static inline float mavlink_msg_pressure_get_ground_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pressure_alt from pressure message
 *
 * @return [m] 高度
 */
static inline float mavlink_msg_pressure_get_pressure_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a pressure message into a struct
 *
 * @param msg The message to decode
 * @param pressure C-struct to decode the message contents into
 */
static inline void mavlink_msg_pressure_decode(const mavlink_message_t* msg, mavlink_pressure_t* pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    pressure->abs_pressure = mavlink_msg_pressure_get_abs_pressure(msg);
    pressure->temperature = mavlink_msg_pressure_get_temperature(msg);
    pressure->ground_pressure = mavlink_msg_pressure_get_ground_pressure(msg);
    pressure->pressure_alt = mavlink_msg_pressure_get_pressure_alt(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PRESSURE_LEN? msg->len : MAVLINK_MSG_ID_PRESSURE_LEN;
        memset(pressure, 0, MAVLINK_MSG_ID_PRESSURE_LEN);
    memcpy(pressure, _MAV_PAYLOAD(msg), len);
#endif
}
