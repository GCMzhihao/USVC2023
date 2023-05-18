#pragma once
// MESSAGE BATTERY_VOLTAGE PACKING

#define MAVLINK_MSG_ID_BATTERY_VOLTAGE 13


typedef struct __mavlink_battery_voltage_t {
 float voltage; /*<  */
} mavlink_battery_voltage_t;

#define MAVLINK_MSG_ID_BATTERY_VOLTAGE_LEN 4
#define MAVLINK_MSG_ID_BATTERY_VOLTAGE_MIN_LEN 4
#define MAVLINK_MSG_ID_13_LEN 4
#define MAVLINK_MSG_ID_13_MIN_LEN 4

#define MAVLINK_MSG_ID_BATTERY_VOLTAGE_CRC 146
#define MAVLINK_MSG_ID_13_CRC 146



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BATTERY_VOLTAGE { \
    13, \
    "BATTERY_VOLTAGE", \
    1, \
    {  { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_battery_voltage_t, voltage) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BATTERY_VOLTAGE { \
    "BATTERY_VOLTAGE", \
    1, \
    {  { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_battery_voltage_t, voltage) }, \
         } \
}
#endif

/**
 * @brief Pack a battery_voltage message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param voltage  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery_voltage_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_VOLTAGE_LEN];
    _mav_put_float(buf, 0, voltage);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY_VOLTAGE_LEN);
#else
    mavlink_battery_voltage_t packet;
    packet.voltage = voltage;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY_VOLTAGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BATTERY_VOLTAGE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BATTERY_VOLTAGE_MIN_LEN, MAVLINK_MSG_ID_BATTERY_VOLTAGE_LEN, MAVLINK_MSG_ID_BATTERY_VOLTAGE_CRC);
}

/**
 * @brief Pack a battery_voltage message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param voltage  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_battery_voltage_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_VOLTAGE_LEN];
    _mav_put_float(buf, 0, voltage);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BATTERY_VOLTAGE_LEN);
#else
    mavlink_battery_voltage_t packet;
    packet.voltage = voltage;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BATTERY_VOLTAGE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BATTERY_VOLTAGE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BATTERY_VOLTAGE_MIN_LEN, MAVLINK_MSG_ID_BATTERY_VOLTAGE_LEN, MAVLINK_MSG_ID_BATTERY_VOLTAGE_CRC);
}

/**
 * @brief Encode a battery_voltage struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param battery_voltage C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery_voltage_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_battery_voltage_t* battery_voltage)
{
    return mavlink_msg_battery_voltage_pack(system_id, component_id, msg, battery_voltage->voltage);
}

/**
 * @brief Encode a battery_voltage struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param battery_voltage C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_battery_voltage_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_battery_voltage_t* battery_voltage)
{
    return mavlink_msg_battery_voltage_pack_chan(system_id, component_id, chan, msg, battery_voltage->voltage);
}

/**
 * @brief Send a battery_voltage message
 * @param chan MAVLink channel to send the message
 *
 * @param voltage  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_battery_voltage_send(mavlink_channel_t chan, float voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BATTERY_VOLTAGE_LEN];
    _mav_put_float(buf, 0, voltage);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_VOLTAGE, buf, MAVLINK_MSG_ID_BATTERY_VOLTAGE_MIN_LEN, MAVLINK_MSG_ID_BATTERY_VOLTAGE_LEN, MAVLINK_MSG_ID_BATTERY_VOLTAGE_CRC);
#else
    mavlink_battery_voltage_t packet;
    packet.voltage = voltage;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_VOLTAGE, (const char *)&packet, MAVLINK_MSG_ID_BATTERY_VOLTAGE_MIN_LEN, MAVLINK_MSG_ID_BATTERY_VOLTAGE_LEN, MAVLINK_MSG_ID_BATTERY_VOLTAGE_CRC);
#endif
}

/**
 * @brief Send a battery_voltage message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_battery_voltage_send_struct(mavlink_channel_t chan, const mavlink_battery_voltage_t* battery_voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_battery_voltage_send(chan, battery_voltage->voltage);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_VOLTAGE, (const char *)battery_voltage, MAVLINK_MSG_ID_BATTERY_VOLTAGE_MIN_LEN, MAVLINK_MSG_ID_BATTERY_VOLTAGE_LEN, MAVLINK_MSG_ID_BATTERY_VOLTAGE_CRC);
#endif
}

#if MAVLINK_MSG_ID_BATTERY_VOLTAGE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_battery_voltage_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, voltage);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_VOLTAGE, buf, MAVLINK_MSG_ID_BATTERY_VOLTAGE_MIN_LEN, MAVLINK_MSG_ID_BATTERY_VOLTAGE_LEN, MAVLINK_MSG_ID_BATTERY_VOLTAGE_CRC);
#else
    mavlink_battery_voltage_t *packet = (mavlink_battery_voltage_t *)msgbuf;
    packet->voltage = voltage;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BATTERY_VOLTAGE, (const char *)packet, MAVLINK_MSG_ID_BATTERY_VOLTAGE_MIN_LEN, MAVLINK_MSG_ID_BATTERY_VOLTAGE_LEN, MAVLINK_MSG_ID_BATTERY_VOLTAGE_CRC);
#endif
}
#endif

#endif

// MESSAGE BATTERY_VOLTAGE UNPACKING


/**
 * @brief Get field voltage from battery_voltage message
 *
 * @return  
 */
static inline float mavlink_msg_battery_voltage_get_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a battery_voltage message into a struct
 *
 * @param msg The message to decode
 * @param battery_voltage C-struct to decode the message contents into
 */
static inline void mavlink_msg_battery_voltage_decode(const mavlink_message_t* msg, mavlink_battery_voltage_t* battery_voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    battery_voltage->voltage = mavlink_msg_battery_voltage_get_voltage(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BATTERY_VOLTAGE_LEN? msg->len : MAVLINK_MSG_ID_BATTERY_VOLTAGE_LEN;
        memset(battery_voltage, 0, MAVLINK_MSG_ID_BATTERY_VOLTAGE_LEN);
    memcpy(battery_voltage, _MAV_PAYLOAD(msg), len);
#endif
}
