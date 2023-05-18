#pragma once
// MESSAGE IMU_RAW PACKING

#define MAVLINK_MSG_ID_IMU_RAW 0


typedef struct __mavlink_imu_raw_t {
 int16_t acc_x_raw; /*<  */
 int16_t acc_y_raw; /*<  */
 int16_t acc_z_raw; /*<  */
 int16_t temp_raw; /*<  */
 int16_t gyro_x_raw; /*<  */
 int16_t gyro_y_raw; /*<  */
 int16_t gyro_z_raw; /*<  */
 int16_t mag_x_raw; /*<  */
 int16_t mag_y_raw; /*<  */
 int16_t mag_z_raw; /*<  */
} mavlink_imu_raw_t;

#define MAVLINK_MSG_ID_IMU_RAW_LEN 20
#define MAVLINK_MSG_ID_IMU_RAW_MIN_LEN 20
#define MAVLINK_MSG_ID_0_LEN 20
#define MAVLINK_MSG_ID_0_MIN_LEN 20

#define MAVLINK_MSG_ID_IMU_RAW_CRC 202
#define MAVLINK_MSG_ID_0_CRC 202



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_IMU_RAW { \
    0, \
    "IMU_RAW", \
    10, \
    {  { "acc_x_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_imu_raw_t, acc_x_raw) }, \
         { "acc_y_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_imu_raw_t, acc_y_raw) }, \
         { "acc_z_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_imu_raw_t, acc_z_raw) }, \
         { "temp_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_imu_raw_t, temp_raw) }, \
         { "gyro_x_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_imu_raw_t, gyro_x_raw) }, \
         { "gyro_y_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_imu_raw_t, gyro_y_raw) }, \
         { "gyro_z_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_imu_raw_t, gyro_z_raw) }, \
         { "mag_x_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_imu_raw_t, mag_x_raw) }, \
         { "mag_y_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_imu_raw_t, mag_y_raw) }, \
         { "mag_z_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_imu_raw_t, mag_z_raw) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_IMU_RAW { \
    "IMU_RAW", \
    10, \
    {  { "acc_x_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_imu_raw_t, acc_x_raw) }, \
         { "acc_y_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_imu_raw_t, acc_y_raw) }, \
         { "acc_z_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_imu_raw_t, acc_z_raw) }, \
         { "temp_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_imu_raw_t, temp_raw) }, \
         { "gyro_x_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_imu_raw_t, gyro_x_raw) }, \
         { "gyro_y_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_imu_raw_t, gyro_y_raw) }, \
         { "gyro_z_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_imu_raw_t, gyro_z_raw) }, \
         { "mag_x_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_imu_raw_t, mag_x_raw) }, \
         { "mag_y_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_imu_raw_t, mag_y_raw) }, \
         { "mag_z_raw", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_imu_raw_t, mag_z_raw) }, \
         } \
}
#endif

/**
 * @brief Pack a imu_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param acc_x_raw  
 * @param acc_y_raw  
 * @param acc_z_raw  
 * @param temp_raw  
 * @param gyro_x_raw  
 * @param gyro_y_raw  
 * @param gyro_z_raw  
 * @param mag_x_raw  
 * @param mag_y_raw  
 * @param mag_z_raw  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int16_t acc_x_raw, int16_t acc_y_raw, int16_t acc_z_raw, int16_t temp_raw, int16_t gyro_x_raw, int16_t gyro_y_raw, int16_t gyro_z_raw, int16_t mag_x_raw, int16_t mag_y_raw, int16_t mag_z_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_RAW_LEN];
    _mav_put_int16_t(buf, 0, acc_x_raw);
    _mav_put_int16_t(buf, 2, acc_y_raw);
    _mav_put_int16_t(buf, 4, acc_z_raw);
    _mav_put_int16_t(buf, 6, temp_raw);
    _mav_put_int16_t(buf, 8, gyro_x_raw);
    _mav_put_int16_t(buf, 10, gyro_y_raw);
    _mav_put_int16_t(buf, 12, gyro_z_raw);
    _mav_put_int16_t(buf, 14, mag_x_raw);
    _mav_put_int16_t(buf, 16, mag_y_raw);
    _mav_put_int16_t(buf, 18, mag_z_raw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_RAW_LEN);
#else
    mavlink_imu_raw_t packet;
    packet.acc_x_raw = acc_x_raw;
    packet.acc_y_raw = acc_y_raw;
    packet.acc_z_raw = acc_z_raw;
    packet.temp_raw = temp_raw;
    packet.gyro_x_raw = gyro_x_raw;
    packet.gyro_y_raw = gyro_y_raw;
    packet.gyro_z_raw = gyro_z_raw;
    packet.mag_x_raw = mag_x_raw;
    packet.mag_y_raw = mag_y_raw;
    packet.mag_z_raw = mag_z_raw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_RAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMU_RAW;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IMU_RAW_MIN_LEN, MAVLINK_MSG_ID_IMU_RAW_LEN, MAVLINK_MSG_ID_IMU_RAW_CRC);
}

/**
 * @brief Pack a imu_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param acc_x_raw  
 * @param acc_y_raw  
 * @param acc_z_raw  
 * @param temp_raw  
 * @param gyro_x_raw  
 * @param gyro_y_raw  
 * @param gyro_z_raw  
 * @param mag_x_raw  
 * @param mag_y_raw  
 * @param mag_z_raw  
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int16_t acc_x_raw,int16_t acc_y_raw,int16_t acc_z_raw,int16_t temp_raw,int16_t gyro_x_raw,int16_t gyro_y_raw,int16_t gyro_z_raw,int16_t mag_x_raw,int16_t mag_y_raw,int16_t mag_z_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_RAW_LEN];
    _mav_put_int16_t(buf, 0, acc_x_raw);
    _mav_put_int16_t(buf, 2, acc_y_raw);
    _mav_put_int16_t(buf, 4, acc_z_raw);
    _mav_put_int16_t(buf, 6, temp_raw);
    _mav_put_int16_t(buf, 8, gyro_x_raw);
    _mav_put_int16_t(buf, 10, gyro_y_raw);
    _mav_put_int16_t(buf, 12, gyro_z_raw);
    _mav_put_int16_t(buf, 14, mag_x_raw);
    _mav_put_int16_t(buf, 16, mag_y_raw);
    _mav_put_int16_t(buf, 18, mag_z_raw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_RAW_LEN);
#else
    mavlink_imu_raw_t packet;
    packet.acc_x_raw = acc_x_raw;
    packet.acc_y_raw = acc_y_raw;
    packet.acc_z_raw = acc_z_raw;
    packet.temp_raw = temp_raw;
    packet.gyro_x_raw = gyro_x_raw;
    packet.gyro_y_raw = gyro_y_raw;
    packet.gyro_z_raw = gyro_z_raw;
    packet.mag_x_raw = mag_x_raw;
    packet.mag_y_raw = mag_y_raw;
    packet.mag_z_raw = mag_z_raw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_RAW_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMU_RAW;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IMU_RAW_MIN_LEN, MAVLINK_MSG_ID_IMU_RAW_LEN, MAVLINK_MSG_ID_IMU_RAW_CRC);
}

/**
 * @brief Encode a imu_raw struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param imu_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_imu_raw_t* imu_raw)
{
    return mavlink_msg_imu_raw_pack(system_id, component_id, msg, imu_raw->acc_x_raw, imu_raw->acc_y_raw, imu_raw->acc_z_raw, imu_raw->temp_raw, imu_raw->gyro_x_raw, imu_raw->gyro_y_raw, imu_raw->gyro_z_raw, imu_raw->mag_x_raw, imu_raw->mag_y_raw, imu_raw->mag_z_raw);
}

/**
 * @brief Encode a imu_raw struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param imu_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_raw_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_imu_raw_t* imu_raw)
{
    return mavlink_msg_imu_raw_pack_chan(system_id, component_id, chan, msg, imu_raw->acc_x_raw, imu_raw->acc_y_raw, imu_raw->acc_z_raw, imu_raw->temp_raw, imu_raw->gyro_x_raw, imu_raw->gyro_y_raw, imu_raw->gyro_z_raw, imu_raw->mag_x_raw, imu_raw->mag_y_raw, imu_raw->mag_z_raw);
}

/**
 * @brief Send a imu_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param acc_x_raw  
 * @param acc_y_raw  
 * @param acc_z_raw  
 * @param temp_raw  
 * @param gyro_x_raw  
 * @param gyro_y_raw  
 * @param gyro_z_raw  
 * @param mag_x_raw  
 * @param mag_y_raw  
 * @param mag_z_raw  
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_imu_raw_send(mavlink_channel_t chan, int16_t acc_x_raw, int16_t acc_y_raw, int16_t acc_z_raw, int16_t temp_raw, int16_t gyro_x_raw, int16_t gyro_y_raw, int16_t gyro_z_raw, int16_t mag_x_raw, int16_t mag_y_raw, int16_t mag_z_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_RAW_LEN];
    _mav_put_int16_t(buf, 0, acc_x_raw);
    _mav_put_int16_t(buf, 2, acc_y_raw);
    _mav_put_int16_t(buf, 4, acc_z_raw);
    _mav_put_int16_t(buf, 6, temp_raw);
    _mav_put_int16_t(buf, 8, gyro_x_raw);
    _mav_put_int16_t(buf, 10, gyro_y_raw);
    _mav_put_int16_t(buf, 12, gyro_z_raw);
    _mav_put_int16_t(buf, 14, mag_x_raw);
    _mav_put_int16_t(buf, 16, mag_y_raw);
    _mav_put_int16_t(buf, 18, mag_z_raw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_RAW, buf, MAVLINK_MSG_ID_IMU_RAW_MIN_LEN, MAVLINK_MSG_ID_IMU_RAW_LEN, MAVLINK_MSG_ID_IMU_RAW_CRC);
#else
    mavlink_imu_raw_t packet;
    packet.acc_x_raw = acc_x_raw;
    packet.acc_y_raw = acc_y_raw;
    packet.acc_z_raw = acc_z_raw;
    packet.temp_raw = temp_raw;
    packet.gyro_x_raw = gyro_x_raw;
    packet.gyro_y_raw = gyro_y_raw;
    packet.gyro_z_raw = gyro_z_raw;
    packet.mag_x_raw = mag_x_raw;
    packet.mag_y_raw = mag_y_raw;
    packet.mag_z_raw = mag_z_raw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_RAW, (const char *)&packet, MAVLINK_MSG_ID_IMU_RAW_MIN_LEN, MAVLINK_MSG_ID_IMU_RAW_LEN, MAVLINK_MSG_ID_IMU_RAW_CRC);
#endif
}

/**
 * @brief Send a imu_raw message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_imu_raw_send_struct(mavlink_channel_t chan, const mavlink_imu_raw_t* imu_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_imu_raw_send(chan, imu_raw->acc_x_raw, imu_raw->acc_y_raw, imu_raw->acc_z_raw, imu_raw->temp_raw, imu_raw->gyro_x_raw, imu_raw->gyro_y_raw, imu_raw->gyro_z_raw, imu_raw->mag_x_raw, imu_raw->mag_y_raw, imu_raw->mag_z_raw);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_RAW, (const char *)imu_raw, MAVLINK_MSG_ID_IMU_RAW_MIN_LEN, MAVLINK_MSG_ID_IMU_RAW_LEN, MAVLINK_MSG_ID_IMU_RAW_CRC);
#endif
}

#if MAVLINK_MSG_ID_IMU_RAW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_imu_raw_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t acc_x_raw, int16_t acc_y_raw, int16_t acc_z_raw, int16_t temp_raw, int16_t gyro_x_raw, int16_t gyro_y_raw, int16_t gyro_z_raw, int16_t mag_x_raw, int16_t mag_y_raw, int16_t mag_z_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_int16_t(buf, 0, acc_x_raw);
    _mav_put_int16_t(buf, 2, acc_y_raw);
    _mav_put_int16_t(buf, 4, acc_z_raw);
    _mav_put_int16_t(buf, 6, temp_raw);
    _mav_put_int16_t(buf, 8, gyro_x_raw);
    _mav_put_int16_t(buf, 10, gyro_y_raw);
    _mav_put_int16_t(buf, 12, gyro_z_raw);
    _mav_put_int16_t(buf, 14, mag_x_raw);
    _mav_put_int16_t(buf, 16, mag_y_raw);
    _mav_put_int16_t(buf, 18, mag_z_raw);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_RAW, buf, MAVLINK_MSG_ID_IMU_RAW_MIN_LEN, MAVLINK_MSG_ID_IMU_RAW_LEN, MAVLINK_MSG_ID_IMU_RAW_CRC);
#else
    mavlink_imu_raw_t *packet = (mavlink_imu_raw_t *)msgbuf;
    packet->acc_x_raw = acc_x_raw;
    packet->acc_y_raw = acc_y_raw;
    packet->acc_z_raw = acc_z_raw;
    packet->temp_raw = temp_raw;
    packet->gyro_x_raw = gyro_x_raw;
    packet->gyro_y_raw = gyro_y_raw;
    packet->gyro_z_raw = gyro_z_raw;
    packet->mag_x_raw = mag_x_raw;
    packet->mag_y_raw = mag_y_raw;
    packet->mag_z_raw = mag_z_raw;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU_RAW, (const char *)packet, MAVLINK_MSG_ID_IMU_RAW_MIN_LEN, MAVLINK_MSG_ID_IMU_RAW_LEN, MAVLINK_MSG_ID_IMU_RAW_CRC);
#endif
}
#endif

#endif

// MESSAGE IMU_RAW UNPACKING


/**
 * @brief Get field acc_x_raw from imu_raw message
 *
 * @return  
 */
static inline int16_t mavlink_msg_imu_raw_get_acc_x_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field acc_y_raw from imu_raw message
 *
 * @return  
 */
static inline int16_t mavlink_msg_imu_raw_get_acc_y_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field acc_z_raw from imu_raw message
 *
 * @return  
 */
static inline int16_t mavlink_msg_imu_raw_get_acc_z_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field temp_raw from imu_raw message
 *
 * @return  
 */
static inline int16_t mavlink_msg_imu_raw_get_temp_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field gyro_x_raw from imu_raw message
 *
 * @return  
 */
static inline int16_t mavlink_msg_imu_raw_get_gyro_x_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field gyro_y_raw from imu_raw message
 *
 * @return  
 */
static inline int16_t mavlink_msg_imu_raw_get_gyro_y_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field gyro_z_raw from imu_raw message
 *
 * @return  
 */
static inline int16_t mavlink_msg_imu_raw_get_gyro_z_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field mag_x_raw from imu_raw message
 *
 * @return  
 */
static inline int16_t mavlink_msg_imu_raw_get_mag_x_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field mag_y_raw from imu_raw message
 *
 * @return  
 */
static inline int16_t mavlink_msg_imu_raw_get_mag_y_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field mag_z_raw from imu_raw message
 *
 * @return  
 */
static inline int16_t mavlink_msg_imu_raw_get_mag_z_raw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Decode a imu_raw message into a struct
 *
 * @param msg The message to decode
 * @param imu_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_imu_raw_decode(const mavlink_message_t* msg, mavlink_imu_raw_t* imu_raw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    imu_raw->acc_x_raw = mavlink_msg_imu_raw_get_acc_x_raw(msg);
    imu_raw->acc_y_raw = mavlink_msg_imu_raw_get_acc_y_raw(msg);
    imu_raw->acc_z_raw = mavlink_msg_imu_raw_get_acc_z_raw(msg);
    imu_raw->temp_raw = mavlink_msg_imu_raw_get_temp_raw(msg);
    imu_raw->gyro_x_raw = mavlink_msg_imu_raw_get_gyro_x_raw(msg);
    imu_raw->gyro_y_raw = mavlink_msg_imu_raw_get_gyro_y_raw(msg);
    imu_raw->gyro_z_raw = mavlink_msg_imu_raw_get_gyro_z_raw(msg);
    imu_raw->mag_x_raw = mavlink_msg_imu_raw_get_mag_x_raw(msg);
    imu_raw->mag_y_raw = mavlink_msg_imu_raw_get_mag_y_raw(msg);
    imu_raw->mag_z_raw = mavlink_msg_imu_raw_get_mag_z_raw(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_IMU_RAW_LEN? msg->len : MAVLINK_MSG_ID_IMU_RAW_LEN;
        memset(imu_raw, 0, MAVLINK_MSG_ID_IMU_RAW_LEN);
    memcpy(imu_raw, _MAV_PAYLOAD(msg), len);
#endif
}
