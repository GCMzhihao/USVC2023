#pragma once
// MESSAGE IMU PACKING

#define MAVLINK_MSG_ID_IMU 1


typedef struct __mavlink_imu_t {
 float acc_x; /*< [m/s/s] */
 float acc_y; /*< [m/s/s] */
 float acc_z; /*< [m/s/s] */
 float temp_C; /*<  */
 float gyro_x; /*< [rad/s] */
 float gyro_y; /*< [rad/s] */
 float gyro_z; /*< [rad/s] */
 float mag_x; /*< [gauss] */
 float mag_y; /*< [gauss] */
 float mag_z; /*< [gauss] */
} mavlink_imu_t;

#define MAVLINK_MSG_ID_IMU_LEN 40
#define MAVLINK_MSG_ID_IMU_MIN_LEN 40
#define MAVLINK_MSG_ID_1_LEN 40
#define MAVLINK_MSG_ID_1_MIN_LEN 40

#define MAVLINK_MSG_ID_IMU_CRC 188
#define MAVLINK_MSG_ID_1_CRC 188



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_IMU { \
    1, \
    "IMU", \
    10, \
    {  { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_imu_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_imu_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_imu_t, acc_z) }, \
         { "temp_C", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_imu_t, temp_C) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_imu_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_imu_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_imu_t, gyro_z) }, \
         { "mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_imu_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_imu_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_imu_t, mag_z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_IMU { \
    "IMU", \
    10, \
    {  { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_imu_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_imu_t, acc_y) }, \
         { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_imu_t, acc_z) }, \
         { "temp_C", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_imu_t, temp_C) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_imu_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_imu_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_imu_t, gyro_z) }, \
         { "mag_x", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_imu_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_imu_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_imu_t, mag_z) }, \
         } \
}
#endif

/**
 * @brief Pack a imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param acc_x [m/s/s] 
 * @param acc_y [m/s/s] 
 * @param acc_z [m/s/s] 
 * @param temp_C  
 * @param gyro_x [rad/s] 
 * @param gyro_y [rad/s] 
 * @param gyro_z [rad/s] 
 * @param mag_x [gauss] 
 * @param mag_y [gauss] 
 * @param mag_z [gauss] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float acc_x, float acc_y, float acc_z, float temp_C, float gyro_x, float gyro_y, float gyro_z, float mag_x, float mag_y, float mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_LEN];
    _mav_put_float(buf, 0, acc_x);
    _mav_put_float(buf, 4, acc_y);
    _mav_put_float(buf, 8, acc_z);
    _mav_put_float(buf, 12, temp_C);
    _mav_put_float(buf, 16, gyro_x);
    _mav_put_float(buf, 20, gyro_y);
    _mav_put_float(buf, 24, gyro_z);
    _mav_put_float(buf, 28, mag_x);
    _mav_put_float(buf, 32, mag_y);
    _mav_put_float(buf, 36, mag_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_LEN);
#else
    mavlink_imu_t packet;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;
    packet.temp_C = temp_C;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMU;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
}

/**
 * @brief Pack a imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param acc_x [m/s/s] 
 * @param acc_y [m/s/s] 
 * @param acc_z [m/s/s] 
 * @param temp_C  
 * @param gyro_x [rad/s] 
 * @param gyro_y [rad/s] 
 * @param gyro_z [rad/s] 
 * @param mag_x [gauss] 
 * @param mag_y [gauss] 
 * @param mag_z [gauss] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float acc_x,float acc_y,float acc_z,float temp_C,float gyro_x,float gyro_y,float gyro_z,float mag_x,float mag_y,float mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_LEN];
    _mav_put_float(buf, 0, acc_x);
    _mav_put_float(buf, 4, acc_y);
    _mav_put_float(buf, 8, acc_z);
    _mav_put_float(buf, 12, temp_C);
    _mav_put_float(buf, 16, gyro_x);
    _mav_put_float(buf, 20, gyro_y);
    _mav_put_float(buf, 24, gyro_z);
    _mav_put_float(buf, 28, mag_x);
    _mav_put_float(buf, 32, mag_y);
    _mav_put_float(buf, 36, mag_z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IMU_LEN);
#else
    mavlink_imu_t packet;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;
    packet.temp_C = temp_C;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IMU;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
}

/**
 * @brief Encode a imu struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_imu_t* imu)
{
    return mavlink_msg_imu_pack(system_id, component_id, msg, imu->acc_x, imu->acc_y, imu->acc_z, imu->temp_C, imu->gyro_x, imu->gyro_y, imu->gyro_z, imu->mag_x, imu->mag_y, imu->mag_z);
}

/**
 * @brief Encode a imu struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_imu_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_imu_t* imu)
{
    return mavlink_msg_imu_pack_chan(system_id, component_id, chan, msg, imu->acc_x, imu->acc_y, imu->acc_z, imu->temp_C, imu->gyro_x, imu->gyro_y, imu->gyro_z, imu->mag_x, imu->mag_y, imu->mag_z);
}

/**
 * @brief Send a imu message
 * @param chan MAVLink channel to send the message
 *
 * @param acc_x [m/s/s] 
 * @param acc_y [m/s/s] 
 * @param acc_z [m/s/s] 
 * @param temp_C  
 * @param gyro_x [rad/s] 
 * @param gyro_y [rad/s] 
 * @param gyro_z [rad/s] 
 * @param mag_x [gauss] 
 * @param mag_y [gauss] 
 * @param mag_z [gauss] 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_imu_send(mavlink_channel_t chan, float acc_x, float acc_y, float acc_z, float temp_C, float gyro_x, float gyro_y, float gyro_z, float mag_x, float mag_y, float mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IMU_LEN];
    _mav_put_float(buf, 0, acc_x);
    _mav_put_float(buf, 4, acc_y);
    _mav_put_float(buf, 8, acc_z);
    _mav_put_float(buf, 12, temp_C);
    _mav_put_float(buf, 16, gyro_x);
    _mav_put_float(buf, 20, gyro_y);
    _mav_put_float(buf, 24, gyro_z);
    _mav_put_float(buf, 28, mag_x);
    _mav_put_float(buf, 32, mag_y);
    _mav_put_float(buf, 36, mag_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU, buf, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
#else
    mavlink_imu_t packet;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.acc_z = acc_z;
    packet.temp_C = temp_C;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU, (const char *)&packet, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
#endif
}

/**
 * @brief Send a imu message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_imu_send_struct(mavlink_channel_t chan, const mavlink_imu_t* imu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_imu_send(chan, imu->acc_x, imu->acc_y, imu->acc_z, imu->temp_C, imu->gyro_x, imu->gyro_y, imu->gyro_z, imu->mag_x, imu->mag_y, imu->mag_z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU, (const char *)imu, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
#endif
}

#if MAVLINK_MSG_ID_IMU_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_imu_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float acc_x, float acc_y, float acc_z, float temp_C, float gyro_x, float gyro_y, float gyro_z, float mag_x, float mag_y, float mag_z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, acc_x);
    _mav_put_float(buf, 4, acc_y);
    _mav_put_float(buf, 8, acc_z);
    _mav_put_float(buf, 12, temp_C);
    _mav_put_float(buf, 16, gyro_x);
    _mav_put_float(buf, 20, gyro_y);
    _mav_put_float(buf, 24, gyro_z);
    _mav_put_float(buf, 28, mag_x);
    _mav_put_float(buf, 32, mag_y);
    _mav_put_float(buf, 36, mag_z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU, buf, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
#else
    mavlink_imu_t *packet = (mavlink_imu_t *)msgbuf;
    packet->acc_x = acc_x;
    packet->acc_y = acc_y;
    packet->acc_z = acc_z;
    packet->temp_C = temp_C;
    packet->gyro_x = gyro_x;
    packet->gyro_y = gyro_y;
    packet->gyro_z = gyro_z;
    packet->mag_x = mag_x;
    packet->mag_y = mag_y;
    packet->mag_z = mag_z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IMU, (const char *)packet, MAVLINK_MSG_ID_IMU_MIN_LEN, MAVLINK_MSG_ID_IMU_LEN, MAVLINK_MSG_ID_IMU_CRC);
#endif
}
#endif

#endif

// MESSAGE IMU UNPACKING


/**
 * @brief Get field acc_x from imu message
 *
 * @return [m/s/s] 
 */
static inline float mavlink_msg_imu_get_acc_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field acc_y from imu message
 *
 * @return [m/s/s] 
 */
static inline float mavlink_msg_imu_get_acc_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field acc_z from imu message
 *
 * @return [m/s/s] 
 */
static inline float mavlink_msg_imu_get_acc_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field temp_C from imu message
 *
 * @return  
 */
static inline float mavlink_msg_imu_get_temp_C(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field gyro_x from imu message
 *
 * @return [rad/s] 
 */
static inline float mavlink_msg_imu_get_gyro_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field gyro_y from imu message
 *
 * @return [rad/s] 
 */
static inline float mavlink_msg_imu_get_gyro_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field gyro_z from imu message
 *
 * @return [rad/s] 
 */
static inline float mavlink_msg_imu_get_gyro_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field mag_x from imu message
 *
 * @return [gauss] 
 */
static inline float mavlink_msg_imu_get_mag_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field mag_y from imu message
 *
 * @return [gauss] 
 */
static inline float mavlink_msg_imu_get_mag_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field mag_z from imu message
 *
 * @return [gauss] 
 */
static inline float mavlink_msg_imu_get_mag_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Decode a imu message into a struct
 *
 * @param msg The message to decode
 * @param imu C-struct to decode the message contents into
 */
static inline void mavlink_msg_imu_decode(const mavlink_message_t* msg, mavlink_imu_t* imu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    imu->acc_x = mavlink_msg_imu_get_acc_x(msg);
    imu->acc_y = mavlink_msg_imu_get_acc_y(msg);
    imu->acc_z = mavlink_msg_imu_get_acc_z(msg);
    imu->temp_C = mavlink_msg_imu_get_temp_C(msg);
    imu->gyro_x = mavlink_msg_imu_get_gyro_x(msg);
    imu->gyro_y = mavlink_msg_imu_get_gyro_y(msg);
    imu->gyro_z = mavlink_msg_imu_get_gyro_z(msg);
    imu->mag_x = mavlink_msg_imu_get_mag_x(msg);
    imu->mag_y = mavlink_msg_imu_get_mag_y(msg);
    imu->mag_z = mavlink_msg_imu_get_mag_z(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_IMU_LEN? msg->len : MAVLINK_MSG_ID_IMU_LEN;
        memset(imu, 0, MAVLINK_MSG_ID_IMU_LEN);
    memcpy(imu, _MAV_PAYLOAD(msg), len);
#endif
}
