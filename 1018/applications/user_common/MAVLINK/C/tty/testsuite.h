/** @file
 *    @brief MAVLink comm protocol testsuite generated from tty.xml
 *    @see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef TTY_TESTSUITE_H
#define TTY_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_tty(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

    mavlink_test_tty(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_imu_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_IMU_RAW >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_imu_raw_t packet_in = {
        17235,17339,17443,17547,17651,17755,17859,17963,18067,18171
    };
    mavlink_imu_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.acc_x_raw = packet_in.acc_x_raw;
        packet1.acc_y_raw = packet_in.acc_y_raw;
        packet1.acc_z_raw = packet_in.acc_z_raw;
        packet1.temp_raw = packet_in.temp_raw;
        packet1.gyro_x_raw = packet_in.gyro_x_raw;
        packet1.gyro_y_raw = packet_in.gyro_y_raw;
        packet1.gyro_z_raw = packet_in.gyro_z_raw;
        packet1.mag_x_raw = packet_in.mag_x_raw;
        packet1.mag_y_raw = packet_in.mag_y_raw;
        packet1.mag_z_raw = packet_in.mag_z_raw;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_IMU_RAW_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_IMU_RAW_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_raw_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_imu_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_raw_pack(system_id, component_id, &msg , packet1.acc_x_raw , packet1.acc_y_raw , packet1.acc_z_raw , packet1.temp_raw , packet1.gyro_x_raw , packet1.gyro_y_raw , packet1.gyro_z_raw , packet1.mag_x_raw , packet1.mag_y_raw , packet1.mag_z_raw );
    mavlink_msg_imu_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.acc_x_raw , packet1.acc_y_raw , packet1.acc_z_raw , packet1.temp_raw , packet1.gyro_x_raw , packet1.gyro_y_raw , packet1.gyro_z_raw , packet1.mag_x_raw , packet1.mag_y_raw , packet1.mag_z_raw );
    mavlink_msg_imu_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_imu_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_raw_send(MAVLINK_COMM_1 , packet1.acc_x_raw , packet1.acc_y_raw , packet1.acc_z_raw , packet1.temp_raw , packet1.gyro_x_raw , packet1.gyro_y_raw , packet1.gyro_z_raw , packet1.mag_x_raw , packet1.mag_y_raw , packet1.mag_z_raw );
    mavlink_msg_imu_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_imu(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_IMU >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_imu_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,241.0,269.0
    };
    mavlink_imu_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.acc_x = packet_in.acc_x;
        packet1.acc_y = packet_in.acc_y;
        packet1.acc_z = packet_in.acc_z;
        packet1.temp_C = packet_in.temp_C;
        packet1.gyro_x = packet_in.gyro_x;
        packet1.gyro_y = packet_in.gyro_y;
        packet1.gyro_z = packet_in.gyro_z;
        packet1.mag_x = packet_in.mag_x;
        packet1.mag_y = packet_in.mag_y;
        packet1.mag_z = packet_in.mag_z;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_IMU_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_IMU_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_pack(system_id, component_id, &msg , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.temp_C , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z );
    mavlink_msg_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.temp_C , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z );
    mavlink_msg_imu_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_imu_send(MAVLINK_COMM_1 , packet1.acc_x , packet1.acc_y , packet1.acc_z , packet1.temp_C , packet1.gyro_x , packet1.gyro_y , packet1.gyro_z , packet1.mag_x , packet1.mag_y , packet1.mag_z );
    mavlink_msg_imu_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_pressure(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PRESSURE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_pressure_t packet_in = {
        17.0,45.0,73.0,101.0
    };
    mavlink_pressure_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.abs_pressure = packet_in.abs_pressure;
        packet1.temperature = packet_in.temperature;
        packet1.ground_pressure = packet_in.ground_pressure;
        packet1.pressure_alt = packet_in.pressure_alt;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PRESSURE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PRESSURE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pressure_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pressure_pack(system_id, component_id, &msg , packet1.abs_pressure , packet1.temperature , packet1.ground_pressure , packet1.pressure_alt );
    mavlink_msg_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pressure_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.abs_pressure , packet1.temperature , packet1.ground_pressure , packet1.pressure_alt );
    mavlink_msg_pressure_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_pressure_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_pressure_send(MAVLINK_COMM_1 , packet1.abs_pressure , packet1.temperature , packet1.ground_pressure , packet1.pressure_alt );
    mavlink_msg_pressure_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sonar(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_SONAR >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_sonar_t packet_in = {
        17.0,45.0
    };
    mavlink_sonar_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.distance = packet_in.distance;
        packet1.distance_alt = packet_in.distance_alt;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_SONAR_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_SONAR_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sonar_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_sonar_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sonar_pack(system_id, component_id, &msg , packet1.distance , packet1.distance_alt );
    mavlink_msg_sonar_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sonar_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.distance , packet1.distance_alt );
    mavlink_msg_sonar_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_sonar_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_sonar_send(MAVLINK_COMM_1 , packet1.distance , packet1.distance_alt );
    mavlink_msg_sonar_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_attitude(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ATTITUDE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_attitude_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0
    };
    mavlink_attitude_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.roll = packet_in.roll;
        packet1.pitch = packet_in.pitch;
        packet1.yaw = packet_in.yaw;
        packet1.roll_speed = packet_in.roll_speed;
        packet1.pitch_speed = packet_in.pitch_speed;
        packet1.yaw_speed = packet_in.yaw_speed;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ATTITUDE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ATTITUDE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_pack(system_id, component_id, &msg , packet1.roll , packet1.pitch , packet1.yaw , packet1.roll_speed , packet1.pitch_speed , packet1.yaw_speed );
    mavlink_msg_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.roll , packet1.pitch , packet1.yaw , packet1.roll_speed , packet1.pitch_speed , packet1.yaw_speed );
    mavlink_msg_attitude_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_attitude_send(MAVLINK_COMM_1 , packet1.roll , packet1.pitch , packet1.yaw , packet1.roll_speed , packet1.pitch_speed , packet1.yaw_speed );
    mavlink_msg_attitude_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_position(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_POSITION >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_position_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,157.0
    };
    mavlink_position_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.z = packet_in.z;
        packet1.x_speed = packet_in.x_speed;
        packet1.y_speed = packet_in.y_speed;
        packet1.z_speed = packet_in.z_speed;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_POSITION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_POSITION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_position_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_position_pack(system_id, component_id, &msg , packet1.x , packet1.y , packet1.z , packet1.x_speed , packet1.y_speed , packet1.z_speed );
    mavlink_msg_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_position_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.x , packet1.y , packet1.z , packet1.x_speed , packet1.y_speed , packet1.z_speed );
    mavlink_msg_position_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_position_send(MAVLINK_COMM_1 , packet1.x , packet1.y , packet1.z , packet1.x_speed , packet1.y_speed , packet1.z_speed );
    mavlink_msg_position_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rocker(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ROCKER >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rocker_t packet_in = {
        17235,17339,17443,17547,17651,17755,17859,17963,18067,18171,18275
    };
    mavlink_rocker_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.leftX = packet_in.leftX;
        packet1.leftY = packet_in.leftY;
        packet1.rightX = packet_in.rightX;
        packet1.rightY = packet_in.rightY;
        packet1.switchA = packet_in.switchA;
        packet1.switchB = packet_in.switchB;
        packet1.switchC = packet_in.switchC;
        packet1.switchD = packet_in.switchD;
        packet1.switchE = packet_in.switchE;
        packet1.switchF = packet_in.switchF;
        packet1.switchG = packet_in.switchG;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ROCKER_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ROCKER_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rocker_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rocker_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rocker_pack(system_id, component_id, &msg , packet1.leftX , packet1.leftY , packet1.rightX , packet1.rightY , packet1.switchA , packet1.switchB , packet1.switchC , packet1.switchD , packet1.switchE , packet1.switchF , packet1.switchG );
    mavlink_msg_rocker_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rocker_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.leftX , packet1.leftY , packet1.rightX , packet1.rightY , packet1.switchA , packet1.switchB , packet1.switchC , packet1.switchD , packet1.switchE , packet1.switchF , packet1.switchG );
    mavlink_msg_rocker_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rocker_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rocker_send(MAVLINK_COMM_1 , packet1.leftX , packet1.leftY , packet1.rightX , packet1.rightY , packet1.switchA , packet1.switchB , packet1.switchC , packet1.switchD , packet1.switchE , packet1.switchF , packet1.switchG );
    mavlink_msg_rocker_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_param_read_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PARAM_READ_REQUEST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_param_read_request_t packet_in = {
        5,72,139
    };
    mavlink_param_read_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.SYS_TYPE = packet_in.SYS_TYPE;
        packet1.DEV_ID = packet_in.DEV_ID;
        packet1.param_id = packet_in.param_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PARAM_READ_REQUEST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PARAM_READ_REQUEST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_read_request_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_param_read_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_read_request_pack(system_id, component_id, &msg , packet1.SYS_TYPE , packet1.DEV_ID , packet1.param_id );
    mavlink_msg_param_read_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_read_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.SYS_TYPE , packet1.DEV_ID , packet1.param_id );
    mavlink_msg_param_read_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_param_read_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_read_request_send(MAVLINK_COMM_1 , packet1.SYS_TYPE , packet1.DEV_ID , packet1.param_id );
    mavlink_msg_param_read_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_param_read_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PARAM_READ_ACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_param_read_ack_t packet_in = {
        17.0,17
    };
    mavlink_param_read_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.value = packet_in.value;
        packet1.param_id = packet_in.param_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PARAM_READ_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PARAM_READ_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_read_ack_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_param_read_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_read_ack_pack(system_id, component_id, &msg , packet1.param_id , packet1.value );
    mavlink_msg_param_read_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_read_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.param_id , packet1.value );
    mavlink_msg_param_read_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_param_read_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_read_ack_send(MAVLINK_COMM_1 , packet1.param_id , packet1.value );
    mavlink_msg_param_read_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_param_write(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PARAM_WRITE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_param_write_t packet_in = {
        17.0,17
    };
    mavlink_param_write_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.value = packet_in.value;
        packet1.param_id = packet_in.param_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PARAM_WRITE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PARAM_WRITE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_write_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_param_write_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_write_pack(system_id, component_id, &msg , packet1.param_id , packet1.value );
    mavlink_msg_param_write_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_write_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.param_id , packet1.value );
    mavlink_msg_param_write_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_param_write_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_write_send(MAVLINK_COMM_1 , packet1.param_id , packet1.value );
    mavlink_msg_param_write_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_param_write_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_PARAM_WRITE_ACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_param_write_ack_t packet_in = {
        5
    };
    mavlink_param_write_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.param_id = packet_in.param_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_PARAM_WRITE_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_PARAM_WRITE_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_write_ack_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_param_write_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_write_ack_pack(system_id, component_id, &msg , packet1.param_id );
    mavlink_msg_param_write_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_write_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.param_id );
    mavlink_msg_param_write_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_param_write_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_param_write_ack_send(MAVLINK_COMM_1 , packet1.param_id );
    mavlink_msg_param_write_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_cmd_write(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CMD_WRITE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_cmd_write_t packet_in = {
        5,72,139
    };
    mavlink_cmd_write_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.SYS_TYPE = packet_in.SYS_TYPE;
        packet1.DEV_ID = packet_in.DEV_ID;
        packet1.cmd_id = packet_in.cmd_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CMD_WRITE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CMD_WRITE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cmd_write_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_cmd_write_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cmd_write_pack(system_id, component_id, &msg , packet1.SYS_TYPE , packet1.DEV_ID , packet1.cmd_id );
    mavlink_msg_cmd_write_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cmd_write_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.SYS_TYPE , packet1.DEV_ID , packet1.cmd_id );
    mavlink_msg_cmd_write_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_cmd_write_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cmd_write_send(MAVLINK_COMM_1 , packet1.SYS_TYPE , packet1.DEV_ID , packet1.cmd_id );
    mavlink_msg_cmd_write_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_cmd_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_CMD_ACK >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_cmd_ack_t packet_in = {
        5,72
    };
    mavlink_cmd_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.cmd_id = packet_in.cmd_id;
        packet1.cmd_ack_id = packet_in.cmd_ack_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_CMD_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_CMD_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cmd_ack_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_cmd_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cmd_ack_pack(system_id, component_id, &msg , packet1.cmd_id , packet1.cmd_ack_id );
    mavlink_msg_cmd_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cmd_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.cmd_id , packet1.cmd_ack_id );
    mavlink_msg_cmd_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_cmd_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_cmd_ack_send(MAVLINK_COMM_1 , packet1.cmd_id , packet1.cmd_ack_id );
    mavlink_msg_cmd_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_battery_voltage(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_BATTERY_VOLTAGE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_battery_voltage_t packet_in = {
        17.0
    };
    mavlink_battery_voltage_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.voltage = packet_in.voltage;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_BATTERY_VOLTAGE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_BATTERY_VOLTAGE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_voltage_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_battery_voltage_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_voltage_pack(system_id, component_id, &msg , packet1.voltage );
    mavlink_msg_battery_voltage_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_voltage_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.voltage );
    mavlink_msg_battery_voltage_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_battery_voltage_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_battery_voltage_send(MAVLINK_COMM_1 , packet1.voltage );
    mavlink_msg_battery_voltage_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_usv_state(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_USV_STATE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_usv_state_t packet_in = {
        17.0,45.0,73.0,101.0,129.0
    };
    mavlink_usv_state_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.x = packet_in.x;
        packet1.y = packet_in.y;
        packet1.speed = packet_in.speed;
        packet1.course = packet_in.course;
        packet1.heading = packet_in.heading;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_USV_STATE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_USV_STATE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_usv_state_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_usv_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_usv_state_pack(system_id, component_id, &msg , packet1.x , packet1.y , packet1.speed , packet1.course , packet1.heading );
    mavlink_msg_usv_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_usv_state_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.x , packet1.y , packet1.speed , packet1.course , packet1.heading );
    mavlink_msg_usv_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_usv_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_usv_state_send(MAVLINK_COMM_1 , packet1.x , packet1.y , packet1.speed , packet1.course , packet1.heading );
    mavlink_msg_usv_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_usv_set(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_USV_SET >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_usv_set_t packet_in = {
        17235,17339,17,84
    };
    mavlink_usv_set_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.MotorSet = packet_in.MotorSet;
        packet1.RudderSet = packet_in.RudderSet;
        packet1.SYS_TYPE = packet_in.SYS_TYPE;
        packet1.DEV_ID = packet_in.DEV_ID;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_USV_SET_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_USV_SET_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_usv_set_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_usv_set_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_usv_set_pack(system_id, component_id, &msg , packet1.SYS_TYPE , packet1.DEV_ID , packet1.MotorSet , packet1.RudderSet );
    mavlink_msg_usv_set_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_usv_set_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.SYS_TYPE , packet1.DEV_ID , packet1.MotorSet , packet1.RudderSet );
    mavlink_msg_usv_set_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_usv_set_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_usv_set_send(MAVLINK_COMM_1 , packet1.SYS_TYPE , packet1.DEV_ID , packet1.MotorSet , packet1.RudderSet );
    mavlink_msg_usv_set_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_uav_set(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_UAV_SET >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_uav_set_t packet_in = {
        17.0,45.0,73.0,41,108
    };
    mavlink_uav_set_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.X_Speed = packet_in.X_Speed;
        packet1.Y_Speed = packet_in.Y_Speed;
        packet1.Z_Speed = packet_in.Z_Speed;
        packet1.SYS_TYPE = packet_in.SYS_TYPE;
        packet1.DEV_ID = packet_in.DEV_ID;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_UAV_SET_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_UAV_SET_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uav_set_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_uav_set_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uav_set_pack(system_id, component_id, &msg , packet1.SYS_TYPE , packet1.DEV_ID , packet1.X_Speed , packet1.Y_Speed , packet1.Z_Speed );
    mavlink_msg_uav_set_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uav_set_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.SYS_TYPE , packet1.DEV_ID , packet1.X_Speed , packet1.Y_Speed , packet1.Z_Speed );
    mavlink_msg_uav_set_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_uav_set_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_uav_set_send(MAVLINK_COMM_1 , packet1.SYS_TYPE , packet1.DEV_ID , packet1.X_Speed , packet1.Y_Speed , packet1.Z_Speed );
    mavlink_msg_uav_set_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_tty(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_imu_raw(system_id, component_id, last_msg);
    mavlink_test_imu(system_id, component_id, last_msg);
    mavlink_test_pressure(system_id, component_id, last_msg);
    mavlink_test_sonar(system_id, component_id, last_msg);
    mavlink_test_attitude(system_id, component_id, last_msg);
    mavlink_test_position(system_id, component_id, last_msg);
    mavlink_test_rocker(system_id, component_id, last_msg);
    mavlink_test_param_read_request(system_id, component_id, last_msg);
    mavlink_test_param_read_ack(system_id, component_id, last_msg);
    mavlink_test_param_write(system_id, component_id, last_msg);
    mavlink_test_param_write_ack(system_id, component_id, last_msg);
    mavlink_test_cmd_write(system_id, component_id, last_msg);
    mavlink_test_cmd_ack(system_id, component_id, last_msg);
    mavlink_test_battery_voltage(system_id, component_id, last_msg);
    mavlink_test_usv_state(system_id, component_id, last_msg);
    mavlink_test_usv_set(system_id, component_id, last_msg);
    mavlink_test_uav_set(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // TTY_TESTSUITE_H
