/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-31     PC-COLD       the first version
 */
#include <include.h>
mavlink_rocker_t rocker;
uint8_t sys_id;
uint8_t dev_id;

void mavlink_msg_proxy(mavlink_message_t *msg , mavlink_status_t* status)
{
    mavlink_message_t mav_msg;
    uint8_t mav_buffer[100];
    uint16_t mav_length;
    if(msg->sysid == SYS_ROCKER)
    {
        USV_State.AutoSail=0;
        mavlink_msg_rocker_decode(msg, &rocker);
    }
    else if(msg->sysid==SYS_GSTATION)
    {
        if(msg->msgid==MAVLINK_MSG_ID_ROCKER)
        {
            USV_State.AutoSail=0;
            mavlink_msg_rocker_decode(msg, &rocker);
        }
        else if(msg->msgid==MAVLINK_MSG_ID_PARAM_READ_REQUEST)//读参数请求
        {
            uint8_t par_id=mavlink_msg_param_read_request_get_param_id(msg);
            rt_sem_take(sem_uart1_tx, RT_WAITING_FOREVER);
            if(par_id==PARAM_ROLL_INNER_P||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_ROLL_INNER_P, parameters.roll.inner.kp);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_ROLL_INNER_I||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_ROLL_INNER_I, parameters.roll.inner.ki);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_ROLL_INNER_D||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_ROLL_INNER_D, parameters.roll.inner.kd);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_ROLL_OUTER_P||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_ROLL_OUTER_P, parameters.roll.outer.kp);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_ROLL_OUTER_I||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_ROLL_OUTER_I, parameters.roll.outer.ki);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_ROLL_OUTER_D||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_ROLL_OUTER_D, parameters.roll.outer.kd);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_PITCH_INNER_P||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_PITCH_INNER_P, parameters.pitch.inner.kp);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_PITCH_INNER_I||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_PITCH_INNER_I, parameters.pitch.inner.ki);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_PITCH_INNER_D||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_PITCH_INNER_D, parameters.pitch.inner.kd);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_PITCH_OUTER_P||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_PITCH_OUTER_P, parameters.pitch.outer.kp);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_PITCH_OUTER_I||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_PITCH_OUTER_I, parameters.pitch.outer.ki);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_PITCH_OUTER_D||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_PITCH_OUTER_D, parameters.pitch.outer.kd);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_YAW_INNER_P||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_YAW_INNER_P, parameters.yaw.inner.kp);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_YAW_INNER_I||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_YAW_INNER_I, parameters.yaw.inner.ki);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_YAW_INNER_D||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_YAW_INNER_D, parameters.yaw.inner.kd);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_YAW_OUTER_P||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_YAW_OUTER_P, parameters.yaw.outer.kp);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_YAW_OUTER_I||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_YAW_OUTER_I, parameters.yaw.outer.ki);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_YAW_OUTER_D||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_YAW_OUTER_D, parameters.yaw.outer.kd);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_ACC_OFFSET_X||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_ACC_OFFSET_X, parameters.acc_self_offset.x);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_ACC_OFFSET_Y||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_ACC_OFFSET_Y, parameters.acc_self_offset.y);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_ACC_OFFSET_Z||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_ACC_OFFSET_Z, parameters.acc_self_offset.z);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_ACC_GAIN_INV_X||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_ACC_GAIN_INV_X, parameters.acc_self_gain_inv.x);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_ACC_GAIN_INV_Y||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_ACC_GAIN_INV_Y, parameters.acc_self_gain_inv.y);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_ACC_GAIN_INV_Z||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_ACC_GAIN_INV_Z, parameters.acc_self_gain_inv.z);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_HORIZ_OFFSET_X||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_HORIZ_OFFSET_X, parameters.horiz_offset.x);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_HORIZ_OFFSET_Y||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_HORIZ_OFFSET_Y, parameters.horiz_offset.y);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_HORIZ_OFFSET_Z||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_HORIZ_OFFSET_Z, parameters.horiz_offset.z);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_GYRO_OFFSET_X||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_GYRO_OFFSET_X, parameters.gyro_offset.x);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_GYRO_OFFSET_Y||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_GYRO_OFFSET_Y, parameters.gyro_offset.y);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_GYRO_OFFSET_Z||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_GYRO_OFFSET_Z, parameters.gyro_offset.z);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_MAG_OFFSET_X||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_MAG_OFFSET_X, parameters.mag_offset.x);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_MAG_OFFSET_Y||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_MAG_OFFSET_Y, parameters.mag_offset.y);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_MAG_OFFSET_Z||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_MAG_OFFSET_Z, parameters.mag_offset.z);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_MAG_GAIN_INV_X||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_MAG_GAIN_INV_X, parameters.mag_gain_inv.x);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_MAG_GAIN_INV_Y||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_MAG_GAIN_INV_Y, parameters.mag_gain_inv.y);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_MAG_GAIN_INV_Z||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_MAG_GAIN_INV_Z, parameters.mag_gain_inv.z);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_USV_SPEED_P||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_USV_SPEED_P, parameters.usv_speed_pid.kp);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_USV_SPEED_I||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_USV_SPEED_I, parameters.usv_speed_pid.ki);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_USV_SPEED_D||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_USV_SPEED_D, parameters.usv_speed_pid.kd);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_USV_HEADING_P||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_USV_HEADING_P, parameters.usv_heading_pid.kp);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_USV_HEADING_I||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_USV_HEADING_I, parameters.usv_heading_pid.ki);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            if(par_id==PARAM_USV_HEADING_D||par_id==PARAM_LIST_REQUEST)
            {
                mavlink_msg_param_read_ack_pack(sys_id, dev_id, &mav_msg, PARAM_USV_HEADING_D, parameters.usv_heading_pid.kd);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            }
            rt_sem_release(sem_uart1_tx);//释放uart1发送
        }
        else if(msg->msgid==MAVLINK_MSG_ID_PARAM_WRITE)//写参数
        {
            uint8_t par_id;
            par_id=mavlink_msg_param_write_get_param_id(msg);
            rt_sem_take(sem_uart1_tx, RT_WAITING_FOREVER);
            switch(par_id)
            {
            case PARAM_ROLL_INNER_P:
                Attitude.Roll.InnerPID.Kp=mavlink_msg_param_write_get_value(msg);
                parameters.roll.inner.kp=Attitude.Roll.InnerPID.Kp;
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_ROLL_INNER_I:
                Attitude.Roll.InnerPID.Ki=mavlink_msg_param_write_get_value(msg);
                parameters.roll.inner.ki=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_ROLL_INNER_D:
                Attitude.Roll.InnerPID.Kd=mavlink_msg_param_write_get_value(msg);
                parameters.roll.inner.kd=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_ROLL_OUTER_P:
                Attitude.Roll.OuterPID.Kp=mavlink_msg_param_write_get_value(msg);
                parameters.roll.outer.kp=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_ROLL_OUTER_I:
                Attitude.Roll.OuterPID.Ki=mavlink_msg_param_write_get_value(msg);
                parameters.roll.outer.ki=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_ROLL_OUTER_D:
                Attitude.Roll.OuterPID.Kd=mavlink_msg_param_write_get_value(msg);
                parameters.roll.outer.kd=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;

            case PARAM_PITCH_INNER_P:
                Attitude.Pitch.InnerPID.Kp=mavlink_msg_param_write_get_value(msg);
                parameters.pitch.inner.kp=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_PITCH_INNER_I:
                Attitude.Pitch.InnerPID.Ki=mavlink_msg_param_write_get_value(msg);
                parameters.pitch.inner.ki=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_PITCH_INNER_D:
                Attitude.Pitch.InnerPID.Kd=mavlink_msg_param_write_get_value(msg);
                parameters.pitch.inner.kd=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_PITCH_OUTER_P:
                Attitude.Pitch.OuterPID.Kp=mavlink_msg_param_write_get_value(msg);
                parameters.pitch.outer.kp=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_PITCH_OUTER_I:
                Attitude.Pitch.OuterPID.Ki=mavlink_msg_param_write_get_value(msg);
                parameters.pitch.outer.ki=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_PITCH_OUTER_D:
                Attitude.Pitch.OuterPID.Kd=mavlink_msg_param_write_get_value(msg);
                parameters.pitch.outer.kd=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;

            case PARAM_YAW_INNER_P:
                Attitude.Yaw.InnerPID.Kp=mavlink_msg_param_write_get_value(msg);
                parameters.yaw.inner.kp=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_YAW_INNER_I:
                Attitude.Yaw.InnerPID.Ki=mavlink_msg_param_write_get_value(msg);
                parameters.yaw.inner.ki=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_YAW_INNER_D:
                Attitude.Yaw.InnerPID.Kd=mavlink_msg_param_write_get_value(msg);
                parameters.yaw.inner.kd=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_YAW_OUTER_P:
                Attitude.Yaw.OuterPID.Kp=mavlink_msg_param_write_get_value(msg);
                parameters.yaw.outer.kp=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_YAW_OUTER_I:
                Attitude.Yaw.OuterPID.Ki=mavlink_msg_param_write_get_value(msg);
                parameters.yaw.outer.ki=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_YAW_OUTER_D:
                Attitude.Yaw.OuterPID.Kd=mavlink_msg_param_write_get_value(msg);
                parameters.yaw.outer.kd=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_USV_SPEED_P:
                USV_Speed_PID.Kp=mavlink_msg_param_write_get_value(msg);
                parameters.usv_speed_pid.kp=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_USV_SPEED_I:
                USV_Speed_PID.Ki=mavlink_msg_param_write_get_value(msg);
                parameters.usv_speed_pid.ki=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
            case PARAM_USV_SPEED_D:
                USV_Speed_PID.Kd=mavlink_msg_param_write_get_value(msg);
                parameters.usv_speed_pid.kd=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_USV_HEADING_P:
                USV_Heading_PID.Kp=mavlink_msg_param_write_get_value(msg);
                parameters.usv_heading_pid.kp=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_USV_HEADING_I:
                USV_Heading_PID.Ki=mavlink_msg_param_write_get_value(msg);
                parameters.usv_heading_pid.ki=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case PARAM_USV_HEADING_D:
                USV_Heading_PID.Kd=mavlink_msg_param_write_get_value(msg);
                parameters.usv_heading_pid.kd=mavlink_msg_param_write_get_value(msg);
                param_write();//保存参数
                mavlink_msg_param_write_ack_pack(sys_id, dev_id, &mav_msg, par_id);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            default:
                break;
            }
            rt_sem_release(sem_uart1_tx);//释放uart1发送
        }
        else if(msg->msgid==MAVLINK_MSG_ID_CMD_WRITE)
        {
            uint8_t cmd_id;
            cmd_id=mavlink_msg_cmd_ack_get_cmd_id(msg);
            rt_sem_take(sem_uart1_tx, RT_WAITING_FOREVER);
            switch(cmd_id)
            {
            case CMD_UNLOCK:
                USV_State.Unlock=1;//解锁
                BuzzerChangeState(beep_unlock);
                mavlink_msg_cmd_ack_pack(sys_id, dev_id, &mav_msg, cmd_id, CMD_ACK_WRITE_SUCESSED);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case CMD_LOCK:
                USV_State.Unlock=0;//锁定
                BuzzerChangeState(beep_lock);
                mavlink_msg_cmd_ack_pack(sys_id, dev_id, &mav_msg, cmd_id, CMD_ACK_WRITE_SUCESSED);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            case CMD_ACC_OFFSET://六面校准指令
                if(!USV_State.HorizOffsetOk
                   ||!USV_State.AccOffsetOk
                   ||!USV_State.GyroOffsetOk
                   ||!USV_State.MagOffsetOk
                   ||!USV_State.BaroOffsetOk
                   ||!USV_State.YawOffsetOk
                   )//检查到有正在校准的状态
                {
                    mavlink_msg_cmd_ack_pack(sys_id, dev_id, &mav_msg, cmd_id, CMD_ACK_WRITE_FAILED);
                    mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                    rt_device_write(uart1, 0, mav_buffer, mav_length);
                }
                else
                {
                    mavlink_msg_cmd_ack_pack(sys_id, dev_id, &mav_msg, cmd_id, CMD_ACK_WRITE_SUCESSED);
                    mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                    rt_device_write(uart1, 0, mav_buffer, mav_length);
                    USV_State.AccOffsetOk=0;//开始加速度计水平校准
                }
                break;
            case CMD_HORIZ_OFFSET://水平校准指令
                if(!USV_State.HorizOffsetOk
                   ||!USV_State.AccOffsetOk
                   ||!USV_State.GyroOffsetOk
                   ||!USV_State.MagOffsetOk
                   ||!USV_State.BaroOffsetOk
                   ||!USV_State.YawOffsetOk
                   )//检查到有正在校准的状态
                {
                    mavlink_msg_cmd_ack_pack(sys_id, dev_id, &mav_msg, cmd_id, CMD_ACK_WRITE_FAILED);
                    mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                    rt_device_write(uart1, 0, mav_buffer, mav_length);
                }
                else
                {
                    mavlink_msg_cmd_ack_pack(sys_id, dev_id, &mav_msg, cmd_id, CMD_ACK_WRITE_SUCESSED);
                    mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                    rt_device_write(uart1, 0, mav_buffer, mav_length);
                    USV_State.HorizOffsetOk=0;//开始加速度计水平校准
                }
                break;
            case CMD_YAW_OFFSET:
                if(!USV_State.HorizOffsetOk
                   ||!USV_State.AccOffsetOk
                   ||!USV_State.GyroOffsetOk
                   ||!USV_State.MagOffsetOk
                   ||!USV_State.BaroOffsetOk
                   ||!USV_State.YawOffsetOk
                   )//检查到有正在校准的状态
                {
                    mavlink_msg_cmd_ack_pack(sys_id, dev_id, &mav_msg, cmd_id, CMD_ACK_WRITE_FAILED);
                    mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                    rt_device_write(uart1, 0, mav_buffer, mav_length);
                }
                else
                {
                    mavlink_msg_cmd_ack_pack(sys_id, dev_id, &mav_msg, cmd_id, CMD_ACK_WRITE_SUCESSED);
                    mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                    rt_device_write(uart1, 0, mav_buffer, mav_length);
                    USV_State.YawOffsetOk=0;//开始加速度计水平校准
                }
                break;
            case CMD_GYRO_OFFSET:
                if(!USV_State.HorizOffsetOk
                   ||!USV_State.AccOffsetOk
                   ||!USV_State.GyroOffsetOk
                   ||!USV_State.MagOffsetOk
                   ||!USV_State.BaroOffsetOk
                   ||!USV_State.YawOffsetOk
                   )//检查到有正在校准的状态
                {
                    mavlink_msg_cmd_ack_pack(sys_id, dev_id, &mav_msg, cmd_id, CMD_ACK_WRITE_FAILED);
                    mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                    rt_device_write(uart1, 0, mav_buffer, mav_length);
                }
                else
                {

                    mavlink_msg_cmd_ack_pack(sys_id, dev_id, &mav_msg, cmd_id, CMD_ACK_WRITE_SUCESSED);
                    mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                    rt_device_write(uart1, 0, mav_buffer, mav_length);
                    USV_State.GyroOffsetOk=0;//开始陀螺仪校准
                }
                break;
            case CMD_MAG_OFFSET:
                if(!USV_State.HorizOffsetOk
                   ||!USV_State.AccOffsetOk
                   ||!USV_State.GyroOffsetOk
                   ||!USV_State.MagOffsetOk
                   ||!USV_State.BaroOffsetOk
                   ||!USV_State.YawOffsetOk
                   )//检查到有正在校准的状态
                {
                    mavlink_msg_cmd_ack_pack(sys_id, dev_id, &mav_msg, cmd_id, CMD_ACK_WRITE_FAILED);
                    mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                    rt_device_write(uart1, 0, mav_buffer, mav_length);
                }
                else
                {

                    mavlink_msg_cmd_ack_pack(sys_id, dev_id, &mav_msg, cmd_id, CMD_ACK_WRITE_SUCESSED);
                    mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                    rt_device_write(uart1, 0, mav_buffer, mav_length);
                    USV_State.MagOffsetOk=0;//开始磁力计校准
                }
                break;
            case CMD_BARO_OFFSET:
                if(!USV_State.HorizOffsetOk
                   ||!USV_State.AccOffsetOk
                   ||!USV_State.GyroOffsetOk
                   ||!USV_State.MagOffsetOk
                   ||!USV_State.BaroOffsetOk
                   ||!USV_State.YawOffsetOk
                   )//检查到有正在校准的状态
                {
                    mavlink_msg_cmd_ack_pack(sys_id, dev_id, &mav_msg, cmd_id, CMD_ACK_WRITE_FAILED);
                    mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                    rt_device_write(uart1, 0, mav_buffer, mav_length);
                }
                else
                {

                    mavlink_msg_cmd_ack_pack(sys_id, dev_id, &mav_msg, cmd_id, CMD_ACK_WRITE_SUCESSED);
                    mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                    rt_device_write(uart1, 0, mav_buffer, mav_length);
                    USV_State.BaroOffsetOk=0;//开始磁力计校准
                }
                break;
            case CMD_AUTO_DRIVE:
                USV_State.AutoSail=1;
                mavlink_msg_cmd_ack_pack(sys_id, dev_id, &mav_msg, cmd_id, CMD_ACK_WRITE_SUCESSED);
                mav_length=mavlink_msg_to_send_buffer(mav_buffer, &mav_msg);
                rt_device_write(uart1, 0, mav_buffer, mav_length);
                break;
            default:
                break;
            }
            rt_sem_release(sem_uart1_tx);//释放uart1发送
        }
        else if(msg->msgid==MAVLINK_MSG_ID_USV_SET
                &&mavlink_msg_usv_set_get_SYS_TYPE(msg)==sys_id
                &&mavlink_msg_usv_set_get_DEV_ID(msg)==dev_id)
        {
            USV_State.AutoSail=1;
            USV_SET.MotorSet=mavlink_msg_usv_set_get_MotorSet(msg);
            USV_SET.RudderSet=mavlink_msg_usv_set_get_RudderSet(msg);
        }
    }
}

void mavlink_msg_send(void)
{
    mavlink_message_t mav_msg;
    uint8_t mav_buffer[300];
    uint16_t mav_length=0;

   if( rt_sem_trytake(sem_uart1_tx)!=RT_EOK)
       return;

   mavlink_msg_usv_state_pack(sys_id, dev_id,&mav_msg,
           USV_State.X,
           USV_State.Y,
           USV_State.Speed,
           USV_State.Course,
           USV_State.Heading,
           USV_State.BatteryVoltage);
   mav_length+=mavlink_msg_to_send_buffer(&mav_buffer[mav_length], &mav_msg);

//    mavlink_msg_attitude_pack(sys_id, dev_id, &mav_msg,
//            Attitude.Roll.Value, Attitude.Pitch.Value, Attitude.Yaw.Value, sensor.gyro.x, sensor.gyro.y, sensor.gyro.z);
//    mav_length+=mavlink_msg_to_send_buffer(&mav_buffer[mav_length], &mav_msg);
//
//    mavlink_msg_imu_pack(sys_id, dev_id, &mav_msg,
//            sensor.acc.x,
//            sensor.acc.y,
//            sensor.acc.z,
//            sensor.temp_C,
//            sensor.gyro.x,
//            sensor.gyro.y,
//            sensor.gyro.z,
//            sensor.mag.x,
//            sensor.mag.y,
//            sensor.mag.z);
//    mav_length+=mavlink_msg_to_send_buffer(&mav_buffer[mav_length], &mav_msg);
//    mavlink_msg_position_pack(sys_id, dev_id, &mav_msg,
//            Position.x.Displacement.Observe,
//            Position.x.Displacement.Estimate,
//            Position.y.Displacement.Observe,
//            Position.y.Displacement.Estimate,
//            Position.x.Speed.Estimate,
//            Position.y.Speed.Estimate
//            );
//    mav_length+=mavlink_msg_to_send_buffer(&mav_buffer[mav_length], &mav_msg);


    rt_device_write(uart1, 0, mav_buffer, mav_length);
    rt_sem_release(sem_uart1_tx);//释放uart1发送
}
