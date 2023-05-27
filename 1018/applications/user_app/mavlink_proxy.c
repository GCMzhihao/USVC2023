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
        else if(msg->msgid==MAVLINK_MSG_ID_PARAM_READ)//读参数请求
        {
            uint8_t par_id=mavlink_msg_param_read_get_param_id(msg);
            rt_sem_take(sem_uart1_tx, RT_WAITING_FOREVER);

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
            USV_SET.Speed=mavlink_msg_usv_set_get_Speed(msg);
            USV_SET.Heading=mavlink_msg_usv_set_get_Heading(msg);
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
           GPS.KSXT.Longitude,
           GPS.KSXT.Latitude,
           GPS.KSXT.Vel,
           GPS.KSXT.heading,
           USV_State.BatteryVoltage);
    mav_length+=mavlink_msg_to_send_buffer(&mav_buffer[mav_length], &mav_msg);
//   mavlink_msg_rocker_pack(sys_id, dev_id, &mav_msg, rocker.leftX, rocker.leftY, rocker.rightX, rocker.rightY, rocker.switchA, rocker.switchB, rocker.switchC, rocker.switchD, rocker.switchE, rocker.switchF, rocker.switchG);
//   mav_length+=mavlink_msg_to_send_buffer(&mav_buffer[mav_length], &mav_msg);
    rt_device_write(uart1, 0, mav_buffer, mav_length);
    rt_sem_release(sem_uart1_tx);//释放uart1发送
}
