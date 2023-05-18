/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-18     PC-COLD       the first version
 */
#ifndef APPLICATIONS_USER_APP_PARAMETER_H_
#define APPLICATIONS_USER_APP_PARAMETER_H_
typedef struct
{
   float kp;
   float ki;
   float kd;
}param_pid;
typedef struct
{
    param_pid inner;
    param_pid outer;
}param_cascade_pid;
typedef struct
{
    param_cascade_pid roll;
    param_cascade_pid pitch;
    param_cascade_pid yaw;
    VectorFloat acc_self_offset;//加计球心零偏
    VectorFloat acc_self_gain_inv;//加计椭球拟合轴长的倒数
    VectorFloat gyro_offset;//陀螺仪零偏
    VectorFloat mag_offset;//磁力计球心零偏
    VectorFloat horiz_offset;//机体水平零偏
    VectorFloat mag_gain_inv;//磁力计椭球拟合轴长的倒数
    int16_t left_rudder_mid_value;
    int16_t right_rudder_mid_value;
}_parameters;

extern _parameters parameters;

void param_read(void);
void param_write(void);
#endif /* APPLICATIONS_USER_APP_PARAMETER_H_ */
