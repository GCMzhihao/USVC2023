/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-08     PC-COLD       the first version
 */
#ifndef APPLICATIONS_USER_COMMON_PID_H_
#define APPLICATIONS_USER_COMMON_PID_H_

typedef struct
{
    float Kp;//比例系数
    float Ki;//积分系数
    float Kd;//微分系数
    float dt;//时间常数

    float SetValue;//设定值
    float ActualValue;//实际值
    float LastErr;//上次偏差
    float dErr;//偏差的偏差

    float Integral;//偏差积分
    float IntMax;// 最大积分

    float LPF_frq;//D项低通截止频率
    float Pout;  //比例输出
    float Iout;  //积分输出
    float Dout;  //微分输出
    float OutPut;//PID输出
    float OutPutMax;//输出限幅
}PID;

void PIDCalculation(PID *pid);
void PIDInit(PID *pid,float kp,float ki,float kd,float intmax,float outputmax,float frq);

#endif /* APPLICATIONS_USER_COMMON_PID_H_ */
