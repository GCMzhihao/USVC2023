/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-19     PC-COLD       the first version
 */
#include <include.h>

_USV_State USV_State;
PID USV_Speed_PID;
PID USV_Heading_PID;
_USV_SET USV_SET;
void USV_PID_Init(void)
{
    PIDInit(&USV_Speed_PID, parameters.usv_speed_pid.kp, parameters.usv_speed_pid.ki, parameters.usv_speed_pid.kd, 100, 800, 0);
    PIDInit(&USV_Heading_PID, parameters.usv_heading_pid.kp, parameters.usv_heading_pid.ki, parameters.usv_heading_pid.kd, 100, 500, 0);
}
int USV_State_Init(void)
{
    USV_State.AutoSail=0;
    USV_State.LeftRudderOk=1;
    USV_State.RightRudderOk=1;

    rocker.leftX=1500;
    rocker.leftY=1000;
    rocker.rightX=1500;
    rocker.rightY=1500;
    rocker.switchA=1500;
    rocker.switchB=1500;
    rocker.switchC=1500;
    rocker.switchD=1500;
    rocker.switchE=1500;
    rocker.switchF=1500;
    rocker.switchG=1500;

    return RT_EOK;
}

static int16_t PWMConvert(int16_t value)//value:0-1000
{
    return value+1024;
}

//////////////////////////////////////////////////////////////////////////////////////////
//小白船：dev_id:1-3
//PWM通道1：推进器
//PWM通道2：转向
//PWM通道3：前进后退
void RockerControl(void)
{
    int16_t pwm1,pwm2,pwm3;
//    if(USV_State.AutoSail)
//        return;
//    if(sys_id==SYS_USV&&dev_id>0)
//    {
//        pwm1=rocker.leftY;
//        pwm2=rocker.rightX;
//        pwm3=1600;
//    }

    if(USV_State.back)
    {
        pwm3 =1750;
    }
    else
    {
        pwm3=1250;
    }
    pwm1 = rocker.leftY;
    pwm2 = rocker.rightX;

    pwm1=353;//调试时用 关闭电机，调试舵机
    MotorPWMSet(pwm1, pwm2, pwm3);
}

void CommandControl(float dt)
{
    uint16_t pwm1,pwm2,pwm3;
    if(!USV_State.AutoSail)
        return;
    if(sys_id==SYS_USV&&dev_id>0)//小白船，编队
    {

    }

  //  MotorPWMSet(pwm1, pwm2, pwm3);
}


