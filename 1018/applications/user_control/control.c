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

static int16_t YellowShipPWMConvert_LeftMotor(int16_t value)//输入1000-2000，输出1460-1000
{
    return -0.46*value+1920;
}

static int16_t YellowShipPWMConvert_RightMotor(int16_t value)//输入1000-2000，输出1501-2000
{
    return 0.499*value+1002;
}


static int16_t YellowShipSteerPWMConvert(int16_t value)
{
    return ((1500-value)*0.3+1500);
}
//////////////////////////////////////////////////////////////////////////////////////////
//小白船：dev_id:1-3
//PWM通道1：推进器
//PWM通道2：舵机
//PWM通道3：排水电机
//推进器电调校准：（校准一次即可）
//1.给主控制器供电并断开推进器电源
//2.遥控器解锁然后将左摇杆Y推至高位
//3.推进器连接电源，此时能听到电调滴滴声，迅速将左摇杆Y拉至地位，接着听到电调滴~滴~滴~，电调校准完毕
/////////////////////////////////////////////////////////////////////////////////////////
//黄色双体船:
//PWM通道1：左推进器1460-1499不转
//PWM通道2：右推进器1460-1501不转
//PWM通道3：舵机
////////////////////////////////////////////////////////////////////////////////////////
void RockerControl(void)
{
    int16_t pwm1,pwm2,pwm3;
    if(USV_State.AutoSail)
        return;
    if(sys_id==SYS_USV&&dev_id>0)
    {
        pwm1=rocker.leftY;
        pwm2=rocker.rightX;
        pwm3=1600;
    }
    else if(sys_id==SYS_USV&&dev_id==0)
    {
        pwm1 = YellowShipPWMConvert_LeftMotor(rocker.leftY);
        pwm2 = YellowShipPWMConvert_RightMotor(rocker.leftY);
        pwm3 = YellowShipSteerPWMConvert(rocker.rightX);
    }
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

    MotorPWMSet(pwm1, pwm2, pwm3);
}


