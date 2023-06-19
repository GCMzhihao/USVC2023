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
rt_uint16_t USV_Rocker_lost_cnt;
void USV_PID_Init(void)
{
    PIDInit(&USV_Speed_PID, parameters.usv_speed_pid.kp, parameters.usv_speed_pid.ki, parameters.usv_speed_pid.kd, 500, 1000, 0);
    PIDInit(&USV_Heading_PID, parameters.usv_heading_pid.kp, parameters.usv_heading_pid.ki, parameters.usv_heading_pid.kd, 100, 250, 0);
}
void USV_Rocker_lost_check(void)
{
    USV_Rocker_lost_cnt++;
    if(USV_Rocker_lost_cnt>=10)
    {
        USV_State.Unlock=0;
        USV_Rocker_lost_cnt=10;
    }
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

void USV_Back_Check(void)
{
    if(rocker.switchC==1000)
    {
        USV_State.back=1;
    }
    else if (rocker.switchC==2000)
    {
        USV_State.back=0;
    }
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
    if(rocker.switchD==2000)
        return;
    //USV_Heading_PID.SetValue=GPS.KSXT.heading;//设定值等于当前朝向
    USV_Heading_PID.SetValue=45;
    USV_Heading_PID.Integral=0;
    USV_Speed_PID.Integral=0;
    if(sys_id==SYS_USV&&dev_id==0)//小黄船
    {
        if(USV_State.back)
        {
            pwm3 =1750;
        }
        else
        {
            pwm3=1250;
        }

        pwm1 = 0.4*(rocker.leftY-1000)+1000;
        pwm2 = 0.5*(rocker.rightX-1500)+1500;
    }
    else if (sys_id==SYS_USV&&dev_id>0) //小白船
    {
        pwm1 = 0.4*(rocker.leftY-1000)+1000;
        pwm2 = 0.5*(rocker.rightX-1500)+1500;
        pwm3 = 1700;

    }

  //  pwm1=1000;//调试时用 关闭电机，调试舵机
    MotorPWMSet(pwm1, pwm2, pwm3);

}

void CommandControl(float dt)
{
    uint16_t pwm1,pwm2,pwm3;
//    if(!USV_State.AutoSail)
//        return;
    if(rocker.switchD==1000)
        return;

    USV_Speed_PID.dt=dt;


    if(rocker.switchA==1000)
    {
        USV_Speed_PID.SetValue=2;
    }
    else if (rocker.switchA==1500)
    {
        USV_Speed_PID.SetValue=2.5;

    }
    else if (rocker.switchA==2000)
    {
        USV_Speed_PID.SetValue=0;
    }


   // USV_Speed_PID.SetValue=1.5;
    USV_Speed_PID.ActualValue=GPS.KSXT.Vel;
    PIDCalculation(&USV_Speed_PID);
    pwm1=USV_Speed_PID.OutPut+1000;

    USV_Heading_PID.dt=dt;

    USV_Heading_PID.ActualValue=GPS.KSXT.heading;
    float err;
    err=USV_Heading_PID.SetValue-USV_Heading_PID.ActualValue;
    if(err>180)
        err-=360;
    else if(err<-180)
        err+=360;
    USV_Heading_PID.ActualValue=USV_Heading_PID.SetValue-err;
    PIDCalculation(&USV_Heading_PID);
    pwm2=USV_Heading_PID.OutPut+1500;
    //pwm2 = 0.5*(rocker.rightX-1500)+1500;

    if(sys_id==SYS_USV&&dev_id==0)//小黄船
        {
            if(USV_State.back)
            {
                pwm3 =1750;
            }
            else
            {
                pwm3=1250;
            }
        }
    else if (sys_id==SYS_USV&&dev_id>0) //小白船
        {
            pwm3 = 1700;
        }

    pwm1=pwm1<1000?1000:pwm1;
    pwm1=pwm1>2000?2000:pwm1;

    pwm2=pwm2<1000?1000:pwm2;
    pwm2=pwm2>2000?2000:pwm2;

    MotorPWMSet(pwm1,pwm2, pwm3);

}


