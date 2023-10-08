/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-08     PC-COLD       the first version
 */
#include <include.h>


void PIDCalculation(PID *pid)
{
    float hz,err;
    hz = 1.0f/pid->dt;
    err = pid->SetValue - pid->ActualValue;
    pid->Pout    =    pid->Kp * err ;
    if(pid->LPF_frq>0)
        _1st_lpf(err - pid->LastErr , &pid->dErr, pid->LPF_frq, pid->dt);
    else
        pid->dErr=err - pid->LastErr;
    pid->Dout=pid->dErr* hz * pid->Kd;
    pid->LastErr =  err ;
//    if((fabs(pid->Integral)<pid->IntMax||pid->Integral*err<0)||pid->IntMax==0)//防止积分饱和
//    {
//        pid->Integral += pid->Ki * err * pid->dt;
//    }
    pid->Integral += pid->Ki * err * pid->dt;
    if(pid->IntMax!=0)
        pid->Integral=LIMIT(pid->Integral,-pid->IntMax,pid->IntMax);
    pid->Iout = pid->Integral;
    if(pid->OutPutMax==0)
        pid->OutPut=(pid->Pout + pid->Iout + pid->Dout);
    else
        pid->OutPut =  LIMIT((pid->Pout + pid->Iout + pid->Dout),-pid->OutPutMax,pid->OutPutMax);
}

void PIDInit(PID *pid,float kp,float ki,float kd,float intmax,float outputmax,float frq)
{
    pid->Kp=kp;
    pid->Ki=ki;
    pid->Kd=kd;

    pid->IntMax=intmax;
    pid->Integral=0;
    pid->LPF_frq=frq;
    pid->OutPutMax=outputmax;
}








