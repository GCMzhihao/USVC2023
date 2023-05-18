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

_Attitude Attitude ;

void AttitudePIDInit(void)
{

/****************************************外环PID*************************************************************/
    PIDInit( &Attitude.Roll.OuterPID,  parameters.roll.outer.kp , parameters.roll.outer.ki , parameters.roll.outer.kd, 50 , 1000 ,0);  //Roll
    PIDInit( &Attitude.Pitch.OuterPID, parameters.pitch.outer.kp , parameters.pitch.outer.ki , parameters.pitch.outer.kd, 50 , 1000 ,0);  //Pitch
    PIDInit( &Attitude.Yaw.OuterPID,   parameters.yaw.outer.kp , parameters.yaw.outer.ki , parameters.yaw.outer.kd, 50 , 1000 ,0);  //Yaw
/************************************************************************************************************/

/****************************************内环PID*************************************************************/
    PIDInit( &Attitude.Roll.InnerPID,  parameters.roll.inner.kp , parameters.roll.inner.ki , parameters.roll.inner.kd, 50 , 1000,0 ); //Roll
    PIDInit( &Attitude.Pitch.InnerPID, parameters.pitch.inner.kp , parameters.pitch.inner.ki , parameters.pitch.inner.kd, 50 , 1000,0 );  //Pitch
    PIDInit( &Attitude.Yaw.InnerPID,   parameters.yaw.inner.kp , parameters.yaw.inner.ki , parameters.yaw.inner.kd, 0 , 1000,0 );  //Yaw
/************************************************************************************************************/

}






