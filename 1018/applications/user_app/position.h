/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-03-20     PC-COLD       the first version
 */
#ifndef APPLICATIONS_USER_APP_POSITION_H_
#define APPLICATIONS_USER_APP_POSITION_H_

typedef struct
{
    float Measure;
    float Observe;
    float Estimate;
}State;

typedef struct
{
    State Displacement;//位移
    State Speed;         //速度
    State Acceleration;//加速度
    PID OuterPID;
    PID InnerPID;
    PID Displacement_Est_PID;//位移估计
    PID Speed_Est_PID;//速度估计
    PID Acceleration_Est_PID;
}Motion;

typedef struct
{
    Motion x;
    Motion y;
    Motion z;
}_Position;

extern _Position Position;
extern float Height;//离地高度

extern void PositionUpdate(float dt);
void PositionPIDInit(void);
void KalmanFilter(float Positional,float *Position,float *Vel,float *Acce,float *R,float Q,float dt);
#endif /* APPLICATIONS_USER_APP_POSITION_H_ */
