/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-03-20     PC-COLD      the first version
 */
#include <include.h>

_Position Position;
float Height = 0;//离地高度

#define POINT_NUM 10

VectorFloat StartPoint;
VectorFloat StartPointTemp[POINT_NUM];

#define TIME_CONTANST_X 1.0f
#define K_ACC_X (1.0f / (TIME_CONTANST_X * TIME_CONTANST_X * TIME_CONTANST_X))
#define K_VEL_X (3.0f / (TIME_CONTANST_X * TIME_CONTANST_X))
#define K_POS_X (3.0f / TIME_CONTANST_X)

#define TIME_CONTANST_Y 1.0f
#define K_ACC_Y (1.0f / (TIME_CONTANST_Y * TIME_CONTANST_Y * TIME_CONTANST_Y))
#define K_VEL_Y (3.0f / (TIME_CONTANST_Y * TIME_CONTANST_Y))
#define K_POS_Y (3.0f / TIME_CONTANST_Y)

#define TIME_CONTANST_Z 1.5f
#define K_ACC_Z (1.0f / (TIME_CONTANST_Z * TIME_CONTANST_Z * TIME_CONTANST_Z))
#define K_VEL_Z (3.0f / (TIME_CONTANST_Z * TIME_CONTANST_Z))
#define K_POS_Z (3.0f / TIME_CONTANST_Z)


static float X_Dealt,Y_Dealt,Z_Dealt;
static float X_acc_correction,X_vel_correction,X_pos_correction,Y_acc_correction,Y_vel_correction,Y_pos_correction,Z_acc_correction,Z_vel_correction,Z_pos_correction;
static float X_last_Acceleration,Y_last_Acceleration,Z_last_Acceleration;
static float X_Acceleration,Y_Acceleration,Z_Acceleration;
static float X_SpeedDealt,Y_SpeedDealt,Z_SpeedDealt;
static float X_Origion_Position,Y_Origion_Position,Z_Origion_Position;
static float X_Origion_Speed,Y_Origion_Speed,Z_Origion_Speed;

void PositionUpdate(float dt)
{
    if(isnan(UWB_data.uwb_pos.x) || isnan(UWB_data.uwb_pos.y) ||
         isnan(UWB_data.uwb_pos.z))
        return;

    Position.x.Displacement.Observe=UWB_data.uwb_pos.x;
    Position.y.Displacement.Observe=UWB_data.uwb_pos.y;
    Position.z.Displacement.Observe=UWB_data.uwb_pos.z;

    //标签输出的速度滞后严重,不能使用

    //x
    Position.x.Acceleration.Measure=sensor.uwb_acc.x;
    X_Dealt=Position.x.Displacement.Observe-Position.x.Displacement.Estimate;
    X_acc_correction += dt*X_Dealt* K_ACC_X ;//加速度校正量
    X_vel_correction += dt*X_Dealt* K_VEL_X ;//速度校正量
    X_pos_correction += dt*X_Dealt* K_POS_X ;//位置校正量

    //加速度计校正后更新
    X_last_Acceleration=X_Acceleration;
    X_Acceleration=Position.x.Acceleration.Measure+X_acc_correction;//
    X_SpeedDealt=(X_last_Acceleration+X_Acceleration)*dt/2.0;//X_last_Acceleration*dt
    //原始位置更新
    X_Origion_Position+=(Position.x.Speed.Estimate+0.5*X_SpeedDealt)*dt;
    //位置校正后更新
    Position.x.Displacement.Estimate=X_Origion_Position+X_pos_correction;
    //原始速度更新
    X_Origion_Speed+=X_SpeedDealt;
    //速度校正后更新
    Position.x.Speed.Estimate=X_Origion_Speed+X_vel_correction;

    //y
    Position.y.Acceleration.Measure=sensor.uwb_acc.y;
    Y_Dealt=Position.y.Displacement.Observe-Position.y.Displacement.Estimate;
    Y_acc_correction += dt*Y_Dealt* K_ACC_Y ;//加速度校正量
    Y_vel_correction += dt*Y_Dealt* K_VEL_Y ;//速度校正量
    Y_pos_correction += dt*Y_Dealt* K_POS_Y ;//位置校正量

    //加速度计校正后更新
    Y_last_Acceleration=Y_Acceleration;
    Y_Acceleration=Position.y.Acceleration.Measure+Y_acc_correction;
    Y_SpeedDealt=(Y_last_Acceleration+Y_Acceleration)*dt/2.0;
    //原始位置更新
    Y_Origion_Position+=(Position.y.Speed.Estimate+0.5*Y_SpeedDealt)*dt;
    //位置校正后更新
    Position.y.Displacement.Estimate=Y_Origion_Position+Y_pos_correction;
    //原始速度更新
    Y_Origion_Speed+=Y_SpeedDealt;
    //速度校正后更新
    Position.y.Speed.Estimate=Y_Origion_Speed+Y_vel_correction;

    //z
    Position.z.Acceleration.Measure=sensor.uwb_acc.z;
    Z_Dealt=Position.z.Displacement.Observe-Position.z.Displacement.Estimate;
    Z_acc_correction += dt*Z_Dealt* K_ACC_Z ;//加速度校正量
    Z_vel_correction += dt*Z_Dealt* K_VEL_Z ;//速度校正量
    Z_pos_correction += dt*Z_Dealt* K_POS_Z ;//位置校正量

    //加速度计校正后更新
    Z_last_Acceleration=Z_Acceleration;
    Z_Acceleration=Position.z.Acceleration.Measure+Z_acc_correction;
    Z_SpeedDealt=(Z_last_Acceleration+Z_Acceleration)*dt/2.0;//=a*t
    //原始位置更新
    Z_Origion_Position+=(Position.z.Speed.Estimate+0.5*Z_SpeedDealt)*dt;//+=v0*t+0.5*a*t*t
    //位置校正后更新
    Position.z.Displacement.Estimate=Z_Origion_Position+Z_pos_correction;
    //原始速度更新
    Z_Origion_Speed+=Z_SpeedDealt;
    //速度校正后更新
    Position.z.Speed.Estimate=Z_Origion_Speed+Z_vel_correction;

    USV_State.X=Position.x.Displacement.Estimate;
    USV_State.Y=Position.y.Displacement.Estimate;
    USV_State.Speed=sqrt(Position.x.Speed.Estimate*Position.x.Speed.Estimate+Position.y.Speed.Estimate*Position.y.Speed.Estimate);
    USV_State.Course=atan2f(Position.y.Speed.Estimate,Position.x.Speed.Estimate)*180/3.1415926f;
    USV_State.Heading=Attitude.Yaw.Value;
}

