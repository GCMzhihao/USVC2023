/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-19     PC-COLD       the first version
 */
#ifndef APPLICATIONS_USER_CONTROL_CONTROL_H_
#define APPLICATIONS_USER_CONTROL_CONTROL_H_

#define MAX_VOLTAGE 2000
#define MIN_VOLTAGE 1000

#define SPEED_CMD 0
#define POSITION_CMD 1


typedef struct
{
    uint8_t SysInitOk:1;
    uint8_t Unlock:1;
    uint8_t LowPower:1;//低电量
    uint8_t RockerIsReady:1;//摇杆接收到数据
    uint8_t AutoSail:1;//自动航行标志
    uint8_t LeftRudderOk:1;//左舵机中位校准标志
    uint8_t RightRudderOk:1;//右舵机中位校准标志
    uint8_t Id;
    float X;
    float Y;
    float Speed;
    float Course;
    float Heading;
    float BatteryVoltage;
}_USV_State;
typedef struct
{
    float Speed;
    float Heading;
}_USV_SET;
extern PID USV_Speed_PID;
extern PID USV_Heading_PID;
extern _USV_State USV_State;
extern _USV_SET USV_SET;

int USV_State_Init(void);
void RockerControl(void);
void CommandControl(float dt);
void USV_PID_Init(void);
#endif /* APPLICATIONS_USER_CONTROL_CONTROL_H_ */
