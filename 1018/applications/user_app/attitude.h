/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-08     PC-COLD       the first version
 */
#ifndef APPLICATIONS_USER_APP_ATTITUDE_H_
#define APPLICATIONS_USER_APP_ATTITUDE_H_

#include "PID.h"

typedef struct
{
    float RawValue;
    float Bias;
    float Value;
    float Deviation;//偏差
    PID OuterPID;
    PID InnerPID;
}EulerAngle;

typedef struct
{
    EulerAngle Roll;
    EulerAngle Pitch;
    EulerAngle Yaw;
}_Attitude;

extern _Attitude Attitude ;

void AttitudePIDInit(void);


#endif /* APPLICATIONS_USER_APP_ATTITUDE_H_ */
