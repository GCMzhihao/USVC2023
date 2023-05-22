/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-18     PC-COLD       the first version
 */
#ifndef APPLICATIONS_USER_APP_PARAMETER_H_
#define APPLICATIONS_USER_APP_PARAMETER_H_
typedef struct
{
   float kp;
   float ki;
   float kd;
}param_pid;
typedef struct
{
    param_pid inner;
    param_pid outer;
}param_cascade_pid;
typedef struct
{
    param_pid usv_speed_pid;
    param_pid usv_heading_pid;
    int16_t left_rudder_mid_value;
    int16_t right_rudder_mid_value;
}_parameters;

extern _parameters parameters;

void param_read(void);
void param_write(void);
#endif /* APPLICATIONS_USER_APP_PARAMETER_H_ */
