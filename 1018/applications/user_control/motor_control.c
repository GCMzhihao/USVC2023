/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-30     PC-COLD       the first version
 */
#include <include.h>

void MotorControl(float dt)
{
    if(USV_State.Unlock==0)//未解锁
    {

        if(sys_id==SYS_USV&&dev_id==0)//大黄船
            MotorPWMSet(1000, parameters.left_rudder_mid_value,parameters.right_rudder_mid_value,1250);
        else if(sys_id==SYS_USV&&dev_id>0&&dev_id<10)//小白船
            MotorPWMSet(1000,parameters.left_rudder_mid_value,parameters.right_rudder_mid_value,1000);
        if(sys_id==SYS_USV&&dev_id>10)//大黄船
            MotorPWMSet(1000, parameters.left_rudder_mid_value,parameters.right_rudder_mid_value,1250);
        USV_Speed_PID.Integral=0;
        USV_Heading_PID.Integral=0;
        return;
    }
     if(USV_State.Unlock==1)//解锁
    {
        USV_Back_Check();
        RockerControl();
        CommandControl(dt);
        return;
    }
}






