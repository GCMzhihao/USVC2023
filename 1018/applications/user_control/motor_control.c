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

        if(sys_id==SYS_USV&&dev_id==0)//小黄船
            MotorPWMSet(1000, 1500, 1400);
        else if(sys_id==SYS_USV&&dev_id>0)//小白船
            MotorPWMSet(1000, 1480, 1500);
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






