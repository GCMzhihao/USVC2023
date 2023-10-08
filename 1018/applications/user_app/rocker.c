/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-30     PC-COLD       the first version
 */
#include <include.h>


void RockerFlagCheck(void)
{
    if(rocker.switchD>=1500)
    {
        USV_State.back=1;
    }
    else if (rocker.switchD<=500)
    {
        USV_State.back=0;
    }



}

static void CheckLockState(void)
{

    if(USV_State.Unlock)//检测到是解锁状态
    {
        if(rocker.leftY==1000&&rocker.leftX==1000&&rocker.rightY==1000&&rocker.rightX==2000)//加锁。左摇杆打到左下，右摇杆打到右下（外八字）
        {
            USV_State.Unlock=0;//锁定
            BuzzerChangeState(beep_lock);
        }
    }
    else//检测到是锁定状态
    {
        if(rocker.rightX==1000&&rocker.rightY==1000&&rocker.leftY==1000&&rocker.leftX==2000)//解锁。左摇杆打到右下，右摇杆打到左下（内八字）
        {
            USV_State.Unlock=1;//解锁
            BuzzerChangeState(beep_unlock);
        }
    }

}


void RockerHandle(void)
{
    CheckLockState();
    RockerFlagCheck();

}

