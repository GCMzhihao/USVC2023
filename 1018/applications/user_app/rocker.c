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


static void CheckLockState(void)
{
    USV_State.Id=rocker.switchA/500-1;
    if(dev_id!=USV_State.Id)
        return;
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
        if(rocker.leftY==1000&&rocker.leftX==2000&&rocker.rightY==1000&&rocker.rightX==1000)//解锁。左摇杆打到右下，右摇杆打到左下（内八字）
        {
            USV_State.Unlock=1;//解锁
            BuzzerChangeState(beep_unlock);
        }
    }

}
static void SensorCalibration(void)
{
    USV_State.Id=rocker.switchA/500-1;
    if(dev_id!=USV_State.Id)
        return;
    if(USV_State.Unlock)//检测到是解锁状态
        return ;
    if(!USV_State.SysOffsetOk)//检查到有正在校准的状态
        return ;
    if(rocker.leftY==1000&&rocker.leftX==1000&&rocker.rightY==2000&&rocker.rightX==1000)//加速度计校准。左摇杆打到左下，右摇杆打到左上
    {
        USV_State.HorizOffsetOk=0;
        USV_State.SysOffsetOk=0;
    }
    else if(rocker.leftY==1000&&rocker.leftX==1000&&rocker.rightY==2000&&rocker.rightX==2000)//陀螺仪校准。左摇杆打到左下，右摇杆打到右上
    {
        USV_State.GyroOffsetOk=0;
        USV_State.SysOffsetOk=0;
    }
    else if(rocker.leftY==1000&&rocker.leftX==2000&&rocker.rightY==2000&&rocker.rightX==1000)//磁力计校准。左摇杆打到右下，右摇杆打到左上
    {
        USV_State.MagOffsetOk=0;
        USV_State.SysOffsetOk=0;
    }
    else if(rocker.leftY==1000&&rocker.leftX==2000&&rocker.rightY==2000&&rocker.rightX==2000)//气压计校准。左摇杆打到右下，右摇杆打到右上
    {
        USV_State.AccOffsetOk=0;
        USV_State.SysOffsetOk=0;
    }
    else if(rocker.leftY==1000&&rocker.leftX==1000&&rocker.rightY==1000&&rocker.rightX==1000)//UWB偏航角校准。左摇杆打到左下，右摇杆打到左下
    {
        USV_State.YawOffsetOk=0;
        USV_State.SysOffsetOk=0;
    }
    else if(rocker.leftY==2000&&&&rocker.leftX==1000&&rocker.rightY==2000&&rocker.rightX==1000)//左舵机中位校准，左摇杆左上，右摇杆左上
    {
        USV_State.LeftRudderOk=0;
        USV_State.SysOffsetOk=0;
    }
    else if(rocker.leftY==2000&&&&rocker.leftX==2000&&rocker.rightY==2000&&rocker.rightX==2000)//左舵机中位校准，左摇杆左上，右摇杆左上
    {
        USV_State.RightRudderOk=0;
        USV_State.SysOffsetOk=0;
    }
}

void RockerHandle(void)
{
    CheckLockState();
    SensorCalibration();
}

