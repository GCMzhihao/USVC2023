/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-15     PC-COLD       the first version
 */
#include <include.h>

static rt_timer_t timer_20ms = RT_NULL;
static rt_timer_t timer_50ms = RT_NULL;
static rt_timer_t timer_100ms = RT_NULL;
static rt_timer_t timer_200ms = RT_NULL;
static rt_timer_t timer_1000ms = RT_NULL;

void timer_20ms_handle(void* parameter)
{
    //rt_sem_control(sem_20ms, RT_IPC_CMD_RESET, 0);//清零信号量
    rt_sem_release(sem_20ms);
}

void timer_50ms_handle(void* parameter)
{
   // rt_sem_control(sem_50ms, RT_IPC_CMD_RESET, 0);//清零信号量
    rt_sem_release(sem_50ms);
}

void timer_100ms_handle(void* parameter)
{
   // rt_sem_control(sem_100ms, RT_IPC_CMD_RESET, 0);//清零信号量
    rt_sem_release(sem_100ms);
}
void timer_200ms_handle(void* parameter)
{
   // rt_sem_control(sem_200ms, RT_IPC_CMD_RESET, 0);//清零信号量
    rt_sem_release(sem_200ms);
}
void timer_1000ms_handle(void* parameter)
{
//    rt_sem_control(sem_1000ms, RT_IPC_CMD_RESET, 0);//清零信号量
    rt_sem_release(sem_1000ms);
}
rt_err_t user_timer_init(void)
{
    /* 创建定时器 */
    timer_20ms = rt_timer_create("timer20ms",
                               timer_20ms_handle,
                               RT_NULL,
                               20,
                               RT_TIMER_FLAG_PERIODIC);
    if (timer_20ms != RT_NULL) rt_timer_start(timer_20ms);/* 启动定时器 */


    /* 创建定时器 */
    timer_50ms = rt_timer_create("timer50ms",
                               timer_50ms_handle,
                               RT_NULL,
                               50,
                               RT_TIMER_FLAG_PERIODIC);
    if (timer_50ms != RT_NULL) rt_timer_start(timer_50ms);/* 启动定时器 */

    /* 创建定时器 */
    timer_100ms = rt_timer_create("timer100ms",
                               timer_100ms_handle,
                               RT_NULL,
                               100,
                               RT_TIMER_FLAG_PERIODIC);
    if (timer_100ms != RT_NULL) rt_timer_start(timer_100ms);/* 启动定时器 */

    /* 创建定时器 */
    timer_200ms = rt_timer_create("timer200ms",
                               timer_200ms_handle,
                               RT_NULL,
                               200,
                               RT_TIMER_FLAG_PERIODIC);
    if (timer_200ms != RT_NULL) rt_timer_start(timer_200ms);/* 启动定时器 */

    /* 创建定时器 */
    timer_1000ms = rt_timer_create("timer1000ms",
                               timer_1000ms_handle,
                               RT_NULL,
                               1000,
                               RT_TIMER_FLAG_PERIODIC);
    if (timer_1000ms != RT_NULL) rt_timer_start(timer_1000ms);/* 启动定时器 */

    return RT_EOK;
}
