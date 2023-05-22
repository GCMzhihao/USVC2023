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

static rt_thread_t task_2ms = RT_NULL;
static rt_thread_t task_5ms = RT_NULL;
static rt_thread_t task_10ms = RT_NULL;
static rt_thread_t task_20ms = RT_NULL;
static rt_thread_t task_50ms = RT_NULL;
static rt_thread_t task_100ms = RT_NULL;
static rt_thread_t task_200ms = RT_NULL;
static rt_thread_t task_1000ms = RT_NULL;
static rt_thread_t task_uart1 = RT_NULL;
static rt_thread_t task_uart2 = RT_NULL;

void task_2ms_entry(void* parameter)
{
    float dt = (uint32_t)parameter*0.001f;
    while(1)
    {
        rt_sem_take(sem_2ms, RT_WAITING_FOREVER);
    }

}

void task_5ms_entry(void* parameter)
{
//    float dt = (uint32_t)parameter*0.001f;
    while(1)
    {
        rt_sem_take(sem_5ms, RT_WAITING_FOREVER);
    }
}

void task_10ms_entry(void* parameter)
{
    float dt = (uint32_t)parameter*0.001f;
    while(1)
    {
        rt_sem_take(sem_10ms, RT_WAITING_FOREVER);
    }
}

void task_20ms_entry(void* parameter)
{
    float dt = (uint32_t)parameter*0.001f;
    while(1)
    {
        rt_sem_take(sem_20ms, RT_WAITING_FOREVER);
        RockerHandle();
        voltage_measure();

    }
}

void task_50ms_entry(void* parameter)
{
//    float dt = (uint32_t)parameter*0.001f;
    while(1)
    {
        rt_sem_take(sem_50ms, RT_WAITING_FOREVER);
        led_run();
        mavlink_msg_send();
    }
}

void task_100ms_entry(void* parameter)
{
    float dt = (uint32_t)parameter*0.001f;
    while(1)
    {
        rt_sem_take(sem_100ms, RT_WAITING_FOREVER);
        BuzzerRun(dt);
        MotorControl(dt);

    }
}

void task_200ms_entry(void* parameter)
{
//    float dt = (uint32_t)parameter*0.001f;
    while(1)
    {
        rt_sem_take(sem_200ms, RT_WAITING_FOREVER);

    }
}

void task_1000ms_entry(void* parameter)
{
//    float dt = (uint32_t)parameter*0.001f;
    while(1)
    {
        rt_sem_take(sem_1000ms, RT_WAITING_FOREVER);
    }
}

void task_uart1_entry(void* parameter)//P900
{
    uint8_t c;
    mavlink_message_t mav_msg;
    mavlink_status_t mav_status;
    while(1)
    {
        while(rt_device_read(uart1, 0, &c, 1)!=1)
        {
            rt_sem_take(sem_uart1_rx, RT_WAITING_FOREVER);
        }
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &mav_msg, &mav_status))
        {
            mavlink_msg_proxy(&mav_msg,&mav_status);
        }
    }
}

void task_uart2_entry(void* parameter)//UWB
{
    uint8_t ch;
    while (1)
    {
        /* 从串口读取一个字节的数据，没有读取到则等待接收信号量 */
        while (rt_device_read(uart2, -1, &ch, 1) != 1)
        {
            /* 阻塞等待接收信号量，等到信号量后再次读取数据 */
            rt_sem_take(sem_uart2_rx, RT_WAITING_FOREVER);
        }
    }
}

rt_err_t user_task_init(void)
{
    /* 创建线程 */
    task_2ms = rt_thread_create("task2ms",
                                task_2ms_entry,
                                (void *)2,
                                TASK_2MS_STACK_SIZE,
                                TASK_2MS_PRIORITY,
                                TASK_2MS_TIMESLICE);
    if(task_2ms!=RT_NULL)/* 启动线程 */
        rt_thread_startup(task_2ms);

    /* 创建线程 */
    task_5ms = rt_thread_create("task5ms",
                                task_5ms_entry,
                                (void *)5,
                                TASK_5MS_STACK_SIZE,
                                TASK_5MS_PRIORITY,
                                TASK_5MS_TIMESLICE);
    if(task_5ms!=RT_NULL)/* 启动线程 */
        rt_thread_startup(task_5ms);

    /* 创建线程 */
    task_10ms = rt_thread_create("task10ms",
                                task_10ms_entry,
                                (void *)10,
                                TASK_10MS_STACK_SIZE,
                                TASK_10MS_PRIORITY,
                                TASK_10MS_TIMESLICE);
    if(task_10ms!=RT_NULL)/* 启动线程 */
        rt_thread_startup(task_10ms);

    /* 创建线程 */
    task_20ms = rt_thread_create("task20ms",
                                task_20ms_entry,
                                (void *)20,
                                TASK_20MS_STACK_SIZE,
                                TASK_20MS_PRIORITY,
                                TASK_20MS_TIMESLICE);

    if(task_20ms!=RT_NULL)/* 启动线程 */
       rt_thread_startup(task_20ms);

    /* 创建线程 */
    task_50ms = rt_thread_create("task50ms",
                                task_50ms_entry,
                                (void *)50,
                                TASK_50MS_STACK_SIZE,
                                TASK_50MS_PRIORITY,
                                TASK_50MS_TIMESLICE);
    if(task_50ms!=RT_NULL)/* 启动线程 */
        rt_thread_startup(task_50ms);
    /* 创建线程 */
    task_100ms = rt_thread_create("task100ms",
                                    task_100ms_entry,
                                    (void *)100,
                                    TASK_100MS_STACK_SIZE,
                                    TASK_100MS_PRIORITY,
                                    TASK_100MS_TIMESLICE);
    if(task_100ms!=RT_NULL)/* 启动线程 */
        rt_thread_startup(task_100ms);

    /* 创建线程 */
    task_200ms = rt_thread_create("task200ms",
                                    task_200ms_entry,
                                    (void *)200,
                                    TASK_200MS_STACK_SIZE,
                                    TASK_200MS_PRIORITY,
                                    TASK_200MS_TIMESLICE);
    if(task_200ms!=RT_NULL)/* 启动线程 */
        rt_thread_startup(task_200ms);

    /* 创建线程 */
    task_1000ms = rt_thread_create("task1000ms",
                                    task_1000ms_entry,
                                    (void *)1000,
                                    TASK_1000MS_STACK_SIZE,
                                    TASK_1000MS_PRIORITY,
                                    TASK_1000MS_TIMESLICE);
    if(task_1000ms!=RT_NULL)/* 启动线程 */
        rt_thread_startup(task_1000ms);
    /* 创建线程 */
    task_uart1 = rt_thread_create("task_uart1",
                                    task_uart1_entry,
                                    (void *)0,
                                    TASK_UART1_STACK_SIZE,
                                    TASK_UART1_PRIORITY,
                                    TASK_UART1_TIMESLICE);
    if(task_uart1!=RT_NULL)/* 启动线程 */
        rt_thread_startup(task_uart1);
    /* 创建线程 */
    task_uart2 = rt_thread_create("task_uart2",
                                    task_uart2_entry,
                                    (void *)0,
                                    TASK_UART2_STACK_SIZE,
                                    TASK_UART2_PRIORITY,
                                    TASK_UART2_TIMESLICE);
    if(task_uart2!=RT_NULL)/* 启动线程 */
        rt_thread_startup(task_uart2);

    return RT_EOK;
}
