/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-06-28     LZH       the first version
 */
#include<include.h>

rt_mutex_t mutex_2ms =RT_NULL;
rt_mutex_t mutex_5ms =RT_NULL;
rt_mutex_t mutex_10ms =RT_NULL;
rt_mutex_t mutex_20ms =RT_NULL;
rt_mutex_t mutex_25ms =RT_NULL;
rt_mutex_t mutex_50ms =RT_NULL;
rt_mutex_t mutex_100ms =RT_NULL;
rt_mutex_t mutex_200ms =RT_NULL;
rt_mutex_t mutex_1000ms =RT_NULL;

void user_mutex_init()
{
    mutex_2ms = rt_mutex_create("mutex_2ms",  RT_IPC_FLAG_FIFO);
    mutex_5ms = rt_mutex_create("mutex_5ms", RT_IPC_FLAG_FIFO);
    mutex_10ms = rt_mutex_create("mutex_10ms",RT_IPC_FLAG_FIFO);
    mutex_20ms = rt_mutex_create("mutex_20ms", RT_IPC_FLAG_FIFO);
    mutex_50ms = rt_mutex_create("mutex_50ms",  RT_IPC_FLAG_FIFO);
    mutex_100ms = rt_mutex_create("mutex_100ms", RT_IPC_FLAG_FIFO);
    mutex_200ms = rt_mutex_create("mutex_200ms", RT_IPC_FLAG_FIFO);
    mutex_1000ms = rt_mutex_create("mutex_1000ms",RT_IPC_FLAG_FIFO);
}
