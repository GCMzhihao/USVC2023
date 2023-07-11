/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-15     PC-COLD       the first version
 */

#include<include.h>

rt_sem_t sem_20ms =RT_NULL;
rt_sem_t sem_25ms =RT_NULL;
rt_sem_t sem_50ms =RT_NULL;
rt_sem_t sem_100ms =RT_NULL;
rt_sem_t sem_200ms =RT_NULL;
rt_sem_t sem_1000ms =RT_NULL;
rt_sem_t sem_uart1_rx =RT_NULL;
rt_sem_t sem_uart1_tx = RT_NULL;
rt_sem_t sem_uart2_rx =RT_NULL;
rt_sem_t sem_uart3_rx =RT_NULL;
void user_sem_init(void)
{
    sem_20ms = rt_sem_create("sem_20ms", 0, RT_IPC_FLAG_FIFO);
    sem_50ms = rt_sem_create("sem_50ms",  0, RT_IPC_FLAG_FIFO);
    sem_100ms = rt_sem_create("sem_100ms",0,  RT_IPC_FLAG_FIFO);
    sem_200ms = rt_sem_create("sem_200ms",0,  RT_IPC_FLAG_FIFO);
    sem_1000ms = rt_sem_create("sem_1000ms",0, RT_IPC_FLAG_FIFO);
    sem_uart1_rx = rt_sem_create("sem_uart1_rx", 0, RT_IPC_FLAG_FIFO);
    sem_uart1_tx = rt_sem_create("sem_uart1_tx", 0, RT_IPC_FLAG_FIFO);
    rt_sem_release(sem_uart1_tx);//释放uart1发送
    sem_uart2_rx = rt_sem_create("sem_uart2_rx", 0, RT_IPC_FLAG_FIFO);
    sem_uart3_rx = rt_sem_create("sem_uart3_rx", 0, RT_IPC_FLAG_FIFO);
}


