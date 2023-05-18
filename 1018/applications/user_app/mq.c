/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-11-21     PC-COLD       the first version
 */

#include <include.h>
//rt_mq_t uart1_mq,uart2_mq;
int  user_mq_init(void)
{
//    uart1_mq = rt_mq_create("uart1_mq", sizeof(rx_msg), 100, RT_IPC_FLAG_FIFO );
//    uart2_mq = rt_mq_create("uart2_mq", sizeof(rx_msg), 100, RT_IPC_FLAG_FIFO );
    return RT_EOK;
}

//INIT_PREV_EXPORT(user_mq_init);



