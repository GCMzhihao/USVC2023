/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-15     PC-COLD       the first version
 */
#ifndef APPLICATIONS_USER_SEM_H_
#define APPLICATIONS_USER_SEM_H_

extern rt_sem_t sem_1ms;
extern rt_sem_t sem_2ms;
extern rt_sem_t sem_5ms;
extern rt_sem_t sem_10ms;
extern rt_sem_t sem_20ms;
extern rt_sem_t sem_50ms;
extern rt_sem_t sem_100ms;
extern rt_sem_t sem_200ms;
extern rt_sem_t sem_1000ms;
extern rt_sem_t sem_uart1_rx;
extern rt_sem_t sem_uart1_tx;
extern rt_sem_t sem_uart2_rx;
extern rt_sem_t sem_uart3_rx;
void user_sem_init(void);

#endif /* APPLICATIONS_USER_SEM_H_ */
