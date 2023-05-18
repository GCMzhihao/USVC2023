/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-15     PC-COLD       the first version
 */
#ifndef APPLICATIONS_USER_TASK_H_
#define APPLICATIONS_USER_TASK_H_



#define TASK_2MS_PRIORITY               11
#define TASK_2MS_STACK_SIZE             3000
#define TASK_2MS_TIMESLICE              5

#define TASK_5MS_PRIORITY               12
#define TASK_5MS_STACK_SIZE             3000
#define TASK_5MS_TIMESLICE              5

#define TASK_10MS_PRIORITY              13
#define TASK_10MS_STACK_SIZE            3000
#define TASK_10MS_TIMESLICE             5

#define TASK_20MS_PRIORITY              14
#define TASK_20MS_STACK_SIZE            3000
#define TASK_20MS_TIMESLICE             5

#define TASK_50MS_PRIORITY              15
#define TASK_50MS_STACK_SIZE            3000
#define TASK_50MS_TIMESLICE             5

#define TASK_100MS_PRIORITY             16
#define TASK_100MS_STACK_SIZE           3000
#define TASK_100MS_TIMESLICE            5

#define TASK_200MS_PRIORITY             17
#define TASK_200MS_STACK_SIZE           3000
#define TASK_200MS_TIMESLICE            5

#define TASK_1000MS_PRIORITY            18
#define TASK_1000MS_STACK_SIZE          3000
#define TASK_1000MS_TIMESLICE           5

#define TASK_UART1_PRIORITY             23
#define TASK_UART1_STACK_SIZE           3000
#define TASK_UART1_TIMESLICE            10

#define TASK_UART2_PRIORITY             23
#define TASK_UART2_STACK_SIZE           3000
#define TASK_UART2_TIMESLICE            10

rt_err_t user_task_init(void);


#endif /* APPLICATIONS_USER_TASK_H_ */
