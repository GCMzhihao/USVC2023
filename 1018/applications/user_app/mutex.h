/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-06-28     LZH       the first version
 */
#ifndef APPLICATIONS_USER_APP_MUTEX_H_
#define APPLICATIONS_USER_APP_MUTEX_H_
extern rt_mutex_t mutex_2ms ;
extern rt_mutex_t mutex_5ms ;
extern rt_mutex_t mutex_10ms;
extern rt_mutex_t mutex_20ms;
extern rt_mutex_t mutex_25ms;
extern rt_mutex_t mutex_50ms;
extern rt_mutex_t mutex_100ms;
extern rt_mutex_t mutex_200ms;
extern rt_mutex_t mutex_1000ms;

void user_mutex_init();
#endif /* APPLICATIONS_USER_APP_MUTEX_H_ */
