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
int user_app_init(void)
{
    user_nvic_init();
    user_sem_init();
    user_timer_init();
    param_read();
    USV_State_Init();
    USV_PID_Init();
    uart_init();
    return RT_EOK;
}
INIT_APP_EXPORT(user_app_init);


