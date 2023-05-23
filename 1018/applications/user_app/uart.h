/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-29     PC-COLD       the first version
 */
#ifndef APPLICATIONS_USER_DRIVER_UART_H_
#define APPLICATIONS_USER_DRIVER_UART_H_

extern rt_device_t uart1;
extern rt_device_t uart2;
void uart_init(void);
void uart2_analysis(uint8_t c);


#endif /* APPLICATIONS_USER_DRIVER_UART_H_ */
