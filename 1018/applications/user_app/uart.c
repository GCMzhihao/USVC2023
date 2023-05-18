/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-29     PC-COLD       the first version
 */
#include"include.h"

rt_device_t uart1;
rt_device_t uart2;
struct serial_configure uart1_cfg=RT_SERIAL_CONFIG_DEFAULT;
struct serial_configure uart2_config = RT_SERIAL_CONFIG_DEFAULT;

rt_err_t uart1_rx_callback(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(sem_uart1_rx);
    return RT_EOK;
}

rt_err_t uart2_rx_callback(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(sem_uart2_rx);
    return RT_EOK;
}

void uart_init(void)
{
    uart1 = rt_device_find("uart1");//P900
    uart1_cfg.baud_rate=230400;
    rt_device_control(uart1, RT_DEVICE_CTRL_CONFIG, &uart1_cfg);
    rt_device_open(uart1,RT_DEVICE_FLAG_DMA_RX);
    rt_device_set_rx_indicate(uart1,uart1_rx_callback);

    uart2 = rt_device_find("uart2");//UWB
    uart2_config.baud_rate = BAUD_RATE_921600;
    rt_device_control(uart2, RT_DEVICE_CTRL_CONFIG, &uart2_config);
    rt_device_open(uart2,RT_DEVICE_FLAG_DMA_RX);
    rt_device_set_rx_indicate(uart2,uart2_rx_callback);
}





