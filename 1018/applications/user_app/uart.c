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
rt_device_t uart3;
struct serial_configure uart1_cfg=RT_SERIAL_CONFIG_DEFAULT;
struct serial_configure uart2_config = RT_SERIAL_CONFIG_DEFAULT;
struct serial_configure uart3_config = RT_SERIAL_CONFIG_DEFAULT;

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
rt_err_t uart3_rx_callback(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(sem_uart3_rx);
    return RT_EOK;
}

void uart_init(void)
{
    uart1 = rt_device_find("uart1");//P900
    uart1_cfg.baud_rate=230400;
    rt_device_control(uart1, RT_DEVICE_CTRL_CONFIG, &uart1_cfg);
    rt_device_open(uart1,RT_DEVICE_FLAG_DMA_RX);
    rt_device_set_rx_indicate(uart1,uart1_rx_callback);

    uart2 = rt_device_find("uart2");//GPS
    uart2_config.baud_rate = BAUD_RATE_115200;
    rt_device_control(uart2, RT_DEVICE_CTRL_CONFIG, &uart2_config);
    rt_device_open(uart2,RT_DEVICE_FLAG_DMA_RX);
    rt_device_set_rx_indicate(uart2,uart2_rx_callback);

    uart3 = rt_device_find("uart3");//遥控器
    uart3_config.baud_rate = BAUD_RATE_100000;
    uart3_config.parity=PARITY_EVEN;
    uart3_config.data_bits=DATA_BITS_9;
    uart3_config.stop_bits=STOP_BITS_2;
    rt_device_control(uart3, RT_DEVICE_CTRL_CONFIG, &uart3_config);
    rt_device_open(uart3,RT_DEVICE_FLAG_DMA_RX);
    rt_device_set_rx_indicate(uart3,uart3_rx_callback);
}

void uart2_analysis(uint8_t c)
{
    static uint8_t rx_state=0;//接收状态机
    static uint16_t cnt=0;
    static uint8_t buf[200];
    switch(rx_state)
    {
    case 0:
        if(c=='$'||c=='#')
        {
            cnt=0;
            rx_state=1;
            rt_memset(buf, 0, 200);
            buf[cnt++]=c;
        }
        break;
    case 1:
        buf[cnt++]=c;
        if(c=='\r')
        {
            rx_state=2;
        }
        if(cnt>=200)
            rx_state=0;
        break;
    case 2:
        rx_state=0;
        if(c=='\n')
        {
            buf[cnt++]=c;
            NMEA0183_Analysis(buf);
        }

        break;
    }
}



