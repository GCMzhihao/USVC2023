/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-09-06     D       the first version
 */
#include <include.h>

static int pin_init(void)
{
    rt_pin_mode(BIT0, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(BIT1, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(BIT2, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(BIT3, PIN_MODE_INPUT_PULLUP);
    rt_pin_mode(BIT4, PIN_MODE_INPUT_PULLUP);

    return RT_EOK;
}
INIT_DEVICE_EXPORT(pin_init);

//上电获取dev_id
int get_dev_id(void)
{
    dev_id = (rt_pin_read(BIT0)<<0)|(rt_pin_read(BIT1)<<1)|(rt_pin_read(BIT2)<<2)
             |(rt_pin_read(BIT3)<<3);
    dev_id = dev_id^0x0F;//低五位取反
    return RT_EOK;
}
INIT_APP_EXPORT(get_dev_id);

int get_sys_id(void)
{
    sys_id = rt_pin_read(BIT4);
    sys_id^=0x01;
    sys_id+=1;
    return RT_EOK;
}
INIT_APP_EXPORT(get_sys_id);
