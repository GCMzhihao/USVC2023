/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-23     PC-COLD       the first version
 */
#include <include.h>

#define ADC1_DEV_NAME "adc1"

static rt_adc_device_t adc1_dev;


static int adc1_init (void)
{
    adc1_dev=(rt_adc_device_t)rt_device_find(ADC1_DEV_NAME);
    if (adc1_dev == RT_NULL)
    {
        rt_kprintf("adc init failed! can't find %s device!\n", ADC1_DEV_NAME);
        return RT_ERROR;
    }

    return RT_EOK;
}

INIT_DEVICE_EXPORT(adc1_init);

void voltage_measure(void)
{
    float tmp;
    rt_adc_enable(adc1_dev, 8);
    tmp=rt_adc_read(adc1_dev, 8);
    rt_adc_disable(adc1_dev, 8);
    USV_State.BatteryVoltage=tmp*3.28/4096*11;

    if(USV_State.BatteryVoltage<22.0f)
    {
        //USV_State.LowPower=1;
        BuzzerChangeState(beep_help);
    }
}

