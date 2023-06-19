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

#define BUZZER_PIN_NUM 21

static _buzzer buzzer;


static int buzzer_init(void)
{
    rt_pin_mode(BUZZER_PIN_NUM, PIN_MODE_OUTPUT);
    rt_pin_write(BUZZER_PIN_NUM, PIN_LOW);
    return RT_EOK;
}

INIT_DEVICE_EXPORT(buzzer_init);

static void BuzzerOn(void)
{
    rt_pin_write(BUZZER_PIN_NUM, PIN_HIGH);
}

static void BuzzerOff(void)
{
    rt_pin_write(BUZZER_PIN_NUM, PIN_LOW);
}

void Buzzer(float dt)
{
    if(buzzer.tick<0.1/dt)
        return;

}



void BuzzerRun(float dt)
{
    uint16_t tick;
    static uint32_t time;
    static uint8_t unlock_state=0,lock_state=0;



    switch (buzzer.buzzer_state)
    {
    case beep_off:
        BuzzerOff();
        break;
    case beep_once_persec:
        tick=1/dt;
        if(time%tick==0)
            BuzzerOn();
        else
            BuzzerOff();
        break;
    case beep_twice_persec:
        tick=0.5/dt;
        if(time%tick==0)
            BuzzerOn();
        else if(time%tick==1)
            BuzzerOff();
        else if(time%tick==2)
            BuzzerOn();
        else
            BuzzerOff();
        break;
    case beep_third_persec:
        tick=0.333/dt;
        if(time%tick==0)
            BuzzerOn();
        else if(time%tick==1)
            BuzzerOff();
        else if(time%tick==2)
            BuzzerOn();
        else if(time%tick==3)
            BuzzerOff();
        else if(time%tick==4)
            BuzzerOn();
        else
            BuzzerOff();
        break;
    case beep_help:
        if(time%20==0)
            {BuzzerOn();
            buzzer.buzzer_state=beep_off;}
        else
            BuzzerOff();
        break;
    case beep_lock:
        unlock_state=0;
        if(lock_state==0)
        {
            BuzzerOn();
            lock_state=1;
        }
        else if(lock_state==1)
        {
            BuzzerOn();
            lock_state=2;
        }
        else if(lock_state==2)
        {
            BuzzerOff();
            buzzer.buzzer_state=beep_off;
            lock_state=0;
        }
        break;
    case beep_unlock:
        lock_state=0;
        if(unlock_state==0)
        {
            BuzzerOn();
            unlock_state=1;
        }
        else if(unlock_state==1)
        {
            BuzzerOff();
            unlock_state=2;
        }
        else if(unlock_state==2)
        {
            BuzzerOn();
            unlock_state=3;
        }
        else if(unlock_state==3)
        {
            BuzzerOff();
            buzzer.buzzer_state=beep_off;
            unlock_state=0;
        }
        break;
    case beep_rudder_offset:

        break;
    default:
        BuzzerOff();
        break;
    }
    time++;
}

void BuzzerChangeState(_buzzer_state state)
{
    buzzer.buzzer_state=state;
}

_buzzer_state GetBuzzerState(void)
{
    return buzzer.buzzer_state;
}

