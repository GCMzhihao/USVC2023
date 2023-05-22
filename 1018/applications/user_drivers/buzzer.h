/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-23     PC-COLD       the first version
 */
#ifndef APPLICATIONS_USER_DRIVERS_BUZZER_H_
#define APPLICATIONS_USER_DRIVERS_BUZZER_H_

typedef enum
{
  beep_off=0,
  beep_once_persec,
  beep_twice_persec,
  beep_third_persec,
  beep_help,
  beep_lock,
  beep_unlock,
  beep_rudder_offset
}_buzzer_state;

typedef struct
{
    _buzzer_state buzzer_state;
    uint32_t beep_time;
    uint8_t tick;//间隔时间，ms
}_buzzer;


void BuzzerRun(float dt);
void BuzzerChangeState(_buzzer_state state);
_buzzer_state GetBuzzerState(void);
#endif /* APPLICATIONS_USER_DRIVERS_BUZZER_H_ */
