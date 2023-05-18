/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-20     PC-COLD       the first version
 */
#ifndef APPLICATIONS_USER_DRIVERS_WS2812_H_
#define APPLICATIONS_USER_DRIVERS_WS2812_H_

#define PIXEL_MAX 2
#define BIT_ONE  75//70/84=0.833us高电平
#define BIT_ZERO 40//40/84=0.476us高电平
typedef enum
{
    black = 0,
    red = 0xff0000,
    yellow = 0xffff00,
    green = 0x00ff00,
    cyan = 0x00ffff,
    blue = 0x0000ff,
    purple = 0xff00ff,
    white = 0xffffff
}_color;
typedef enum
{
    led_fun=0,//功能led
    led_sta=1//状态led
}_led_number;
typedef enum
{
    normal_status=0,
    led_error,
    acc_offset_status,
    horiz_offset__status,
    yaw_offset_status,
    gyro_offset_status,
    mag_offset_status
}_led_status;
typedef enum
{
    bright=0,
    fast_flash=1,
    medium_flash=2,
    low_flash=3,
    extr_low_flash=4
}_led_flash;
typedef struct
{
    const uint16_t header[3];
    uint16_t data[24*PIXEL_MAX];
    const uint16_t tail;
}_ws2812_frame;

typedef struct
{
    _led_number number;//编号
    _color color;//颜色
    uint32_t lighttime;//开始点亮的时间
    _led_flash tick;//持续时间
}_led;

void led_run(void);
void led_change_status(_led_number num , uint8_t status);
void led_change_tick(_led_number num , _led_flash tick);
void led_closed(void);
void led_warning(void);
#endif /* APPLICATIONS_USER_DRIVERS_WS2812_H_ */
