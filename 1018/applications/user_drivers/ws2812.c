/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-20     PC-COLD       the first version
 */
#include <include.h>
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);

TIM_HandleTypeDef htim3;

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 104;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
}

static int ws2812_init(void)
{
    MX_DMA_Init();
    MX_TIM3_Init();
    return RT_EOK;
}
INIT_DEVICE_EXPORT(ws2812_init);


_ws2812_frame ws2812_frame={
                                .header[0]=0,
                                .header[1]=0,
                                .header[2]=0,
                                .tail=0,
                           };

static _led led0={0,0,0,0};//功能指示灯
static _led led1={1,0,0,0};//状态指示灯
static _color color[PIXEL_MAX]={};
static void led_light(void)
{
    uint8_t i,j;
    for(i=0;i<PIXEL_MAX;i++)
    {
        for(j=0;j<8;j++)
        {
            ws2812_frame.data[24*i+j]    = ((*(((uint8_t*)(&color[i]))+1))&0x80>>j)?BIT_ONE:BIT_ZERO;//green
            ws2812_frame.data[24*i+j+8]  = ((*(((uint8_t*)(&color[i]))+2))&0x80>>j)?BIT_ONE:BIT_ZERO;//red
            ws2812_frame.data[24*i+j+16] = ((*(((uint8_t*)(&color[i]))))&0x80>>j)?BIT_ONE:BIT_ZERO;//blue
        }
    }
    HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t*)&ws2812_frame,3+24*PIXEL_MAX+1);
}
uint32_t led_time=0;
//功能：点亮led并以时间tick翻转
//入口参数led ：待操作的led
static void led_tick(_led *led )
{
    if(led->tick==0)
        color[led->number] = led->color;
    else
    {
        if(led_time < led->lighttime + led->tick)//点亮led
        {
            color[led->number] = led->color;
        }
        else//熄灭led
        {
            color[led->number] = 0;
            if(led_time >= led->lighttime + led->tick*2-1)
            {
                led->lighttime = led_time+1;
            }
        }
    }
}
void led_run(void)
{
    led_tick(&led0);
    led_tick(&led1);
    led_light();
    led_time++;
}
//功能：修改led状态
//入口参数num ：led编号
//入口参数status：颜色状态
void led_change_status(_led_number num , uint8_t status)
{
    _led *led;
    if(num==led0.number)
        led = &led0;
    else if (num==led1.number)
        led = &led1;
    else
        return;
    switch (status)
    {
        case 0:
            led->color=black;
            break;
        case 1:
            led->color=red;
            break;
        case 2:
            led->color=yellow;
            break;
        case 3:
            led->color=green;
            break;
        case 4:
            led->color=cyan;
            break;
        case 5:
            led->color=blue;
            break;
        case 6:
            led->color=purple;
            break;
        case 7:
            led->color=white;
            break;
        default:
            break;
    }
}
//功能：修改led tick
//入口参数num ：led编号
//入口参数tick：点亮时间
void led_change_tick(_led_number num , _led_flash tick)
{
    _led *led;
    if(num==led0.number)
        led = &led0;
    else if (num==led1.number)
        led = &led1;
    else
        return;
    led->tick = tick;
}

void led_warning(void)
{
    led_change_status(led_fun, led_error);
    led_change_status(led_sta, led_error);
    led_change_tick(led_fun, fast_flash);
    led_change_tick(led_sta, fast_flash);
}

void led_closed(void)
{
    led_change_status(led_fun, normal_status);
    led_change_tick(led_fun, bright);
    led_change_status(led_sta, normal_status);
    led_change_tick(led_sta, bright);
}


extern void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
extern DMA_HandleTypeDef hdma_tim3_ch1_trig;
void DMA1_Stream4_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_DMA_IRQHandler(&hdma_tim3_ch1_trig);
    rt_interrupt_leave();
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM3)
        HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
}



