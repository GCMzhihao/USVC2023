/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-15     RealThread   first version
 */

#include <rtthread.h>
#include <board.h>
#include <drv_common.h>

RT_WEAK void rt_hw_board_init()
{
    extern void hw_board_init(char *clock_src, int32_t clock_src_freq, int32_t clock_target_freq);

    /* Heap initialization */
#if defined(RT_USING_HEAP)
    rt_system_heap_init((void *) HEAP_BEGIN, (void *) HEAP_END);
#endif

    hw_board_init(BSP_CLOCK_SOURCE, BSP_CLOCK_SOURCE_FREQ_MHZ, BSP_CLOCK_SYSTEM_FREQ_MHZ);

    /* Set the shell console output device */
#if defined(RT_USING_DEVICE) && defined(RT_USING_CONSOLE)
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

    /* Board underlying hardware initialization */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif

}
//void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  if(hspi->Instance==SPI1)
//  {
//    __HAL_RCC_SPI1_CLK_ENABLE();
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//  }
//  else if(hspi->Instance==SPI2)
//  {
//    __HAL_RCC_SPI2_CLK_ENABLE();
//    __HAL_RCC_GPIOB_CLK_ENABLE();
//    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//  }
//
//}

//DMA_HandleTypeDef hdma_tim3_ch1_trig;
//
//void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
//{
//    if(htim_base->Instance==TIM3)
//    {
//        __HAL_RCC_TIM3_CLK_ENABLE();
//        hdma_tim3_ch1_trig.Instance = DMA1_Stream4;
//        hdma_tim3_ch1_trig.Init.Channel = DMA_CHANNEL_5;
//        hdma_tim3_ch1_trig.Init.Direction = DMA_MEMORY_TO_PERIPH;
//        hdma_tim3_ch1_trig.Init.PeriphInc = DMA_PINC_DISABLE;
//        hdma_tim3_ch1_trig.Init.MemInc = DMA_MINC_ENABLE;
//        hdma_tim3_ch1_trig.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
//        hdma_tim3_ch1_trig.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
//        hdma_tim3_ch1_trig.Init.Mode = DMA_NORMAL;
//        hdma_tim3_ch1_trig.Init.Priority = DMA_PRIORITY_VERY_HIGH;
//        hdma_tim3_ch1_trig.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
//        if (HAL_DMA_Init(&hdma_tim3_ch1_trig) != HAL_OK)
//        {
//          Error_Handler();
//        }
//        __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_CC1],hdma_tim3_ch1_trig);
//        __HAL_LINKDMA(htim_base,hdma[TIM_DMA_ID_TRIGGER],hdma_tim3_ch1_trig);
//    }
//    else if(htim_base->Instance==TIM4)
//    {
//      __HAL_RCC_TIM4_CLK_ENABLE();
//    }
//}
//
//void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
//{
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//    if(htim->Instance==TIM3)
//    {
//        __HAL_RCC_GPIOB_CLK_ENABLE();
//        /**TIM3 GPIO Configuration
//        PB4     ------> TIM3_CH1
//        */
//        GPIO_InitStruct.Pin = GPIO_PIN_4;
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
//        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//    }
//    else if(htim->Instance==TIM4)
//    {
//        __HAL_RCC_GPIOD_CLK_ENABLE();
//        /**TIM4 GPIO Configuration
//        PD12     ------> TIM4_CH1
//        PD13     ------> TIM4_CH2
//        PD14     ------> TIM4_CH3
//        PD15     ------> TIM4_CH4
//        */
//        GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
//        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
//        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
//    }
//}
//
//void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
//{
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//    if(hadc->Instance==ADC1)
//    {
//        __HAL_RCC_ADC1_CLK_ENABLE();
//
//        __HAL_RCC_GPIOB_CLK_ENABLE();
//        /**ADC1 GPIO Configuration
//        PB0     ------> ADC1_IN8
//        */
//        GPIO_InitStruct.Pin = GPIO_PIN_0;
//        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//        GPIO_InitStruct.Pull = GPIO_NOPULL;
//        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//    }
//
//}
//
////void HAL_PCD_MspInit(PCD_HandleTypeDef* pcdHandle)
////{
////    GPIO_InitTypeDef GPIO_InitStruct;
////    if(pcdHandle->Instance==USB_OTG_FS)
////    {
////        __HAL_RCC_GPIOA_CLK_ENABLE();
////        /**USB_OTG_FS GPIO Configuration
////        PA11     ------> USB_OTG_FS_DM
////        PA12     ------> USB_OTG_FS_DP
////        */
////        GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
////        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
////        GPIO_InitStruct.Pull = GPIO_NOPULL;
////        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
////        GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
////        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
////        /* Peripheral clock enable */
////        __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
////        /* Peripheral interrupt init */
////        HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);
////        HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
////    }
////}
