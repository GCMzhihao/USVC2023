/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-15     PC-COLD       the first version
 */
#include <include.h>
#include <drv_spi.h>

#define SPI2_BUS_NAME "spi2"
#define SPI2_DEV_NAME_AK8975 "ak8975"
static struct stm32_hw_spi_cs ak8975_cs;
static struct rt_spi_device *spi_dev_ak8975;

static rt_size_t ak8975_readbytes(uint8_t reg ,uint8_t *buff,rt_size_t buff_len)
{
    reg|=0x80;
    return rt_spi_send_then_recv(spi_dev_ak8975, &reg, 1, buff, buff_len);
}
void ak8975_read(rt_int16_t *buff)
{
    uint8_t tmp[2];
    tmp[0]=AK8975_CNTL_REG;
    tmp[1]=0x01;
    ak8975_readbytes(AK8975_HXL_REG, (uint8_t *)buff, 6);//get xyz magnetic data
    rt_spi_send(spi_dev_ak8975,tmp,2);//single measurement mode
}

static int ak8975_init(void)
{
    rt_err_t res;

    ak8975_cs.GPIOx = GPIOB;
    ak8975_cs.GPIO_Pin = GPIO_PIN_12;
    res = rt_hw_spi_device_attach(SPI2_BUS_NAME,SPI2_DEV_NAME_AK8975,ak8975_cs.GPIOx,ak8975_cs.GPIO_Pin);
    if(res!=RT_EOK)
    {
        rt_kprintf("ak8975 cs_pin attach failed!");
        return RT_ERROR;
    }
    spi_dev_ak8975 = (struct rt_spi_device *)rt_device_find(SPI2_DEV_NAME_AK8975);

    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
    cfg.max_hz = 20 * 1000 *1000; /* 20M,SPI max 42MHz,ssd1351 4-wire spi */
    res=rt_spi_configure(spi_dev_ak8975, &cfg);
    if(res!=RT_EOK)
    {
       rt_kprintf("ak8975 configure failed!");
       return RT_ERROR;
    }


    return RT_EOK;
}
INIT_DEVICE_EXPORT(ak8975_init);

int ak8975_check(void)
{
    uint8_t tmp;
    ak8975_readbytes(AK8975_WIA_REG,&tmp,1);
    if(tmp!=0x48)
    {
       rt_kprintf("ak8975 hardware error:0x%02x!\n",tmp);
       return RT_ERROR;
    }
    return RT_EOK;
}
INIT_APP_EXPORT(ak8975_check);



