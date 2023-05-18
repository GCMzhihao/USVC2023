/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-06-30     PC-COLD       the first version
 */
#include <include.h>
#include <drv_spi.h>

#define SPI2_BUS_NAME "spi2"
#define SPI2_DEV_ICM20602 "icm20602"
static struct stm32_hw_spi_cs icm20602_cs;
static struct rt_spi_device *spi_dev_icm20602;

static int icm20602_init(void)
{
    rt_err_t res;
    icm20602_cs.GPIOx = GPIOB;
    icm20602_cs.GPIO_Pin = GPIO_PIN_10;
    res = rt_hw_spi_device_attach(SPI2_BUS_NAME,SPI2_DEV_ICM20602,icm20602_cs.GPIOx,icm20602_cs.GPIO_Pin);
    if(res!=RT_EOK)
    {
        rt_kprintf("icm20602 cs_pin attach failed!");
        return RT_ERROR;
    }
    spi_dev_icm20602 = (struct rt_spi_device *)rt_device_find(SPI2_DEV_ICM20602);

    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_3 | RT_SPI_MSB;
    cfg.max_hz = 20 * 1000 *1000;
    res=rt_spi_configure(spi_dev_icm20602, &cfg);
    if(res!=RT_EOK)
    {
       rt_kprintf("icm20602 configure failed!");
       return RT_ERROR;
    }

    return RT_EOK;
}
INIT_DEVICE_EXPORT(icm20602_init);

static rt_size_t icm20602_writebyte(rt_uint8_t reg ,rt_uint8_t data)
{
    rt_uint8_t tmp[2];
    tmp[0]=reg;
    tmp[1]=data;
    return rt_spi_send(spi_dev_icm20602, tmp, 2);
}
static rt_size_t icm20602_readbytes(rt_uint8_t reg , rt_uint8_t *buff,rt_size_t buff_len)
{
    reg|=0x80;
    return rt_spi_send_then_recv(spi_dev_icm20602, &reg, 1,buff, buff_len);
}
static int icm20602_config(void)
{
    rt_uint8_t res;
    icm20602_readbytes(MPUREG_WHOAMI,&res,1);
    if(res!=MPU_WHOAMI_20602)
    {
        rt_kprintf("icm20602 hardware error:0x%02x!\n",res);
        return RT_ERROR;
    }
    icm20602_writebyte(MPU_RA_PWR_MGMT_1,0x80);//Device reset
    rt_thread_delay(10);

    icm20602_writebyte(MPU_RA_PWR_MGMT_1,0x01);//clock source
    rt_thread_delay(10);

    icm20602_writebyte(MPU_RA_SIGNAL_PATH_RESET,0x03);//Reset temp digital signal path
    rt_thread_delay(10);

    icm20602_writebyte(MPU_RA_USER_CTRL,0x01);//Reset all gyro digital signal path, accel digital signal path,
    rt_thread_delay(10);                      //and temp digital signal path.
                                              //This bit also clears all the sensor registers.

    icm20602_writebyte(0x70,0x40);//Disable I2C Slave module and put the serial interface in SPI mode only.
    rt_thread_delay(10);

    icm20602_writebyte(MPU_RA_PWR_MGMT_2,0x00);// XYZ accelerometer is on
    rt_thread_delay(10);                       // XYZ gyro is on

    icm20602_writebyte(MPU_RA_SMPLRT_DIV,0);//1khz sample rate
    rt_thread_delay(10);

    icm20602_writebyte(MPU_RA_CONFIG,ICM20602_LPF_20HZ);//Gyro and temp 20Hz Low-pass filter
    rt_thread_delay(10);

    icm20602_writebyte(MPU_RA_GYRO_CONFIG,ICM20602_2000DPS );//±2000dps
    rt_thread_delay(10);

    icm20602_writebyte(MPU_RA_ACCEL_CONFIG,ICM20602_16G);//±16g
    rt_thread_delay(10);

    icm20602_writebyte(MPU_RA_ACCEL_CONFIG2,ICM20602_LPF_10HZ);//acc 10HZ Low-pass filter
    rt_thread_delay(10);

    icm20602_writebyte(MPU_RA_LP_MODE_CFG,0x00);//low-power mode is disable
    rt_thread_delay(10);

    icm20602_writebyte(MPU_RA_FIFO_EN,0x00);//FIFO is disable
    rt_thread_delay(10);

    return 0;
}
INIT_COMPONENT_EXPORT(icm20602_config);

void icm20602_read(int16_t *buff)
{
    uint8_t tmp[14],i;
    icm20602_readbytes(MPU_RA_ACCEL_XOUT_H, tmp, 14);
    for(i=0;i<7;i++)
    {
        buff[i]=(int16_t)tmp[i*2]<<8|tmp[i*2+1];
    }
}


