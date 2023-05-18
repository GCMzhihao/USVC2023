/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-17     PC-COLD       the first version
 */


#include <include.h>
#include <drv_spi.h>

static struct stm32_hw_spi_cs w25q_cs;
static struct rt_spi_device *spi_dev_w25q;
static flash_info_t flash_info;

static int w25q_config(void)
{
    rt_err_t res;
    w25q_cs.GPIOx = GPIOE;
    w25q_cs.GPIO_Pin = GPIO_PIN_10;
    res = rt_hw_spi_device_attach("spi1","w25q",w25q_cs.GPIOx,w25q_cs.GPIO_Pin);
    if(res!=RT_EOK)
    {
        rt_kprintf("w25q cs_pin attach failed!");
        return RT_ERROR;
    }
    spi_dev_w25q = (struct rt_spi_device *)rt_device_find("w25q");

    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB;
    cfg.max_hz = 20 * 1000 *1000;
    res=rt_spi_configure(spi_dev_w25q, &cfg);
    if(res!=RT_EOK)
    {
       rt_kprintf("w25q configure failed!");
       return RT_ERROR;
    }

    return RT_EOK;
}
INIT_DEVICE_EXPORT(w25q_config);

static void w25q_WaitForEnd ( void )
{
    rt_uint8_t FLASH_Status = 0,reg;
    reg=JEDEC_READ_STATUS;
    do
    {
        rt_spi_send_then_recv(spi_dev_w25q, &reg, 1, &FLASH_Status, 1);
    } while (FLASH_Status & JEDEC_STATUS_BUSY);
}

//Erases the specified FLASH sector.
static void w25q_SectorErase ( uint32_t address)
{
    rt_uint8_t reg;
    rt_uint8_t buff[4];
    reg=JEDEC_WRITE_ENABLE;
    rt_spi_send(spi_dev_w25q, &reg, 1);
    buff[0]=JEDEC_SECTOR_ERASE;
    buff[1]=*((uint8_t *)(&address)+2);
    buff[2]=*((uint8_t *)(&address)+1);
    buff[3]=*((uint8_t *)(&address)+0);
    rt_spi_send(spi_dev_w25q, buff, 4);
    w25q_WaitForEnd();
}
/**
  * @brief  Writes more than one byte to the FLASH with a single WRITE
  *         cycle(Page WRITE sequence). The number of byte can't exceed
  *         the FLASH page size.
  * @param pBuffer : pointer to the buffer  containing the data to be
  *                  written to the FLASH.
  * @param WriteAddr : FLASH's internal address to write to.
  * @param NumByteToWrite : number of bytes to write to the FLASH,
  *                       must be equal or less than "SPI_FLASH_PageSize" value.
  * @retval : None
  */

static void w25q_PageWrite ( uint32_t address, uint8_t* buffer,  uint32_t len )
{
    rt_uint8_t reg;
    rt_uint8_t send_buff[4];
    reg=JEDEC_WRITE_ENABLE;
    rt_spi_send(spi_dev_w25q, &reg, 1);
    send_buff[0]=JEDEC_PAGE_WRITE;
    send_buff[1]=*((uint8_t *)(&address)+2);
    send_buff[2]=*((uint8_t *)(&address)+1);
    send_buff[3]=*((uint8_t *)(&address)+0);
    rt_spi_send_then_send(spi_dev_w25q, send_buff, 4, buffer, len);
    w25q_WaitForEnd();
}

static void w25q_ReadID ( jedec_id_t *id )
{
    rt_uint8_t reg;
    reg=JEDEC_DEVICE_ID;
    rt_spi_send_then_recv(spi_dev_w25q, &reg, 1, ( uint8_t* ) id, 3);
}


rt_err_t w25q_ReadBuffers ( uint32_t address, uint8_t* buffer,  uint32_t len )
{
    rt_uint8_t send_buff[4];

    if(address+len>flash_info.sector_count*flash_info.sector_size)
        return -RT_ERROR;

    send_buff[0]=JEDEC_READ_DATA;
    send_buff[1]=*((uint8_t *)(&address)+2);
    send_buff[2]=*((uint8_t *)(&address)+1);
    send_buff[3]=*((uint8_t *)(&address)+0);
    rt_spi_send_then_recv(spi_dev_w25q, send_buff, 4, buffer, len);
    return RT_EOK;
}

static void w25q_OneSectorWrite ( uint32_t start, uint32_t end,uint8_t *buffer)
{
    uint16_t i = 0 , j = 0 , k = 0 , page = 0;
    uint16_t sector_num=0;
    uint8_t *tmp;
    uint32_t address=0;
    tmp=rt_malloc(W25Q_SECTOR_SIZE);
    if(tmp==RT_NULL)
    {
        rt_kprintf("malloc memory failed!\r\n");
        return;
    }
    sector_num=start/flash_info.sector_size;
    w25q_ReadBuffers(sector_num*flash_info.sector_size,tmp,flash_info.sector_size);//将整块扇区数据存起来
    w25q_SectorErase(sector_num*flash_info.sector_size);//将该扇区数据擦除

    for(i=start%flash_info.sector_size;i<end%flash_info.sector_size+1;i++)
    {
        tmp[i]=buffer[j];//将需要写入的数据覆盖原来位置的数据
        j++;
    }
    address=sector_num*flash_info.sector_size;
    page = flash_info.sector_size / 256;
    while ( page-- )
    {
        w25q_PageWrite ( address, &tmp[k*256], 256 );
        k++;
        address += 256;
    }//写入整个扇区数据
    rt_free(tmp);
}
rt_err_t w25q_WriteBuffers ( uint32_t address, uint8_t* buffer,  uint32_t len )
{
    uint32_t address_start=0,address_end=0;
    uint32_t w_start=0,w_end=0;
    uint32_t address_sector_end=0;
    uint32_t cnt=0;
    if(address+len>flash_info.sector_count*flash_info.sector_size)
        return -RT_ERROR;

    address_start=address;
    address_end=address+len-1;//写数据终止地址

    while(w_end<address_end)
    {
        if(w_start==0&&w_end==0)//第一次写入
            w_start=address_start;
        else
            w_start=w_end+1;
        address_sector_end=(w_start/flash_info.sector_size+1)*flash_info.sector_size-1;//获取该扇区末尾地址
        if(address_sector_end>address_end)
            w_end=address_end;
        else
            w_end=address_sector_end;

        w25q_OneSectorWrite(w_start,w_end,buffer+cnt);
        cnt+=w_end-w_start+1;
    }
    return RT_EOK;
}

static int w25q_init(void)
{
    jedec_id_t flash_id;
    rt_uint8_t reg;
    reg=DUMMY_BYTE;
    rt_spi_send(spi_dev_w25q, &reg, 1);
    /* read flash id */
    w25q_ReadID ( &flash_id );

    if ( flash_id.Manufacturer == JEDEC_MANUFACTURER_WINBOND )
    {
        flash_info.sector_size = W25Q_SECTOR_SIZE;                         /* Page Erase (4096 Bytes) */
        if ( flash_id.Capacity == ( JEDEC_W25Q128_BV & 0xff ) )
        {
            w25qxx_debug ( "W25Q128_BV detection\r\n" );

            flash_info.sector_count = 4096;                        /* 128Mbit / 8 / 4096 = 4096 */
        }
        else if ( flash_id.Capacity == ( JEDEC_W25Q64_DW & 0xff ) )
        {
            w25qxx_debug ( "W25Q64_DW or W25Q64_BV or W25Q64_CV detection\r\n" );
            flash_info.sector_count = 2048;                       /* 64Mbit / 8 / 4096 = 2048 */
        }
        else if ( flash_id.Capacity == ( JEDEC_W25Q32_DW & 0xff ) )
        {
            w25qxx_debug ( "W25Q32_DW or W25Q32_BV detection\r\n" );
            flash_info.sector_count = 1024;                       /* 32Mbit / 8 / 4096 = 1024 */
        }
        else if ( flash_id.Capacity == ( JEDEC_W25Q16_DW & 0xff ) )
        {
            w25qxx_debug ( "W25Q16_DW or W25Q16_BV detection\r\n" );
            flash_info.sector_count = 512;                       /* 16Mbit / 8 / 4096 = 512 */
        }
        else
        {
            w25qxx_debug ( "error flash capacity\r\n" );
            flash_info.sector_count = 0;
        }

        flash_info.capacity = flash_info.sector_size * flash_info.sector_count;
    }
    else
    {
        w25qxx_debug ( "Unknow Manufacturer ID!%02X\r\n", flash_id.Manufacturer );
        flash_info.initialized = 0;
        return RT_ERROR;
    }

    flash_info.initialized = 1;

    return RT_EOK;
}
INIT_COMPONENT_EXPORT(w25q_init);
