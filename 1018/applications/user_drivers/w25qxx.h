/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-17     PC-COLD       the first version
 */
#ifndef APPLICATIONS_USER_DRIVERS_W25QXX_H_
#define APPLICATIONS_USER_DRIVERS_W25Qxx_H_

/* Private define ------------------------------------------------------------*/

//#define W25QXX_DEBUG
#ifdef W25QXX_DEBUG
#define w25qxx_debug(fmt, ...)  rt_kprintf(fmt, ##__VA_ARGS__)
#else
#define w25qxx_debug(fmt, ...)
#endif

#define JEDEC_MANUFACTURER_ST       0x20
#define JEDEC_MANUFACTURER_MACRONIX 0xC2
#define JEDEC_MANUFACTURER_WINBOND  0xEF

/* JEDEC Device ID: Memory type and Capacity */
#define JEDEC_W25Q16_BV_CL_CV   (0x4015) /* W25Q16BV W25Q16CL W25Q16CV  */
#define JEDEC_W25Q16_DW         (0x6015) /* W25Q16DW  */
#define JEDEC_W25Q32_BV         (0x4016) /* W25Q32BV */
#define JEDEC_W25Q32_DW         (0x6016) /* W25Q32DW */
#define JEDEC_W25Q64_BV_CV      (0x4017) /* W25Q64BV W25Q64CV */
#define JEDEC_W25Q64_DW         (0x4017) /* W25Q64DW */
#define JEDEC_W25Q128_BV        (0x4018) /* W25Q128BV */


#define JEDEC_WRITE_ENABLE           0x06
#define JEDEC_WRITE_DISABLE          0x04
#define JEDEC_READ_STATUS            0x05
#define JEDEC_WRITE_STATUS           0x01
#define JEDEC_READ_DATA              0x03
#define JEDEC_FAST_READ              0x0b
#define JEDEC_DEVICE_ID              0x9F
#define JEDEC_PAGE_WRITE             0x02
#define JEDEC_SECTOR_ERASE           0x20

#define JEDEC_STATUS_BUSY            0x01
#define JEDEC_STATUS_WRITEPROTECT    0x02
#define JEDEC_STATUS_BP0             0x04
#define JEDEC_STATUS_BP1             0x08
#define JEDEC_STATUS_BP2             0x10
#define JEDEC_STATUS_TP              0x20
#define JEDEC_STATUS_SEC             0x40
#define JEDEC_STATUS_SRP0            0x80

#define DUMMY_BYTE     0xFF
#define W25Q_SECTOR_SIZE    4096



/* Jedec Flash Information structure */
typedef struct
{
    uint8_t initialized;
    uint16_t sector_size;
    uint16_t sector_count;
    uint32_t capacity;
} flash_info_t;

typedef struct
{
    uint8_t Manufacturer;             /* Manufacturer ID */
    uint8_t Memory;               /* Density Code */
    uint8_t Capacity;                /* Family Code */
    uint8_t rev;

} jedec_id_t;

rt_err_t w25q_ReadBuffers ( uint32_t address, uint8_t* buffer,  uint32_t len );
rt_err_t w25q_WriteBuffers ( uint32_t address, uint8_t* buffer,  uint32_t len );
#endif /* APPLICATIONS_USER_DRIVERS_W25QXX_H_ */
