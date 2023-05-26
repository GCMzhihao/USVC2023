/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-05-23     LZH       the first version
 */
#include<Sbus.h>

uint16_t RevSbus[16];
uint8_t RevBuf[100];
uint8_t cnt=0;
uint8_t SbusFlag;
uint8_t SbusRevFlag=0;
void SbusReceive(uint8_t data)
{
    RevBuf[cnt++]=data;
    if(cnt>=25)
   {
       cnt=0;
       SbusRevFlag=1;
   }
}

void SbusRocker_conver(void)
{
    rocker.rightX=RevSbus[0];
    rocker.leftY=RevSbus[1];
    rocker.rightY=RevSbus[2];
    rocker.leftX=RevSbus[3];
    rocker.switchA=RevSbus[4];
    rocker.switchB=RevSbus[5];
    rocker.switchC=RevSbus[6];
    rocker.switchD=RevSbus[7];
    rocker.switchE=RevSbus[8];
    rocker.switchF=RevSbus[9];
    rocker.switchG=RevSbus[10];
}


void SbusHandle(void)
{
    if(SbusRevFlag==1)
    {
        SbusRevFlag=0;
        if(RevBuf[0]==0x0f&&RevBuf[24]==0x00)
         {
            RevSbus[0] = (uint16_t)(RevBuf[2] & 0x07) << 8 | RevBuf[1];
            RevSbus[1] = (uint16_t)(RevBuf[3] & 0x3f) << 5 | (RevBuf[2] >> 3);
            RevSbus[2] = (uint16_t)(RevBuf[5] & 0x01) << 10 | ((uint16_t)RevBuf[4] << 2) | (RevBuf[3] >> 6);
            RevSbus[3] = (uint16_t)(RevBuf[6] & 0x0F) << 7 | (RevBuf[5] >> 1);
            RevSbus[4] = (uint16_t)(RevBuf[7] & 0x7F) << 4 | (RevBuf[6] >> 4);
            RevSbus[5] = (uint16_t)(RevBuf[9] & 0x03) << 9 | ((uint16_t)RevBuf[8] << 1) | (RevBuf[7] >> 7);
            RevSbus[6] = (uint16_t)(RevBuf[10] & 0x1F) << 6 | (RevBuf[9] >> 2);
            RevSbus[7] = (uint16_t)RevBuf[11] << 3 | (RevBuf[10] >> 5);

            RevSbus[8] = (uint16_t)(RevBuf[13] & 0x07) << 8 | RevBuf[12];
            RevSbus[9] = (uint16_t)(RevBuf[14] & 0x3f) << 5 | (RevBuf[13] >> 3);
            RevSbus[10] = (uint16_t)(RevBuf[16] & 0x01) << 10 | ((uint16_t)RevBuf[15] << 2) | (RevBuf[14] >> 6);
            RevSbus[11] = (uint16_t)(RevBuf[17] & 0x0F) << 7 | (RevBuf[16] >> 1);
            RevSbus[12] = (uint16_t)(RevBuf[18] & 0x7F) << 4 | (RevBuf[17] >> 4);
            RevSbus[13] = (uint16_t)(RevBuf[20] & 0x03) << 9 | ((uint16_t)RevBuf[19] << 1) | (RevBuf[18] >> 7);
            RevSbus[14] = (uint16_t)(RevBuf[21] & 0x1F) << 6 | (RevBuf[20] >> 2);
            RevSbus[15] = (uint16_t)RevBuf[22] << 3 | (RevBuf[21] >> 5);
            SbusFlag=RevBuf[23];

        //SbusRocker_conver();


         }

    }


}






