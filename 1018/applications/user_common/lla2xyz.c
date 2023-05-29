/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-05-29     LZH       the first version
 */

#include<lla2xyz.h>

const double_t DEG_TO_RED_LOCAL=3.1415926535897932/180.0;

void lla2xyz(double_t longitude,double_t latitude,double_t height)
{
    double_t lon=longitude*DEG_TO_RED_LOCAL;
    double_t lat=latitude*DEG_TO_RED_LOCAL;
    double_t hei=height;

    double_t a=6378137.0;//赤道半径，单位m
    double_t b=6356752.31424518;//地球短半轴，单位m

    double_t N=a/(sqrt(1-((a*a-b*b)/(a*a))*sin(lat)*sin(lat)));

    USV_State.X=(N+hei)*cos(lat)*cos(lon);
    USV_State.Y=(N+hei)*cos(lat)*sin(lon);
   // z=((b*b*N)/(a*a)+hei)*sin(lat);

}
