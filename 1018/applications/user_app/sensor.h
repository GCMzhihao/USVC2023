/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-19     PC-COLD       the first version
 */
#ifndef APPLICATIONS_USER_APP_USER_SENSOR_H_
#define APPLICATIONS_USER_APP_USER_SENSOR_H_
#include <common.h>

#define ADCToRadianSpeed  1.06526443e-3f//(1/65536*4000/180*3.14159265f)// 1.06526443e-3f
#define ADCToAngularSpeed 6.10351563e-2f//(1.0f/65536*4000)//  6.10351563e-2f
#define RaSpeedToAngSpeed  5.72957795e1f//1*180/pi
#define ADCToGravAcc       4.8828125e-4f// 1/65536*32
#define ADCToMagFlux       0.3e-06f
typedef struct
{
    //原始数据
    VectorInt16 acc_raw;
    int16_t temp_raw;
    VectorInt16 gyro_raw;
    VectorInt16 mag_raw;
    //校正后数据
    VectorFloat acc_self_crt;
    VectorFloat acc_horiz_crt;
    VectorFloat gyro_crt;
    VectorFloat mag_crt;
    float temp_crt;
    //最终数据
    VectorFloat acc;
    VectorFloat gyro;
    VectorFloat mag;
    float temp_C;

    VectorFloat w_acc;//世界坐标系加速度
    VectorFloat uwb_acc;//uwb坐标系下加速度
}_sensor;

typedef struct EllipsoidFitting
{
    int16_t points_cnt;
    //一次项统计和
    double x_sum,y_sum,z_sum;
    //二次项统计和
    double xx_sum,yy_sum,zz_sum,xy_sum,xz_sum,yz_sum;
    //三次项统计和
    double xxx_sum,xxy_sum,xxz_sum,xyy_sum,xzz_sum,yyy_sum,yyz_sum,yzz_sum,zzz_sum;
    //四次项统计和
    double yyyy_sum,zzzz_sum,xxzz_sum,yyzz_sum,xxyy_sum;
}elf;

extern _sensor sensor;
void GetAccGyroData(float dt);
void GetMagData(float dt);
void SensorCorrection(float dt);

#endif /* APPLICATIONS_USER_APP_USER_SENSOR_H_ */
