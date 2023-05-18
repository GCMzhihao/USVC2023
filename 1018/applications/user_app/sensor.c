/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs: sensor handle
 * Date           Author       Notes
 * 2020-07-19     PC-COLD       the first version
 */
#include<include.h>

VectorFloat AccFilter = {0.0f,0.0f,0.0f};
VectorFloat GyroFilter = {0.0f,0.0f,0.0f};
VectorFloat MagFilter = {0.0f,0.0f,0.0f};
_sensor sensor;
elf elf_acc, elf_mag;
float res_mag[6],res_acc[6];
void HorizCorrection(void);//机体水平校准
void GetAccGyroData(float dt)
{
    icm20602_read(&sensor.acc_raw.x);
    //计算校准后数据
    sensor.acc_self_crt.x = (sensor.acc_raw.x - parameters.acc_self_offset.x)*parameters.acc_self_gain_inv.x;
    sensor.acc_self_crt.y = (sensor.acc_raw.y - parameters.acc_self_offset.y)*parameters.acc_self_gain_inv.y;
    sensor.acc_self_crt.z = (sensor.acc_raw.z - parameters.acc_self_offset.z)*parameters.acc_self_gain_inv.z;//六面校准

    sensor.temp_crt = sensor.temp_raw/326.8+20;
    sensor.gyro_crt.x = (sensor.gyro_raw.x - parameters.gyro_offset.x)*ADCToRadianSpeed;
    sensor.gyro_crt.y = (sensor.gyro_raw.y - parameters.gyro_offset.y)*ADCToRadianSpeed;
    sensor.gyro_crt.z = (sensor.gyro_raw.z - parameters.gyro_offset.z)*ADCToRadianSpeed;

    _1st_lpf(sensor.acc_self_crt.x, &sensor.acc.x, 5, dt);
    _1st_lpf(sensor.acc_self_crt.y, &sensor.acc.y, 5, dt);
    _1st_lpf(sensor.acc_self_crt.z, &sensor.acc.z, 5, dt);

    _1st_lpf(sensor.gyro_crt.x, &sensor.gyro.x, 20, dt);
    _1st_lpf(sensor.gyro_crt.y, &sensor.gyro.y, 20, dt);
    _1st_lpf(sensor.gyro_crt.z, &sensor.gyro.z, 20, dt);
}

void GetMagData(float dt)
{
    ak8975_read(&sensor.mag_raw.x);
    sensor.mag_crt.x = -(sensor.mag_raw.x - parameters.mag_offset.x)*parameters.mag_gain_inv.x;
    sensor.mag_crt.y =  (sensor.mag_raw.y - parameters.mag_offset.y)*parameters.mag_gain_inv.y;
    sensor.mag_crt.z = -(sensor.mag_raw.z - parameters.mag_offset.z)*parameters.mag_gain_inv.z;
    _1st_lpf(sensor.mag_crt.x, &sensor.mag.x, 5, dt);//将传感器所在坐标系转到机体坐标系，机头方向为x正向
    _1st_lpf(sensor.mag_crt.y, &sensor.mag.y, 5, dt);//将传感器所在坐标系转到机体坐标系，机头方向为x正向
    _1st_lpf(sensor.mag_crt.z, &sensor.mag.z, 5, dt);//将传感器所在坐标系转到机体坐标系，机头方向为x正向
}
void HorizCorrection(void)//机体水平校准
{
    static float x=0,y=0;

    static uint16_t cnt=0,cnt1=0;
    if(USV_State.HorizOffsetOk)
        return;
    led_change_status(led_fun, horiz_offset__status);
    led_change_tick(led_fun, bright);
    led_change_status(led_sta, horiz_offset__status);
    led_change_tick(led_sta, medium_flash);
    if(cnt==0&&cnt1<10)//等待数据稳定
    {
        x=0;
        y=0;
        cnt1++;
        return;
    }
    x+=Attitude.Roll.RawValue;
    y+=Attitude.Pitch.RawValue;
    cnt++;
    if(cnt>=300)
    {
        parameters.horiz_offset.x=x/cnt;
        parameters.horiz_offset.y=y/cnt;
        cnt=0;
        cnt1=0;
        USV_State.HorizOffsetOk=1;
        USV_State.SysOffsetOk=1;
        led_closed();
        param_write();
    }
}
static void YawCorrection(void)//偏航角校准
{
    static float z=0;

    static uint16_t cnt=0,cnt1=0;
    if(USV_State.YawOffsetOk)
        return;
    led_change_status(led_fun, yaw_offset_status);
    led_change_tick(led_fun, bright);
    led_change_status(led_sta, yaw_offset_status);
    led_change_tick(led_sta, medium_flash);
    if(cnt==0&&cnt1<1000)//等待数据稳定
    {
        z=0;
        cnt1++;
        return;
    }
    z+=Attitude.Yaw.RawValue;
    cnt++;
    if(cnt>=300)
    {
        parameters.horiz_offset.z=z/cnt;
        cnt=0;
        cnt1=0;
        USV_State.YawOffsetOk=1;
        USV_State.SysOffsetOk=1;
        led_closed();
        param_write();
    }
}
static void AccCorrection(void)
{
    static uint8_t hold_still_cnt=0;//静止次数
    static uint8_t acc_collect_flag=1;//采集数据标志
    uint8_t hold_still_max=50;//最大保持静止次数
    int16_t points_oneface_num=10;//每个面采集样本数
    if(USV_State.AccOffsetOk)
        return;
    if(acc_collect_flag==0)
    {
        led_change_status(led_fun, acc_offset_status);
        led_change_tick(led_fun, bright);
        led_change_status(led_sta, elf_acc.points_cnt/points_oneface_num+1);
        led_change_tick(led_sta, fast_flash);
        if(fabs(sensor.gyro.x)*RaSpeedToAngSpeed>30
           ||fabs(sensor.gyro.y)*RaSpeedToAngSpeed>30
           ||fabs(sensor.gyro.z)*RaSpeedToAngSpeed>30)//切换方向
        {
            acc_collect_flag=1;
        }
        return;
    }
    else
    {
        led_change_status(led_fun, acc_offset_status);
        led_change_tick(led_fun, bright);
        led_change_status(led_sta, elf_acc.points_cnt/points_oneface_num+1);//读取数据状态
        led_change_tick(led_sta, bright);//常亮
    }
    if(fabs(sensor.gyro.x)*RaSpeedToAngSpeed>2.0
       ||fabs(sensor.gyro.y)*RaSpeedToAngSpeed>2.0
       ||fabs(sensor.gyro.z)*RaSpeedToAngSpeed>2.0)//判断此时是否静止
    {
        hold_still_cnt=0;
        return;
    }
    else
        hold_still_cnt++;//静止次数+1
    if(hold_still_cnt<hold_still_max)
        return;
    else//达到最大静止次数。开始采集数据
        hold_still_cnt=0;
    elf_acc.x_sum+=sensor.acc_raw.x;
    elf_acc.y_sum+=sensor.acc_raw.y;
    elf_acc.z_sum+=sensor.acc_raw.z;
    elf_acc.xx_sum+=(1.0f*sensor.acc_raw.x*sensor.acc_raw.x);
    elf_acc.yy_sum+=(1.0f*sensor.acc_raw.y*sensor.acc_raw.y);
    elf_acc.zz_sum+=(1.0f*sensor.acc_raw.z*sensor.acc_raw.z);
    elf_acc.xy_sum+=(1.0f*sensor.acc_raw.x*sensor.acc_raw.y);
    elf_acc.xz_sum+=(1.0f*sensor.acc_raw.x*sensor.acc_raw.z);
    elf_acc.yz_sum+=(1.0f*sensor.acc_raw.y*sensor.acc_raw.z);

    elf_acc.xxx_sum+=(1.0f*sensor.acc_raw.x*sensor.acc_raw.x*sensor.acc_raw.x);
    elf_acc.xxy_sum+=(1.0f*sensor.acc_raw.x*sensor.acc_raw.x*sensor.acc_raw.y);
    elf_acc.xxz_sum+=(1.0f*sensor.acc_raw.x*sensor.acc_raw.x*sensor.acc_raw.z);
    elf_acc.xyy_sum+=(1.0f*sensor.acc_raw.x*sensor.acc_raw.y*sensor.acc_raw.y);
    elf_acc.xzz_sum+=(1.0f*sensor.acc_raw.x*sensor.acc_raw.z*sensor.acc_raw.z);
    elf_acc.yyy_sum+=(1.0f*sensor.acc_raw.y*sensor.acc_raw.y*sensor.acc_raw.y);
    elf_acc.yyz_sum+=(1.0f*sensor.acc_raw.y*sensor.acc_raw.y*sensor.acc_raw.z);
    elf_acc.yzz_sum+=(1.0f*sensor.acc_raw.y*sensor.acc_raw.z*sensor.acc_raw.z);
    elf_acc.zzz_sum+=(1.0f*sensor.acc_raw.z*sensor.acc_raw.z*sensor.acc_raw.z);

    elf_acc.yyyy_sum+=(1.0f*sensor.acc_raw.y*sensor.acc_raw.y*sensor.acc_raw.y*sensor.acc_raw.y);
    elf_acc.zzzz_sum+=(1.0f*sensor.acc_raw.z*sensor.acc_raw.z*sensor.acc_raw.z*sensor.acc_raw.z);
    elf_acc.xxyy_sum+=(1.0f*sensor.acc_raw.x*sensor.acc_raw.x*sensor.acc_raw.y*sensor.acc_raw.y);
    elf_acc.xxzz_sum+=(1.0f*sensor.acc_raw.x*sensor.acc_raw.x*sensor.acc_raw.z*sensor.acc_raw.z);
    elf_acc.yyzz_sum+=(1.0f*sensor.acc_raw.y*sensor.acc_raw.y*sensor.acc_raw.z*sensor.acc_raw.z);
    elf_acc.points_cnt++;

    if(elf_acc.points_cnt%points_oneface_num!=0)//一个面所有点数据没有采集完，继续采集
        return;
    else//一个面所有点数据采集完了，采集数据标志清零
        acc_collect_flag=0;
    if(elf_acc.points_cnt>=points_oneface_num*6)//六面数据全部采集完
    {
        elf_acc.x_sum/=elf_acc.points_cnt;
        elf_acc.y_sum/=elf_acc.points_cnt;
        elf_acc.z_sum/=elf_acc.points_cnt;

        elf_acc.xx_sum/=elf_acc.points_cnt;
        elf_acc.yy_sum/=elf_acc.points_cnt;
        elf_acc.zz_sum/=elf_acc.points_cnt;
        elf_acc.xy_sum/=elf_acc.points_cnt;
        elf_acc.xz_sum/=elf_acc.points_cnt;
        elf_acc.yz_sum/=elf_acc.points_cnt;

        elf_acc.xxx_sum/=elf_acc.points_cnt;
        elf_acc.xxy_sum/=elf_acc.points_cnt;
        elf_acc.xxz_sum/=elf_acc.points_cnt;
        elf_acc.xyy_sum/=elf_acc.points_cnt;
        elf_acc.xzz_sum/=elf_acc.points_cnt;
        elf_acc.yyy_sum/=elf_acc.points_cnt;
        elf_acc.yyz_sum/=elf_acc.points_cnt;
        elf_acc.yzz_sum/=elf_acc.points_cnt;
        elf_acc.zzz_sum/=elf_acc.points_cnt;

        elf_acc.yyyy_sum/=elf_acc.points_cnt;
        elf_acc.zzzz_sum/=elf_acc.points_cnt;
        elf_acc.xxyy_sum/=elf_acc.points_cnt;
        elf_acc.xxzz_sum/=elf_acc.points_cnt;
        elf_acc.yyzz_sum/=elf_acc.points_cnt;


        float val1[] = {  elf_acc.yyyy_sum ,elf_acc.yyzz_sum ,elf_acc.xyy_sum ,elf_acc.yyy_sum ,elf_acc.yyz_sum ,elf_acc.yy_sum,
                          elf_acc.yyzz_sum ,elf_acc.zzzz_sum ,elf_acc.xzz_sum ,elf_acc.yzz_sum ,elf_acc.zzz_sum ,elf_acc.zz_sum,
                          elf_acc.xyy_sum  ,elf_acc.xzz_sum  ,elf_acc.xx_sum  ,elf_acc.xy_sum  ,elf_acc.xz_sum  ,elf_acc.x_sum,
                          elf_acc.yyy_sum  ,elf_acc.yzz_sum  ,elf_acc.xy_sum  ,elf_acc.yy_sum  ,elf_acc.yz_sum  ,elf_acc.y_sum,
                          elf_acc.yyz_sum  ,elf_acc.zzz_sum  ,elf_acc.xz_sum  ,elf_acc.yz_sum  ,elf_acc.zz_sum  ,elf_acc.z_sum,
                          elf_acc.yy_sum   ,elf_acc.zz_sum   ,elf_acc.x_sum   ,elf_acc.y_sum   ,elf_acc.z_sum   ,1};
        float val2[] = { -elf_acc.xxyy_sum,
                         -elf_acc.xxzz_sum,
                         -elf_acc.xxx_sum,
                         -elf_acc.xxy_sum,
                         -elf_acc.xxz_sum,
                         -elf_acc.xx_sum};
        float val3[36];
        float val4[6];
        struct matrix_t mat_A0,mat_b,inv_mat_A0,mul_mat_A0_b;
        MATRIX_INIT(mat_A0, val1,6,6);
        MATRIX_INIT(mat_b,  val2,6,1);
        MATRIX_INIT(inv_mat_A0,val3,6,6);
        MATRIX_INIT(mul_mat_A0_b,val4,6,1);

        matrix_t_inv(&inv_mat_A0,&mat_A0);
        matrix_t_mul(&mul_mat_A0_b,&inv_mat_A0,&mat_b,1);
        uint8_t i;
        for(i=0;i<6;i++)
        {
            res_acc[i]=mul_mat_A0_b.m[i];
        }
        parameters.acc_self_offset.x=-res_acc[2]/2;
        parameters.acc_self_offset.y=-res_acc[3]/(2*res_acc[0]);
        parameters.acc_self_offset.z=-res_acc[4]/(2*res_acc[1]);
        parameters.acc_self_gain_inv.x= 1.0f/sqrt(parameters.acc_self_offset.x*parameters.acc_self_offset.x
                                        +res_acc[0]*parameters.acc_self_offset.y*parameters.acc_self_offset.y
                                        +res_acc[1]*parameters.acc_self_offset.z*parameters.acc_self_offset.z-res_acc[5]);

        parameters.acc_self_gain_inv.y=parameters.acc_self_gain_inv.x*sqrt(res_acc[0]);
        parameters.acc_self_gain_inv.z=parameters.acc_self_gain_inv.x*sqrt(res_acc[1]);
        rt_memset(&elf_acc,0,sizeof(elf));
        USV_State.AccOffsetOk=1;
        USV_State.SysOffsetOk=1;
        beta_count=0;//校准完成后使姿态快速收敛
        led_closed();
        param_write();
        QuaternionReset();
    }
}
static void GyroCorrection(void)
{
    static float gx=0,gy=0,gz=0;
    static uint16_t cnt=0;
    if(USV_State.GyroOffsetOk)//判断陀螺仪校准状态
        return;
    led_change_status(led_fun, gyro_offset_status);
    led_change_tick(led_fun, bright);
    led_change_status(led_sta, gyro_offset_status);
    led_change_tick(led_sta, medium_flash);

    if(cnt==0)
    {
        gx=0;
        gy=0;
        gz=0;
    }
    gx+=sensor.gyro_raw.x;
    gy+=sensor.gyro_raw.y;
    gz+=sensor.gyro_raw.z;
    cnt++;
    if(cnt>=300)
    {
        parameters.gyro_offset.x = gx/cnt;
        parameters.gyro_offset.y = gy/cnt;
        parameters.gyro_offset.z = gz/cnt;

        USV_State.GyroOffsetOk = 1;
        USV_State.SysOffsetOk=1;
        beta_count=0;//校准完成后使姿态快速收敛
        cnt=0;
        led_closed();
        param_write();
        QuaternionReset();
    }
}

void MagCorrection(void)
{
    if(USV_State.MagOffsetOk)//判断磁力计校准状态
        return;

    led_change_status(led_fun, mag_offset_status);
    led_change_tick(led_fun, bright);
    led_change_status(led_sta, mag_offset_status);
    led_change_tick(led_sta, medium_flash);

    if(fabs(sensor.gyro.x)*RaSpeedToAngSpeed<5
       ||fabs(sensor.gyro.y)*RaSpeedToAngSpeed<5
       ||fabs(sensor.gyro.z)*RaSpeedToAngSpeed<5)//没有旋转
        return;
    led_change_tick(led_sta, bright);
//    mavlink_msg_send();
    elf_mag.points_cnt++;
    elf_mag.x_sum+=sensor.mag_raw.x;
    elf_mag.y_sum+=sensor.mag_raw.y;
    elf_mag.z_sum+=sensor.mag_raw.z;
    elf_mag.xx_sum+=(1.0f*sensor.mag_raw.x*sensor.mag_raw.x);
    elf_mag.yy_sum+=(1.0f*sensor.mag_raw.y*sensor.mag_raw.y);
    elf_mag.zz_sum+=(1.0f*sensor.mag_raw.z*sensor.mag_raw.z);
    elf_mag.xy_sum+=(1.0f*sensor.mag_raw.x*sensor.mag_raw.y);
    elf_mag.xz_sum+=(1.0f*sensor.mag_raw.x*sensor.mag_raw.z);
    elf_mag.yz_sum+=(1.0f*sensor.mag_raw.y*sensor.mag_raw.z);

    elf_mag.xxx_sum+=(1.0f*sensor.mag_raw.x*sensor.mag_raw.x*sensor.mag_raw.x);
    elf_mag.xxy_sum+=(1.0f*sensor.mag_raw.x*sensor.mag_raw.x*sensor.mag_raw.y);
    elf_mag.xxz_sum+=(1.0f*sensor.mag_raw.x*sensor.mag_raw.x*sensor.mag_raw.z);
    elf_mag.xyy_sum+=(1.0f*sensor.mag_raw.x*sensor.mag_raw.y*sensor.mag_raw.y);
    elf_mag.xzz_sum+=(1.0f*sensor.mag_raw.x*sensor.mag_raw.z*sensor.mag_raw.z);
    elf_mag.yyy_sum+=(1.0f*sensor.mag_raw.y*sensor.mag_raw.y*sensor.mag_raw.y);
    elf_mag.yyz_sum+=(1.0f*sensor.mag_raw.y*sensor.mag_raw.y*sensor.mag_raw.z);
    elf_mag.yzz_sum+=(1.0f*sensor.mag_raw.y*sensor.mag_raw.z*sensor.mag_raw.z);
    elf_mag.zzz_sum+=(1.0f*sensor.mag_raw.z*sensor.mag_raw.z*sensor.mag_raw.z);

    elf_mag.yyyy_sum+=(1.0f*sensor.mag_raw.y*sensor.mag_raw.y*sensor.mag_raw.y*sensor.mag_raw.y);
    elf_mag.zzzz_sum+=(1.0f*sensor.mag_raw.z*sensor.mag_raw.z*sensor.mag_raw.z*sensor.mag_raw.z);
    elf_mag.xxyy_sum+=(1.0f*sensor.mag_raw.x*sensor.mag_raw.x*sensor.mag_raw.y*sensor.mag_raw.y);
    elf_mag.xxzz_sum+=(1.0f*sensor.mag_raw.x*sensor.mag_raw.x*sensor.mag_raw.z*sensor.mag_raw.z);
    elf_mag.yyzz_sum+=(1.0f*sensor.mag_raw.y*sensor.mag_raw.y*sensor.mag_raw.z*sensor.mag_raw.z);

    if(elf_mag.points_cnt>=5000)
    {
        elf_mag.x_sum/=elf_mag.points_cnt;
        elf_mag.y_sum/=elf_mag.points_cnt;
        elf_mag.z_sum/=elf_mag.points_cnt;

        elf_mag.xx_sum/=elf_mag.points_cnt;
        elf_mag.yy_sum/=elf_mag.points_cnt;
        elf_mag.zz_sum/=elf_mag.points_cnt;
        elf_mag.xy_sum/=elf_mag.points_cnt;
        elf_mag.xz_sum/=elf_mag.points_cnt;
        elf_mag.yz_sum/=elf_mag.points_cnt;

        elf_mag.xxx_sum/=elf_mag.points_cnt;
        elf_mag.xxy_sum/=elf_mag.points_cnt;
        elf_mag.xxz_sum/=elf_mag.points_cnt;
        elf_mag.xyy_sum/=elf_mag.points_cnt;
        elf_mag.xzz_sum/=elf_mag.points_cnt;
        elf_mag.yyy_sum/=elf_mag.points_cnt;
        elf_mag.yyz_sum/=elf_mag.points_cnt;
        elf_mag.yzz_sum/=elf_mag.points_cnt;
        elf_mag.zzz_sum/=elf_mag.points_cnt;

        elf_mag.yyyy_sum/=elf_mag.points_cnt;
        elf_mag.zzzz_sum/=elf_mag.points_cnt;
        elf_mag.xxyy_sum/=elf_mag.points_cnt;
        elf_mag.xxzz_sum/=elf_mag.points_cnt;
        elf_mag.yyzz_sum/=elf_mag.points_cnt;


        float val1[] = {  elf_mag.yyyy_sum ,elf_mag.yyzz_sum ,elf_mag.xyy_sum ,elf_mag.yyy_sum ,elf_mag.yyz_sum ,elf_mag.yy_sum,
                          elf_mag.yyzz_sum ,elf_mag.zzzz_sum ,elf_mag.xzz_sum ,elf_mag.yzz_sum ,elf_mag.zzz_sum ,elf_mag.zz_sum,
                          elf_mag.xyy_sum  ,elf_mag.xzz_sum  ,elf_mag.xx_sum  ,elf_mag.xy_sum  ,elf_mag.xz_sum  ,elf_mag.x_sum,
                          elf_mag.yyy_sum  ,elf_mag.yzz_sum  ,elf_mag.xy_sum  ,elf_mag.yy_sum  ,elf_mag.yz_sum  ,elf_mag.y_sum,
                          elf_mag.yyz_sum  ,elf_mag.zzz_sum  ,elf_mag.xz_sum  ,elf_mag.yz_sum  ,elf_mag.zz_sum  ,elf_mag.z_sum,
                          elf_mag.yy_sum   ,elf_mag.zz_sum   ,elf_mag.x_sum   ,elf_mag.y_sum   ,elf_mag.z_sum   ,1};
        float val2[] = { -elf_mag.xxyy_sum,
                         -elf_mag.xxzz_sum,
                         -elf_mag.xxx_sum,
                         -elf_mag.xxy_sum,
                         -elf_mag.xxz_sum,
                         -elf_mag.xx_sum};
        float val3[36];
        float val4[6];
        struct matrix_t mat_A0,mat_b,inv_mat_A0,mul_mat_A0_b;
        MATRIX_INIT(mat_A0, val1,6,6);
        MATRIX_INIT(mat_b,  val2,6,1);
        MATRIX_INIT(inv_mat_A0,val3,6,6);
        MATRIX_INIT(mul_mat_A0_b,val4,6,1);

        matrix_t_inv(&inv_mat_A0,&mat_A0);
        matrix_t_mul(&mul_mat_A0_b,&inv_mat_A0,&mat_b,1);
        uint8_t i;
        for(i=0;i<6;i++)
        {
            res_mag[i]=mul_mat_A0_b.m[i];
        }
        parameters.mag_offset.x=-res_mag[2]/2;
        parameters.mag_offset.y=-res_mag[3]/(2*res_mag[0]);
        parameters.mag_offset.z=-res_mag[4]/(2*res_mag[1]);
        parameters.mag_gain_inv.x= 1.0f/sqrt(parameters.mag_offset.x*parameters.mag_offset.x
                                        +res_mag[0]*parameters.mag_offset.y*parameters.mag_offset.y
                                        +res_mag[1]*parameters.mag_offset.z*parameters.mag_offset.z-res_mag[5]);

        parameters.mag_gain_inv.y=parameters.mag_gain_inv.x*sqrt(res_mag[0]);
        parameters.mag_gain_inv.z=parameters.mag_gain_inv.x*sqrt(res_mag[1]);
        rt_memset(&elf_mag,0,sizeof(elf));
        USV_State.MagOffsetOk=1;
        USV_State.SysOffsetOk=1;
        beta_count=0;//校准完成后使姿态快速收敛
        led_closed();
        param_write();
        QuaternionReset();
    }
}
void LeftRudderCorrection(float dt)
{
    static uint32_t cnt;
    static int16_t switch_f_last;
    uint32_t tick;

    if(USV_State.LeftRudderOk)
        return;
    if(switch_f_last==1000&&rocker.switchF==2000)
    {
        USV_State.LeftRudderOk=1;
        USV_State.SysOffsetOk=1;
        return;
    }
    switch_f_last=rocker.switchF;
    tick = 0.5/dt;//500ms写入flash一次
    cnt++;

    if(cnt%tick==0)
    {
        if(rocker.leftY>2000&rocker.leftY<1000)
            return;
        parameters.left_rudder_mid_value=rocker.leftY;
        param_write();
    }

}
void SensorCorrection(float dt)
{
    AccCorrection();    //加速度计六面校准
    GyroCorrection();       //陀螺仪校准
    MagCorrection();        //磁力计校准
    HorizCorrection();      //机体水平校准
    YawCorrection();      //偏航角校准

}




