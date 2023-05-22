/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-19     PC-COLD       the first version
 */
#ifndef APPLICATIONS_USER_DRIVERS_GPS_H_
#define APPLICATIONS_USER_DRIVERS_GPS_H_

//GPS NMEA-0183协议重要参数结构体定义
//卫星信息
typedef struct
{
 	uint8_t Num;		//卫星编号
	uint8_t Elevation;	//卫星仰角
	uint16_t Azimuth;	//卫星方位角
	uint8_t Sn;		//信噪比
}SatelliteInformation;

typedef struct
{
    uint8_t NumOfVisSat;//卫星总数
    SatelliteInformation SatInfo[12];//卫星信息
}_GPGSV;
//UTC时间信息

typedef struct
{
    uint32_t Time;
    uint32_t Date;
}_UTC;
typedef struct
{
    _UTC UTC;
    double Latitude;//纬度
    double Longitude;//经度
    uint8_t GPS_State;//GPS状态:0,未定位;1,非差分定位;2,差分定位;6,正在估算.
    uint8_t PosSolSatNum;//用于定位的卫星数,0~12.
    float HDOP;//水平经度因子0.5~99.9
    float Altitude;//海拔高度
    float Height;// 地球椭球面相对大地水准面的高度
    uint32_t DiffTime;//差分GPS信息，即差分时间（从最近一次接收到差分信号开始的秒数，如果不是差分定位将为空）
    uint16_t DiffStationNum;//差分站ID号0000 - 1023（前导位数不足则补0，如果不是差分定位将为空）
}_GPGGA;
typedef struct
{
    _UTC UTC;
    double Latitude;//纬度
    double Longitude;//经度
    float Speed;//地面速率(000.0~999.9节，前面的0也将被传输)
    float Course;//地面航向(000.0~359.9度，以真北为参考基准，前面的0也将被传输)
    float Declination;//磁偏角
}_GPRMC;

typedef struct
{
    float Course;// 以正北为参考基准的地面航向(000~359度，前面的0也将被传输)
    float Course1;//以磁北为参考基准的地面航向(000~359度，前面的0也将被传输)
    float Speed;//地面速率(000.0~999.9节，前面的0也将被传输)
    float Speed1;//地面速率(0000.0~1851.8公里/小时，前面的0也将被传输)
}_GPVTG;

typedef struct
{
    float Yaw;//偏航角0～360°
    float Pitch;//俯仰角-90～90°
    float Roll;//横滚角-180～180°
    double Latitude;//纬度
    double Longitude;//经度
    float Altitude;//海拔高度 m
    float EastSpeed;//东向速度，单位：m/s
    float NorthSpeed;//北向速度，单位：m/s
    float SkySpeed;//天向速度，单位：m/s
    float BaseLineLength;//基线长度，单位：m
    uint8_t Ant1Num;//天线1卫星数
    uint8_t Ant2Num;//天线2卫星数
    char SolState;//0：初始化；1：GPS位置、速度和航向有效；2：GPS位置和速度有效；3：纯惯性模式；11:GPS差分、速度和航向有效；12：GPS差分有效。
}_GPFPD;

typedef struct
{
    _GPGSV GPGSV;
    _GPGGA GPGGA;
    _GPRMC GPRMC;
    _GPVTG GPVTG;
    _GPFPD GPFPD;
}_NMEA_MSG;
extern _NMEA_MSG GPS;
void NMEA0183_Analysis(uint8_t *buf);

#endif /* APPLICATIONS_USER_DRIVERS_GPS_H_ */
