/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-05-19     PC-COLD       the first version
 */
#include <include.h>
_NMEA_MSG GPS;

/**
 *******************************************************************************
 * @brief   十六进制格式字符串 转 十六进制格式整数 函数
 * @param   [in] *str 字符串指针
 * @param   [in] size 字符串个数
 * @param   [in] *result 转换结果缓存指针
 * @return  0--转换成功; 1--数据不在范围内;
 * @note
 *******************************************************************************
*/
uint8_t AsciiToHex(char *str, uint8_t size, uint8_t *result)
{
	unsigned char temp;

	for(*result = 0; size; size--, str++)
	{
		if(('9' >= *str) && (*str >='0') )
			temp = *str - '0';
		else if(('F' >= *str) && (*str >='A') )
			temp = *str - 'A' + 10;
		else if(('f' >= *str) && (*str >='a') )
			temp = *str - 'a' + 10;
		else
			return 1; //数据不在范围内
		*result |= temp<<((size-1)*4);
	}

	return 0; //转换成功
}


/**
 *******************************************************************************
 * @brief   NMEA 数据帧校验 函数
 * @param   [in] *buf 校验数据帧指针
 * @return  0--校验成功; 1--校验失败; 2--校验数据超长; 3--校验和错误 不是校验和
 * @note
 *******************************************************************************
 */
uint8_t NMEA0183_CheckSum(uint8_t *buf)
{
	unsigned char i;
	unsigned char chk, result;

	for(chk=buf[1], i=2; (buf[i]!='*')&&(i<255); i++)
	{
		chk ^= buf[i];
	}

	if( AsciiToHex((char*)(&buf[i+1]), 2, &result) )
		return 3; //校验和错误 不是校验和
	if(i>=255)
		return 2; //校验数据超长
	if(chk != result)
		return 1; //校验失败

	return 0; //校验成功
}
//获取第num个字符串偏移地址
//返回 偏移地址
char* GetNMEA_Pos(uint8_t*buf,uint8_t num)
{
	while(num)
	{
		if(*buf=='\0')
			num--;
		buf++;
	}
	return (char*)buf;
}
/**
 *******************************************************************************
 * @brief   逗号替换为空字符，并返回字符串个数
 * @param   [in] *buf 字符串指针
 * @param   [in] len 字符串长度
 * @return  [out]替换后的字符串个数
 * @note
 *******************************************************************************
*/
uint8_t NullCharReplaceComma(uint8_t*buf)
{
    uint8_t i=0;
    uint8_t len;
    int8_t strcnt=0;//字符串个数
    if(NMEA0183_CheckSum(buf))//校验失败
        return 0;
    len=strlen((char*)buf);
    for(i=0;i<len;i++)
    {
        if(buf[i]=='*')
            break;
        if(buf[i]==',')
        {
            buf[i]='\0';
            strcnt++;
        }
    }
    strcnt++;
    return strcnt;
}

void GPGSV_Analysis(uint8_t*buf)
{
    SatelliteInformation temp;
    SatelliteInformation temp1;
	uint8_t num;//本条报文包括卫星数目
	uint8_t cnt=0;
	uint8_t i;

	uint8_t strcnt=NullCharReplaceComma(buf);
	if(strcnt==0)////校验失败
	    return;
	num = strcnt/4 - 1;
	while(num--)
	{
	    GPS.GPGSV.NumOfVisSat = atoi(GetNMEA_Pos(buf,cnt*4+3));
		temp.Num = atoi(GetNMEA_Pos(buf,cnt*4+4));
		temp.Elevation = atoi(GetNMEA_Pos(buf,cnt*4+5));
		temp.Azimuth=atoi(GetNMEA_Pos(buf,cnt*4+6));
		temp.Sn=atoi(GetNMEA_Pos(buf,cnt*4+7));
		cnt++;
		for(i=0;i<12;i++)//由高到低排序
		{
			if(temp.Num >GPS.GPGSV.SatInfo[i].Num)
			{
				temp1.Num=GPS.GPGSV.SatInfo[i].Num;
				temp1.Elevation=GPS.GPGSV.SatInfo[i].Elevation;
				temp1.Azimuth=GPS.GPGSV.SatInfo[i].Azimuth;
				temp1.Sn=GPS.GPGSV.SatInfo[i].Sn;

				GPS.GPGSV.SatInfo[i].Num=temp.Num;
				GPS.GPGSV.SatInfo[i].Elevation=temp.Elevation;
				GPS.GPGSV.SatInfo[i].Azimuth=temp.Azimuth;
				GPS.GPGSV.SatInfo[i].Sn=temp.Sn;

				temp.Num=temp1.Num;
				temp.Elevation=temp1.Elevation;
				temp.Azimuth=temp1.Azimuth;
				temp.Sn=temp1.Sn;
			}
			else if(temp.Num==GPS.GPGSV.SatInfo[i].Num)
			{
                GPS.GPGSV.SatInfo[i].Num=temp.Num;
                GPS.GPGSV.SatInfo[i].Elevation=temp.Elevation;
                GPS.GPGSV.SatInfo[i].Azimuth=temp.Azimuth;
                GPS.GPGSV.SatInfo[i].Sn=temp.Sn;
				break;
			}
		}
	}
}
void GPGGA_Analysis(uint8_t *buf)
{
    float ftmp;
    uint32_t utmp;

    uint8_t strcnt=NullCharReplaceComma(buf);
    if(strcnt==0)////校验失败
        return;
    GPS.GPGGA.UTC.Time = atof(GetNMEA_Pos(buf,1))*1000;
    ftmp=atof(GetNMEA_Pos(buf,2));
    utmp=ftmp;
    utmp/=100;
    ftmp=(ftmp-utmp*100)/60;
    GPS.GPGGA.Latitude=utmp+ftmp;//纬度
    if(strcmp(GetNMEA_Pos(buf,3),"S")==0)//南纬
        GPS.GPGGA.Latitude=-GPS.GPGGA.Latitude;
    ftmp=atof(GetNMEA_Pos(buf,4));
    utmp=ftmp;
    utmp/=100;
    ftmp=(ftmp-utmp*100)/60;
    GPS.GPGGA.Longitude=utmp+ftmp;//经度
    if(strcmp(GetNMEA_Pos(buf,5),"W")==0)//西经
        GPS.GPGGA.Longitude=-GPS.GPGGA.Longitude;
    GPS.GPGGA.GPS_State=atoi(GetNMEA_Pos(buf,6));
    GPS.GPGGA.PosSolSatNum=atoi(GetNMEA_Pos(buf,7));
    GPS.GPGGA.HDOP=atof(GetNMEA_Pos(buf,8));
    GPS.GPGGA.Altitude=atof(GetNMEA_Pos(buf,9));
    GPS.GPGGA.Height=atof(GetNMEA_Pos(buf,11));
    GPS.GPGGA.DiffTime=atoi(GetNMEA_Pos(buf,13));
    GPS.GPGGA.DiffStationNum=atoi(GetNMEA_Pos(buf,14));
}
void GPRMC_Analysis(uint8_t *buf)
{
    float ftmp;
    uint32_t utmp;

    uint8_t strcnt=NullCharReplaceComma(buf);
    if(strcnt==0)////校验失败
        return;
    GPS.GPRMC.UTC.Time = atof(GetNMEA_Pos(buf,1))*1000;
    GPS.GPRMC.UTC.Date=atoi(GetNMEA_Pos(buf,9));
    ftmp=atof(GetNMEA_Pos(buf,3));
    utmp=ftmp;
    utmp/=100;
    ftmp=(ftmp-utmp*100)/60;
    GPS.GPRMC.Latitude=utmp+ftmp;//纬度
    if(strcmp(GetNMEA_Pos(buf,4),"S")==0)//南纬
        GPS.GPRMC.Latitude=-GPS.GPRMC.Latitude;
    ftmp=atof(GetNMEA_Pos(buf,5));
    utmp=ftmp;
    utmp/=100;
    ftmp=(ftmp-utmp*100)/60;
    GPS.GPRMC.Longitude=utmp+ftmp;//经度
    if(strcmp(GetNMEA_Pos(buf,6),"W")==0)//西经
        GPS.GPRMC.Longitude=-GPS.GPRMC.Longitude;
    GPS.GPRMC.Speed=atof(GetNMEA_Pos(buf,7));
    GPS.GPRMC.Course=atof(GetNMEA_Pos(buf,8));
    GPS.GPRMC.Declination=atof(GetNMEA_Pos(buf,10));
    if(strcmp(GetNMEA_Pos(buf,11),"W")==0)//西
        GPS.GPRMC.Declination=-GPS.GPRMC.Declination;

}
void GPVTG_Analysis(uint8_t *buf)
{
    uint8_t strcnt=NullCharReplaceComma(buf);
    if(strcnt==0)////校验失败
        return;
    GPS.GPVTG.Course=atof(GetNMEA_Pos(buf,1));
    GPS.GPVTG.Course1=atof(GetNMEA_Pos(buf,3));
    GPS.GPVTG.Speed=atof(GetNMEA_Pos(buf,5));
    GPS.GPVTG.Speed1=atof(GetNMEA_Pos(buf,7));
}
void GPFPD_Analysis(uint8_t *buf)
{
    uint8_t strcnt=NullCharReplaceComma(buf);
    if(strcnt==0)////校验失败
        return;
    GPS.GPFPD.Yaw=atof(GetNMEA_Pos(buf,3));
    GPS.GPFPD.Pitch=atof(GetNMEA_Pos(buf,4));
    GPS.GPFPD.Roll=atof(GetNMEA_Pos(buf,5));
    GPS.GPFPD.Latitude=atof(GetNMEA_Pos(buf,6));
    GPS.GPFPD.Longitude=atof(GetNMEA_Pos(buf,7));
    GPS.GPFPD.Altitude=atof(GetNMEA_Pos(buf,8));
    GPS.GPFPD.EastSpeed=atof(GetNMEA_Pos(buf,9));
    GPS.GPFPD.NorthSpeed=atof(GetNMEA_Pos(buf,10));
    GPS.GPFPD.SkySpeed=atof(GetNMEA_Pos(buf,11));
    GPS.GPFPD.BaseLineLength=atof(GetNMEA_Pos(buf,12));
    GPS.GPFPD.Ant1Num=atoi(GetNMEA_Pos(buf,13));
    GPS.GPFPD.Ant2Num=atoi(GetNMEA_Pos(buf,14));
    GPS.GPFPD.SolState=atoi(GetNMEA_Pos(buf,15));
}
void GPTRA_Analysis(uint8_t *buf)
{
    uint8_t strcnt=NullCharReplaceComma(buf);
    if(strcnt==0)////校验失败
        return;
    GPS.GPTRA.heading=atof(GetNMEA_Pos(buf, 2));
    GPS.GPTRA.pitch=atof(GetNMEA_Pos(buf, 3));
    GPS.GPTRA.roll=atof(GetNMEA_Pos(buf, 4));
    GPS.GPTRA.QF=atoi(GetNMEA_Pos(buf, 5));
    GPS.GPTRA.SatNum=atoi(GetNMEA_Pos(buf, 6));
    GPS.GPTRA.DiffDelay=atof(GetNMEA_Pos(buf, 7));
    GPS.GPTRA.StnID=atoi(GetNMEA_Pos(buf, 8));


}
void Heading_Analysis(uint8_t *buf)
{
    uint8_t strcnt=NullCharReplaceComma(buf);
    if(strcnt==0)////校验失败
        return;
    GPS.Headinga.Length=atof(GetNMEA_Pos(buf, 3));
    GPS.Headinga.heading=atof(GetNMEA_Pos(buf, 4));
    GPS.Headinga.pitch=atof(GetNMEA_Pos(buf, 5));
    GPS.Headinga.hdgstddev=atof(GetNMEA_Pos(buf, 7));
    GPS.Headinga.ptchstddev=atof(GetNMEA_Pos(buf, 8));
    GPS.Headinga.StnID=atoi(GetNMEA_Pos(buf, 9));
    GPS.Headinga.SVs=atoi(GetNMEA_Pos(buf, 10));
    GPS.Headinga.SolnSVs=atoi(GetNMEA_Pos(buf, 11));
    GPS.Headinga.Obs=atoi(GetNMEA_Pos(buf, 12));
    GPS.Headinga.Multi=atoi(GetNMEA_Pos(buf, 13));
    GPS.Headinga.ExtSolSta=atoi(GetNMEA_Pos(buf, 15));
    GPS.Headinga.SigMask=atoi(GetNMEA_Pos(buf, 17));


}
void KSXT_Analysis(uint8_t *buf)
{
    uint8_t strcnt=NullCharReplaceComma(buf);
    if(strcnt==0)////校验失败
        return;
    strcpy(GPS.KSXT.UTC.str,GetNMEA_Pos(buf, 1));
    GPS.KSXT.Longitude=atof(GetNMEA_Pos(buf, 2));
    GPS.KSXT.Latitude=atof(GetNMEA_Pos(buf, 3));
    GPS.KSXT.Altitude=atof(GetNMEA_Pos(buf, 4));
    GPS.KSXT.heading=atof(GetNMEA_Pos(buf, 5));
    GPS.KSXT.pitch=atof(GetNMEA_Pos(buf, 6));
    GPS.KSXT.TrackTure=atof(GetNMEA_Pos(buf, 7));
    GPS.KSXT.Vel=atof(GetNMEA_Pos(buf, 8));
    GPS.KSXT.Roll=atof(GetNMEA_Pos(buf, 9));
    GPS.KSXT.PosQual=atoi(GetNMEA_Pos(buf, 10));
    GPS.KSXT.HeadingQual=atoi(GetNMEA_Pos(buf, 11));
    GPS.KSXT.SsolnSvs=atoi(GetNMEA_Pos(buf, 12));
    GPS.KSXT.MsolnSvs=atoi(GetNMEA_Pos(buf, 13));
    GPS.KSXT.PosEast=atof(GetNMEA_Pos(buf, 14));
    GPS.KSXT.PosNorth=atof(GetNMEA_Pos(buf, 15));
    GPS.KSXT.PosUp=atof(GetNMEA_Pos(buf, 16));
    GPS.KSXT.VelEast=atof(GetNMEA_Pos(buf, 17));
    GPS.KSXT.VelNorth=atof(GetNMEA_Pos(buf,18));
    GPS.KSXT.VelUp=atof(GetNMEA_Pos(buf, 19));
    GPS.KSXT.MSNR=atoi(GetNMEA_Pos(buf, 20));
    GPS.KSXT.SSNR=atoi(GetNMEA_Pos(buf, 21));


}
void NMEA0183_Analysis(uint8_t *buf)
{
//    if(strstr((char*)buf,"$GPRMC")!=NULL)
//        GPRMC_Analysis(buf);
//    else if(strstr((char*)buf,"$GPFPD")!=NULL)
//        GPFPD_Analysis(buf);
//    else if(strstr((char*)buf,"$GPTRA")!=NULL)
//        GPTRA_Analysis(buf);
//    else if(strstr((char*)buf,"$GPGGA")!=NULL)
//        GPGGA_Analysis(buf);
//    else if(strstr((char*)buf,"#HEADINGA")!=NULL)
//        Heading_Analysis(buf);
    if(strstr((char*)buf,"$KSXT")!=NULL)
        KSXT_Analysis(buf);

}

