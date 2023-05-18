/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-21     D       the first version
 */
#include "nlink_linktrack_tagframe0.h"

#include "nlink_utils.h"

#include "Filter.h"
#include<include.h>

uint8_t RxBuffer[200];
_UWB_data UWB_data;
VectorFloat Uwb_Raw_Data;
VectorFloat Uwb_Filter_Data;

#pragma pack(1)
typedef struct
{
  uint8_t header[2];
  uint8_t id;
  uint8_t role;
  nint24_t pos_3d[3];
  nint24_t vel_3d[3];
  nint24_t dis_arr[8];
  float imu_gyro_3d[3];
  float imu_acc_3d[3];
  uint8_t reserved1[12];
  int16_t angle_3d[3];
  float quaternion[4];
  uint8_t reserved2[4];
  uint32_t local_time;
  uint32_t system_time;
  uint8_t reserved3[1];
  uint8_t eop_3d[3];
  uint16_t voltage;
  uint8_t reserved4[5];
  uint8_t check_sum;
} nlt_tagframe0_raw_t;
#pragma pack()

static nlt_tagframe0_raw_t g_frame;

uint8_t UnpackData(const uint8_t *data, size_t data_length)
{
  if (data_length < g_nlt_tagframe0.fixed_part_size ||
      data[0] != g_nlt_tagframe0.frame_header ||
      data[1] != g_nlt_tagframe0.function_mark)
    return 0;
  if (!NLINK_VerifyCheckSum(data, g_nlt_tagframe0.fixed_part_size))
    return 0;

  memcpy(&g_frame, data, g_nlt_tagframe0.fixed_part_size);
  g_nlt_tagframe0.result.role = g_frame.role;
  g_nlt_tagframe0.result.id = g_frame.id;
  g_nlt_tagframe0.result.local_time = g_frame.local_time;
  g_nlt_tagframe0.result.system_time = g_frame.system_time;
  g_nlt_tagframe0.result.voltage = g_frame.voltage / MULTIPLY_VOLTAGE;

  NLINK_TRANSFORM_ARRAY_INT24(g_nlt_tagframe0.result.pos_3d, g_frame.pos_3d,
                              MULTIPLY_POS)
  NLINK_TRANSFORM_ARRAY_INT24(g_nlt_tagframe0.result.vel_3d, g_frame.vel_3d,
                              MULTIPLY_VEL)
  NLINK_TRANSFORM_ARRAY_INT24(g_nlt_tagframe0.result.dis_arr, g_frame.dis_arr,
                              MULTIPLY_DIS)
  NLINK_TRANSFORM_ARRAY(g_nlt_tagframe0.result.imu_gyro_3d, g_frame.imu_gyro_3d,
                        1)
  NLINK_TRANSFORM_ARRAY(g_nlt_tagframe0.result.imu_acc_3d, g_frame.imu_acc_3d,
                        1)
  NLINK_TRANSFORM_ARRAY(g_nlt_tagframe0.result.quaternion, g_frame.quaternion,
                        1)
  NLINK_TRANSFORM_ARRAY(g_nlt_tagframe0.result.angle_3d, g_frame.angle_3d,
                        MULTIPLY_ANGLE)
  NLINK_TRANSFORM_ARRAY(g_nlt_tagframe0.result.eop_3d, g_frame.eop_3d,
                        MULTIPLY_EOP)

  return 1;
}

nlt_tagframe0_t g_nlt_tagframe0 = {.fixed_part_size = 128,
                                   .frame_header = 0x55,
                                   .function_mark = 0x01,
                                   .UnpackData = UnpackData};
void tagframe0_phrase(uint8_t ch)
{
    static uint8_t RxState = 0;
    static uint8_t _data_len = 128,_data_cnt = 0;

    if(RxState==0&&ch==0x55)
    {
        RxState=1;
        RxBuffer[0]=ch;
        _data_cnt=1;
    }
    else if(RxState==1&&ch==0x01)
    {
        RxState=2;
        RxBuffer[1]=ch;
        _data_cnt=2;
    }
    else if(RxState==2)
    {
        RxBuffer[_data_cnt]=ch;
        _data_cnt++;
        if(_data_cnt==_data_len)
        {
            if(UnpackData(RxBuffer, _data_len))
            {
                GetUwb_data();
            }
            RxState = 0;
            _data_cnt=0;
        }
    }
    else
        RxState = 0;
}

void GetUwb_data()
{
//    static uint32_t Tick=0;
//    uint32_t LastTick;
//    uint32_t dt=0;
//    LastTick=Tick;
//    Tick =  rt_tick_get();
//    dt = Tick-LastTick;

//    Uwb_Raw_Data.x = g_nlt_tagframe0.result.pos_3d[0];
//    Uwb_Raw_Data.y = g_nlt_tagframe0.result.pos_3d[1];
//    Uwb_Raw_Data.z = g_nlt_tagframe0.result.pos_3d[2];
//
//    SortAver_FilterXYZ(&Uwb_Raw_Data, &Uwb_Filter_Data, 6);
//    Aver_FilterXYZ(&Uwb_Filter_Data,&UWB_data.uwb_pos,6);


    UWB_data.uwb_pos.x = g_nlt_tagframe0.result.pos_3d[0];
    UWB_data.uwb_pos.y = g_nlt_tagframe0.result.pos_3d[1];
    UWB_data.uwb_pos.z = g_nlt_tagframe0.result.pos_3d[2];
}
