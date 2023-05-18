/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-21     D       the first version
 */
#ifndef APPLICATIONS_USER_APP_NLINK_LINKTRACK_TAGFRAME0_H_
#define APPLICATIONS_USER_APP_NLINK_LINKTRACK_TAGFRAME0_H_
#include <common.h>
typedef struct
{
    VectorFloat uwb_pos;//坐标
    float dis_to_base[8];//到各个基站的距离
    VectorFloat vel;//xyz速度,无z轴速度数据
}_UWB_data;

#ifdef __cplusplus
extern "C"
{
#endif
#include "nlink_typedef.h"

  typedef struct
  {
    linktrack_role_e role;
    uint8_t id;
    float pos_3d[3];
    float eop_3d[3];
    float vel_3d[3];
    float dis_arr[8];
    float imu_gyro_3d[3];
    float imu_acc_3d[3];
    float angle_3d[3];
    float quaternion[4];
    uint32_t local_time;
    uint32_t system_time;
    float voltage;
  } nlt_tagframe0_result_t;

  typedef struct
  {
    const size_t fixed_part_size;
    const uint8_t frame_header;
    const uint8_t function_mark;
    nlt_tagframe0_result_t result;

    uint8_t (*const UnpackData)(const uint8_t *data, size_t data_length);
  } nlt_tagframe0_t;

extern nlt_tagframe0_t g_nlt_tagframe0;
extern uint8_t RxBuffer[200];
extern _UWB_data UWB_data;
extern VectorFloat Uwb_Filter_Data;

#ifdef __cplusplus
}
#endif

uint8_t UnpackData(const uint8_t *data, size_t data_length);
void tagframe0_phrase(uint8_t ch);
void GetUwb_data();

#endif /* APPLICATIONS_USER_APP_NLINK_LINKTRACK_TAGFRAME0_H_ */
