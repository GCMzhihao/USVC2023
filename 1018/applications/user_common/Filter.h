#ifndef _FILTER_H__
#define _FILTER_H__
#include <common.h>

void _1st_lpf(float X,float *Y,float Hz,float dt);

void  SortAver_Filter1(float value,float *filter,unsigned int n);
void QuiteSort(float* a,int low,int high);
float FindPos(float*a,int low,int high);
void Aver_Filter1(float data,float *filt_data,unsigned int n);
void Aver_FilterXYZ(VectorFloat *acc,VectorFloat *Acc_filt,uint8_t n);
void  SortAver_FilterXYZ(VectorFloat *acc,VectorFloat *Acc_filt,uint8_t n);
#endif
