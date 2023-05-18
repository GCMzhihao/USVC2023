#include "Filter.h"

#define PI 3.141592653f

void _1st_lpf(float X,float *Y,float Hz,float dt)
{
	float k;
	k=2*PI*dt*Hz;
	*Y=k*(X)+(1-k)*(*Y);
}


/*******************************************************************************
* 函  数 ：float FindPos(float*a,int low,int high)
* 功  能 ：确定一个元素位序
* 参  数 ：a  数组首地址
*          low数组最小下标
*          high数组最大下标
* 返回值 ：返回元素的位序low
* 备  注 : 无
*******************************************************************************/
float FindPos(float*a,int low,int high)
{
    float val = a[low];                      //选定一个要确定值val确定位置
    while(low<high)
    {
        while(low<high && a[high]>=val)
             high--;                       //如果右边的数大于VAL下标往前移
             a[low] = a[high];             //当右边的值小于VAL则复值给A[low]

        while(low<high && a[low]<=val)
             low++;                        //如果左边的数小于VAL下标往后移
             a[high] = a[low];             //当左边的值大于VAL则复值给右边a[high]
    }
    a[low] = val;
    return low;
}

/*******************************************************************************
* 函  数 ：void QuiteSort(float* a,int low,int high)
* 功  能 ：快速排序
* 参  数 ：a  数组首地址
*          low数组最小下标
*          high数组最大下标
* 返回值 ：无
* 备  注 : 无
*******************************************************************************/
 void QuiteSort(float* a,int low,int high)
 {
     int pos;
     if(low<high)
     {
         pos = FindPos(a,low,high); //排序一个位置
         QuiteSort(a,low,pos-1);    //递归调用
         QuiteSort(a,pos+1,high);
     }
 }

 /*******************************************************************************
 * 函  数 ：float  SortAver_Filter1(float value)
 * 功  能 ：去最值平均值滤波一组数据
 * 参  数 ：value 采样的数据
 *          *filter 滤波以后的数据地址
 * 返回值 ：无
 * 备  注 : 无
 *******************************************************************************/
 void  SortAver_Filter1(float value,float *filter,unsigned int n)
 {
     static float buf[20];
     static unsigned int cnt =0 ,flag = 1;
     float temp=0;
     unsigned int i;
     buf[cnt++] = value;
     if(cnt<n && flag)
         return;   //数组填不满不计算
     else
         flag = 0;
     QuiteSort(buf,0,n-1);
     for(i=1;i<n-1;i++)
      {
         temp += buf[i];
      }
     if(cnt>=n) cnt = 0;

     *filter = temp/(n-2);

 }

 /*******************************************************************************
 * 函  数 ：void Aver_Filter1(float data,float *filt_data,uint8_t n
 * 功  能 ：滑动窗口滤波一组数据
 * 参  数 ：data  要滤波数据
 *          *filt_data 滤波后数据地址
 * 返回值 ：返回滤波后的数据
 * 备  注 : 无
 *******************************************************************************/
 void Aver_Filter1(float data,float *filt_data,unsigned int n)
 {
   static float buf[20];
     static unsigned int cnt =0,flag = 1;
     float temp=0;
     unsigned int i;
     buf[cnt++] = data;
     if(cnt<n && flag)
         return;   //数组填不满不计算
     else
         flag = 0;
     for(i=0;i<n;i++)
     {
         temp += buf[i];
     }
     if(cnt>=n) cnt = 0;
      *filt_data = temp/n;
 }

 /********************************************************************************
 * 函  数 ：void  SortAver_FilterXYZ(INT16_XYZ *acc,FLOAT_XYZ *Acc_filt,uint8_t n)
 * 功  能 ：去最值平均值滤波三组数据
 * 参  数 ：*acc 要滤波数据地址
 *          *Acc_filt 滤波后数据地址
 * 返回值 ：无
 * 备  注 : 无
 ********************************************************************************/
 void  SortAver_FilterXYZ(VectorFloat *acc,VectorFloat *Acc_filt,uint8_t n)
 {
     static float bufx[20],bufy[20],bufz[20];
     static uint8_t cnt =0,flag = 1;
     float temp1=0,temp2=0,temp3=0;
     uint8_t i;
     bufx[cnt] = acc->x;
     bufy[cnt] = acc->y;
     bufz[cnt] = acc->z;
     cnt++;      //这个的位置必须在赋值语句后，否则bufx[0]不会被赋值
     if(cnt<n && flag)
         return;   //数组填不满不计算
     else
         flag = 0;

     QuiteSort(bufx,0,n-1);
     QuiteSort(bufy,0,n-1);
     QuiteSort(bufz,0,n-1);
     for(i=1;i<n-1;i++)
      {
         temp1 += bufx[i];
         temp2 += bufy[i];
         temp3 += bufz[i];
      }

      if(cnt>=n) cnt = 0;
      Acc_filt->x  = temp1/(n-2);
      Acc_filt->y  = temp2/(n-2);
      Acc_filt->z  = temp3/(n-2);
 }


 /*******************************************************************************
 * 函  数 ：void Aver_FilterXYZ(INT16_XYZ *acc,FLOAT_XYZ *Acc_filt,uint8_t n)
 * 功  能 ：滑动窗口滤波三组数据
 * 参  数 ：*acc  要滤波数据地址
 *          *Acc_filt 滤波后数据地址
 * 返回值 ：无
 * 备  注 : 无
 *******************************************************************************/
 void Aver_FilterXYZ(VectorFloat *acc,VectorFloat *Acc_filt,uint8_t n)
 {
     static float bufax[20],bufay[20],bufaz[20];
     static uint8_t cnt =0,flag = 1,i;
     float temp1=0,temp2=0,temp3=0;
     bufax[cnt] = acc->x;
     bufay[cnt] = acc->y;
     bufaz[cnt] = acc->z;
     cnt++;      //这个的位置必须在赋值语句后，否则bufax[0]不会被赋值
     if(cnt<n && flag)
         return;   //数组填不满不计算
     else
         flag = 0;
     for(i=0;i<n;i++)
     {
         temp1 += bufax[i];
         temp2 += bufay[i];
         temp3 += bufaz[i];
     }
      if(cnt>=n)  cnt = 0;
      Acc_filt->x  = temp1/n;
      Acc_filt->y  = temp2/n;
      Acc_filt->z  = temp3/n;
 }

