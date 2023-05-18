/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-08     PC-COLD       the first version
 */
#ifndef APPLICATIONS_USER_COMMON_MYMATH_H_
#define APPLICATIONS_USER_COMMON_MYMATH_H_

#define intabs(x) ((x<0)?(-x):x)
#define fabs(x) ((x<0.0f)?(-x):x)
//#define MAX(a,b) (a>b?a:b)
//#define MIN(a,b) (a<b?a:b)
#define LIMIT( x,min,max ) ( ((x) <= (min)) ? (min) : ( ((x) > (max))? (max) : (x) ) )

#define my_pow(a) ((a)*(a))
#define safe_div(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator)/(denominator)) )
#define ABS(x) ( (x)>0?(x):-(x) )

float mysqrt(float x);
float mysin(float rad );
float mycos(float rad );
float myatan2(float y, float x);
float myasin(float x);
float myatan2_t(float y,float x);
#endif /* APPLICATIONS_USER_COMMON_MYMATH_H_ */
