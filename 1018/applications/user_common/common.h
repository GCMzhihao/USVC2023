/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-15     PC-COLD       the first version
 */
#ifndef APPLICATIONS_COMMON_H_
#define APPLICATIONS_COMMON_H_
#include <stdint.h>
typedef struct
{
                int16_t x;
                int16_t y;
                int16_t z;
}VectorInt16;

typedef struct
{
                float x;
                float y;
                float z;
}VectorFloat;


#endif /* APPLICATIONS_COMMON_H_ */
