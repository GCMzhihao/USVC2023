/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-18     PC-COLD       the first version
 */
#include <include.h>

_parameters parameters;

void param_read(void)
{
    w25q_ReadBuffers(0,(uint8_t*)&parameters,sizeof(_parameters));
}

void param_write(void)
{
    w25q_WriteBuffers(0,(uint8_t*)&parameters,sizeof(_parameters));
}

