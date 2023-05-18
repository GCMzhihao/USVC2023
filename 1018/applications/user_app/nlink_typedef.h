/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-20     D       the first version
 */
#ifndef APPLICATIONS_USER_APP_NLINK_TYPEDEF_H_
#define APPLICATIONS_USER_APP_NLINK_TYPEDEF_H_

#ifdef __cplusplus
extern "C"
{
#endif
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

  typedef enum
  {
    LINKTRACK_ROLE_NODE,
    LINKTRACK_ROLE_ANCHOR,
    LINKTRACK_ROLE_TAG,
    LINKTRACK_ROLE_CONSOLE,
    LINKTRACK_ROLE_DT_MASTER,
    LINKTRACK_ROLE_DT_SLAVE,
    LINKTRACK_ROLE_MONITOR,
  } linktrack_role_e;

  typedef uint32_t id_t;

#ifdef __cplusplus
}
#endif

#endif /* APPLICATIONS_USER_APP_NLINK_TYPEDEF_H_ */
