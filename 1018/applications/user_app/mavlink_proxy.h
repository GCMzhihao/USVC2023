/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-12-31     PC-COLD       the first version
 */
#ifndef APPLICATIONS_USER_APP_MAVLINK_PROXY_H_
#define APPLICATIONS_USER_APP_MAVLINK_PROXY_H_

extern mavlink_rocker_t rocker;
extern uint8_t dev_id;
extern uint8_t sys_id;
void mavlink_msg_proxy(mavlink_message_t *msg , mavlink_status_t* status);
void mavlink_msg_send(void);
#endif /* APPLICATIONS_USER_APP_MAVLINK_PROXY_H_ */
