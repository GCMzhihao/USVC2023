/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-07-15     PC-COLD       the first version
 */
#ifndef APPLICATIONS_AHRS_H_
#define APPLICATIONS_AHRS_H_
extern float AttMatrix[3][3];
void QuaternionReset(void);
void AHRSupdate(float dt);
void QuaternionUpdate(float dt);
extern float beta;
extern uint16_t beta_count;
#endif /* APPLICATIONS_AHRS_H_ */
