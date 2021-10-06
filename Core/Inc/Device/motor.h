/*
 * motor.h
 *
 *  Created on: Jan 26, 2021
 *      Author: 90797
 */

#ifndef INC_DEV_MOTOR_H_
#define INC_DEV_MOTOR_H_

#include "stm32f4xx_hal.h"

void Motor_Lock(void);
void Motor_PWMSet(uint32_t Motor1,uint32_t Motor2,uint32_t Motor3,uint32_t Motor4);

#endif /* INC_DEV_MOTOR_H_ */
