/*
 * IMU.h
 *
 *  Created on: Jan 26, 2021
 *      Author: 90797
 */

#ifndef INC_DATAFUSION_IMU_H_
#define INC_DATAFUSION_IMU_H_

#include "stm32f4xx_hal.h"

typedef struct
{
	float angle[3];		//0:Roll 1:Pitch 2:Yaw
	// float innerData[3];
}IMU_TypeDef;

void IMU_AccCalRollPitch(const float *acc,float *angle);
float IMU_MagCalYaw(const float *mag,const float *angle);
void IMU_Process(IMU_TypeDef *IMU_Data,const float *acc,const float *gyro,const float *mag);

#endif /* INC_DATAFUSION_IMU_H_ */
