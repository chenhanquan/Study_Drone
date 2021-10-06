/*
 * Space.h
 *
 *  Created on: 2021年2月10日
 *      Author: 90797
 */

#ifndef INC_DATAFUSION_SPACE_H_
#define INC_DATAFUSION_SPACE_H_

#include "stm32f4xx_hal.h"
enum Axis_dir
{
	n2b,
	b2n
};

void Space_RotationMatrix(float *axis_body,float *axis_n,float *angle,enum Axis_dir dir);
void Space_RotationPlant(float *axis_body,float *axis_n,float Yaw_angle,enum Axis_dir dir);

#endif /* INC_DATAFUSION_SPACE_H_ */
