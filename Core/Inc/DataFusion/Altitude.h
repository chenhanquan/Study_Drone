/*
 * Altitude.h
 *
 *  Created on: Apr 7, 2021
 *      Author: 90797
 */

#ifndef INC_DATAFUSION_ALTITUDE_H_
#define INC_DATAFUSION_ALTITUDE_H_

#include "stm32f4xx_hal.h"

typedef struct{
	float Fusion_Altitude;
	float Estimation_Velocity_Z;
	
}Altitude_TypeDef;

float Altitude_PressureSensorDataFusion(const float PressureSensor1,const float PressureSensor2);
void Altitude_GetRelAltitudeAndZVelocity(Altitude_TypeDef *Altitude_Struct,const float Acc_Z,const float Rel_Height_Pressure,const float Abs_Height_Ultrasonic,const float Origin_Height_Ultrasonic,const float *Drone_Angle);
void Altitude_GetRelAltitudeAndZVelocity_Ultrasonic(Altitude_TypeDef *Altitude_Struct,const float Acc_Z,const float Rel_Height_Ultrasonic);

#endif /* INC_DATAFUSION_ALTITUDE_H_ */
