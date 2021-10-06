/*
 * DataHandle.h
 *
 *  Created on: Apr 15, 2021
 *      Author: 90797
 */

#ifndef INC_FLITER_DATAHANDLE_H_
#define INC_FLITER_DATAHANDLE_H_

#include "stm32f4xx_hal.h"

typedef struct
{
	float Acc[3];
	// float Acc_ellipsoid[3];		
	float Gyro[3];
	float Mag[3];

}DataHandle_AccGyroMag_TypeDef;

void DataHandle_Init(void);
void DataHandle_GyroFilter(DataHandle_AccGyroMag_TypeDef *FilterData,const float *RawGyro,float *RotateGyro);
void DataHandle_AccFilter(DataHandle_AccGyroMag_TypeDef *FilterData,const float *RawAcc,float *RotateAcc);
void DataHandle_MagFilter(DataHandle_AccGyroMag_TypeDef *FilterData,const float *RawMag);

float DataHandle_PressureSensor1HeightFilter(const float PressureHeight);
// float DataHandle_PressureSensor2HeightFilter(const float PressureHeight);
float DataHandle_UltrasonicHeightFilter(const float UltrasonicHeight);
void DataHandle_UWB(const float Raw_UWB_X,const float Raw_UWB_Y,const float Angle_Yaw,float *Body_X,float *Body_Y);
void DataHandle_OpticalFlowFilter(const float *Raw_OpticalFlowData,float *Filter_OpticalFlowData);

// void DataHandle_RawGyroRotate(const float *RawGyro,float *RotateGyro);
// void DataHandle_RawAccRotate(const float *RawAcc,float *RotateAcc);

#endif /* INC_FLITER_DATAHANDLE_H_ */
