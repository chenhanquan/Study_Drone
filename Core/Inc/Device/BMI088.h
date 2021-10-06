/*
 * BMI088.h
 *
 *  Created on: Jun 3, 2021
 *      Author: 90797
 */

#ifndef INC_DEVICE_BMI088_H_
#define INC_DEVICE_BMI088_H_

#include "stm32f4xx_hal.h"

void BMI088_I2C_Write_Port(const uint8_t Dev_Addr,const uint8_t Reg_Addr,const uint8_t data);
void BMI088_I2C_Read_Port(const uint8_t Dev_Addr,const uint8_t Reg_Addr,uint8_t *data,const uint8_t len);
void BMI088_Init(void);
void BMI088_Read_Gyro(float *RawGyroData);
void BMI088_Read_Acc(float *RawAccData,float *SensorTime);
uint8_t BMI088_Read_ID();


#endif /* INC_DEVICE_BMI088_H_ */
