#ifndef INC_DEV_SPL_H_
#define INC_DEV_SPL_H_

#include "stm32f4xx_hal.h"


uint8_t SPL06_ReadID(uint8_t Sensor_Addr);
void SPL06_Init();
float SPL06_GetSensor1PressureData();
float SPL06_GetSensor2PressureData();
float SPL06_GetSensor1Height();
float SPL06_GetSensor2Height();


#endif
