#ifndef __UWB_H__
#define __UWB_H__

#include "stm32f4xx_hal.h"

// void UWB_Init(void);
void UWB_IdleIT(void);
void UWB_GetData_DMA(void);
void UWB_GetData(int *X_Local,int *Y_Local);

#endif