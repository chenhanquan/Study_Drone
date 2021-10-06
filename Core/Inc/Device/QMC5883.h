#ifndef _QMC5883_H_
#define _QMC5883_H_

#include "stm32f4xx_hal.h"

void QMC5883_Init(void);
void QMC5883_Read(float *RawMagData);
uint8_t QMC5883_ReadID(void);

#endif /* INC_DEVICE_BMI088_H_ */

