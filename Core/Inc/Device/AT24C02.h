#ifndef _AT24C02_H
#define _AT24C02_H

#include "stm32f4xx_hal.h"
#include <stdio.h>

void AT24C02_ByteWrite(uint8_t writeAddr ,uint8_t data);
void AT24C02_PageWrite(uint8_t WritePage,uint8_t* pBuffer);    //ҳд,0-31ҳ
uint8_t AT24C02_ByteCurrentRead(void);
void AT24C02_NByteRandomRead(uint8_t readAddr ,uint8_t len,uint8_t* pBuffer);
void AT24C02_NByteSequentialRead(uint8_t len,uint8_t* pBuffer);
uint8_t AT24C02_Test(void);//����,255λ 


void AT24C02_SaveFloatData(const float DataToSave,uint8_t pos);
float AT24C02_ReadFloatData(uint8_t pos);
void AT24C02_ReadNFloatData(const uint8_t pos,const uint8_t float_num,float *OutputData);

#endif
