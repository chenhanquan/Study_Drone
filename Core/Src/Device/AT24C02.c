#include "AT24C02.h"
#include "main.h"
#include "I2C_Software.h"

#define AT24C02_ADDR 0xA0  //AT24c02  IIC��ַַ

uint8_t AT24C02_ByteCurrentRead()
{
	uint8_t temp=0;
	I2C_Software_Receive(AT24C02_ADDR, 1, &temp);
	return temp;
}

void AT24C02_NByteRandomRead(uint8_t readAddr ,uint8_t len,uint8_t* pBuffer)
{
	I2C_Software_ReadNByte(AT24C02_ADDR, readAddr,len, pBuffer);
}

void AT24C02_NByteSequentialRead(uint8_t len,uint8_t* pBuffer)
{
	I2C_Software_Receive(AT24C02_ADDR, len, pBuffer);
}


uint8_t AT24C02_Test(void)//����
{
	uint8_t test = 0;
	AT24C02_ByteWrite(0xFF,0xAA);
	AT24C02_NByteRandomRead(0xFF,1,&test);
	if(test != 0xAA) //�洢���쳣
		return 0;
	else
		return 1;
}
	


/**
 * @brief  д����������I2C�ӿ�
 * @param  slaveAddr : slave address MPU6050_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer  containing the data to be written to the MPU6050.
 * @param  writeAddr : address of the register in which the data will be written
 * @return None
 */
void AT24C02_ByteWrite(uint8_t writeAddr ,uint8_t data)
{
	I2C_Software_Write1Byte(AT24C02_ADDR, writeAddr,data);
	HAL_Delay(2);
}

void AT24C02_PageWrite(uint8_t WritePage,uint8_t* pBuffer)
{
	I2C_Software_WriteNByte(AT24C02_ADDR, 8*WritePage, 8, pBuffer);
	HAL_Delay(5);
}

void AT24C02_SaveFloatData(const float DataToSave,uint8_t pos)
{
	uint8_t *point_temp;
	uint8_t i=0;
	point_temp=(uint8_t*)&DataToSave;
	
	for(i=0;i<4;i++)
	{
		AT24C02_ByteWrite(pos+i ,*(point_temp+i));
	}
	
}

float AT24C02_ReadFloatData(uint8_t pos)
{
	uint8_t ReadDataBuf[4]={0};
	float OutputData=0;
	AT24C02_NByteRandomRead(pos,4,ReadDataBuf);
	OutputData=*((float*)ReadDataBuf);
	return OutputData;
}

void AT24C02_ReadNFloatData(const uint8_t pos,const uint8_t float_num,float *OutputData)
{
	uint8_t ReadDataBuf[4 * float_num];
	AT24C02_NByteRandomRead(pos,4*float_num,ReadDataBuf);
	for(uint8_t i=0;i<float_num;i++)
	{
		OutputData[i]=*(((float*)ReadDataBuf)+i);
	}
}


