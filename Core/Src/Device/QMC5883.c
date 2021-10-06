#include "QMC5883.h"
#include "main.h"

#define QMC5883_Addr (0x0D<<1)

void QMC5883_I2C_Write_Port(const uint8_t Reg_Addr,const uint8_t data)
{
//	uint8_t Data[2];
//	Data[0]=Reg_Addr;
//	Data[1]=data;
//	HAL_I2C_Master_Transmit(&hi2c1, Dev_Addr, Data, 2,1);

	I2C_Software_Write1Byte(QMC5883_Addr,Reg_Addr,data);
}

void QMC5883_I2C_Read_Port(const uint8_t Reg_Addr,uint8_t *data,const uint8_t len)
{
//	uint8_t Data;
//	Data=Reg_Addr;
//	HAL_I2C_Master_Transmit(&hi2c1, Dev_Addr, &Data, 1,1);
//	HAL_I2C_Master_Receive(&hi2c1, Dev_Addr, data, len,1);

	I2C_Software_ReadNByte(QMC5883_Addr, Reg_Addr,len, data);
}

void QMC5883_Init(void)
{
	// QMC5883_I2C_Write_Port(0x0A,0x09);
	// HAL_Delay(1);
	QMC5883_I2C_Write_Port(0x0B,0x01);
	QMC5883_I2C_Write_Port(0x09,0x09);		//2G 512 100Hz
}

#define SCALE 2.0f
void QMC5883_Read(float *RawMagData)
{
	uint8_t QMC5883_data[6];
	QMC5883_I2C_Read_Port(0x00,QMC5883_data,6);
	
	// *RawMagData=((int16_t)((QMC5883_data[1]<<8) | QMC5883_data[0]))/32768.0f * SCALE;
	// *(RawMagData+1)=((int16_t)((QMC5883_data[3]<<8) | QMC5883_data[2]))/32768.0f * SCALE;
	// *(RawMagData+2)=((int16_t)((QMC5883_data[5]<<8) | QMC5883_data[4]))/32768.0f * SCALE;

	*RawMagData=((float)((int16_t)(QMC5883_data[1]<<8) | QMC5883_data[0]))/12000.0f;
	*(RawMagData+1)=((float)((int16_t)(QMC5883_data[3]<<8) | QMC5883_data[2]))/12000.0f;
	*(RawMagData+2)=((float)((int16_t)(QMC5883_data[5]<<8) | QMC5883_data[4]))/12000.0f;
	}
	

uint8_t QMC5883_ReadID(void)
{
	uint8_t ID;
	QMC5883_I2C_Read_Port(0x0D,&ID,1);
	return ID;
}