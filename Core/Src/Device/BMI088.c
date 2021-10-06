/*
 * BMI088.c
 *
 *  Created on: Jun 3, 2021
 *      Author: 90797
 */
#include "BMI088.h"
#include "math.h"
#include "main.h"

#define Gyro_Addr (0x68<<1)
#define Acc_Addr (0x18<<1)


void BMI088_I2C_Write_Port(const uint8_t Dev_Addr,const uint8_t Reg_Addr,const uint8_t data)
{
//	uint8_t Data[2];
//	Data[0]=Reg_Addr;
//	Data[1]=data;
//	HAL_I2C_Master_Transmit(&hi2c1, Dev_Addr, Data, 2,1);

	I2C_Software_Write1Byte(Dev_Addr,Reg_Addr,data);
}

void BMI088_I2C_Read_Port(const uint8_t Dev_Addr,const uint8_t Reg_Addr,uint8_t *data,const uint8_t len)
{
//	uint8_t Data;
//	Data=Reg_Addr;
//	HAL_I2C_Master_Transmit(&hi2c1, Dev_Addr, &Data, 1,1);
//	HAL_I2C_Master_Receive(&hi2c1, Dev_Addr, data, len,1);

	I2C_Software_ReadNByte(Dev_Addr, Reg_Addr,len, data);
}

#define Gyro_scale 250
#define Acc_scale 3
void BMI088_Init()
{
	uint8_t res=1;
	while(res)
	{
		res=!BMI088_Read_ID();
		HAL_Delay(2);
	}

	//Gyro初始化
	BMI088_I2C_Write_Port(Gyro_Addr,0x0F,0x03);		//scale +-250 o/s
	//BMI088_I2C_Write_Port(Gyro_Addr,0x10,0x03);		//ODR 400Hz filter bandwidth 47Hz
	BMI088_I2C_Write_Port(Gyro_Addr,0x10,0x05);		//ODR 100Hz filter bandwidth 12Hz
	BMI088_I2C_Write_Port(Gyro_Addr,0x11,0x00);		//mode:normal
	//Acc初始化
	BMI088_I2C_Write_Port(Acc_Addr,0x40,0x88);		//ODR 100Hz filter bandwidth 10Hz
	BMI088_I2C_Write_Port(Acc_Addr,0x41,0x00);		//scale +-3g
	BMI088_I2C_Write_Port(Acc_Addr,0x7C,0x00);
	BMI088_I2C_Write_Port(Acc_Addr,0x7D,0x04);
}

void BMI088_Read_Gyro(float *RawGyroData)
{
	uint8_t sensor_data[6];
	float factor;
	factor=Gyro_scale/32768.0f;
	BMI088_I2C_Read_Port(Gyro_Addr,0x02,sensor_data,6);

	*RawGyroData=((int16_t)((sensor_data[1]<<8)|sensor_data[0]))*factor;
	*(RawGyroData+1)=((int16_t)((sensor_data[3]<<8)|sensor_data[2]))*factor;
	*(RawGyroData+2)=((int16_t)((sensor_data[5]<<8)|sensor_data[4]))*factor;
}

void BMI088_Read_Acc(float *RawAccData,float *SensorTime)
{
	uint8_t sensor_data[9];
	float factor;
	factor=Acc_scale/32768.0f;
	BMI088_I2C_Read_Port(Acc_Addr,0x12,sensor_data,9);

	*RawAccData=((int16_t)((sensor_data[1]<<8)|sensor_data[0]))*factor;
	*(RawAccData+1)=((int16_t)((sensor_data[3]<<8)|sensor_data[2]))*factor;
	*(RawAccData+2)=((int16_t)((sensor_data[5]<<8)|sensor_data[4]))*factor;

	*SensorTime = sensor_data[7]*10.0f;		//ms
}

uint8_t BMI088_Read_ID()
{
	uint8_t id_gyro=0,id_acc=0;
	BMI088_I2C_Read_Port(Gyro_Addr,0x00,&id_gyro,1);
	BMI088_I2C_Read_Port(Acc_Addr,0x00,&id_acc,1);

	if(id_gyro!=0&&id_acc!=0)
	{
		return 1;
	}
	return 0;
}
