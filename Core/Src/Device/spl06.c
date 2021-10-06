#include "spl06.h"
#include "math.h"
#include "main.h"

#define SPL06_1_ADDR (0x77<<1)
#define SPL06_2_ADDR (0x77<<1)
#define kT 3670016.0f
#define kP 3670016.0f

static int16_t spl06_1_c0,spl06_1_c1,spl06_1_c01,spl06_1_c11,spl06_1_c20,spl06_1_c21,spl06_1_c30;
static int32_t spl06_1_c00,spl06_1_c10;

static int16_t spl06_2_c0,spl06_2_c1,spl06_2_c01,spl06_2_c11,spl06_2_c20,spl06_2_c21,spl06_2_c30;
static int32_t spl06_2_c00,spl06_2_c10;

static uint8_t SPL06_ReadByte(uint8_t Sensor_Addr,uint8_t regAddr);
static void SPL06_ReadNByte(uint8_t Sensor_Addr,uint8_t regAddr,uint8_t len,  uint8_t *data);
static void SPL06_ByteWrite(uint8_t Sensor_Addr,uint8_t writeAddr ,uint8_t data);
	

void SPL06_Init()
{
	uint8_t spl06_1_coe[18];
	uint8_t spl06_2_coe[18];

	SPL06_ByteWrite(SPL06_1_ADDR,0x0C,0x89);    //reset
	SPL06_ByteWrite(SPL06_2_ADDR,0x0C,0x89);    //reset
	HAL_Delay(100);
	
	SPL06_ReadNByte(SPL06_1_ADDR,0x10,18,spl06_1_coe);
	spl06_1_c0=(spl06_1_coe[0]<<4)|((spl06_1_coe[1]&0xF0)>>4);
	spl06_1_c0=(spl06_1_c0&0x0800)?(0xF000|spl06_1_c0):spl06_1_c0;
	spl06_1_c1=((spl06_1_coe[1]&0x0F)<<8)|spl06_1_coe[2];
	spl06_1_c1=(spl06_1_c1&0x0800)?(0xF000|spl06_1_c1):spl06_1_c1;
	spl06_1_c00=(spl06_1_coe[3]<<12)|(spl06_1_coe[4]<<4)|(spl06_1_coe[5]>>4);
	spl06_1_c10=((spl06_1_coe[5]&0x0F)<<16)|(spl06_1_coe[6]<<8)|(spl06_1_coe[7]);
	spl06_1_c00 = (spl06_1_c00&0x080000)?(0xFFF00000|spl06_1_c00):spl06_1_c00;
	spl06_1_c10 = (spl06_1_c10&0x080000)?(0xFFF00000|spl06_1_c10):spl06_1_c10;
	spl06_1_c01=(spl06_1_coe[8]<<8)|spl06_1_coe[9];
	spl06_1_c11=(spl06_1_coe[10]<<8)|spl06_1_coe[11];
	spl06_1_c20=(spl06_1_coe[12]<<8)|spl06_1_coe[13];
	spl06_1_c21=(spl06_1_coe[14]<<8)|spl06_1_coe[15];
	spl06_1_c30=(spl06_1_coe[16]<<8)|spl06_1_coe[17];

	SPL06_ReadNByte(SPL06_2_ADDR,0x10,18,spl06_2_coe);
	spl06_2_c0=(spl06_2_coe[0]<<4)|((spl06_2_coe[1]&0xF0)>>4);
	spl06_2_c0=(spl06_2_c0&0x0800)?(0xF000|spl06_2_c0):spl06_2_c0;
	spl06_2_c1=((spl06_2_coe[1]&0x0F)<<8)|spl06_2_coe[2];
	spl06_2_c1=(spl06_2_c1&0x0800)?(0xF000|spl06_2_c1):spl06_2_c1;
	spl06_2_c00=(spl06_2_coe[3]<<12)|(spl06_2_coe[4]<<4)|(spl06_2_coe[5]>>4);
	spl06_2_c10=((spl06_2_coe[5]&0x0F)<<16)|(spl06_2_coe[6]<<8)|(spl06_2_coe[7]);
	spl06_2_c00 = (spl06_2_c00&0x080000)?(0xFFF00000|spl06_2_c00):spl06_2_c00;
	spl06_2_c10 = (spl06_2_c10&0x080000)?(0xFFF00000|spl06_2_c10):spl06_2_c10;
	spl06_2_c01=(spl06_2_coe[8]<<8)|spl06_2_coe[9];
	spl06_2_c11=(spl06_2_coe[10]<<8)|spl06_2_coe[11];
	spl06_2_c20=(spl06_2_coe[12]<<8)|spl06_2_coe[13];
	spl06_2_c21=(spl06_2_coe[14]<<8)|spl06_2_coe[15];
	spl06_2_c30=(spl06_2_coe[16]<<8)|spl06_2_coe[17];
	
	SPL06_ByteWrite(SPL06_1_ADDR,0x06,0x72);    //4times oversampling  128time pre sec
	SPL06_ByteWrite(SPL06_1_ADDR,0x07,0xF2);    //4times oversampling  128time pre sec
	SPL06_ByteWrite(SPL06_1_ADDR,0x08,0x07);    //��������
	SPL06_ByteWrite(SPL06_1_ADDR,0x09,0x00);

	SPL06_ByteWrite(SPL06_2_ADDR,0x06,0x72);    //4times oversampling  128time pre sec
	SPL06_ByteWrite(SPL06_2_ADDR,0x07,0xF2);    //4times oversampling  128time pre sec
	SPL06_ByteWrite(SPL06_2_ADDR,0x08,0x07);    //��������
	SPL06_ByteWrite(SPL06_2_ADDR,0x09,0x00);
}

float SPL06_GetSensor1PressureData()
{
	uint8_t Sensor_RawData[6]={0};
	int32_t Pressure_RawData0,Temperature_RawData0;
	float Pressure_RawData,Temperature_RawData,Pressure_Output;//,Temperature;
	SPL06_ReadNByte(SPL06_1_ADDR,0x00,6,Sensor_RawData);
	Pressure_RawData0=(Sensor_RawData[0]<<16)|(Sensor_RawData[1]<<8)|(Sensor_RawData[2]);
	Temperature_RawData0=(Sensor_RawData[3]<<16)|(Sensor_RawData[4]<<8)|(Sensor_RawData[5]);
	
	Pressure_RawData0=(Pressure_RawData0&0x800000)?(0xFF000000|Pressure_RawData0):Pressure_RawData0;
	Temperature_RawData0=(Temperature_RawData0&0x800000)?(0xFF000000|Temperature_RawData0):Temperature_RawData0;
	
	Pressure_RawData=(float)Pressure_RawData0/kP;
	Temperature_RawData=(float)Temperature_RawData0/kT;
	Pressure_Output=spl06_1_c00+Pressure_RawData*(spl06_1_c10+Pressure_RawData*(spl06_1_c20+Pressure_RawData*spl06_1_c30))+Temperature_RawData*spl06_1_c01+Temperature_RawData*Pressure_RawData*(spl06_1_c11+Pressure_RawData*spl06_1_c21);
//	Temperature=c0*0.5f+c1*Temperature_RawData;
	return Pressure_Output;
}

float SPL06_GetSensor2PressureData()
{
	uint8_t Sensor_RawData[6]={0};
	int32_t Pressure_RawData0,Temperature_RawData0;
	float Pressure_RawData,Temperature_RawData,Pressure_Output;//,Temperature;
	SPL06_ReadNByte(SPL06_2_ADDR,0x00,6,Sensor_RawData);
	Pressure_RawData0=(Sensor_RawData[0]<<16)|(Sensor_RawData[1]<<8)|(Sensor_RawData[2]);
	Temperature_RawData0=(Sensor_RawData[3]<<16)|(Sensor_RawData[4]<<8)|(Sensor_RawData[5]);
	
	Pressure_RawData0=(Pressure_RawData0&0x800000)?(0xFF000000|Pressure_RawData0):Pressure_RawData0;
	Temperature_RawData0=(Temperature_RawData0&0x800000)?(0xFF000000|Temperature_RawData0):Temperature_RawData0;
	
	Pressure_RawData=(float)Pressure_RawData0/kP;
	Temperature_RawData=(float)Temperature_RawData0/kT;
	Pressure_Output=spl06_2_c00+Pressure_RawData*(spl06_2_c10+Pressure_RawData*(spl06_2_c20+Pressure_RawData*spl06_2_c30))+Temperature_RawData*spl06_2_c01+Temperature_RawData*Pressure_RawData*(spl06_2_c11+Pressure_RawData*spl06_2_c21);
	return Pressure_Output;
}

#define Pa2mmHg 0.0075006f
float SPL06_GetSensor1Height()
{
	float P,Height;
	P=SPL06_GetSensor1PressureData();
	P=P*Pa2mmHg;
	Height=-7924.0f*log(P/760.0f);
	return Height * 100.0f;    //cm
	
}

float SPL06_GetSensor2Height()
{
	float P,Height;
	P=SPL06_GetSensor2PressureData();
	P=P*Pa2mmHg;
	Height=-7924.0f*log(P/760.0f);
	return Height * 100.0f;    //cm
	
}


uint8_t SPL06_ReadID(uint8_t Sensor_Addr)
{
	uint8_t ID=0;
	ID=SPL06_ReadByte(Sensor_Addr,0x0D);
	return ID;
}


/** �����ݺ���������I2C�ӿ�
 * @param slaveAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for single bit value
 */
static uint8_t SPL06_ReadByte(uint8_t Sensor_Addr,uint8_t regAddr)
{
	uint8_t data;
	I2C_Software_Read1Byte(Sensor_Addr,regAddr,&data);
	return data;
}

static void SPL06_ReadNByte(uint8_t Sensor_Addr,uint8_t regAddr,uint8_t len,  uint8_t *data)
{
	I2C_Software_ReadNByte(Sensor_Addr,regAddr, len, data);
}

/**
 * @brief  д����������I2C�ӿ�
 * @param  slaveAddr : slave address SPL06_DEFAULT_ADDRESS
 * @param  pBuffer : pointer to the buffer  containing the data to be written to the SPL06.
 * @param  writeAddr : address of the register in which the data will be written
 * @return None
 */
static void SPL06_ByteWrite(uint8_t Sensor_Addr,uint8_t writeAddr ,uint8_t data)
{
	I2C_Software_Write1Byte(Sensor_Addr,writeAddr,data);
}
