/*
 * ANO_DT.c
 *
 *  Created on: 2021年2月5日
 *      Author: 90797
 */
#include "ANO_DT.h"
#include "main.h"

extern Drone_Type MyDrone;

extern UART_HandleTypeDef huart6;
#define Drone_USART huart6

#define DRONE_ADDR  0xFF
#define COMPUTER_ADDR  0xFA

//定义数据拆分
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

uint8_t FrameSendBuf_Gen[30];
uint8_t FrameSendBuf_AccGyro[20];

static void ANO_DT_SendDataPort(uint8_t	*DateToSend,uint8_t len);
static void ANO_DT_SendOneFrame(const uint8_t address ,const uint8_t id ,const uint8_t len,const uint8_t *data,uint8_t *SendBuf);

//uint8_t DataToSend[20];

void ANO_SendData_AccGyro()
{
	uint8_t DataToSend[20];
	//发送MPU6050数据
	float temp[6]={0};
	int16_t temp1;
	for(uint8_t i=0;i<3;i++){temp[i]=MyDrone.Filter_AccGyroMag.Acc[i]*100;temp[i+3]=MyDrone.Filter_AccGyroMag.Gyro[i];}
	// for(uint8_t i=0;i<3;i++){temp[i]=MyDrone.Raw_AccData[i]*100;temp[i+3]=MyDrone.Raw_GyroData[i];}
	// for(uint8_t i=0;i<3;i++){temp[i]=MyDrone.DataHandle_Raw_Acc_Rotate[i]*100.0f;temp[i+3]=MyDrone.Fliter_AccGyroMag.Gyro[i];}
	for(uint8_t i=0;i<6;i++)
	{
	temp1=(int16_t)temp[i];
	DataToSend[2*i]=BYTE0(temp1);
	DataToSend[2*i+1]=BYTE1(temp1);
	}
	DataToSend[12]=0;
	ANO_DT_SendOneFrame(DRONE_ADDR, SEND_ID_SENSOR_IMU,13,DataToSend,FrameSendBuf_AccGyro);

}

void ANO_SendData_Angle()
{
	uint8_t DataToSend[20];
	//发送无人机姿态角
	float imu_temp[3]={0};
	int16_t data_temp;
	imu_temp[0]=MyDrone.Fusion_IMUData.angle[0]*100.0f;    //100/57.2 ·¢ËÍµÄÎª»¡¶ÈÊý¾Ý 0.39
	imu_temp[1]=MyDrone.Fusion_IMUData.angle[1]*100.0f;
	imu_temp[2]=MyDrone.Fusion_IMUData.angle[2]*100.0f;
	data_temp = (int16_t)imu_temp[0];
	DataToSend[0]= BYTE0(data_temp);
	DataToSend[1]= BYTE1(data_temp);
	data_temp = (int16_t)imu_temp[1];
	DataToSend[2]= BYTE0(data_temp);
	DataToSend[3]= BYTE1(data_temp);
	data_temp = (int16_t)imu_temp[2];
	DataToSend[4]= BYTE0(data_temp);
	DataToSend[5]= BYTE1(data_temp);

	DataToSend[6]= 0;
	ANO_DT_SendOneFrame(DRONE_ADDR, SEND_ID_EULER,7,DataToSend,FrameSendBuf_Gen);
}
	//发送4元数
//	{
//		float temp[4]={0};
//		int16_t data_temp;
//		for(uint8_t i=0;i<4;i++){temp[i]=MyDrone.Drone_IMUData.quad[i]*10000;}
//		data_temp = (int16_t)temp[0];
//		DataToSend[0]= BYTE0(data_temp);
//		DataToSend[1]= BYTE1(data_temp);
//		data_temp = (int16_t)temp[1];
//		DataToSend[2]= BYTE0(data_temp);
//		DataToSend[3]= BYTE1(data_temp);
//		data_temp = (int16_t)temp[2];
//		DataToSend[4]= BYTE0(data_temp);
//		DataToSend[5]= BYTE1(data_temp);
//		data_temp = (int16_t)temp[3];
//		DataToSend[6]= BYTE0(data_temp);
//		DataToSend[7]= BYTE1(data_temp);
//
//		DataToSend[8]= 0;
//		ANO_DT_SendOneFrame(DRONE_ADDR, SEND_ID_QUAD,9,DataToSend);
//	}
void ANO_SendData_Mag()
{
	uint8_t DataToSend[20];
	//发送磁力计数据
	int16_t mag_temp[3]={0};
	// for(uint8_t i=0;i<3;i++){mag_temp[i]=MyDrone.Fliter_AccGyroMag.Mag[i]*100000;}
	for(uint8_t i=0;i<3;i++){mag_temp[i]=MyDrone.Raw_MagData[i]*10000;}
	for(uint8_t i=0;i<3;i++)
	{
		DataToSend[2*i]=BYTE0(mag_temp[i]);
		DataToSend[2*i+1]=BYTE1(mag_temp[i]);
	}
	for(uint8_t i=0;i<8;i++){DataToSend[i+6]=0;}
	ANO_DT_SendOneFrame(DRONE_ADDR,SEND_ID_SENSOR_COMPASS,14,DataToSend,FrameSendBuf_Gen);
}

void ANO_SendData_Altitude()
{
	// uint8_t DataToSend[20];
	// //发送高度数据
	// int32_t ALT_FU,ALT_ADD;
	// ALT_FU=MyDrone.Drone_AltitudeData.RelHeight;
	// ALT_ADD=MyDrone.Drone_AltitudeData.Z_Velocity;
	// DataToSend[0]=BYTE0(ALT_FU);
	// DataToSend[1]=BYTE1(ALT_FU);
	// DataToSend[2]=BYTE2(ALT_FU);
	// DataToSend[3]=BYTE3(ALT_FU);
	// DataToSend[4]=BYTE0(ALT_ADD);
	// DataToSend[5]=BYTE1(ALT_ADD);
	// DataToSend[6]=BYTE2(ALT_ADD);
	// DataToSend[7]=BYTE3(ALT_ADD);
	// DataToSend[8]=0;
	// ANO_DT_SendOneFrame(DRONE_ADDR,0x05,9,DataToSend,FrameSendBuf_Gen);
}

void ANO_SendData_PWM()
{
	uint8_t DataToSend[20];
	//发送PWM
	uint16_t temp1;
	temp1=MyDrone.Control.PWM[0];
	//temp1=TIM4->CCR1;
	DataToSend[0]=BYTE0(temp1);
	DataToSend[1]=BYTE1(temp1);
	temp1=MyDrone.Control.PWM[1];
	//temp1=TIM4->CCR2;
	DataToSend[2]=BYTE0(temp1);
	DataToSend[3]=BYTE1(temp1);
	temp1=MyDrone.Control.PWM[2];
	//temp1=TIM4->CCR3;
	DataToSend[4]=BYTE0(temp1);
	DataToSend[5]=BYTE1(temp1);
	temp1=MyDrone.Control.PWM[3];
	//temp1=TIM4->CCR4;
	DataToSend[6]=BYTE0(temp1);
	DataToSend[7]=BYTE1(temp1);

	ANO_DT_SendOneFrame(DRONE_ADDR,SEND_ID_CONTROL_PWM,8,DataToSend,FrameSendBuf_Gen);
}

void ANO_SendData_NAxisAcc()
{
	// uint8_t DataToSend[20];
	// //发送Space acc vel
	// float temp[3]={0};
	// int32_t temp1;
	// for(uint8_t i=0;i<3;i++){temp[i]=MyDrone.DataHandle_Acceleration_Axis_N[i]*100;}
	// for(uint8_t i=0;i<3;i++)
	// {
	// 	temp1=(int32_t)temp[i];
	// 	DataToSend[4*i]=BYTE0(temp1);
	// 	DataToSend[4*i+1]=BYTE1(temp1);
	// 	DataToSend[4*i+2]=BYTE2(temp1);
	// 	DataToSend[4*i+3]=BYTE3(temp1);
	// }
	// ANO_DT_SendOneFrame(DRONE_ADDR, 0xF1,12,DataToSend,FrameSendBuf_Gen);
}

void ANO_SendData_NAxisVel()
{
	// uint8_t DataToSend[20];
	// float temp[3]={0};
	// int32_t temp1;
	// for(uint8_t i=0;i<2;i++){temp[i]=MyDrone.Drone_XYPosition.RelVelocity[i]*100;}
	// for(uint8_t i=0;i<2;i++)
	// {
	// 	temp1=(int32_t)temp[i];
	// 	DataToSend[4*i]=BYTE0(temp1);
	// 	DataToSend[4*i+1]=BYTE1(temp1);
	// 	DataToSend[4*i+2]=BYTE2(temp1);
	// 	DataToSend[4*i+3]=BYTE3(temp1);
	// }
	// ANO_DT_SendOneFrame(DRONE_ADDR, 0xF2,8,DataToSend,FrameSendBuf_Gen);
}

void ANO_SendData_NAxisPos()
{
//	uint8_t DataToSend[20];
//	float temp[3]={0};
//	int32_t temp1;
//	for(uint8_t i=0;i<2;i++){temp[i]=MyDrone.Drone_XYPosition.RelDistance[i]*100;}
//	for(uint8_t i=0;i<2;i++)
//	{
//		temp1=(int32_t)temp[i];
//		DataToSend[4*i]=BYTE0(temp1);
//		DataToSend[4*i+1]=BYTE1(temp1);
//		DataToSend[4*i+2]=BYTE2(temp1);
//		DataToSend[4*i+3]=BYTE3(temp1);
//	}
//	ANO_DT_SendOneFrame(DRONE_ADDR, 0xF3,8,DataToSend,FrameSendBuf_Gen);
}

void ANO_SendData_UserData()
{
	uint8_t DataToSend[20];
	//发送所需数据
	float temp[5]={0};
	int32_t temp1;
	for(uint8_t i=0;i<5;i++){temp[i]=MyDrone.UserData[i]*100;}
	for(uint8_t i=0;i<5;i++)
	{
		temp1=(int32_t)temp[i];
		DataToSend[4*i]=BYTE0(temp1);
		DataToSend[4*i+1]=BYTE1(temp1);
		DataToSend[4*i+2]=BYTE2(temp1);
		DataToSend[4*i+3]=BYTE3(temp1);
	}
	ANO_DT_SendOneFrame(DRONE_ADDR, 0xF1,20,DataToSend,FrameSendBuf_Gen);
}

void ANO_SendData_UserData1()
{
	uint8_t DataToSend[20];
	//发送所需数据
	float temp[5]={0};
	int32_t temp1;
	for(uint8_t i=0;i<5;i++){temp[i]=MyDrone.UserData1[i]*100;}
	for(uint8_t i=0;i<5;i++)
	{
		temp1=(int32_t)temp[i];
		DataToSend[4*i]=BYTE0(temp1);
		DataToSend[4*i+1]=BYTE1(temp1);
		DataToSend[4*i+2]=BYTE2(temp1);
		DataToSend[4*i+3]=BYTE3(temp1);
	}
	ANO_DT_SendOneFrame(DRONE_ADDR, 0xF2,20,DataToSend,FrameSendBuf_Gen);
}

void ANO_SendData_UserData2()
{
	uint8_t DataToSend[20];
	//发送所需数据
	float temp[5]={0};
	int32_t temp1;
	for(uint8_t i=0;i<5;i++){temp[i]=MyDrone.UserData2[i]*100;}
	for(uint8_t i=0;i<5;i++)
	{
		temp1=(int32_t)temp[i];
		DataToSend[4*i]=BYTE0(temp1);
		DataToSend[4*i+1]=BYTE1(temp1);
		DataToSend[4*i+2]=BYTE2(temp1);
		DataToSend[4*i+3]=BYTE3(temp1);
	}
	ANO_DT_SendOneFrame(DRONE_ADDR, 0xF3,20,DataToSend,FrameSendBuf_Gen);
}

void ANO_SendData_UserData3()
{
	uint8_t DataToSend[20];
	//发送所需数据
	float temp[5]={0};
	int32_t temp1;
	for(uint8_t i=0;i<5;i++){temp[i]=MyDrone.UserData3[i]*100;}
	for(uint8_t i=0;i<5;i++)
	{
		temp1=(int32_t)temp[i];
		DataToSend[4*i]=BYTE0(temp1);
		DataToSend[4*i+1]=BYTE1(temp1);
		DataToSend[4*i+2]=BYTE2(temp1);
		DataToSend[4*i+3]=BYTE3(temp1);
	}
	ANO_DT_SendOneFrame(DRONE_ADDR, 0xF4,20,DataToSend,FrameSendBuf_Gen);
}

void ANO_SendData_Standard()
{
	// uint8_t DataToSend[20];
	// //发送所需数据
	// float temp[3]={0};
	// int32_t temp1;
	// for(uint8_t i=0;i<3;i++){temp[i]=MyDrone.standard[i]*100;}
	// for(uint8_t i=0;i<3;i++)
	// {
	// 	temp1=(int32_t)temp[i];
	// 	DataToSend[4*i]=BYTE0(temp1);
	// 	DataToSend[4*i+1]=BYTE1(temp1);
	// 	DataToSend[4*i+2]=BYTE2(temp1);
	// 	DataToSend[4*i+3]=BYTE3(temp1);
	// }
	// ANO_DT_SendOneFrame(DRONE_ADDR, 0xF6,12,DataToSend,FrameSendBuf_Gen);
}

static void ANO_DT_SendDataPort(uint8_t	*DateToSend,uint8_t len)
{
//	HAL_UART_Transmit(&Drone_USART,DateToSend, len, 2);

	// osThreadSuspendAll();
	HAL_UART_Transmit_DMA(&Drone_USART, DateToSend, len);
	// osThreadResumeAll();
}


///**
// * @brief  ·¢ËÍÒ»Ö¡º¯Êý
// * @param  µØÖ·£¬id£¬Êý¾Ý³¤¶È£¬Òª·¢ËÍµÄÊý¾Ý
// * @return None
// */
static void ANO_DT_SendOneFrame(const uint8_t address ,const uint8_t id ,const uint8_t len,const uint8_t *data,uint8_t *SendBuf)
{
	uint8_t sumcheck=0;    //ºÍÐ£Ñé
	uint8_t addcheck=0;    //¸½¼ÓÐ£Ñé
	*SendBuf = 0xAA;
	*(SendBuf+1) = address;
	*(SendBuf+2) = id;
	*(SendBuf+3) = len;
	for(uint8_t i=0;i<len;i++)    //ÔØÈëÊý¾Ý
	{
		*(SendBuf+4+i) = data[i];
	}
	for(uint8_t i=0;i<(len+4);i++)    //Ð£Ñé¼ÆËã
	{
		sumcheck += *(SendBuf+i);
		addcheck += sumcheck;
	}
	*(SendBuf+len+4)=sumcheck;
	*(SendBuf+len+5)=addcheck;
	ANO_DT_SendDataPort(SendBuf,(len+6));
}
