/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//系统�??
#include "MyDelay.h"
#include "cmsis_os.h"
//设备文件
#include "led.h"
#include "remote.h"
// #include "BMI055.h"
#include "QMC5883.h"
#include "motor.h"
#include "Ultrasonic.h"
#include "AT24C02.h"
#include "spl06.h"
#include "OpticalFlow.h"
// #include "mpu6050.h"
#include "BMI088.h"

//通信文件
#include "I2C_Software.h"
#include "ANO_DT.h"

//数学�??
#include "pid.h"
#include "ADRC_lib.h"
#include "NonlinearFunction.h"
#include "Space.h"

//数据处理
#include "DataHandle.h"
// #include "FIRFilter.h"
#include "Altitude.h"
#include "IMU.h"
#include "UWB.h"
#include "UWB_Position.h"
#include "Fusion_OpticalFlow.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum Drone_Mode
{
	Lock,
	Normal,
	Altitude,
	Space_Control,
	Calibration
};

typedef struct
{
  //无人机模�??
	enum Drone_Mode Mode;
  //原始数据
	float Raw_GyroData[3];
	float Raw_AccData[3];
	float Raw_MagData[3];
	float Raw_PressureHeight1;
	float Raw_PressureHeight2;
	float Raw_UltrasonicHeight;
	int Raw_UWB_Pos_X;
	int Raw_UWB_Pos_Y;
	float Raw_OpticalFlowData[2];
	Remote_TypeDef Raw_RemoteData;

  //滤波处理数据
	DataHandle_AccGyroMag_TypeDef Filter_AccGyroMag;
	float Filter_PressureHeightSensor1;
	float Filter_PressureHeightSensor2;
	float Filter_UltrasonicHeight;
	float Filter_UWB_Pos[2];
	float Filter_Acceleration_Axis_Body_Plane[2];
	float Filter_Acceleration_Axis_Z;
	float Filter_OpticalFlowData[2];

  //融合数据
	IMU_TypeDef Fusion_IMUData;
	float Fusion_PressureAltitude;
	Altitude_TypeDef Fusion_AltitudeData;
	UWB_Position_TypeDef Fusion_Position_UWB;
	float Fusion_Est_v_OpticalFlow[2];

	//修正处理数据
	float DataHandle_Yaw_Angle;
	float DataHandle_Height_Ultrasonic;
	float DataHandle_Acceleration_Axis_Z;
	//   float DataHandle_Acceleration_Axis_Body_Plane[2];
	float DataHandle_Raw_Gyro_Rotate[3];
	float DataHandle_Raw_Acc_Rotate[3];

	//原点数据
	float Origin_Height_Pressure;
	float Origin_Height_Ultrasonic;

	float Drone_Raw_Position[2];
	struct Control_Info
	{
		uint32_t Thro;
		uint32_t PWM[4];
	}Control;
	struct Target_Info
	{
		float Roll;
		float Pitch;
		float Yaw;
		float Yaw_gyro;
		float Altitude_target;
		float Vel_X;
		float Vel_Y;
		// float Drone_North;
		// float Drone_East;
		// float BodyXY[2];
		// float Position_UWB[2];
	}Target;
	float UserData[5];
	float UserData1[5];
	float UserData2[5];
	float UserData3[5];
	float standard[3];
}Drone_Type;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Drone_Init(void);
void Drone_ModeControl(void);
void Drone_DataHandle(void);
void Drone_TargetSet(void);
void Drone_Control_100Hz(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Drone_USART1 huart1
#define Drone_USART6 huart6
#define LED_RED_Pin GPIO_PIN_2
#define LED_RED_GPIO_Port GPIOE
#define LED_GREEN_Pin GPIO_PIN_3
#define LED_GREEN_GPIO_Port GPIOE
#define LED_BLUE_Pin GPIO_PIN_4
#define LED_BLUE_GPIO_Port GPIOE
#define PPM_Pin GPIO_PIN_15
#define PPM_GPIO_Port GPIOE
#define PPM_EXTI_IRQn EXTI15_10_IRQn
#define I2C_SCL_Pin GPIO_PIN_6
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_7
#define I2C_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
