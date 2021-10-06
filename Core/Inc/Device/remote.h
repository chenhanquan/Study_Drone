#ifndef __REMOTE_H__
#define __REMOTE_H__

#include "stm32f4xx_hal.h"

typedef struct
{
	uint16_t Ch1_Roll;    //通道1
	uint16_t Ch2_Pitch;   //通道2
	uint16_t Ch3_Thro;    //通道3
	uint16_t Ch4_Yaw;     //通道4
	uint16_t Ch5_SWA;     //通道5
	uint16_t Ch6_SWB;     //通道6
	uint16_t Ch7_SWC;     //通道7
	uint16_t Ch8_SWD;     //通道8
}Remote_TypeDef;

void PPM_Callback(TIM_TypeDef* TIMx);	//在外部中断回调函数中调用

void Remote_DataHandle(Remote_TypeDef* RemoteData);

#endif
