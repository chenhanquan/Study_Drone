/*
 * Ultrasonic.c
 *
 *  Created on: 2021年2月23日
 *      Author: 90797
 */
#include "Ultrasonic.h"
#include "main.h"

extern UART_HandleTypeDef huart4;

#define Ultrasonic_UART huart4

uint8_t Ultrasonic_Buff[2]={0};


float Ultrasonic_GetDistance(void)
{
	uint8_t order=0x55;
	uint16_t res;
	HAL_UART_Receive_IT(&Ultrasonic_UART, Ultrasonic_Buff, 2);
	HAL_UART_Transmit(&Ultrasonic_UART,&order,1,1);
	res=((Ultrasonic_Buff[0]<<8)|Ultrasonic_Buff[1])*0.1;		//单位cm
	return res;
}


