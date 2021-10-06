/**
  ******************************************************************************
  * @file    led.c
  * @author  CHEN HANQUAN
  * @version V2.0
  * @date    15-January-2021
  * @brief   �˺������ڿ���zinС���RGB�ƣ�ʹ��ʱ��Ҫ����Systick��ʱ����������һ
	*          ��ȫ�ּ�ʱ����uSystickCount���ڼ�ʱ��
  * @note    �˺��������޸ı��˵Ĵ��룬�������ļ����ļ�֮��Ĺ����ԡ��˺�����ʹ��
	*          ��ѯˢ��ʱ��ķ���������˸������ʹ����ʱ�ķ��������ϵͳЧ�ʡ�
	*
  ******************************************************************************
	                               ʹ�÷���
	******************************************************************************
	* ����һ��LED_Type�ṹ�壬���úõƵ���˸ʱ�䡢��ʽ����ɫ�����ṹ�崫��vLEDDisplay����
  */
#include "led.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

#define LED_PORT GPIOE
#define LED_RED GPIO_PIN_2
#define LED_GREEN GPIO_PIN_3
#define LED_BLUE GPIO_PIN_4

#define BLUE_OFF LED_PORT->BSRR |= LED_BLUE
#define BLUE_ON LED_PORT->BSRR |= (LED_BLUE << 16)

#define GREEN_OFF LED_PORT->BSRR |= LED_GREEN
#define GREEN_ON LED_PORT->BSRR |= (LED_GREEN << 16)

#define RED_OFF LED_PORT->BSRR |= LED_RED
#define RED_ON LED_PORT->BSRR |= (LED_RED << 16)

void LED_Display(enum mode LED_mode,enum color LED_color)
{
		RED_OFF;GREEN_OFF;BLUE_OFF;
		if(LED_mode==On)
		{
			switch(LED_color)
			{
				case RED:RED_ON; break;
				case GREEN: GREEN_ON; break;
				case BLUE: BLUE_ON; break;
				case CYAN: BLUE_ON;GREEN_ON; break;
				case PINK: RED_ON;BLUE_ON; break;
				case YELLOW: RED_ON;GREEN_ON; break;
				case WHITE: RED_ON;GREEN_ON;BLUE_ON; break;
				default: RED_ON; break;

			}
		}
}
