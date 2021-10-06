/*
 * MyDelay.c
 *
 *  Created on: 2021年2月9日
 *      Author: 90797
 */

#include "main.h"
#include "MyDelay.h"

extern TIM_HandleTypeDef htim2;
#define CLOCK_TIM htim2	//分频84  周期10000

uint32_t Int_Count = 0;
static uint32_t Tim_Count = 0;

uint32_t HAL_GetTick(void)
{
	Tim_Count=CLOCK_TIM.Instance->CNT;
  return 10*Int_Count+Tim_Count/1000;	//返回ms
}

uint32_t HAL_GetTick_us(void)
{
	Tim_Count=CLOCK_TIM.Instance->CNT;
  return 10000*Int_Count+Tim_Count;	//返回us
}

void HAL_Delay_us(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick_us();
  uint32_t wait = Delay;

  /* Add a freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }

  while((HAL_GetTick_us() - tickstart) < wait)
  {
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==(&CLOCK_TIM))
	{
		Int_Count++;
	}
}
