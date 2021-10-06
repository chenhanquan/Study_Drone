/*
 * motor.c
 *
 *  Created on: Jan 26, 2021
 *      Author: 90797
 */
#include "motor.h"

#define PWM_MAX 999

//Tim配置为400Hz 计数值10000  航模占空比标准输出40%~80%
void Motor_PWMSet(uint32_t Motor1,uint32_t Motor2,uint32_t Motor3,uint32_t Motor4)
{
	if(Motor1 > PWM_MAX) Motor1=PWM_MAX;
	if(Motor2 > PWM_MAX) Motor2=PWM_MAX;
	if(Motor3 > PWM_MAX) Motor3=PWM_MAX;
	if(Motor4 > PWM_MAX) Motor4=PWM_MAX;

	TIM4->CCR1=(uint32_t)(Motor1+1000);
	TIM4->CCR2=(uint32_t)(Motor2+1000);
	TIM4->CCR3=(uint32_t)(Motor3+1000);
	TIM4->CCR4=(uint32_t)(Motor4+1000);

//	TIM3->CCR4=(uint32_t)(Motor1+3850);
//	TIM3->CCR3=(uint32_t)(Motor2+3850);
//	TIM3->CCR2=(uint32_t)(Motor3+3850);
//	TIM3->CCR1=(uint32_t)(Motor4+3850);

//	TIM3->CCR4=Motor1+3850;    //电机测试
//	TIM3->CCR3=Motor2+3850;
//	TIM3->CCR2=Motor3+3850;
//	TIM3->CCR1=Motor4+3850;


}

//电机锁定
void Motor_Lock()
{
	Motor_PWMSet(0,0,0,0);
}
