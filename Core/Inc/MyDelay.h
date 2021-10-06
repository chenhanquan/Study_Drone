/*
 * MyDelay.h
 *
 *  Created on: 2021年2月9日
 *      Author: 90797
 */

#ifndef INC_MYDELAY_H_
#define INC_MYDELAY_H_

uint32_t HAL_GetTick(void);
uint32_t HAL_GetTick_us(void);
void HAL_Delay_us(uint32_t Delay);

#endif /* INC_MYDELAY_H_ */
