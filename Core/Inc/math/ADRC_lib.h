/*
 * ADRC_lib.h
 *
 *  Created on: Jan 27, 2021
 *      Author: 90797
 */

#ifndef INC_MATH_ADRC_LIB_H_
#define INC_MATH_ADRC_LIB_H_

#include "stm32f4xx_hal.h"

typedef struct ADRC_TD
{
	//需要修改
	float r;
	float h;
	float T;

	//不需要修改
	float x[2];
}ADRC_TD_TypeDef;

typedef struct ADRC_ESO
{
	//需要修改
	float b0;
	float h;

	float u;

	//不需要修改
	float z[3];
	float beta[3];

}ADRC_ESO_TypeDef;

void ADRC_TD_fhan(ADRC_TD_TypeDef *ADRC_TD,float input);
void ADRC_TD_fsun(ADRC_TD_TypeDef *ADRC_TD,float input);
float ADRC_ESO_Normal(ADRC_ESO_TypeDef *ADRC_ESO,float y);
float ADRC_ESO_PartFunction(ADRC_ESO_TypeDef *ADRC_ESO,float y,float PartFuntion);

#endif /* INC_MATH_ADRC_LIB_H_ */
