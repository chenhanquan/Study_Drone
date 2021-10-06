/*
 * ADRC_lib.c
 *
 *  Created on: Jan 27, 2021
 *      Author: 90797
 */

#include "ADRC_lib.h"
#include "NonlinearFunction.h"
#include "math.h"

void ADRC_TD_fhan(ADRC_TD_TypeDef *ADRC_TD,float input)
{
	float u;
	u=fhan(ADRC_TD->x[0]-input,ADRC_TD->x[1],ADRC_TD->r,ADRC_TD->h);
	ADRC_TD->x[0]=ADRC_TD->x[0]+ADRC_TD->T*ADRC_TD->x[1];
	ADRC_TD->x[1]=ADRC_TD->x[1]+ADRC_TD->T*u;
}

void ADRC_TD_fsun(ADRC_TD_TypeDef *ADRC_TD,float input)
{
	float u;
	u=fsun(ADRC_TD->x[0]-input,ADRC_TD->x[1],ADRC_TD->r,ADRC_TD->h);
	ADRC_TD->x[0]=ADRC_TD->x[0]+ADRC_TD->T*ADRC_TD->x[1];
	ADRC_TD->x[1]=ADRC_TD->x[1]+ADRC_TD->T*u;
}

float ADRC_ESO_Normal(ADRC_ESO_TypeDef *ADRC_ESO,float y)
{
	float Err=0,fe1=0,fe2=0;;
	if(ADRC_ESO->beta[0]==0||ADRC_ESO->beta[1]==0||ADRC_ESO->beta[2]==0)
	{
		ADRC_ESO->beta[0]=1.0f/ADRC_ESO->h;
		ADRC_ESO->beta[1]=1.186f/pow(ADRC_ESO->h,1.47f);
		ADRC_ESO->beta[2]=0.283f/pow(ADRC_ESO->h,2.21f);
	}
	Err=ADRC_ESO->z[0]-y;
	fe1=fal(Err,0.5,ADRC_ESO->h);
	fe2=fal(Err,0.25,ADRC_ESO->h);
	ADRC_ESO->z[0]=ADRC_ESO->z[0]+ADRC_ESO->h*(ADRC_ESO->z[1]-ADRC_ESO->beta[0]*Err);
	ADRC_ESO->z[1]=ADRC_ESO->z[1]+ADRC_ESO->h*(ADRC_ESO->z[2]-ADRC_ESO->beta[1]*fe1+ADRC_ESO->b0*ADRC_ESO->u);
	ADRC_ESO->z[2]=ADRC_ESO->z[2]+ADRC_ESO->h*(-ADRC_ESO->beta[2]*fe2);

	return (ADRC_ESO->z[2]/ADRC_ESO->b0);
}


float ADRC_ESO_PartFunction(ADRC_ESO_TypeDef *ADRC_ESO,float y,float PartFuntion)
{
	float Err=0,fe1=0,fe2=0;;
	if(ADRC_ESO->beta[0]==0||ADRC_ESO->beta[1]==0||ADRC_ESO->beta[2]==0)
	{
		ADRC_ESO->beta[0]=1.0f/ADRC_ESO->h;
		ADRC_ESO->beta[1]=1.186f/pow(ADRC_ESO->h,1.47f);
		ADRC_ESO->beta[2]=0.283f/pow(ADRC_ESO->h,2.21f);
	}
	Err=ADRC_ESO->z[0]-y;
	fe1=fal(Err,0.5,5*ADRC_ESO->h);
	fe2=fal(Err,0.25,5*ADRC_ESO->h);
	ADRC_ESO->z[0]=ADRC_ESO->z[0]+ADRC_ESO->h*(ADRC_ESO->z[1]-ADRC_ESO->beta[0]*Err);
	ADRC_ESO->z[1]=ADRC_ESO->z[1]+ADRC_ESO->h*(ADRC_ESO->z[2]-ADRC_ESO->beta[1]*fe1+PartFuntion+ADRC_ESO->b0*ADRC_ESO->u);
	ADRC_ESO->z[2]=ADRC_ESO->z[2]+ADRC_ESO->h*(-ADRC_ESO->beta[2]*fe2);

	return (ADRC_ESO->z[2]/ADRC_ESO->b0);
}
