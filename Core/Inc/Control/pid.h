#ifndef __PID_H__
#define __PID_H__

#include "stm32f4xx.h"

void PID_Init();
float PID_Roll(const float Target,const float Outer_m,const float Inner_m);
float PID_Pitch(const float Target,const float Outer_m,const float Inner_m);
float PID_Yaw(const float Target,const float Outer_m,const float Inner_m);
float PID_Altitude(const float Target,const float Pos_m,const float Vel_m);
float PID_OpticalFlow_X(const float Target,const float Outer_m);
float PID_OpticalFlow_Y(const float Target,const float Outer_m);

float PID_P(float target,float measure,float Kp);

void PID_Altitude_Reset(void);
void PID_Attitude_Reset(void);

#endif 
