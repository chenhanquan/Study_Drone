/*
 * Altitude.c
 *
 *  Created on: Apr 7, 2021
 *      Author: 90797
 */
#include "Altitude.h"
#include "DataHandle.h"
#include "arm_math.h"

#define FUSIONTIME 0.01f
#define DEG2RAD 0.01745329f

float Altitude_PressureSensorDataFusion(const float PressureSensor1,const float PressureSensor2)
{
	// static float PressureSensor1_LastData,PressureSensor2_LastData;
	// float D_PressureSensor1,D_PressureSensor2;
	// float Fusion_Height,factor;

	// D_PressureSensor1=(PressureSensor1-PressureSensor1_LastData)/FUSIONTIME;
	// D_PressureSensor2=(PressureSensor2-PressureSensor2_LastData)/FUSIONTIME;

	// factor=fabs(D_PressureSensor1)/(fabs(D_PressureSensor1)+fabs(D_PressureSensor2));
	// // if(factor>1.0f){factor=1.0f;}
	// // if(factor<0.0f){factor=0.0f;}

	// Fusion_Height=factor*PressureSensor2+(1.0f-factor)*PressureSensor1;
	// //Fusion_Height=(PressureSensor2+PressureSensor1)/2.0f;

	// PressureSensor1_LastData=PressureSensor1;
	// PressureSensor2_LastData=PressureSensor2;

	// return Fusion_Height;
	
	float Fusion_Height;
	return Fusion_Height=(PressureSensor1+PressureSensor2)/2.0f;
}

void Altitude_GetRelAltitudeAndZVelocity(Altitude_TypeDef *Altitude_Struct,const float Acc_Z,const float Rel_Height_Pressure,const float Abs_Height_Ultrasonic,const float Origin_Height_Ultrasonic,const float *Drone_Angle)
{
	static float Acc_Z_LastTime;
	static float Rel_Height_Pressure_LastTime;
	static float Rel_Estimation_Velocity_LastTime;
	float Estimation_Pressure_Velocity_Z;
	float Rel_Height_Ultrasonic;

	Rel_Height_Ultrasonic=Abs_Height_Ultrasonic*cos(Drone_Angle[0]*DEG2RAD)*cos(Drone_Angle[1]*DEG2RAD);

	Estimation_Pressure_Velocity_Z=(Rel_Height_Pressure-Rel_Height_Pressure_LastTime)/FUSIONTIME;
	Rel_Height_Pressure_LastTime=Rel_Height_Pressure;

	Altitude_Struct->Estimation_Velocity_Z+=Acc_Z_LastTime*FUSIONTIME;
	if(fabs(Estimation_Pressure_Velocity_Z)<50)
	{
		Altitude_Struct->Estimation_Velocity_Z+=0.015f*(Estimation_Pressure_Velocity_Z-Altitude_Struct->Estimation_Velocity_Z);
	}

	Altitude_Struct->Fusion_Altitude+=Rel_Estimation_Velocity_LastTime*FUSIONTIME;
	// if(Abs_Height_Ultrasonic>10&&Abs_Height_Ultrasonic<250&&fabs(Rel_Height_Ultrasonic-Rel_Height_Pressure)<250)
	if(Abs_Height_Ultrasonic>10&&Abs_Height_Ultrasonic<250)
	{
		Altitude_Struct->Fusion_Altitude+=0.1f*(Rel_Height_Ultrasonic-Origin_Height_Ultrasonic-Altitude_Struct->Fusion_Altitude);
	}
	else
	{
		if(fabs(Estimation_Pressure_Velocity_Z)<50){Altitude_Struct->Fusion_Altitude+=0.015f*(Rel_Height_Pressure-Altitude_Struct->Fusion_Altitude);}
	}

	Acc_Z_LastTime=Acc_Z;
	Rel_Estimation_Velocity_LastTime=Altitude_Struct->Estimation_Velocity_Z;

}

// void Altitude_GetRelAltitudeAndZVelocity_Ultrasonic(Altitude_TypeDef *Altitude_Struct,const float Acc_Z,const float Rel_Height_Ultrasonic)
// {
// 	static float Acc_Z_LastTime;
// 	static float Rel_Height_Ultrasonic_LastTime;
// 	static float Rel_Estimation_Velocity_LastTime;
// 	float Estimation_Velocity_Z;

// 	Estimation_Velocity_Z = (Rel_Height_Ultrasonic - Rel_Height_Ultrasonic_LastTime)/FUSIONTIME;
// 	if(fabs(Estimation_Velocity_Z)<5000.0f)
// 	{	
// 		Rel_Height_Ultrasonic_LastTime = Rel_Height_Ultrasonic;
// 	}
	
// 	Altitude_Struct->Estimation_Velocity_Z+=Acc_Z_LastTime*FUSIONTIME;
// 	if(fabs(Estimation_Velocity_Z)<5000.0f)
// 	{
// 		Altitude_Struct->Estimation_Velocity_Z+=0.3f*(Estimation_Velocity_Z-Altitude_Struct->Estimation_Velocity_Z);
// 	}

// 	Altitude_Struct->Fusion_Altitude+=Rel_Estimation_Velocity_LastTime*FUSIONTIME;
// 	if(fabs(Estimation_Velocity_Z)<5000.0f)
// 	{
// 		Altitude_Struct->Fusion_Altitude+=0.01f*(Rel_Height_Ultrasonic-Altitude_Struct->Fusion_Altitude);
// 	}

// 	//test
	
// 	// Rel_Height_Ultrasonic_LastTime = Rel_Height_Ultrasonic;
	
// 	// Altitude_Struct->Estimation_Velocity_Z+=Acc_Z_LastTime*FUSIONTIME;
// 	// Altitude_Struct->Estimation_Velocity_Z+=0.005f*(Estimation_Velocity_Z-Altitude_Struct->Estimation_Velocity_Z);

// 	// Altitude_Struct->Fusion_Altitude+=Rel_Estimation_Velocity_LastTime*FUSIONTIME;
// 	// Altitude_Struct->Fusion_Altitude+=0.005f*(Rel_Height_Ultrasonic-Altitude_Struct->Fusion_Altitude);

// 	Acc_Z_LastTime=Acc_Z;
// 	Rel_Estimation_Velocity_LastTime=Altitude_Struct->Estimation_Velocity_Z;
// }

void Altitude_GetRelAltitudeAndZVelocity_Ultrasonic(Altitude_TypeDef *Altitude_Struct, const float Acc_Z, const float Rel_Height_Ultrasonic)
{
	static float Acc_Z_LastTime;
	static float Rel_Height_Ultrasonic_LastTime;
	static float Rel_Estimation_Velocity_LastTime;
	float Estimation_Velocity_Z;

	Estimation_Velocity_Z = (Rel_Height_Ultrasonic - Rel_Height_Ultrasonic_LastTime) / FUSIONTIME;
	Rel_Height_Ultrasonic_LastTime = Rel_Height_Ultrasonic;

	Altitude_Struct->Estimation_Velocity_Z += Acc_Z_LastTime * FUSIONTIME;
	Altitude_Struct->Estimation_Velocity_Z += 0.3f * (Estimation_Velocity_Z - Altitude_Struct->Estimation_Velocity_Z);

	Altitude_Struct->Fusion_Altitude += Rel_Estimation_Velocity_LastTime * FUSIONTIME;
	if (Rel_Height_Ultrasonic < 350.0f && Rel_Height_Ultrasonic > -20.0f)
	{
		Altitude_Struct->Fusion_Altitude += 0.01f * (Rel_Height_Ultrasonic - Altitude_Struct->Fusion_Altitude);
	}
	Acc_Z_LastTime = Acc_Z;
	Rel_Estimation_Velocity_LastTime = Altitude_Struct->Estimation_Velocity_Z;
}
