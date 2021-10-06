/*
 * DataHandle.c
 *
 *  Created on: 2021年4月15日
 *      Author: 90797
 */
#include "DataHandle.h"
#include "arm_math.h"


#define STAGES_ONE 1
#define STAGES_TWO 2

/*******************************FIRFilter**********************************************/


/**********************************IIRFilter******************************************************/

static const float IIRCoeffsFre100to15[5*STAGES_ONE]=
{
	1,  2,  1, 0.747789178258503439700177750637521967292f,  -0.272214937925007283148204351164167746902f
};	//100--15Hz
static const float IIRScaleValueFre100to15=0.131106439916625960862006650131661444902f;

// static const float IIRCoeffsFre100to10[5*STAGES_ONE]=
// {
// 	1,  2,  1,  1.142980502539901133118860343529377132654f,  -0.412801598096188770981029847462195903063f
// };	//100--10Hz
// static const float IIRScaleValueFre100to10=0.067455273889071895587754568168747937307f;

static const float IIRCoeffsFre100to5[5*STAGES_ONE]=
{
	1,  2,  1,  1.561018075800718163392843962355982512236f,  -0.641351538057563175243558362126350402832f
};	//100--5Hz
static const float IIRScaleValueFre100to5=0.02008336556421123561544384017452102853f;

static const float IIRCoeffsFre100to2[5*STAGES_ONE]=
{
	1,  2,  1,  1.822694925196308268766642868285998702049f,  -0.837181651256022618667884671594947576523f
};	//100--2Hz
static const float IIRScaleValueFre100to2=0.003621681514928642119099944096660692594f;

static const float IIRCoeffsFre100to1[5*STAGES_ONE]=
{
	1,  2,  1,  1.911197067426073203932901378720998764038f,  -0.914975834801433740572917940880870446563f
};	//100--1Hz
static const float IIRScaleValueFre100to1=0.000944691843840150748297379568185760945f;

// static const float IIRCoeffsFre100to0dot5[5*STAGES_ONE]=
// {
// 	1,  2,  1,  1.955578240315035465357595967361703515053f,  -0.956543676511203422307971777627244591713f
// };	//100--0.5Hz
// static const float IIRScaleValueFre100to0dot5=0.000241359049041980726606898555175462207f;

// static const float IIRCoeffsBP1Hz5Hz[5*STAGES_TWO]=
// {
// 	1,  0,  -1,  1.925051594732948068156019871821627020836f,  -0.929923473764920682782531002885662019253f,
// 	1,  0,  -1,  1.686278256752978599308789853239431977272f,  -0.753714473246627880698156332073267549276f
// };	
// static const float IIRScaleValueBP1Hz5Hz = 0.115582005640396529022595473179535474628f*0.115582005640396529022595473179535474628f;


//加速度1Hz滤波

arm_biquad_casd_df1_inst_f32 Butterworth_Acc_x,Butterworth_Acc_y,Butterworth_Acc_z;
static float Butterworth_Acc_x_State[4*STAGES_ONE];
static float Butterworth_Acc_y_State[4*STAGES_ONE];
static float Butterworth_Acc_z_State[4*STAGES_ONE];

//角速度15Hz滤波
arm_biquad_casd_df1_inst_f32 Butterworth_Gyro_x,Butterworth_Gyro_y,Butterworth_Gyro_z;
static float Butterworth_Gyro_x_State[4*STAGES_ONE];
static float Butterworth_Gyro_y_State[4*STAGES_ONE];
static float Butterworth_Gyro_z_State[4*STAGES_ONE];

//磁力计5Hz滤波
arm_biquad_casd_df1_inst_f32 Butterworth_Mag_x,Butterworth_Mag_y,Butterworth_Mag_z;
static float Butterworth_Mag_x_State[4*STAGES_ONE];
static float Butterworth_Mag_y_State[4*STAGES_ONE];
static float Butterworth_Mag_z_State[4*STAGES_ONE];

//气压计滤波
arm_biquad_casd_df1_inst_f32 Butterworth_Pressure_Sensor1;
static float Butterworth_Pressure_Sensor1_State[4*STAGES_ONE];
// static float Butterworth_Pressure_Sensor2[4*STAGES_ONE];

//超声波滤波
arm_biquad_casd_df1_inst_f32 Butterworth_Ultrasonic;
static float Butterworth_Ultrasonic_State[4*STAGES_ONE];

//光流滤波
arm_biquad_casd_df1_inst_f32 Butterworth_OpticalFlow_x,Butterworth_OpticalFlow_y;
static float Butterworth_OpticalFlow_x_State[4*STAGES_ONE];
static float Butterworth_OpticalFlow_y_State[4*STAGES_ONE];

//UWB
// FIRFilter_Type FIRFilter_UWB_x,FIRFilter_UWB_y;
// static float FIRFilter_UWB_x_Bufx[4],FIRFilter_UWB_y_Bufx[4];

void DataHandle_Init()
{
	//Acc	
	arm_biquad_cascade_df1_init_f32	(&Butterworth_Acc_x,STAGES_ONE,IIRCoeffsFre100to1,Butterworth_Acc_x_State);
	arm_biquad_cascade_df1_init_f32	(&Butterworth_Acc_y,STAGES_ONE,IIRCoeffsFre100to1,Butterworth_Acc_y_State);
	arm_biquad_cascade_df1_init_f32	(&Butterworth_Acc_z,STAGES_ONE,IIRCoeffsFre100to1,Butterworth_Acc_z_State);

	//Gyro
	arm_biquad_cascade_df1_init_f32	(&Butterworth_Gyro_x,STAGES_ONE,IIRCoeffsFre100to15,Butterworth_Gyro_x_State);
	arm_biquad_cascade_df1_init_f32	(&Butterworth_Gyro_y,STAGES_ONE,IIRCoeffsFre100to15,Butterworth_Gyro_y_State);
	arm_biquad_cascade_df1_init_f32	(&Butterworth_Gyro_z,STAGES_ONE,IIRCoeffsFre100to15,Butterworth_Gyro_z_State);

	//Mag
	arm_biquad_cascade_df1_init_f32	(&Butterworth_Mag_x,STAGES_ONE,IIRCoeffsFre100to5,Butterworth_Mag_x_State);
	arm_biquad_cascade_df1_init_f32	(&Butterworth_Mag_y,STAGES_ONE,IIRCoeffsFre100to5,Butterworth_Mag_y_State);
	arm_biquad_cascade_df1_init_f32	(&Butterworth_Mag_z,STAGES_ONE,IIRCoeffsFre100to5,Butterworth_Mag_z_State);

	//Ultrasonic
	arm_biquad_cascade_df1_init_f32(&Butterworth_Ultrasonic,STAGES_ONE,IIRCoeffsFre100to5,Butterworth_Ultrasonic_State);

	//Pressure
	arm_biquad_cascade_df1_init_f32(&Butterworth_Pressure_Sensor1,STAGES_ONE,IIRCoeffsFre100to2,Butterworth_Pressure_Sensor1_State);

	//OpticalFlow
	arm_biquad_cascade_df1_init_f32(&Butterworth_OpticalFlow_x,STAGES_ONE,IIRCoeffsFre100to5,Butterworth_OpticalFlow_x_State);
	arm_biquad_cascade_df1_init_f32(&Butterworth_OpticalFlow_y,STAGES_ONE,IIRCoeffsFre100to5,Butterworth_OpticalFlow_y_State);

	//UWB
	// FIRFilter_Init(&FIRFilter_UWB_x,FIR_Num_Fre100to30,FIRFilter_UWB_x_Bufx,3);
	// FIRFilter_Init(&FIRFilter_UWB_y,FIR_Num_Fre100to30,FIRFilter_UWB_y_Bufx,3);

}

void DataHandle_GyroFilter(DataHandle_AccGyroMag_TypeDef *FilterData,const float *RawGyro,float *RotateGyro)
{

	RotateGyro[0]=RawGyro[1];
	RotateGyro[1]=RawGyro[0];
	RotateGyro[2]=-RawGyro[2];
	
	arm_biquad_cascade_df1_f32	(&Butterworth_Gyro_x,RotateGyro,FilterData->Gyro,1);
	arm_biquad_cascade_df1_f32	(&Butterworth_Gyro_y,RotateGyro+1,FilterData->Gyro+1,1);
	arm_biquad_cascade_df1_f32	(&Butterworth_Gyro_z,RotateGyro+2,FilterData->Gyro+2,1);	

	FilterData->Gyro[0]*=IIRScaleValueFre100to15;
	FilterData->Gyro[1]*=IIRScaleValueFre100to15;
	FilterData->Gyro[2]*=IIRScaleValueFre100to15;
	
}

void DataHandle_AccFilter(DataHandle_AccGyroMag_TypeDef *FilterData,const float *RawAcc,float *RotateAcc)
{
	RotateAcc[0]=RawAcc[1]*9.80665f;
	RotateAcc[1]=RawAcc[0]*9.80665f;
	RotateAcc[2]=-RawAcc[2]*9.80665f;

	arm_biquad_cascade_df1_f32	(&Butterworth_Acc_x,RotateAcc,FilterData->Acc,1);
	arm_biquad_cascade_df1_f32	(&Butterworth_Acc_y,RotateAcc+1,FilterData->Acc+1,1);
	arm_biquad_cascade_df1_f32	(&Butterworth_Acc_z,RotateAcc+2,FilterData->Acc+2,1);

	FilterData->Acc[0]*=IIRScaleValueFre100to1;
	FilterData->Acc[1]*=IIRScaleValueFre100to1;
	FilterData->Acc[2]*=IIRScaleValueFre100to1;

}

void DataHandle_MagFilter(DataHandle_AccGyroMag_TypeDef *FilterData,const float *RawMag)
{
	float RotateMag[3];

	RotateMag[0]=RawMag[1];
	RotateMag[1]=RawMag[0];
	RotateMag[2]=-RawMag[2];

	arm_biquad_cascade_df1_f32	(&Butterworth_Mag_x,RotateMag,FilterData->Mag,1);
	arm_biquad_cascade_df1_f32	(&Butterworth_Mag_y,RotateMag+1,FilterData->Mag+1,1);
	arm_biquad_cascade_df1_f32	(&Butterworth_Mag_z,RotateMag+2,FilterData->Mag+2,1);

	FilterData->Mag[0]*=IIRScaleValueFre100to5;
	FilterData->Mag[1]*=IIRScaleValueFre100to5;
	FilterData->Mag[2]*=IIRScaleValueFre100to5;
}

float DataHandle_PressureSensor1HeightFilter(const float PressureHeight)
{
	float Output;
	arm_biquad_cascade_df1_f32	(&Butterworth_Pressure_Sensor1,&PressureHeight,&Output,1);
	return Output*=IIRScaleValueFre100to2;
}

// float DataHandle_PressureSensor2HeightFilter(const float PressureHeight)
// {
// 	return FIRFilter_Filter(&FIRFilter_Pressure_Sensor2,PressureHeight);
// }

float DataHandle_UltrasonicHeightFilter(const float UltrasonicHeight)
{
	float Filter_UltrasonicHeight;
	arm_biquad_cascade_df1_f32	(&Butterworth_Ultrasonic,&UltrasonicHeight,&Filter_UltrasonicHeight,1);
	return Filter_UltrasonicHeight*IIRScaleValueFre100to5;
}

void DataHandle_UWB(const float Raw_UWB_X,const float Raw_UWB_Y,const float Angle_Yaw,float *Body_X,float *Body_Y)
{
	*Body_X= Raw_UWB_X * arm_cos_f32(Angle_Yaw) - Raw_UWB_Y * arm_sin_f32(Angle_Yaw);
	*Body_Y= Raw_UWB_X * arm_sin_f32(Angle_Yaw) + Raw_UWB_Y * arm_cos_f32(Angle_Yaw);
}

void DataHandle_OpticalFlowFilter(const float *Raw_OpticalFlowData,float *Filter_OpticalFlowData)
{
	arm_biquad_cascade_df1_f32	(&Butterworth_OpticalFlow_x,Raw_OpticalFlowData,Filter_OpticalFlowData,1);
	arm_biquad_cascade_df1_f32	(&Butterworth_OpticalFlow_y,Raw_OpticalFlowData+1,Filter_OpticalFlowData+1,1);

	Filter_OpticalFlowData[0]*=IIRScaleValueFre100to5;
	Filter_OpticalFlowData[1]*=IIRScaleValueFre100to5;

}
