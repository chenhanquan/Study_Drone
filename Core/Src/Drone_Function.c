/*
 * Drone_Function.c
 *
 *  Created on: Jan 23, 2021
 *      Author: 90797
 */
#include "main.h"
#include "arm_math.h"

#define DEG2RAD           0.01745329f	//角度转弧度
#define RAD2DEG           57.29577951f	//弧度转角度

extern Drone_Type MyDrone;

//安排过渡过程
ADRC_TD_TypeDef ADRC_TD_Roll;
ADRC_TD_TypeDef ADRC_TD_Pitch;

float Roll_Compensation,Pitch_Compensation;

ADRC_TD_TypeDef ADRC_TD_Altitude;

static void Drone_ParaInit();

void Drone_Init(void)
{
	I2C_Software_Stop();
	HAL_Delay(5); 
	BMI088_Init();
	QMC5883_Init();
	SPL06_Init();
//	GPS_Init();
	MyDrone.Mode=Lock;
	Drone_ParaInit();
	DataHandle_Init();
	OpticalFlow_Init();

	PID_Init();
}

static void Drone_ParaInit()
{

	ADRC_TD_Roll.T=0.02;
	ADRC_TD_Roll.h=0.02;
	ADRC_TD_Roll.r=800;

	ADRC_TD_Pitch.T=0.02;
	ADRC_TD_Pitch.h=0.02;
	ADRC_TD_Pitch.r=800;

	ADRC_TD_Altitude.T=0.02;
	ADRC_TD_Altitude.h=0.04;
	ADRC_TD_Altitude.r=150;

}

void Drone_ModeControl(void)
{
	Remote_DataHandle(&(MyDrone.Raw_RemoteData));
	if(MyDrone.Raw_RemoteData.Ch5_SWA<1500)		//判断是否锁定
	{
		MyDrone.Mode=Lock;
		LED_Display(On, RED);
		Motor_Lock();

	}
	else if(MyDrone.Raw_RemoteData.Ch5_SWA>1500&&MyDrone.Raw_RemoteData.Ch3_Thro<1050)	//进入手飞模式
	{
		if(MyDrone.Mode==Lock)
		{
			MyDrone.Mode=Normal;
			MyDrone.Target.Yaw=MyDrone.Fusion_IMUData.angle[2];	//获取当前偏航
			
			MyDrone.Origin_Height_Pressure=MyDrone.Fusion_PressureAltitude;
			MyDrone.Origin_Height_Ultrasonic =MyDrone.Filter_UltrasonicHeight;

			MyDrone.Fusion_AltitudeData.Fusion_Altitude = 0.0f;
			MyDrone.Fusion_AltitudeData.Estimation_Velocity_Z = 0.0f;

			// MyDrone.Target.Position_UWB[0]=MyDrone.Filter_UWB_Pos[0];
			// MyDrone.Target.Position_UWB[1]=MyDrone.Filter_UWB_Pos[1];

			PID_Altitude_Reset();
			PID_Attitude_Reset();

		}
		else
		{
			if(MyDrone.Raw_RemoteData.Ch7_SWC>1200)	//判断定高定点
			{
				if(MyDrone.Raw_RemoteData.Ch7_SWC>1800)		//定点模式
				{
					LED_Display(On, GREEN);
					MyDrone.Mode=Space_Control;
					// if(MyDrone.Raw_GPSData.numSV<3){LED_Display(On, PINK);}		//GPS信号指示
				}
				else
				{
					LED_Display(On, YELLOW);				//定高模式
					MyDrone.Mode=Altitude;
				}
			}else{MyDrone.Mode=Normal;}
		}
		if(MyDrone.Mode==Normal){LED_Display(On, BLUE);}
	}
}

void Drone_DataHandle(void)
{
	float SensorTime;
	//角速度计数据读取
	BMI088_Read_Gyro(MyDrone.Raw_GyroData);
	DataHandle_GyroFilter(&MyDrone.Filter_AccGyroMag,MyDrone.Raw_GyroData,MyDrone.DataHandle_Raw_Gyro_Rotate);

	osDelay(1);
	//获取磁力计数据
	QMC5883_Read(MyDrone.Raw_MagData);			
	DataHandle_MagFilter(&MyDrone.Filter_AccGyroMag,MyDrone.Raw_MagData);
	
	//获取加速度计数据
	BMI088_Read_Acc(MyDrone.Raw_AccData,&SensorTime);
    DataHandle_AccFilter(&MyDrone.Filter_AccGyroMag,MyDrone.Raw_AccData,MyDrone.DataHandle_Raw_Acc_Rotate);

	//计算机体姿态
	IMU_Process(&(MyDrone.Fusion_IMUData),MyDrone.Filter_AccGyroMag.Acc,MyDrone.DataHandle_Raw_Gyro_Rotate,MyDrone.Filter_AccGyroMag.Mag);
	MyDrone.Fusion_IMUData.angle[0]+=2.5f;
	MyDrone.Fusion_IMUData.angle[1]-=0.6f;

	//修正磁力计角度
	// float Fix_Yaw;
	// if(MyDrone.Control.Thro>300){Fix_Yaw=(MyDrone.Control.Thro-150)/850.0f*25.0f;}else{Fix_Yaw=0.0f;}
	// MyDrone.DataHandle_Yaw_Angle=MyDrone.Fusion_IMUData.angle[2]+Fix_Yaw;
	// if(MyDrone.DataHandle_Yaw_Angle>180.0f){MyDrone.DataHandle_Yaw_Angle-=360.0f;}
	// if(MyDrone.DataHandle_Yaw_Angle<-180.0f){MyDrone.DataHandle_Yaw_Angle+=360.0f;}
	MyDrone.DataHandle_Yaw_Angle=MyDrone.Fusion_IMUData.angle[2];

	/********************************一系列坐标旋转************************/
	//计算原始加速度转到世界坐标
	// float Fix_Angle[3];
	// Fix_Angle[0]=MyDrone.Fusion_IMUData.angle[0];
	// Fix_Angle[1]=MyDrone.Fusion_IMUData.angle[1];
	// Fix_Angle[2]=MyDrone.DataHandle_Yaw_Angle;	

	//计算原始加速度转到机体平面（融合数据用）
	// MyDrone.DataHandle_Acceleration_Axis_Body_Plane[0]=-MyDrone.Fliter_AccGyroMag.Acc[0] * arm_cos_f32(MyDrone.Fusion_IMUData.angle[1]) * 100.0f;
	// MyDrone.DataHandle_Acceleration_Axis_Body_Plane[1]=-MyDrone.Fliter_AccGyroMag.Acc[1] * arm_cos_f32(MyDrone.Fusion_IMUData.angle[0]) * 100.0f;

	//计算滤波加速度转到机体Z轴（融合数据用）
	MyDrone.DataHandle_Acceleration_Axis_Z = -MyDrone.Filter_AccGyroMag.Acc[2]*arm_cos_f32(MyDrone.Fusion_IMUData.angle[0]*DEG2RAD)*arm_cos_f32(MyDrone.Fusion_IMUData.angle[1]*DEG2RAD);

	MyDrone.DataHandle_Acceleration_Axis_Z -=9.80f;
	MyDrone.DataHandle_Acceleration_Axis_Z *=100.0f;

	//计算滤波加速度转到机体平面（控制反馈用）
	MyDrone.Filter_Acceleration_Axis_Body_Plane[0]=MyDrone.Filter_AccGyroMag.Acc[0] * arm_cos_f32(MyDrone.Fusion_IMUData.angle[1]) * 100.0f;
	MyDrone.Filter_Acceleration_Axis_Body_Plane[1]=MyDrone.Filter_AccGyroMag.Acc[1] * arm_cos_f32(MyDrone.Fusion_IMUData.angle[0]) * 100.0f;

	//计算滤波加速度转到机体Z轴（控制反馈用）
	MyDrone.Filter_Acceleration_Axis_Z = -MyDrone.Filter_AccGyroMag.Acc[2]*arm_cos_f32(MyDrone.Fusion_IMUData.angle[0]*DEG2RAD)*arm_cos_f32(MyDrone.Fusion_IMUData.angle[1]*DEG2RAD);

	MyDrone.Filter_Acceleration_Axis_Z -=9.80f;
	MyDrone.Filter_Acceleration_Axis_Z *=100.0f;
	/***************************坐标旋转结束********************************/

	/*****************************获取高度*********************************/
	osDelay(1);
	MyDrone.Raw_PressureHeight1=SPL06_GetSensor1Height();
	MyDrone.Filter_PressureHeightSensor1=DataHandle_PressureSensor1HeightFilter(MyDrone.Raw_PressureHeight1);
	// MyDrone.Raw_PressureHeight2=SPL06_GetSensor2Height();
	// MyDrone.Fliter_PressureHeightSensor2=DataHandle_PressureSensor2HeightFliter(MyDrone.Raw_PressureHeight2);
	// MyDrone.Fusion_PressureAltitude = Altitude_PressureSensorDataFusion((MyDrone.Fliter_PressureHeightSensor1),(MyDrone.Fliter_PressureHeightSensor2));
	
	MyDrone.Raw_UltrasonicHeight = Ultrasonic_GetDistance();
	MyDrone.Filter_UltrasonicHeight = DataHandle_UltrasonicHeightFilter(MyDrone.Raw_UltrasonicHeight);
	// MyDrone.DataHandle_Height_Ultrasonic=MyDrone.Fliter_UltrasonicHeight*cos(MyDrone.Fusion_IMUData.angle[0]*DEG2RAD)*cos(MyDrone.Fusion_IMUData.angle[1]*DEG2RAD);

	if(MyDrone.Mode!=Lock)
	{
		float Rel_Height_Ultrasonic;
		Rel_Height_Ultrasonic = MyDrone.Filter_UltrasonicHeight*arm_cos_f32(MyDrone.Fusion_IMUData.angle[0]*DEG2RAD)*arm_cos_f32(MyDrone.Fusion_IMUData.angle[1]*DEG2RAD)-MyDrone.Origin_Height_Ultrasonic;

		Altitude_GetRelAltitudeAndZVelocity_Ultrasonic(&(MyDrone.Fusion_AltitudeData),MyDrone.DataHandle_Acceleration_Axis_Z,Rel_Height_Ultrasonic);
		// }

		// MyDrone.UserData1[4] = Rel_Height_Ultrasonic;
	}

	// MyDrone.UserData[0] = MyDrone.Raw_PressureHeight1 ;
	// MyDrone.UserData[1] = MyDrone.Filter_PressureHeightSensor1 ;
	// MyDrone.UserData[2] = MyDrone.Fusion_AltitudeData.Estimation_Velocity_Z ;
	// MyDrone.UserData[3] = MyDrone.Fusion_AltitudeData.Fusion_Altitude ;

	// MyDrone.UserData1[0] = MyDrone.Fliter_Acceleration_Axis_Body_Plane[0] ;
	// MyDrone.UserData1[1] = MyDrone.Fliter_Acceleration_Axis_Body_Plane[1] ;
	// MyDrone.UserData1[2] = MyDrone.Fliter_Acceleration_Axis_Z ;

	/***************************获取高度结束*******************************/

	/*****************************获取定点*********************************/
	OpticalFlow_Read(MyDrone.Raw_OpticalFlowData);
	DataHandle_OpticalFlowFilter(MyDrone.Raw_OpticalFlowData,MyDrone.Filter_OpticalFlowData);
	MyDrone.Filter_OpticalFlowData[0]=MyDrone.Filter_OpticalFlowData[0]/200.0f;
	MyDrone.Filter_OpticalFlowData[1]=MyDrone.Filter_OpticalFlowData[1]/200.0f;

	MyDrone.Filter_OpticalFlowData[0]=(MyDrone.Filter_OpticalFlowData[0]+0.2f*MyDrone.Filter_AccGyroMag.Gyro[1]*0.01745329f)*MyDrone.Filter_UltrasonicHeight;
	MyDrone.Filter_OpticalFlowData[1]=(MyDrone.Filter_OpticalFlowData[1]-0.2f*MyDrone.Filter_AccGyroMag.Gyro[0]*0.01745329f)*MyDrone.Filter_UltrasonicHeight;

	Fusion_OpticalFlowData(MyDrone.Filter_Acceleration_Axis_Body_Plane,MyDrone.Filter_OpticalFlowData,MyDrone.Fusion_Est_v_OpticalFlow);

	UWB_GetData(&MyDrone.Raw_UWB_Pos_X,&MyDrone.Raw_UWB_Pos_Y);
	DataHandle_UWB(MyDrone.Raw_UWB_Pos_X,MyDrone.Raw_UWB_Pos_Y,MyDrone.DataHandle_Yaw_Angle,MyDrone.Filter_UWB_Pos,(MyDrone.Filter_UWB_Pos+1));
	
	if(MyDrone.Mode!=Lock)
	{
		// UWB_Position_GetPositionAndVelocity(&MyDrone.Fusion_Position_UWB,MyDrone.DataHandle_Acceleration_Axis_Body_Plane[0],MyDrone.DataHandle_Acceleration_Axis_Body_Plane[1],MyDrone.Fliter_UWB_Pos[0],MyDrone.Fliter_UWB_Pos[1]);
	}

	/****************************获取定点结束*******************************/

	// MyDrone.UserData[0] = MyDrone.Raw_UltrasonicHeight ;
	// MyDrone.UserData[1] = MyDrone.Filter_UltrasonicHeight ;
	// MyDrone.UserData[2] = MyDrone.Fusion_AltitudeData.Estimation_Velocity_Z ;
	// MyDrone.UserData[3] = MyDrone.Fusion_AltitudeData.Fusion_Altitude ;

	// MyDrone.UserData1[3] = MyDrone.Raw_UltrasonicHeight ;
	// MyDrone.UserData1[4] = MyDrone.Fliter_UltrasonicHeight ;

	// MyDrone.UserData[0] = MyDrone.Filter_OpticalFlowData[0] ;
	// MyDrone.UserData[1] = MyDrone.Filter_OpticalFlowData[1] ;
	// MyDrone.UserData[2] = MyDrone.Filter_Acceleration_Axis_Body_Plane[0] ;
	// MyDrone.UserData[3] = MyDrone.Filter_Acceleration_Axis_Body_Plane[1] ;
	// MyDrone.UserData[4] = MyDrone.Fliter_UltrasonicHeight ;

	// MyDrone.UserData1[0] = MyDrone.Fusion_Est_v_OpticalFlow[0] ;
	// MyDrone.UserData1[1] = MyDrone.Fusion_Est_v_OpticalFlow[1] ;
	// MyDrone.UserData[2] = MyDrone.Filter_OpticalFlowData[0] ;
	// MyDrone.UserData[3] = MyDrone.Filter_OpticalFlowData[1] ;

}

#define ANGLE_MAX 25.0f
#define VEL_MAX 150.0f
void Drone_TargetSet(void)		//50Hz
{
	float Roll_target,Pitch_target;
	float Yaw_gyro_target=0.0f;
	float Altitude_target=0;
	static uint8_t Yaw_Flag=0;
	float Thro_target;
	//油门目标值处理
	if(MyDrone.Raw_RemoteData.Ch3_Thro<1050)
	{
		Thro_target=0;
		Altitude_target=0;
		MyDrone.Control.Thro=Thro_target;
	}
	else
	{
		Thro_target=MyDrone.Raw_RemoteData.Ch3_Thro-1000;
		if(MyDrone.Mode==Normal){MyDrone.Control.Thro=Thro_target/1000.0f*700.0f+250;}		//手飞模式油门控制
		else															//定高定点油门控制
		{
			Altitude_target=((Thro_target-50.0f)/950.0f)*200.0f;
			if(Altitude_target<0){Altitude_target=0.0f;}
		}
	}
	//横滚目标值处理
	if(fabs(MyDrone.Raw_RemoteData.Ch1_Roll-1500)<50.0f)
	{

		Roll_target=0.0f;
		if(MyDrone.Mode==Space_Control)
		{
			if(MyDrone.Fusion_AltitudeData.Fusion_Altitude>5.0f)
			{
				Roll_target=PID_OpticalFlow_Y(0.0f,MyDrone.Filter_OpticalFlowData[1]);
			}
		}
	}
	else
	{
		if(MyDrone.Mode==Space_Control)
		{
			if(MyDrone.Raw_RemoteData.Ch1_Roll>1500){MyDrone.Target.Vel_Y=(MyDrone.Raw_RemoteData.Ch1_Roll-1550)/450.0f*VEL_MAX;}
			else{MyDrone.Target.Vel_Y=(MyDrone.Raw_RemoteData.Ch1_Roll-1450)/450.0f*VEL_MAX;}

			Roll_target=PID_OpticalFlow_Y(MyDrone.Target.Vel_Y,MyDrone.Filter_OpticalFlowData[1]);
		}
		else
		{
			if(MyDrone.Raw_RemoteData.Ch1_Roll>1500){Roll_target=(MyDrone.Raw_RemoteData.Ch1_Roll-1550)/450.0f*ANGLE_MAX;}
			else{Roll_target=(MyDrone.Raw_RemoteData.Ch1_Roll-1450)/450.0f*ANGLE_MAX;}
		}
		
	}
	if(Roll_target>ANGLE_MAX){Roll_target=ANGLE_MAX;}
	if(Roll_target<-ANGLE_MAX){Roll_target=-ANGLE_MAX;}
	//俯仰目标值处理
	if(fabs(MyDrone.Raw_RemoteData.Ch2_Pitch-1500)<50.0f)
	{
		Pitch_target=0.0f;
		if(MyDrone.Mode==Space_Control)
		{
			// Roll_target=PID_OpticalFlow_X(0.0f,MyDrone.Fusion_Est_v_OpticalFlow[0]);
			Pitch_target=-PID_OpticalFlow_X(0.0f,MyDrone.Filter_OpticalFlowData[0]);
		}
	}
	else
	{
		if(MyDrone.Mode==Space_Control)
		{
			if(MyDrone.Raw_RemoteData.Ch2_Pitch>1500){MyDrone.Target.Vel_X=(MyDrone.Raw_RemoteData.Ch2_Pitch-1550)/450.0f*VEL_MAX;}
			else{MyDrone.Target.Vel_X=(MyDrone.Raw_RemoteData.Ch2_Pitch-1450)/450.0f*VEL_MAX;}

			Pitch_target=-PID_OpticalFlow_X(MyDrone.Target.Vel_X,MyDrone.Filter_OpticalFlowData[0]);
		}
		else
		{
			if(MyDrone.Raw_RemoteData.Ch2_Pitch>1500){Pitch_target=(MyDrone.Raw_RemoteData.Ch2_Pitch-1550)/450.0f*ANGLE_MAX;}
			else{Pitch_target=(MyDrone.Raw_RemoteData.Ch2_Pitch-1450)/450.0f*ANGLE_MAX;}
		}
	}
	if(Pitch_target>ANGLE_MAX){Pitch_target=ANGLE_MAX;}
	if(Pitch_target<-ANGLE_MAX){Pitch_target=-ANGLE_MAX;}
	//偏航目标值处理
	if(fabs(MyDrone.Raw_RemoteData.Ch4_Yaw-1500)<50.0f&&Yaw_Flag==0){MyDrone.Target.Yaw=MyDrone.DataHandle_Yaw_Angle;Yaw_Flag=1;}
		else
		{
			if(MyDrone.Raw_RemoteData.Ch4_Yaw>1550){Yaw_gyro_target=(MyDrone.Raw_RemoteData.Ch4_Yaw-1550)/1550*45;Yaw_Flag=0;}
			if(MyDrone.Raw_RemoteData.Ch4_Yaw<1450){Yaw_gyro_target=(MyDrone.Raw_RemoteData.Ch4_Yaw-1450)/1450*45;Yaw_Flag=0;}
		}

	//横滚俯仰安排过渡
	if(MyDrone.Mode!=Space_Control)
	{
		ADRC_TD_fsun(&ADRC_TD_Roll,Roll_target);
		ADRC_TD_fsun(&ADRC_TD_Pitch,Pitch_target);

		MyDrone.Target.Roll=ADRC_TD_Roll.x[0];
		MyDrone.Target.Pitch=ADRC_TD_Pitch.x[0];
	}
	else
	{
		MyDrone.Target.Roll=Roll_target;
		MyDrone.Target.Pitch=Pitch_target;
	}
	

	//高度安排过渡
	ADRC_TD_fsun(&ADRC_TD_Altitude,Altitude_target);

	MyDrone.Target.Altitude_target=Altitude_target;

	//偏航赋值
	MyDrone.Target.Yaw_gyro=Yaw_gyro_target;
}

static float PWM_Roll=0,PWM_Pitch=0,PWM_Yaw=0;
void Drone_Control_100Hz(void)		//100Hz
{
	if(MyDrone.Mode==Altitude||MyDrone.Mode==Space_Control)
	{
		MyDrone.Control.Thro=PID_Altitude(MyDrone.Target.Altitude_target,
											MyDrone.Fusion_AltitudeData.Fusion_Altitude,
											MyDrone.Fusion_AltitudeData.Estimation_Velocity_Z);
	}

	if(MyDrone.Raw_RemoteData.Ch3_Thro>1050&&MyDrone.Mode!=Lock)
	{
		PWM_Roll = PID_Roll(MyDrone.Target.Roll,MyDrone.Fusion_IMUData.angle[0],MyDrone.Filter_AccGyroMag.Gyro[0]);
		PWM_Pitch =  PID_Pitch(MyDrone.Target.Pitch,MyDrone.Fusion_IMUData.angle[1],MyDrone.Filter_AccGyroMag.Gyro[1]);
		PWM_Yaw =  PID_Yaw(MyDrone.Target.Yaw,MyDrone.Fusion_IMUData.angle[2],MyDrone.Filter_AccGyroMag.Gyro[2]);


		// float last_output_Roll,last_output_Pitch,last_output_Yaw;

		// static float last_gyro_x,last_gyro_y,last_gyro_z;
		// float D_gyro_x,D_gyro_y,D_gyro_z;
		// D_gyro_x=(MyDrone.Fliter_AccGyroMag.Gyro[0]-last_gyro_x)/0.005f;
		// D_gyro_y=(MyDrone.Fliter_AccGyroMag.Gyro[1]-last_gyro_y)/0.005f;
		// D_gyro_z=(MyDrone.Fliter_AccGyroMag.Gyro[2]-last_gyro_z)/0.005f;
		// last_gyro_x=MyDrone.Fliter_AccGyroMag.Gyro[0];
		// last_gyro_y=MyDrone.Fliter_AccGyroMag.Gyro[1];
		// last_gyro_z=MyDrone.Fliter_AccGyroMag.Gyro[2];

		// Roll_Compensation=(D_gyro_x-5.0f*last_output_Roll)/20.0f;
		// Pitch_Compensation=(D_gyro_y-5.0f*last_output_Pitch)/20.0f;
		// Yaw_Compensation=(D_gyro_z-5.0f*last_output_Yaw)/25.0f;

		// if(Roll_Compensation > 300.0f){Roll_Compensation=300.0f;}
		// if(Roll_Compensation < -300.0f){Roll_Compensation=-300.0f;}
		// if(Pitch_Compensation > 300.0f){Pitch_Compensation=300.0f;}
		// if(Pitch_Compensation < -300.0f){Pitch_Compensation=-300.0f;}
		// if(Yaw_Compensation > 300.0f){Yaw_Compensation=300.0f;}
		// if(Yaw_Compensation < -300.0f){Yaw_Compensation=-300.0f;}

		// PWM_Roll-=Roll_Compensation;
		// PWM_Pitch-=Pitch_Compensation;
		// PWM_Yaw-=Yaw_Compensation;

		// last_output_Roll=PWM_Roll;
		// last_output_Pitch=PWM_Pitch;
		// last_output_Yaw=PWM_Yaw;

		// MyDrone.UserData[0]=D_gyro_x;
		// MyDrone.UserData[1]=last_output_Roll;
		// MyDrone.UserData[2]=12.0f*last_output_Roll;
		// MyDrone.UserData[3]=Roll_Compensation;

		MyDrone.Control.PWM[0] = (MyDrone.Control.Thro + PWM_Roll + PWM_Pitch - PWM_Yaw);
		MyDrone.Control.PWM[1] = (MyDrone.Control.Thro - PWM_Roll + PWM_Pitch + PWM_Yaw);
		MyDrone.Control.PWM[2] = (MyDrone.Control.Thro - PWM_Roll - PWM_Pitch - PWM_Yaw);
		MyDrone.Control.PWM[3] = (MyDrone.Control.Thro + PWM_Roll - PWM_Pitch + PWM_Yaw);
	}
	else
	{
		PID_Altitude_Reset();
		PID_Attitude_Reset();
		MyDrone.Control.PWM[0] = 0;MyDrone.Control.PWM[1] = 0;MyDrone.Control.PWM[2] = 0;MyDrone.Control.PWM[3] = 0;
	}

	Motor_PWMSet(MyDrone.Control.PWM[0],MyDrone.Control.PWM[1],MyDrone.Control.PWM[2],MyDrone.Control.PWM[3]);
	// Motor_PWMSet(0,0,0,0);			//测试用，禁止电机输出
}



