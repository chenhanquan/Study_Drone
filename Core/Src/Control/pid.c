#include "pid.h"
#include "main.h"
#include "arm_math.h"

extern Drone_Type MyDrone;

#define TEST_PID

//Roll and Pitch Parameter
arm_pid_instance_f32 PID_Roll_outer,PID_Pitch_outer;
arm_pid_instance_f32 PID_Roll_inner,PID_Pitch_inner;

//不带架
// float PID_Para_RollAndPitch_outer[3] = {3.5f,0.0f,0.0f};  //6.2			//朗宇电机
// float PID_Para_RollAndPitch_inner[3] = {0.6f,0.0f,0.1f};  //0.5-0.8

// 带架
// float PID_Para_RollAndPitch_outer[3] = {7.0f,0.0f,0.0f};  //6.0			//朗宇电机
// float PID_Para_RollAndPitch_inner[3] = {0.8f,0.0f,0.15f};  //0.5-0.8

float PID_Para_RollAndPitch_outer[3] = {6.2f,0.01f,0.1f};  //6.0			//朗宇电机
float PID_Para_RollAndPitch_inner[3] = {0.4f,0.0f,0.3f};  //0.5-0.8

//Yaw Parameter
arm_pid_instance_f32 PID_Yaw_outer,PID_Yaw_inner; 

// 不带架
// float PID_Para_Yaw_outer[3] = {2.0f,0.0f,0.0f};
// float PID_Para_Yaw_inner[3] = {12.0f,0.0f,0.2f};

float PID_Para_Yaw_outer[3] = {3.5f,0.0f,0.0f};
float PID_Para_Yaw_inner[3] = {5.5f,0.0f,0.2f};

// float PID_Para_Yaw_outer[3] = {2.0f,0.0f,0.0f};
// float PID_Para_Yaw_inner[3] = {6.0f,0.0f,0.2f};

//带架
// float PID_Para_Yaw_outer[3] = {1.0f,0.0f,0.0f};
// float PID_Para_Yaw_inner[3] = {1.0f,0.0f,0.2f};

//Altitude Parameter
arm_pid_instance_f32 PID_Altitude_Pos,PID_Altitude_Vel,PID_Altitude_Acc;
float PID_Para_Altitude_Pos[3] = {3.4f,0.05f,0.12f};
float PID_Para_Altitude_Vel[3] = {1.4f,0.0f,0.0f};

//OpticalFlow Parameter
arm_pid_instance_f32 PID_OpticalFlow_X_Outer,PID_OpticalFlow_Y_Outer;
float PID_Para_OpticalFlow_XY_Outer[3] = {0.05f, 0.0f, 0.0f}; //0.05(有稳态误差，无抖动，无超调) 0.1（有稳态误差，有抖动，无超调）

void PID_Init()
{
	//Roll and Pitch PID Init
    PID_Roll_inner.Kp = PID_Para_RollAndPitch_inner[0];
    PID_Roll_inner.Ki = PID_Para_RollAndPitch_inner[1];
    PID_Roll_inner.Kd = PID_Para_RollAndPitch_inner[2];

	PID_Pitch_inner.Kp = PID_Para_RollAndPitch_inner[0];
    PID_Pitch_inner.Ki = PID_Para_RollAndPitch_inner[1];
    PID_Pitch_inner.Kd = PID_Para_RollAndPitch_inner[2];
    arm_pid_init_f32(&PID_Roll_inner,1);
	arm_pid_init_f32(&PID_Pitch_inner,1);

	PID_Roll_outer.Kp = PID_Para_RollAndPitch_outer[0];
    PID_Roll_outer.Ki = PID_Para_RollAndPitch_outer[1];
    PID_Roll_outer.Kd = PID_Para_RollAndPitch_outer[2];

	PID_Pitch_outer.Kp = PID_Para_RollAndPitch_outer[0];
    PID_Pitch_outer.Ki = PID_Para_RollAndPitch_outer[1];
    PID_Pitch_outer.Kd = PID_Para_RollAndPitch_outer[2];
    arm_pid_init_f32(&PID_Roll_outer,1);
	arm_pid_init_f32(&PID_Pitch_outer,1);

	//Yaw PID Init
	PID_Yaw_inner.Kp = PID_Para_Yaw_inner[0];
    PID_Yaw_inner.Ki = PID_Para_Yaw_inner[1];
    PID_Yaw_inner.Kd = PID_Para_Yaw_inner[2];

	PID_Yaw_outer.Kp = PID_Para_Yaw_outer[0];
    PID_Yaw_outer.Ki = PID_Para_Yaw_outer[1];
    PID_Yaw_outer.Kd = PID_Para_Yaw_outer[2];
	arm_pid_init_f32(&PID_Yaw_inner,1);
	arm_pid_init_f32(&PID_Yaw_outer,1);

	//Altitude PID Init
	PID_Altitude_Pos.Kp = PID_Para_Altitude_Pos[0];
    PID_Altitude_Pos.Ki = PID_Para_Altitude_Pos[1];
    PID_Altitude_Pos.Kd = PID_Para_Altitude_Pos[2];

	PID_Altitude_Vel.Kp = PID_Para_Altitude_Vel[0];
    PID_Altitude_Vel.Ki = PID_Para_Altitude_Vel[1];
    PID_Altitude_Vel.Kd = PID_Para_Altitude_Vel[2];

	arm_pid_init_f32(&PID_Altitude_Pos,1);
	arm_pid_init_f32(&PID_Altitude_Vel,1);

	//OpticalFlow PID Init
	PID_OpticalFlow_X_Outer.Kp = PID_Para_OpticalFlow_XY_Outer[0];
    PID_OpticalFlow_X_Outer.Ki = PID_Para_OpticalFlow_XY_Outer[1];
    PID_OpticalFlow_X_Outer.Kd = PID_Para_OpticalFlow_XY_Outer[2];

	PID_OpticalFlow_Y_Outer.Kp = PID_Para_OpticalFlow_XY_Outer[0];
    PID_OpticalFlow_Y_Outer.Ki = PID_Para_OpticalFlow_XY_Outer[1];
    PID_OpticalFlow_Y_Outer.Kd = PID_Para_OpticalFlow_XY_Outer[2];
	arm_pid_init_f32(&PID_OpticalFlow_X_Outer,1);
	arm_pid_init_f32(&PID_OpticalFlow_Y_Outer,1);

}

void PID_Attitude_Reset(void)
{
	arm_pid_reset_f32(&PID_Roll_outer);
	arm_pid_reset_f32(&PID_Roll_inner);

	arm_pid_reset_f32(&PID_Pitch_outer);
	arm_pid_reset_f32(&PID_Pitch_inner);

	arm_pid_reset_f32(&PID_Yaw_outer);
	arm_pid_reset_f32(&PID_Yaw_inner);
}

float PID_Roll(const float Target,const float Outer_m,const float Inner_m)
{
	float Outer_Err,Inner_Err,Inner_Target,Output;

	Outer_Err = Target - Outer_m;

	Inner_Target = arm_pid_f32(&PID_Roll_outer,Outer_Err);

	Inner_Err = Inner_Target - Inner_m;

	Output = arm_pid_f32(&PID_Roll_inner,Inner_Err);

#ifdef TEST_PID

	MyDrone.UserData[0] = Target ;
	MyDrone.UserData[1] = Outer_m ;
	MyDrone.UserData[2] = Inner_Target ;
	MyDrone.UserData[3] = Inner_m ;
	MyDrone.UserData[4] = Output ;

#endif

	return Output;
}

float PID_Pitch(const float Target,const float Outer_m,const float Inner_m)
{
	float Outer_Err,Inner_Err,Inner_Target,Output;

	Outer_Err = Target - Outer_m;

	Inner_Target = arm_pid_f32(&PID_Pitch_outer,Outer_Err);

	Inner_Err = Inner_Target - Inner_m;

	Output = arm_pid_f32(&PID_Pitch_inner,Inner_Err);

#ifdef TEST_PID


	// MyDrone.UserData1[0] = Target ;
	// MyDrone.UserData1[1] = Outer_m ;
	// MyDrone.UserData1[2] = Inner_Target ;
	// MyDrone.UserData1[3] = Inner_m ;
	// MyDrone.UserData1[4] = Output ;

#endif

	return Output;

}

float PID_Yaw(const float Target,const float Outer_m,const float Inner_m)
{
	float Outer_Err,Inner_Err,Inner_Target,Output;

	Outer_Err = Target - Outer_m;

	if(Outer_Err>180){Outer_Err-=360;}
	if(Outer_Err<-180){Outer_Err+=360;}

	Inner_Target = arm_pid_f32(&PID_Yaw_outer,Outer_Err);

	Inner_Err = Inner_Target - Inner_m;

	Output = arm_pid_f32(&PID_Yaw_inner,Inner_Err);

#ifdef TEST_PID

	// MyDrone.UserData2[0] = Target ;
	// MyDrone.UserData2[1] = Outer_m ;
	// MyDrone.UserData2[2] = Inner_Target ;
	// MyDrone.UserData2[3] = Inner_m ;
	// MyDrone.UserData2[4] = Output ;

#endif

	return Output;
}
void PID_Altitude_Reset(void)
{
	arm_pid_reset_f32(&PID_Altitude_Pos);
	arm_pid_reset_f32(&PID_Altitude_Vel);
}



float PID_Altitude(const float Target,const float Pos_m,const float Vel_m)
{
	float Pos_Err,Vel_Err,Vel_Target,Output;

	Pos_Err = Target - Pos_m;

	Vel_Target = arm_pid_f32(&PID_Altitude_Pos,Pos_Err);

	Vel_Err = Vel_Target - Vel_m;

	Output = arm_pid_f32(&PID_Altitude_Vel,Vel_Err);

#ifdef TEST_PID

	MyDrone.UserData3[0] = Target ;
	MyDrone.UserData3[1] = Pos_m ;
	MyDrone.UserData3[2] = Vel_Target ;
	MyDrone.UserData3[3] = Vel_m ;
	MyDrone.UserData3[4] = Output + 300 ;

#endif

	return Output+300.0f;
}

float PID_OpticalFlow_X(const float Target,const float Outer_m)
{
	float Outer_Err,Output;

	Outer_Err = Target - Outer_m;

	Output = -arm_pid_f32(&PID_OpticalFlow_X_Outer,Outer_Err);

	MyDrone.UserData1[0] = Target ;
	MyDrone.UserData1[1] = Outer_m ;
	// MyDrone.UserData1[2] = Inner_Target ;
	// MyDrone.UserData1[3] = Inner_m ;
	MyDrone.UserData1[4] = Output ;

	return Output;
}

float PID_OpticalFlow_Y(const float Target,const float Outer_m)
{
	float Outer_Err,Output;

	Outer_Err = Target - Outer_m;

	Output = arm_pid_f32(&PID_OpticalFlow_Y_Outer,Outer_Err);

	MyDrone.UserData2[0] = Target ;
	MyDrone.UserData2[1] = Outer_m ;
	// MyDrone.UserData2[2] = Inner_Target ;
	// MyDrone.UserData2[3] = Inner_m ;
	MyDrone.UserData2[4] = Output ;

	return Output;
}

float PID_P(float target,float measure,float Kp)
{
	return Kp*(target-measure);
}
