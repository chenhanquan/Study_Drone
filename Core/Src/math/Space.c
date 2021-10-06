/*
 * Space.c
 *
 *  Created on: 2021年2月10日
 *      Author: 90797
 */
#include "Space.h"
#include "arm_math.h"

void Space_RotationMatrix(float *axis_body,float *axis_n,float *angle,enum Axis_dir dir)
{
	float pi_alpha,pi_beta,pi_gama;
	float sin_alpha,sin_beta,sin_gama,cos_alpha,cos_beta,cos_gama;
	float Mat_Rotation_buf[9];
	arm_matrix_instance_f32 Mat_Rotation;
	arm_matrix_instance_f32 Mat_axis_body;
	arm_matrix_instance_f32 Mat_axis_n;

	arm_mat_init_f32(&Mat_axis_body,3,1,axis_body);
	arm_mat_init_f32(&Mat_axis_n,3,1,axis_n);

	pi_alpha=angle[0]*0.01745329f;
	pi_beta=angle[1]*0.01745329f;
	pi_gama=angle[2]*0.01745329f;

	sin_alpha=arm_sin_f32(pi_alpha);
	sin_beta=arm_sin_f32(pi_beta);
	sin_gama=arm_sin_f32(pi_gama);

	cos_alpha=arm_cos_f32(pi_alpha);
	cos_beta=arm_cos_f32(pi_beta);
	cos_gama=arm_cos_f32(pi_gama);

	Mat_Rotation_buf[0]=cos_beta*cos_gama;
	Mat_Rotation_buf[1]=cos_beta*sin_gama;
	Mat_Rotation_buf[2]=-sin_beta;

	Mat_Rotation_buf[3]=cos_gama*sin_alpha*sin_beta-cos_alpha*sin_gama;
	Mat_Rotation_buf[4]=cos_alpha*cos_gama+sin_alpha*sin_beta*sin_gama;
	Mat_Rotation_buf[5]=cos_beta*sin_alpha;

	Mat_Rotation_buf[6]=sin_alpha*sin_gama+cos_alpha*cos_gama*sin_beta;
	Mat_Rotation_buf[7]=cos_alpha*sin_beta*sin_gama-cos_gama*sin_alpha;
	Mat_Rotation_buf[8]=cos_alpha*cos_beta;

	arm_mat_init_f32(&Mat_Rotation,3,3,Mat_Rotation_buf);

	if(dir==n2b)
	{
		arm_mat_mult_f32(&Mat_Rotation,&Mat_axis_n,&Mat_axis_body);
	}
	else
	{
		float Mat_Rotation_T_buf[9]={0};
		arm_matrix_instance_f32 Mat_Rotation_T;
		arm_mat_init_f32(&Mat_Rotation_T,3,3,Mat_Rotation_T_buf);
		arm_mat_trans_f32(&Mat_Rotation,&Mat_Rotation_T);
		arm_mat_mult_f32(&Mat_Rotation_T,&Mat_axis_body,&Mat_axis_n);
	}

}

void Space_RotationPlant(float *axis_body,float *axis_n,float Yaw_angle,enum Axis_dir dir)
{
	float Yaw_angle_rad;
	Yaw_angle_rad=Yaw_angle*0.0174532925f;
	if(dir==n2b)
	{
		axis_body[0]=axis_n[0]*cos(Yaw_angle_rad)+axis_n[1]*sin(Yaw_angle_rad);
		axis_body[1]=axis_n[1]*cos(Yaw_angle_rad)-axis_n[0]*sin(Yaw_angle_rad);
	}
	else
	{
		axis_n[0]=axis_body[0]*cos(Yaw_angle_rad)+axis_body[1]*sin(Yaw_angle_rad);
		axis_n[1]=axis_body[0]*sin(Yaw_angle_rad)+axis_body[1]*cos(Yaw_angle_rad);
	}
}


