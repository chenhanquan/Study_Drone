#include "IMU.h"
#include "main.h"
#include "arm_math.h"

#define RAD2DEG  57.29578f
#define DEG2RAD  0.017453f
#define IMU_dt 0.01f

// #define LIMIT( x,min,max ) ( (x) < (min)  ? (min) : ( (x) > (max) ? (max) : (x) ) )

// static float invSqrt(float x) ;

void IMU_Process(IMU_TypeDef *IMU_Data,const float *acc,const float *gyro,const float *mag)
{		
	float angle_k[3];
	static float angle_rad[3];

	float norm,Fusion_k;

	//k时刻计算角度，融合角度
	IMU_AccCalRollPitch(acc,angle_k);

	// arm_sqrt_f32(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2], &norm);

	// if(norm>=9.80665f){Fusion_k=(9.80665f/norm);}
	// else{Fusion_k=(norm/9.80665f);}

	// angle_rad[0]=angle_rad[0]+0.01f*Fusion_k*Fusion_k*(angle_k[0]-angle_rad[0]);
	// angle_rad[1]=angle_rad[1]+0.01f*Fusion_k*Fusion_k*(angle_k[1]-angle_rad[1]);

	angle_rad[0]=angle_rad[0]+0.01f*(angle_k[0]-angle_rad[0]);
	angle_rad[1]=angle_rad[1]+0.01f*(angle_k[1]-angle_rad[1]);

	// angle_rad[0]=angle_rad[0];

	IMU_Data->angle[0]=angle_rad[0]*RAD2DEG;
	IMU_Data->angle[1]=angle_rad[1]*RAD2DEG;
	// IMU_Data->angle[1]=angle_k[0]*RAD2DEG;
	// IMU_Data->angle[1]=angle_rad[0]*RAD2DEG;

	// float Yaw_Fusion_rad;
	angle_k[2]=IMU_MagCalYaw(mag,angle_rad);	//磁力计计算偏航

	if(fabs(angle_k[2]-angle_rad[2])>PI&&fabs(angle_rad[2])>2.7925268)
	{
		angle_rad[2]=angle_k[2];
	}
	else
	{
		if(fabs(IMU_Data->angle[0])<10.0f&&fabs(IMU_Data->angle[1])<10.0f)
		{
			angle_rad[2]=angle_rad[2]+0.01f*(angle_k[2]-angle_rad[2]);
			if(angle_rad[2]>PI){angle_rad[2]-=2*PI;}
			if(angle_rad[2]<-PI){angle_rad[2]+=2*PI;}
		}
		
	}
	IMU_Data->angle[2]=angle_rad[2]*RAD2DEG;
	

	//预测下一时刻角度
	angle_rad[2]=angle_rad[2]+gyro[2]*IMU_dt*DEG2RAD;
	if(angle_rad[2]>PI){angle_rad[2]-=2*PI;}
	if(angle_rad[2]<-PI){angle_rad[2]+=2*PI;}


	angle_rad[0]=angle_rad[0]+gyro[0]*IMU_dt*DEG2RAD;
	angle_rad[1]=angle_rad[1]+gyro[1]*IMU_dt*DEG2RAD;

}

// /*********************************************************************************************************
// *函  数：void AHRS_compute(float gx, float gy, float gz, float ax, float ay, float az) 
// *功　能：梯度下降姿态解算
// *参  数：无
// *返回值：无
// *备  注：无
// **********************************************************************************************************/	
// #define Quad_Num 10                     //四元数最大存储深度
// float Acceleration_Length=0.0f,hisGYRO_Length=0.0f,GYRO_Length=0.0f,Tmep_gyro_Length[350],Tmep_Acce_Length=0,Quad_Buf[10][4]={0},att_q[4]={1,0,0,0}; 
// float GYRO_rms,BETADEF,Beta_Adjust[2]={0.03f,0.025f};
// float CNTLCYCLE1=0.005f,CNTLCYCLE2=0.000005f,CNTLCYCLE_X=0.0f,CNTLCYCLE_Y=0.0f,CNTLCYCLE_Z=0.0f;
// uint16_t Quad_Delay=0,numGYRO_Length=0;
// float delta;               				//角速度模长 
// float BETADEF=0.04f;
// float Begain_Imu_Time,End_Imu_time,CNTLCYCLE=0.01f;//周期

// extern uint32_t Int_Count;

// void IMU_Process(IMU_TypeDef *IMU_Data,const float *acc,const float *gyro,const float *mag)
// {		
// 	float recipNorm;					           	//平方根分之一
// 	float s0, s1, s2, s3;					     	 //梯度下降算子求出来的姿态四元数微分
// 	float qDot1, qDot2, qDot3, qDot4;				//四元数姿态微分
// 	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;//四元数乘积组合                                      
// 	uint16_t i;

// 	float gx,gy,gz,ax,ay,az;

// 	gx = gyro[1] * DEG2RAD;
// 	gy = gyro[0] * DEG2RAD;
// 	gz = gyro[2] * DEG2RAD;

// 	ax = -acc[1];
// 	ay = -acc[0];
// 	az = acc[2];
	
// 	End_Imu_time=Begain_Imu_Time;//结束时间
// 	Begain_Imu_Time=(10000*Int_Count+TIM2->CNT)/1000; //开始时间 ms
// 	CNTLCYCLE=(Begain_Imu_Time-End_Imu_time)/1000;//姿态解算周期 ms化为s

// 										/////////////陀螺仪数据处理////////////
// 	GYRO_Length=1/invSqrt(gx*RAD2DEG*gx*RAD2DEG+gy*RAD2DEG*gy*RAD2DEG);      //角速度角度制的模长 
// 	//角速度数据更新四元数姿态微分
// 	qDot1 = 0.5f * (-Quad_Buf[Quad_Delay][1] * gx - Quad_Buf[Quad_Delay][2] * gy - Quad_Buf[Quad_Delay][3] * gz); 
// 	qDot2 = 0.5f * ( Quad_Buf[Quad_Delay][0] * gx + Quad_Buf[Quad_Delay][2] * gz - Quad_Buf[Quad_Delay][3] * gy);
// 	qDot3 = 0.5f * ( Quad_Buf[Quad_Delay][0] * gy - Quad_Buf[Quad_Delay][1] * gz + Quad_Buf[Quad_Delay][3] * gx);
// 	qDot4 = 0.5f * ( Quad_Buf[Quad_Delay][0] * gz + Quad_Buf[Quad_Delay][1] * gy - Quad_Buf[Quad_Delay][2] * gx);


// 										////////////加速度计数据处理//////////
// 	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))      //判断加速度计数据是否有效                                                
// 	{         
// 		recipNorm=invSqrt(ax * ax + ay * ay + az * az);     	//数据归一化
// 		Acceleration_Length=fabs(1/recipNorm-9.80665f);                 //加速度模长
// 		ax *= recipNorm;				//加速度计数据归一化
// 		ay *= recipNorm;
// 		az *= recipNorm;

// 		_2q0 = 2.0f * att_q[0];        //定义过程变量减少计算量
// 		_2q1 = 2.0f * att_q[1];
// 		_2q2 = 2.0f * att_q[2]; 
// 		_2q3 = 2.0f * att_q[3];
// 		_4q0 = 4.0f * att_q[0];
// 		_4q1 = 4.0f * att_q[1];
// 		_4q2 = 4.0f * att_q[2]; 
// 		_8q1 = 8.0f * att_q[1];
// 		_8q2 = 8.0f * att_q[2];
// 		q0q0 = att_q[0] * att_q[0];
// 		q1q1 = att_q[1] * att_q[1];
// 		q2q2 = att_q[2] * att_q[2];
// 		q3q3 = att_q[3] * att_q[3];
		
// 		//加速度计更新四元数微分方程，梯度下降算法，计算误差的函数梯度                                                                                                                        
// 		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;             
// 		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * att_q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
// 		s2 = 4.0f * q0q0 * att_q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
// 		s3 = 4.0f * q1q1 * att_q[3] - _2q1 * ax + 4.0f * q2q2 * att_q[3] - _2q2 * ay;
		
// 		/*梯度归一化 */                         
// 		recipNorm=invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); 
// 		s0 *= recipNorm;
// 		s1 *= recipNorm;
// 		s2 *= recipNorm;
// 		s3 *= recipNorm;

// 		Tmep_Acce_Length=LIMIT(Acceleration_Length,0,20);//限幅 正常在10以内
// 		GYRO_rms=LIMIT(GYRO_Length,0,1000);//限幅 正常在500以内
// 		BETADEF=Beta_Adjust[0]-Tmep_Acce_Length*0.005f+GYRO_rms*0.0001f;//动态步长 加速度模长抑制步长，角速度模长加速步长，抑制重力加速度的同时加速收敛
// 		qDot1 -= BETADEF * s0;
// 		qDot2 -= BETADEF * s1;
// 		qDot3 -= BETADEF * s2;
// 		qDot4 -= BETADEF * s3;  
// 	}


// 	delta = (CNTLCYCLE * gx) * (CNTLCYCLE * gx) + (CNTLCYCLE * gy) * (CNTLCYCLE * gy) + (CNTLCYCLE * gz) * (CNTLCYCLE * gz);
// 	//二阶毕卡求解微分方程，补偿四元数微分方程引入的误差，积分四元数的导数得到姿态四元数
// 	att_q[0] = (1.0f - delta / 8.0f) * att_q[0] + qDot1 * CNTLCYCLE;                                       
// 	att_q[1] = (1.0f - delta / 8.0f) * att_q[1] + qDot2 * CNTLCYCLE;
// 	att_q[2] = (1.0f - delta / 8.0f) * att_q[2] + qDot3 * CNTLCYCLE;
// 	att_q[3] = (1.0f - delta / 8.0f) * att_q[3] + qDot4 * CNTLCYCLE;

// 	//四元数归一化
// 	recipNorm=invSqrt(att_q[0] * att_q[0] + att_q[1] * att_q[1] + att_q[2] * att_q[2] + att_q[3] * att_q[3]); 
// 	att_q[0] *= recipNorm;
// 	att_q[1] *= recipNorm;
// 	att_q[2] *= recipNorm;
// 	att_q[3] *= recipNorm;
	
	
// 	IMU_Data->angle[1] = atan2(2.0f * att_q[2] * att_q[3] + 2.0f * att_q[0] * att_q[1], -2.0f * att_q[1] * att_q[1] - 2.0f * att_q[2]* att_q[2] + 1.0f) * RAD2DEG;
// 	IMU_Data->angle[0]= asin(2.0f * att_q[0]* att_q[2]-2.0f * att_q[1] * att_q[3]) * RAD2DEG;								

// 	float Real_Yaw,Rad_Angle[3];  
// 	static float Yaw_Pre;
// 	Rad_Angle[0] = IMU_Data->angle[0] * DEG2RAD;
// 	Rad_Angle[1] = IMU_Data->angle[1] * DEG2RAD;
// 	Real_Yaw=IMU_MagCalYaw(mag,Rad_Angle);	//磁力计计算偏航

// 	if(fabs(Real_Yaw-Yaw_Pre)>PI&&fabs(Yaw_Pre)>2.7925268)
// 	{
// 		Yaw_Pre=Real_Yaw;
// 	}
// 	else
// 	{
// 		if(fabs(IMU_Data->angle[0])<10.0f&&fabs(IMU_Data->angle[1])<10.0f)
// 		{
// 			Yaw_Pre=Yaw_Pre+0.01f*(Real_Yaw-Yaw_Pre);
// 			if(Yaw_Pre>PI){Yaw_Pre-=2*PI;}
// 			if(Yaw_Pre<-PI){Yaw_Pre+=2*PI;}
// 		}
// 	}

// 	IMU_Data->angle[2]=Yaw_Pre*RAD2DEG;

// 	//预测下一时刻角度
// 	Yaw_Pre=Yaw_Pre+gyro[2]*IMU_dt*DEG2RAD;
// 	if(Yaw_Pre>PI){Yaw_Pre-=2*PI;}
// 	if(Yaw_Pre<-PI){Yaw_Pre+=2*PI;}
		 
// 	for(i=Quad_Num-1;i>0;i--) //保存四元数历史值    
// 	{
// 		Quad_Buf[i][0]=Quad_Buf[i-1][0];
// 		Quad_Buf[i][1]=Quad_Buf[i-1][1];
// 		Quad_Buf[i][2]=Quad_Buf[i-1][2];
// 		Quad_Buf[i][3]=Quad_Buf[i-1][3];
// 	}
// 		Quad_Buf[0][0]=att_q[0];     //最新四元数更新到末端备用
// 		Quad_Buf[0][1]=att_q[1];
// 		Quad_Buf[0][2]=att_q[2];
// 		Quad_Buf[0][3]=att_q[3];

// }

// // /**************************实现函数*********************************************************************
// // 函  数：static float invSqrt(float x) 
// // 功　能: 快速计算 1/Sqrt(x) 	
// // 参  数：要计算的值
// // 返回值：结果
// // 备  注：比普通Sqrt()函数要快四倍See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
// // *********************************************************************************************************/
// static float invSqrt(float x) 
// {
// 	float halfx = 0.5f * x;
// 	float y = x;
// 	long i = *(long*)&y;
// 	i = 0x5f3759df - (i>>1);
// 	y = *(float*)&i;
// 	y = y * (1.5f - (halfx * y * y));
// 	return y;
// }

void IMU_AccCalRollPitch(const float *acc,float *angle)
{
	float norm,acc_norm[3];
	if(acc[0]==0&&acc[1]==0&&acc[2]==0)
	{
		angle[0]=0.0000001f;
		angle[1]=0.0000001f;
	}
	else
	{
		arm_sqrt_f32(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2], &norm);		//加速度计归一化
		acc_norm[0]=-acc[0]/norm;
		acc_norm[1]=-acc[1]/norm;
		acc_norm[2]=-acc[2]/norm;

		angle[0]=atan2(acc_norm[1],acc_norm[2]);		//roll
		angle[1]=asin(-acc_norm[0]);					//pitch
	}
}

float IMU_MagCalYaw(const float *mag,const float *angle)
{
	float Bx,By;
	float angle_Yaw=0.0f;

	Bx=mag[0]*cos(angle[1]) + mag[1]*sin(angle[1])*sin(angle[0])+mag[2]*sin(angle[1])*cos(angle[0]);
	By=mag[1]*cos(angle[0]) - mag[2]*sin(angle[0]);

	if(By==0.0f&&Bx>0){angle_Yaw=0;}
	else if(By<0){angle_Yaw=PI/2.0f+atan(Bx/By);}
	else if(By==0.0f&&Bx<0){angle_Yaw=PI;}
	else if(By>0.0f){angle_Yaw=-PI/2.0f+atan(Bx/By);}

// 	float TempX,TempY;
// 	float YAW;
// 	TempX = mag[0] * cos(angle[0])+ mag[2] * sin(angle[1]);
// 	TempY = mag[0] * sin(angle[0]) * sin(angle[1])
// 					+mag[1] * cos(angle[0])
// 					-mag[2] * cos(angle[1])*sin(angle[0]);

// 	YAW = atan2(TempX,TempY);

// //	return angle_Yaw;
// 	return YAW;

	return angle_Yaw;
}







