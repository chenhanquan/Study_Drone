/*
 * NonlinearFunction.c
 *
 *  Created on: 2021年1月22日
 *      Author: 90797
 */
#include "NonlinearFunction.h"
#include <stdint.h>
#include "math.h"

//符号函数：x>0,y=1  x<0,y=-1 x=0.y=0
float sign(const float x)
{
	float y=0;
	if(x>0){y=1;}
	else if(x<0){y=-1;}
	else{y=0;}
	return y;
}

//区间函数：区间内取值为1
float fsg(const float x,const float a,const float b)
{
	float y=0;
	y=0.5f*(sign(x-a)-sign(x-b));
	return y;
}

//死区函数：区间内取值为0
float fdb(const float x,const float a,const float b)
{
	float y=0;
	y=0.5f*(sign(x-a)+sign(x-b));
	return y;
}

//最优开关函数：(a<b)
float fss(const float x,const float a,const float b)
{
	float y=0;
	y=0.5f*(sign(x-a)*(1.0f-sign(x-b)));
	return y;
}

//零点线性饱和函数
float fst_zero(const float x,const float d)
{
	float y=0,x_abs=0;
	x_abs=fabs(x);
	if(x_abs<d){y=x;}
	else{y=sign(x);}
	return y;
}

//任意区间饱和函数
float fst(const float x,const float a,const float b)
{
	float y=0;
	if(x>b){y=1.0f;}
	else if(x<a){y=0;}
	else{y=(x-a)/(b-a);}
	return y;
}

//幂次函数
float fal(const float x,const float alpha,const float gama)
{
	float x_abs,y;
	x_abs=fabs(x);
	if(x_abs<gama){y=x/pow(gama,1.0f-alpha);}
	else{y=pow(x_abs,alpha)*sign(x);}
	return y;
}

float sat(const float x,const float sigma)
{
	float x_abs,y;
	x_abs=fabs(x);
	if(x_abs>sigma){y=sign(x);}
	else{y=x/sigma;}
	return y;
}

float xsign(const float x,const float alpha)
{
	float y;
	y=pow(fabs(x),alpha)*sign(x);
	return y;
}

//二阶最优最速综合函数
float fhan(const float x1,const float x2,const float r,const float h)
{
	float d,a0,y0,a1,a2,a,y;
	d=r*h*h;
	a0=x2*h;
	y0=x1+a0;
	a1=sqrt(d*(d+8*fabs(y0)));
	a2=a0+sign(y0)*(a1-d)*0.5f;
	a=(a0+y0)*fsg(y0,-d,d)+a2*(1-fsg(y0,-d,d));
	y=-r*(a/d)*fsg(a,-d,d)-r*sign(a)*(1-fsg(a,-d,d));
	return y;
}

//二阶最优最速综合函数
float fsun(const float x1,const float x2,const float r,const float h)
{
	float y,y_abs,k_,k,h2r,f;
	uint32_t fix_k_;
	y=x1+h*x2;
	y_abs=fabs(y);
	h2r=h*h*r;
	k_=0.5f*(1+sqrt(1+8*y_abs/h2r));
	fix_k_=k_;
	k=sign(k_-fix_k_)+fix_k_;
	if(y_abs>h2r)
	{
		f=r*sat((1.0f-0.5f*k)*sign(x1)-(x1+k*h*x2)/((k-1.0f)*h2r),1);
	}
	else
	{
		f=-r*sat(x2+y/h,h*r);
	}
	return f;
}
