/*
 * NonlinearFunction.h
 *
 *  Created on: 2021年1月22日
 *      Author: 90797
 */

#ifndef INC_MATH_NONLINEARFUNCTION_H_
#define INC_MATH_NONLINEARFUNCTION_H_

float sign(const float x);
float fsg(const float x,const float a,const float b);
float fdb(const float x,const float a,const float b);
float fss(const float x,const float a,const float b);
float fst_zero(const float x,const float d);
float fst(const float x,const float a,const float b);
float fal(const float x,const float alpha,const float gama);
float sat(const float x,const float sigma);
float xsign(const float x,const float alpha);
float fhan(const float x1,const float x2,const float r,const float h);
float fsun(const float x1,const float x2,const float r,const float h);

#endif /* INC_MATH_NONLINEARFUNCTION_H_ */
