/*
 * OpticalFlow.h
 *
 *  Created on: May 6, 2021
 *      Author: 90797
 */

#ifndef INC_DEV_OPTICALFLOW_H_
#define INC_DEV_OPTICALFLOW_H_

void OpticalFlow_Init(void);
void OpticalFlow_Read(float *data);
void OpticalFlow_DMA(void);


#endif /* INC_DEV_OPTICALFLOW_H_ */
