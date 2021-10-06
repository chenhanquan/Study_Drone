#ifndef INC_UWB_POSITION_H_
#define INC_UWB_POSITION_H_

#include "stm32f4xx_hal.h"

typedef struct{
	float Fusion_Pos_X;
    float Fusion_Pos_Y;
	float Velocity_X;
    float Velocity_Y;
	
}UWB_Position_TypeDef;

void UWB_Position_GetPositionAndVelocity(UWB_Position_TypeDef *UWB_Position_Struct,const float X_Acc,const float Y_Acc,const float X_Position,const float Y_Position);

#endif /* INC_DATAFUSION_ALTITUDE_H_ */