#include "UWB_Position.h"


#define FUSIONTIME 0.02f


void UWB_Position_GetPositionAndVelocity(UWB_Position_TypeDef *UWB_Position_Struct,const float X_Acc,const float Y_Acc,const float X_Position,const float Y_Position)
{
	static float Position_X_Pre;
	static float Velocity_X_Pre;
	static float Last_Position_X;
    float Velocity_X;

    static float Position_Y_Pre;
	static float Velocity_Y_Pre;
	static float Last_Position_Y;
	float Velocity_Y;

    //X轴
	UWB_Position_Struct->Fusion_Pos_X+=0.03f*(X_Position-Position_X_Pre);

	Velocity_X=(X_Position-Last_Position_X)/FUSIONTIME;
	UWB_Position_Struct->Velocity_X +=0.03f*(Velocity_X-Velocity_X_Pre);

	Velocity_X_Pre=UWB_Position_Struct->Velocity_X+X_Acc*FUSIONTIME;
	Position_X_Pre=UWB_Position_Struct->Fusion_Pos_X + UWB_Position_Struct->Velocity_X * FUSIONTIME;

    Last_Position_X=X_Position;

    //Y轴
    UWB_Position_Struct->Fusion_Pos_Y+=0.03f*(Y_Position-Position_Y_Pre);

	Velocity_Y=(Y_Position-Last_Position_Y)/FUSIONTIME;
	UWB_Position_Struct->Velocity_Y +=0.03f*(Velocity_Y-Velocity_Y_Pre);

	Velocity_Y_Pre=UWB_Position_Struct->Velocity_Y + Y_Acc*FUSIONTIME;
	Position_Y_Pre=UWB_Position_Struct->Fusion_Pos_Y + UWB_Position_Struct->Velocity_Y * FUSIONTIME;

    Last_Position_Y=Y_Position;
}