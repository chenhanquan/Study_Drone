#include "Fusion_OpticalFlow.h"

#define FUSION_TIME 0.01f

void Fusion_OpticalFlowData(float *Acc_Plant,float *OpticalFlowData,float *OutputData)
{
    static float Est_v[2];

    OutputData[0] = Est_v[0] + 0.2f*(OpticalFlowData[0]-Est_v[0]);
    OutputData[1] = Est_v[1] + 0.2f*(OpticalFlowData[1]-Est_v[1]);

    Est_v[0] = OutputData[0];
    Est_v[1] = OutputData[1];

    Est_v[0] += Acc_Plant[0]*FUSION_TIME;
    Est_v[1] += Acc_Plant[1]*FUSION_TIME;
}


