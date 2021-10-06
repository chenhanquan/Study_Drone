#include "UWB.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

extern UART_HandleTypeDef huart3;

#define UWB_UART huart3
#define UWB_DATASIZE 166

char UWB_Data[UWB_DATASIZE];
int X_Pos,Y_Pos;

// void UWB_Init(void)
// {
//     HAL_UART_Receive_DMA(&UWB_UART, (uint8_t *)UWB_Data, 163);
// }

void UWB_IdleIT(void)
{
    if(__HAL_UART_GET_FLAG(&UWB_UART,UART_FLAG_IDLE))
    {
        HAL_UART_DMAStop(&UWB_UART);
        HAL_UART_Receive_DMA(&UWB_UART, (uint8_t *)UWB_Data, UWB_DATASIZE);
        __HAL_UART_CLEAR_IDLEFLAG(&UWB_UART);
    };
    
}

void UWB_GetData_DMA(void)
{
    char X_Data[10],Y_Data[10];
    char *X_Buf,*Y_Buf;
    uint8_t X_num,Y_num;
    
    char *X_equ,*Y_equ;
    char *X_cm,*Y_cm;
    char *Local_X=strchr(UWB_Data,'X');
    char *Local_Y=strchr(UWB_Data,'Y');

    strncpy(X_Data, Local_X, 10);
    strncpy(Y_Data, Local_Y, 10);
    X_equ=strchr(X_Data,'=');
    Y_equ=strchr(Y_Data,'=');
    X_cm=strstr(X_Data,'cm');
    Y_cm=strstr(Y_Data,'cm');
    X_num=((X_cm-2)-(X_equ+1));
    Y_num=((Y_cm-2)-(Y_equ+1));
    X_Buf = (char *)malloc(sizeof(char)*X_num);
    Y_Buf = (char *)malloc(sizeof(char)*Y_num);
    strncpy(X_Buf, (X_equ+2), X_num);
    strncpy(Y_Buf, (Y_equ+2), Y_num);

    X_Pos=atoi(X_Buf);
    Y_Pos=atoi(Y_Buf);
    free(X_Buf);
    free(Y_Buf);
}

void UWB_GetData(int *X_Local,int *Y_Local)
{
    *X_Local = X_Pos;
    *Y_Local = -Y_Pos;
}