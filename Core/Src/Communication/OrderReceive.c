#include "OrderReceive.h"
#include "main.h"


extern UART_HandleTypeDef huart4;
#define RECEIVE_USART huart4
#define Receive_Size 7


uint8_t Receive_Buf[Receive_Size];
int16_t Pos_X,Pos_Y,Pos_Z;

void OrderReceive_Init(void)
{
    HAL_UART_Receive_DMA(&RECEIVE_USART, Receive_Buf, Receive_Size);
}

void OrderReceive_Analysis_DMA(void)
{
    switch (Receive_Buf[0])
    {
    case 0x00/* constant-expression */:
        HAL_UART_DMAStop(&RECEIVE_USART);
        OrderReceive_Init();
        break;

    case 0x01/* constant-expression */:
        Pos_X=(int16_t)((Receive_Buf[1]<<8) | Receive_Buf[2]);
        Pos_Y=(int16_t)((Receive_Buf[3]<<8) | Receive_Buf[4]);
        Pos_Z=(int16_t)((Receive_Buf[5]<<8) | Receive_Buf[6]);
        break;

    case 0x02/* constant-expression */:
        /* code */
        break;
    
    default:
        break;
    }
}