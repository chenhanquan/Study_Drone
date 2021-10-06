/*
 * I2C_Software.c
 *
 *  Created on: Feb 9, 2021
 *      Author: 90797
 */
#include "I2C_Software.h"
#include "main.h"
#include "MyDelay.h"

#define I2C_Port	GPIOB
#define I2C_Pin_SCL		GPIO_PIN_6
#define I2C_Pin_SDA		GPIO_PIN_7

#define SCL_H         I2C_Port->BSRR |= I2C_Pin_SCL
#define SCL_L         I2C_Port->BSRR |= (I2C_Pin_SCL << 16)
#define SDA_H         I2C_Port->BSRR |= I2C_Pin_SDA
#define SDA_L         I2C_Port->BSRR |= (I2C_Pin_SDA << 16)
#define SCL_read      I2C_Port->IDR  & I2C_Pin_SCL
#define SDA_read      I2C_Port->IDR  & I2C_Pin_SDA

#undef SUCCESS
#define SUCCESS 0
#undef FAILED
#define FAILED  1

void I2C_Software_Delay()
{
//	__NOP();__NOP();__NOP();
//	__NOP();__NOP();__NOP();
//	__NOP();__NOP();__NOP();

//	uint8_t i=20;
//	while(i--);
	
	HAL_Delay_us(3);

}

int I2C_Software_Start()
{
	SDA_H;
	SCL_H;
	I2C_Software_Delay();
	if(~SDA_read)return FAILED;	//SDAÏßÎªµÍµçÆ½Ôò×ÜÏßÃ¦,ÍË³ö
	SDA_L;
	I2C_Software_Delay();
	if(SDA_read) return FAILED;	//SDAÏßÎª¸ßµçÆ½Ôò×ÜÏß³ö´í,ÍË³ö
	SDA_L;
	I2C_Software_Delay();
	return SUCCESS;

}

void I2C_Software_Stop()
{
	SCL_L;
	I2C_Software_Delay();
	SDA_L;
	I2C_Software_Delay();
	SCL_H;
	I2C_Software_Delay();
	SDA_H;
	I2C_Software_Delay();
}

void I2C_Software_Ask()
{
	SCL_L;
	I2C_Software_Delay();
	SDA_L;
	I2C_Software_Delay();
	SCL_H;
	I2C_Software_Delay();
	SCL_L;
	I2C_Software_Delay();
}

void I2C_Software_NoAsk()
{
	SCL_L;
	I2C_Software_Delay();
	SDA_H;
	I2C_Software_Delay();
	SCL_H;
	I2C_Software_Delay();
	SCL_L;
	I2C_Software_Delay();
}

int I2C_Software_WaitAsk(void) 	 //·µ»ØÎª:=1ÎÞASK,=0ÓÐASK
{
  uint8_t ErrTime = 0;
	SCL_L;
	I2C_Software_Delay();
	SDA_H;
	I2C_Software_Delay();
	SCL_H;
	I2C_Software_Delay();
	while(SDA_read)
	{
		ErrTime++;
		if(ErrTime>50)
		{
			I2C_Software_Stop();
			return FAILED;
		}
	}
	SCL_L;
	I2C_Software_Delay();
	return SUCCESS;
}

void I2C_Software_SendByte(uint8_t SendByte) //Êý¾Ý´Ó¸ßÎ»µ½µÍÎ»//
{
    uint8_t i=8;
    while(i--)
    {
        SCL_L;
        I2C_Software_Delay();
      if(SendByte&0x80)
        SDA_H;
      else
        SDA_L;
        SendByte<<=1;
        I2C_Software_Delay();
		SCL_H;
		I2C_Software_Delay();
    }
    SCL_L;
}

//¶Á1¸ö×Ö½Ú£¬ack=1Ê±£¬·¢ËÍACK£¬ack=0£¬·¢ËÍNACK
uint8_t I2C_Software_ReadByte(uint8_t ask)  //Êý¾Ý´Ó¸ßÎ»µ½µÍÎ»//
{
    uint8_t i=8;
    uint8_t ReceiveByte=0;

    SDA_H;
    while(i--)
    {
      ReceiveByte<<=1;
      SCL_L;
      I2C_Software_Delay();
			SCL_H;
      I2C_Software_Delay();
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L;

	if (ask)
		I2C_Software_Ask();
	else
		I2C_Software_NoAsk();
    return ReceiveByte;
}


// IICÐ´Ò»¸ö×Ö½ÚÊý¾Ý
uint8_t I2C_Software_Write1Byte(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data)
{
	I2C_Software_Start();
	I2C_Software_SendByte(SlaveAddress);
	if(I2C_Software_WaitAsk() == FAILED)
	{
		I2C_Software_Stop();
		return FAILED;
	}
	I2C_Software_SendByte(REG_Address);
	I2C_Software_WaitAsk();
	I2C_Software_SendByte(REG_data);
	I2C_Software_WaitAsk();
	I2C_Software_Stop();
	return SUCCESS;
}

// IIC¶Á1×Ö½ÚÊý¾Ý
uint8_t I2C_Software_Read1Byte(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t *REG_data)
{
	I2C_Software_Start();
	I2C_Software_SendByte(SlaveAddress);
	if(I2C_Software_WaitAsk() == FAILED)
	{
		I2C_Software_Stop();
		return FAILED;
	}
	I2C_Software_SendByte(REG_Address);
	I2C_Software_WaitAsk();
	I2C_Software_Start();
	I2C_Software_SendByte(SlaveAddress+1);
	I2C_Software_WaitAsk();
	*REG_data= I2C_Software_ReadByte(0);
	I2C_Software_Stop();
	return SUCCESS;
}

// IICÐ´n×Ö½ÚÊý¾Ý
uint8_t I2C_Software_WriteNByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t len, uint8_t *buf)
{
	I2C_Software_Start();
	I2C_Software_SendByte(SlaveAddress);
	if(I2C_Software_WaitAsk() == FAILED)
	{
		I2C_Software_Stop();
		return FAILED;
	}
	I2C_Software_SendByte(REG_Address);
	I2C_Software_WaitAsk();
	while(len--)
	{
		I2C_Software_SendByte(*buf++);
		I2C_Software_WaitAsk();
	}
	I2C_Software_Stop();
	return SUCCESS;
}

uint8_t I2C_Software_ReadNByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t len, uint8_t *buf)
{
    if (I2C_Software_Start() == FAILED)
        return FAILED;
    I2C_Software_SendByte(SlaveAddress);
    if (I2C_Software_WaitAsk() == FAILED)
    {
        I2C_Software_Stop();
        return FAILED;
    }
    I2C_Software_SendByte(REG_Address);
    I2C_Software_WaitAsk();
	I2C_Software_Stop();
    I2C_Software_Start();
    I2C_Software_SendByte(SlaveAddress+1);
    if (I2C_Software_WaitAsk() == FAILED)
    {
        I2C_Software_Stop();
        return FAILED;
    }
	while(len)
	{
		if(len == 1)
		{
			*buf = I2C_Software_ReadByte(0);
		}
		else
		{
			*buf = I2C_Software_ReadByte(1);
		}
		buf++;
		len--;
	}
    I2C_Software_Stop();
    return SUCCESS;
}

uint8_t I2C_Software_Transmit(uint8_t SlaveAddress, uint8_t len, uint8_t *buf)
{
	I2C_Software_Start();
	I2C_Software_SendByte(SlaveAddress);
	if(I2C_Software_WaitAsk() == FAILED)
	{
		I2C_Software_Stop();
		return FAILED;
	}
	while(len--)
	{
		I2C_Software_SendByte(*buf++);
		I2C_Software_WaitAsk();
	}
	I2C_Software_Stop();
	return SUCCESS;
}

uint8_t I2C_Software_Receive(uint8_t SlaveAddress, uint8_t len, uint8_t *buf)
{
    if (I2C_Software_Start() == FAILED)
        return FAILED;
    I2C_Software_SendByte(SlaveAddress);
    if (I2C_Software_WaitAsk() == FAILED) {
        I2C_Software_Stop();
        return FAILED;
    }
	while(len)
	{
		if(len == 1)
		{
			*buf = I2C_Software_ReadByte(0);
		}
		else
		{
			*buf = I2C_Software_ReadByte(1);
		}
		buf++;
		len--;
	}
    I2C_Software_Stop();
    return SUCCESS;
}

