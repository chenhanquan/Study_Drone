/*
 * I2C_Software.h
 *
 *  Created on: Feb 9, 2021
 *      Author: 90797
 */

#ifndef INC_DEV_I2C_SOFTWARE_H_
#define INC_DEV_I2C_SOFTWARE_H_

#include "stm32f4xx_hal.h"

//I2C内部函数
void I2C_Software_Delay(void);
int I2C_Software_Start(void);
void I2C_Software_Stop(void);
void I2C_Software_Ask(void);
void I2C_Software_NoAsk(void);
int I2C_Software_WaitAsk(void);
void I2C_Software_SendByte(uint8_t SendByte);
uint8_t I2C_Software_ReadByte(uint8_t ask);


//I2C外部函数
uint8_t I2C_Software_Write1Byte(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data);
uint8_t I2C_Software_Read1Byte(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t *REG_data);
uint8_t I2C_Software_WriteNByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t len, uint8_t *buf);
uint8_t I2C_Software_ReadNByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t len, uint8_t *buf);
uint8_t I2C_Software_Transmit(uint8_t SlaveAddress, uint8_t len, uint8_t *buf);
uint8_t I2C_Software_Receive(uint8_t SlaveAddress, uint8_t len, uint8_t *buf);



#endif /* INC_DEV_I2C_SOFTWARE_H_ */
