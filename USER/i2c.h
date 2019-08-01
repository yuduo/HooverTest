/*
 * 	i2c.h
 *
 *	Created on: Jun 25, 2013
 *		Author: Denis aka caat
 */

#ifndef I2C_H_
#define I2C_H_

//#include <stdint.h>
//#include "stm32f10x_gpio.h"

#define ULTRASONIC_I2CADDR 0x6c
#define DISTANCE_I2CADDR 	0x5A

extern int I2Cerror;		//need them to make local!;
extern int I2Cerrorcount;	//need to make them local;

#define I2C_ULTRASONIC_DATA_PORT	GPIOC
#define I2C_ULTRASONIC_DATA_PIN		GPIO_Pin_7

#define I2C_ULTRASONIC_CLK_PORT		GPIOC
#define I2C_ULTRASONIC_CLK_PIN		GPIO_Pin_6

#define READ_DATA()  GPIO_ReadInputDataBit(I2C_ULTRASONIC_DATA_PORT, I2C_ULTRASONIC_DATA_PIN)

#define SDAH GPIO_WriteBit(I2C_ULTRASONIC_DATA_PORT, I2C_ULTRASONIC_DATA_PIN,   Bit_SET);
#define SDAL GPIO_WriteBit(I2C_ULTRASONIC_DATA_PORT, I2C_ULTRASONIC_DATA_PIN,   Bit_RESET);

#define SCLH GPIO_SetBits(I2C_ULTRASONIC_CLK_PORT, I2C_ULTRASONIC_CLK_PIN);
#define SCLL GPIO_ResetBits(I2C_ULTRASONIC_CLK_PORT, I2C_ULTRASONIC_CLK_PIN);

void I2C1_GPIO_Init(void);
void I2C_delay(void);
void I2C1_Start(void);
void I2C1_Stop(void);
void I2C1_Ack(void);
void I2C1_NoAck(void);
void I2C1_SendByte(unsigned char SendByte);
void I2C1_WaitAck(void);
uint8_t I2C1_ReceiveByte(void);
uint8_t I2C1_ReadByte(uint8_t i2c_addr, uint8_t reg_addr);
void I2C1_WriteByte(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data);
#endif /* I2C_H_*/
