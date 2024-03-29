/***************************************************
* Author	:	lijinzhi
* Data		:	2011.8.15
* Descirbe  :	USART2
***************************************************/
#include "stdio.h"
#include "string.h"
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_it.h"
#include "MCUPeriph_Init.h"
/***************************************************
* Descirbe  :
* Input     :
* Output    :
* Attention :
* author 	:
***************************************************/
void USART2_GPIOConfig(void)
{
   GPIO_InitTypeDef	GPIO_InitStruct;
 #if(0)  
   /* 配置 USART1 Tx (PA2) */
   GPIO_InitStruct.GPIO_Pin 	= GPIO_Pin_2;
   GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_AF_PP;	 			/* 复用推挽输出模式 */
   GPIO_InitStruct.GPIO_Speed 	= GPIO_Speed_50MHz;				/* 输出最大频率为50MHz */
   GPIO_Init(GPIOA, &GPIO_InitStruct);
    
   /* 配置 USART1 Rx (PA3)  */
   GPIO_InitStruct.GPIO_Pin 	= GPIO_Pin_3;
   GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_IN_FLOATING;		/* 浮空输入模式 */
   GPIO_Init(GPIOA, &GPIO_InitStruct);
   #endif
}
void USART2_NVICConfing(void)
{
  NVIC_InitTypeDef NVIC_InitStruct;
 	
  NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0; 
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);
}
void USART2_Config(void)
{
    USART_InitTypeDef USART_InitStruct;
	/* USART and USART2 configured as follow:
        - BaudRate = 9600 baud  
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled */
 	USART_InitStruct.USART_BaudRate            = 9600;	  					
	USART_InitStruct.USART_StopBits            = USART_StopBits_1;	 
	USART_InitStruct.USART_WordLength          = USART_WordLength_8b; 
	USART_InitStruct.USART_Parity              = USART_Parity_No;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

	/* 配置串口2 */
	USART_Init(USART2, &USART_InitStruct);
	/* 串口1接收中断使能 USART_IT_RXNE : 接收不空中断 */
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	/* 使能串口2 */
	USART_Cmd(USART2, ENABLE);
}
void USART2_SendString(char* data)
{
   unsigned char i = 0;
   for (i=0; data[i] != '\0'; i++)
   {
	  USART_SendData( USART2, data[i]);
	  while( USART_GetFlagStatus(USART2 , USART_FLAG_TXE) == RESET);
		/* 发送数据直到 USART_FLAG_TXE 发送空为真,就可以进行下次发送 */
   }
}
