#include "sys.h"
#include <stdarg.h>
#include <stdio.h>

void USART_PutChar(uint8_t ch)
{
	uint16_t i;
	USART1->DR = ch;	
	for(i=0;i<890;i++);
}

void USART_PutString(uint8_t * str)
{	
	while(*str != 0)
	{
		uint16_t i;
		USART_SendData(USART1, (unsigned char) *str);
		for(i=0;i<890;i++);
			str++;		
	}
}

void uasrt_write(uint8_t *buff,int len)
{
	int i;
	for(i=0;i<len;i++)
	{
		USART_PutChar(*buff++);
	}
}

void USART1_IRQHandler(void)
{
	int sr = USART1->SR;

    if (sr & USART_FLAG_TXE)
    {
        USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    }

    if (sr & USART_FLAG_RXNE)
    {
        unsigned char c = USART1->DR;
    }  
} 

void usartx_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO,ENABLE);//开启端口B和复用功能时钟
	GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);//使能端口重映射

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate =115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE,ENABLE);
	USART_Cmd(USART1, ENABLE);
}

void proc_uart_task(void)
{
}

void usart_write(uint8_t *buf,int len)
{
	int i;
 	for (i=0;i<len;i++) 
	{
		USART_PutChar(buf[i]);
	}
}

char string[512];
void log_printf(const char *format,...)
{
	va_list ap;
	va_start(ap,format);
	vsprintf(string,format,ap);

	USART_PutString((uint8_t *)string);
	va_end(ap);
}

