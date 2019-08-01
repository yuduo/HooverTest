

#include "sys.h"
 int I2Cerror;		//need them to make local!;
 int I2Cerrorcount;	//need to make them local

//I2C1的模拟IO口初始化
void I2C1_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;


	GPIO_InitStructure.GPIO_Pin =I2C_ULTRASONIC_CLK_PIN;//SDA1_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2C_ULTRASONIC_CLK_PORT, &GPIO_InitStructure);//SDA1_PORT

	GPIO_InitStructure.GPIO_Pin =I2C_ULTRASONIC_DATA_PIN ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2C_ULTRASONIC_DATA_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(I2C_ULTRASONIC_DATA_PORT, I2C_ULTRASONIC_DATA_PIN);
	

}

void I2C_delay(void)
{
//   uint8_t i = 10;  //3 for 400khz
//
//   while(i)
//   {
//		i--;
//   }
	int8_t i;
	for(i = 11; i > 0; i--);//3->8->11->20
}

void I2C1_Start(void)
{
	SDAH;
	SCLH;
	I2C_delay();
	SDAL;
	I2C_delay();
	SCLL;
	I2C_delay();
	//while(1);

}

void I2C1_Stop(void)
{
	SCLL;
	I2C_delay();
	SDAL;
	I2C_delay();
	SCLH;
	I2C_delay();
	SDAH;
	I2C_delay();
}

void I2C1_Ack(void)
{ 
	SCLL;
	I2C_delay();
	SDAL;
	I2C_delay();
	SCLH;
	I2C_delay();
	SCLL;
	I2C_delay();
}


void I2C1_NoAck(void)
{ 
	SCLL;
	I2C_delay();
	SDAH;
	I2C_delay();
	SCLH;
	I2C_delay();
	SCLL;
	I2C_delay();
}

void I2C1_SendByte(unsigned char SendByte) 
{
//    int8_t i = 8;
	unsigned char i = 8;
    while(i--)
//	for(; i > 0; i--)	
    {
        SCLL;
        I2C_delay();
        if(SendByte & 0x80) 
		{
			SDAH;
		}

        if(!(SendByte & 0x80))
		{
			SDAL;
		}  
        SendByte<<=1;
        I2C_delay();
		SCLH;
        I2C_delay();
    }
    SCLL;
}

uint8_t I2C1_ReceiveByte(void)  
{ 
//	int InputLevel[9];
  /*  unsigned char i=8;
    unsigned char ReceiveByte=0;


    SDAH;    
    while(i--)
    {
      ReceiveByte<<=1;      
      SCLL;
      I2C_delay();
      SCLH;
      I2C_delay(); 
      if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)==1)
      {
        ReceiveByte|=0x01;
      }
    }
    SCLL;
    return ReceiveByte;*/
	
	unsigned char i=8;
	unsigned char ReceiveByte=0;
	uint8_t t;
	uint8_t data;

	SDAH;    
	while(i--)
	{
		ReceiveByte<<=1;      
		SCLL;
		I2C_delay();
		SCLH;
		/*
		InputLevel[0]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
		InputLevel[1]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
		InputLevel[2]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
		InputLevel[3]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
		InputLevel[4]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
		InputLevel[5]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
		InputLevel[6]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
		InputLevel[7]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
		InputLevel[8]=GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11);
		if((InputLevel[0]+InputLevel[1]+InputLevel[2]+InputLevel[3]+InputLevel[4]+InputLevel[5]+InputLevel[6]+InputLevel[7]+InputLevel[8])>=4)
		{
			ReceiveByte|=0x01;
		}
		*/ 
		data = 0;
		for(t = 0; t < 8; t++)
		{
			//data += GPIO_ReadInputDataBit(SDA1_PORT, SDA1_PIN);	
			//data += GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15);	
			data += (GPIOB->IDR & GPIO_Pin_9)?1:0;//GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15);	
		}
		if(data >= 4)
		{
			ReceiveByte |= 0x01;
		}

	}
	SCLL;
	return ReceiveByte;	
}

void I2C1_WaitAck(void)   
{
	SCLL;
	I2C_delay();
	SDAH;   
	I2C_delay();
	SCLH;
	I2C_delay();
	if(GPIOB->IDR & GPIO_Pin_9) 
	//if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15)==1) 
	//if(GPIO_ReadInputDataBit(SDA1_PORT, SDA1_PIN)==1) 
	{
		I2Cerror=1; 
		//DEBUG_LEDon; 
		I2Cerrorcount++;
	}
	SCLL;
 
}


uint8_t I2C1_ReadByte(uint8_t i2c_addr, uint8_t reg_addr)
{
	uint8_t res;
	
    I2C1_Start();
    I2C1_SendByte((i2c_addr & 0xfe));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(reg_addr);//fe-0(Write)
    I2C1_WaitAck();
	
    //I2C1_Stop();
    
    I2C1_NoAck();
    //I2C_delay();
    //I2C_delay();
    //I2C_delay();
    
    I2C1_Start();
    I2C1_SendByte((i2c_addr | 0x01));//fe-0(Write)
    I2C1_WaitAck();
    

    res = I2C1_ReceiveByte(); //receive
    
    //I2C1_WaitAck();
    I2C1_NoAck();
	I2C1_Stop();
	return res;
}


void I2C1_WriteByte(uint8_t i2c_addr, uint8_t reg_addr, uint8_t data)
{
//	uint8_t res;
	
	
    I2C1_Start();
    I2C1_SendByte((i2c_addr & 0xfe));//fe-0(Write)
    I2C1_WaitAck();
    I2C1_SendByte(reg_addr);//fe-0(Write)
    I2C1_WaitAck();
    
    //I2C1_Start();
    //I2C1_SendByte((i2c_addr | 0x01));//fe-0(Write)
    //I2C1_WaitAck();
    I2C1_SendByte(data);//fe-0(Write)
    
    I2C1_NoAck();
	I2C1_Stop();
}

